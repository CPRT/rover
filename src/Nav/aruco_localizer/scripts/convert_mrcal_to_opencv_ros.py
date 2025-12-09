#!/usr/bin/env python3
import argparse
import sys
import numpy as np
import mrcal
import yaml


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert mrcal cameramodel to ROS 2 YAML format."
    )
    parser.add_argument(
        "--input",
        "-i",
        type=str,
        default="camera-0.cameramodel",
        help="Path to input mrcal .cameramodel file",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default="camera-0.yaml",
        help="Path to output .yaml file",
    )
    parser.add_argument(
        "--name",
        "-n",
        type=str,
        default="camera",
        help="Name of the camera (default: 'camera')",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # 1. Load the mrcal model
    try:
        model = mrcal.cameramodel(args.input)
    except Exception as e:
        print(f"Error loading model file '{args.input}': {e}")
        sys.exit(1)

    # 2. Extract Data using correct API
    # documentation: lensmodel, intrinsics_data = model.intrinsics()
    lensmodel, intrinsics_data = model.intrinsics()

    # documentation: width, height = model.imagersize()
    width, height = model.imagersize()

    # 3. Parse Intrinsics (Core 4 params: fx, fy, cx, cy)
    fx = intrinsics_data[0]
    fy = intrinsics_data[1]
    cx = intrinsics_data[2]
    cy = intrinsics_data[3]

    # Raw distortion coefficients from mrcal
    raw_dist = intrinsics_data[4:]

    # 4. Map to ROS 2 Distortion Models
    ros_distortion_model = "plumb_bob"
    ros_dist_coeffs = []

    if lensmodel == "LENSMODEL_OPENCV8":
        ros_distortion_model = "rational_polynomial"
        # ROS rational_polynomial expects 8: [k1, k2, p1, p2, k3, k4, k5, k6]
        # mrcal OPENCV8 matches this exactly.
        ros_dist_coeffs = raw_dist

    elif lensmodel == "LENSMODEL_OPENCV5":
        ros_distortion_model = "plumb_bob"
        # ROS plumb_bob expects 5: [k1, k2, p1, p2, k3]
        # mrcal OPENCV5 matches this exactly.
        ros_dist_coeffs = raw_dist

    elif lensmodel == "LENSMODEL_OPENCV4":
        ros_distortion_model = "plumb_bob"
        # mrcal OPENCV4 provides 4: [k1, k2, p1, p2]
        # ROS plumb_bob expects 5: [k1, k2, p1, p2, k3]
        # We append 0.0 for k3.
        ros_dist_coeffs = np.append(raw_dist, 0.0)

    elif lensmodel == "LENSMODEL_PINHOLE":
        ros_distortion_model = "plumb_bob"
        # Pinhole has 0 distortion params.
        ros_dist_coeffs = np.zeros(5, dtype=np.float64)

    elif "SPLINED" in lensmodel:
        print(
            f"ERROR: Cannot directly convert splined model '{lensmodel}' to standard OpenCV/ROS parameters."
        )
        print("You must re-optimize to an OpenCV model or use a lookup table.")
        sys.exit(1)

    else:
        print(
            f"WARNING: Unknown lens model '{lensmodel}'. Defaulting to passing raw coeffs as plumb_bob."
        )
        ros_dist_coeffs = raw_dist

    # 5. Construct Matrices
    # Camera Matrix (K)
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    # Rectification Matrix (R) - Identity for monocular/unrectified
    R = np.eye(3, dtype=np.float64)

    # Projection Matrix (P) - K[I|0]
    P = np.zeros((3, 4), dtype=np.float64)
    P[:3, :3] = K

    # Print matrices for verification
    print(f"--- Loaded Model: {args.input} ---")
    print(f"Image Size: {width} x {height}")
    print(f"Lens Model: {lensmodel}")
    print(f"Distortion Model: {ros_distortion_model} (ROS Name)")
    print("")

    # Manual formatting for K to ensure exact copy-paste syntax with commas
    print("Copy pasteable camera matrices for OpenCV:")
    print("K = np.array([")
    print(f"    [{K[0,0]:.9f}, {K[0,1]:.9f}, {K[0,2]:.9f}],")
    print(f"    [{K[1,0]:.9f}, {K[1,1]:.9f}, {K[1,2]:.9f}],")
    print(f"    [{K[2,0]:.9f}, {K[2,1]:.9f}, {K[2,2]:.9f}]")
    print("])")

    # Use numpy's array2string to handle the D array gracefully with commas
    # We use threshold=np.inf to ensure it doesn't truncate large arrays with "..."
    d_str = np.array2string(
        ros_dist_coeffs, separator=", ", threshold=np.inf, precision=9
    )
    print(f"D = np.array({d_str})")
    print("")

    # 6. Prepare Dictionary for YAML
    ros_camera_info = {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": args.name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": K.flatten().tolist()},
        "distortion_model": ros_distortion_model,
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(ros_dist_coeffs),
            "data": ros_dist_coeffs.tolist()
            if isinstance(ros_dist_coeffs, np.ndarray)
            else list(ros_dist_coeffs),
        },
        "rectification_matrix": {"rows": 3, "cols": 3, "data": R.flatten().tolist()},
        "projection_matrix": {"rows": 3, "cols": 4, "data": P.flatten().tolist()},
    }

    # 7. Write to YAML file
    try:
        with open(args.output, "w") as f:
            yaml.safe_dump(ros_camera_info, f, default_flow_style=None, sort_keys=False)
        print(f"\nSuccessfully wrote ROS 2 calibration to: {args.output}")
    except Exception as e:
        print(f"Error writing output file: {e}")


if __name__ == "__main__":
    main()
