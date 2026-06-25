#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
import time
import sys
from scipy.spatial.transform import Rotation as R

# ================= USER CONFIGURATION =================
CAMERA_INPUT = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_B0561_16MP_Klarity_SN0001-video-index0"
CAMERA_YAML_PATH = "camera-0.yaml"  # Path to your intrinsics file
OUTPUT_BOARD_YAML = "camera-0_ArucoBoard_Tag6-Tag7.yaml"
TAG_SIZE = 0.1995  # Size of the printed tag in METERS (Measure this!)
ID_ANCHOR = 6  # The ID you want to be at (0,0,0)
ID_TARGET = 7  # The ID you want to calculate the position of
CAPTURE_INTERVAL = 0.2  # Seconds between captures
WIDTH = 1280  # Camera width
HEIGHT = 720  # Camera height
# ======================================================


def load_camera_info(yaml_path):
    """Parses the ROS-style camera YAML file."""
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    # Extract Camera Matrix
    camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(
        3, 3
    )

    # Extract Distortion Coefficients
    dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float64)

    # Setup for 8 coefficients (Rational Polynomial) if needed
    if dist_coeffs.size > 5:
        dist_coeffs = dist_coeffs.reshape(1, -1)

    return camera_matrix, dist_coeffs


def main():
    # 1. Load Intrinsics
    try:
        mtx, dist = load_camera_info(CAMERA_YAML_PATH)
        print(f"Loaded camera parameters from {CAMERA_YAML_PATH}")
    except FileNotFoundError:
        print(f"Error: Could not find {CAMERA_YAML_PATH}")
        sys.exit(1)

    # 2. Setup Camera
    cap = cv2.VideoCapture(CAMERA_INPUT, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    captured_frames = []
    last_capture_time = time.time()

    print("\n" + "=" * 50)
    print(f" PHASE 1: DATA COLLECTION")
    print(f" Point camera at BOTH tags ({ID_ANCHOR} and {ID_TARGET}).")
    print(f" Capturing every {CAPTURE_INTERVAL}s when valid.")
    print(f" Press 'ENTER' in the window to finish and process.")
    print("=" * 50 + "\n")

    # 3. Capture Loop
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect just for visualization and validity check
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame,
            aruco_dict,
            parameters=parameters,
        )
        display_frame = frame.copy()

        valid_frame = False
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
            flat_ids = ids.flatten()
            if ID_ANCHOR in flat_ids and ID_TARGET in flat_ids:
                valid_frame = True

        current_time = time.time()

        # Capture logic
        if valid_frame and (current_time - last_capture_time > CAPTURE_INTERVAL):
            captured_frames.append(frame)  # Store raw frame in RAM
            last_capture_time = current_time
            print(f"Captured Frame #{len(captured_frames)}", end="\r")

            # Flash effect
            cv2.rectangle(display_frame, (0, 0), (1280, 720), (0, 255, 0), 10)

        # UI
        cv2.imshow("Calibration Capture (Press Enter to Stop)", display_frame)

        key = cv2.waitKey(1)
        if key == 13:  # Enter key
            break
        elif key == 27:  # ESC key
            print("Cancelled.")
            cap.release()
            sys.exit(0)

    cap.release()
    cv2.destroyAllWindows()

    if len(captured_frames) == 0:
        print("\nNo frames captured. Exiting.")
        sys.exit(0)

    # 4. Processing Phase
    print(f"\n\nPHASE 2: PROCESSING {len(captured_frames)} FRAMES...")

    relative_translations = []
    relative_rotations = []  # Quaternions

    for i, frame in enumerate(captured_frames):
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame,
            aruco_dict,
            parameters=parameters,
        )

        # We know both IDs exist because we filtered in Phase 1, but check again to be safe
        idx_anchor = np.where(ids == ID_ANCHOR)[0][0]
        idx_target = np.where(ids == ID_TARGET)[0][0]

        # Solve PnP for Anchor (Tag A)
        _, rvec_a, tvec_a = cv2.solvePnP(
            np.array(
                [
                    [-TAG_SIZE / 2, TAG_SIZE / 2, 0],
                    [TAG_SIZE / 2, TAG_SIZE / 2, 0],
                    [TAG_SIZE / 2, -TAG_SIZE / 2, 0],
                    [-TAG_SIZE / 2, -TAG_SIZE / 2, 0],
                ],
                dtype=np.float32,
            ),
            corners[idx_anchor],
            mtx,
            dist,
        )

        # Solve PnP for Target (Tag B)
        _, rvec_b, tvec_b = cv2.solvePnP(
            np.array(
                [
                    [-TAG_SIZE / 2, TAG_SIZE / 2, 0],
                    [TAG_SIZE / 2, TAG_SIZE / 2, 0],
                    [TAG_SIZE / 2, -TAG_SIZE / 2, 0],
                    [-TAG_SIZE / 2, -TAG_SIZE / 2, 0],
                ],
                dtype=np.float32,
            ),
            corners[idx_target],
            mtx,
            dist,
        )

        # Convert to Matrix
        R_a, _ = cv2.Rodrigues(rvec_a)
        T_cam_a = np.eye(4)
        T_cam_a[:3, :3] = R_a
        T_cam_a[:3, 3] = tvec_a.flatten()

        R_b, _ = cv2.Rodrigues(rvec_b)
        T_cam_b = np.eye(4)
        T_cam_b[:3, :3] = R_b
        T_cam_b[:3, 3] = tvec_b.flatten()

        # Calculate Transform from Anchor to Target
        # T_a_b = inv(T_cam_a) * T_cam_b
        T_a_b = np.linalg.inv(T_cam_a) @ T_cam_b

        relative_translations.append(T_a_b[:3, 3])
        relative_rotations.append(R.from_matrix(T_a_b[:3, :3]).as_quat())

    # 5. Compute Statistics & Average
    trans_array = np.array(relative_translations)
    avg_translation = np.mean(trans_array, axis=0)
    std_translation = np.std(trans_array, axis=0)

    # Average Rotations (Mean of Quaternions)
    quat_array = np.array(relative_rotations)
    # Simple mean and normalize is sufficient for clustered data
    mean_quat = np.mean(quat_array, axis=0)
    mean_quat /= np.linalg.norm(mean_quat)

    avg_rot_matrix = R.from_quat(mean_quat).as_matrix()

    # Calculate Rotation Jitter (RMSE in degrees)
    angular_errors = []
    for q in quat_array:
        # Angle between mean_quat and current q
        # 2 * arccos(|<q1, q2>|)
        dot = abs(np.dot(q, mean_quat))
        if dot > 1.0:
            dot = 1.0
        angle = 2 * np.arccos(dot)
        angular_errors.append(np.degrees(angle))
    rmse_rotation = np.sqrt(np.mean(np.array(angular_errors) ** 2))

    # 6. Generate Board Model
    # Anchor is at (0,0,0) with no rotation
    d = TAG_SIZE / 2.0
    corners_anchor = [[-d, d, 0.0], [d, d, 0.0], [d, -d, 0.0], [-d, -d, 0.0]]

    # Target is transformed by avg_translation and avg_rotation
    base_corners = np.array(
        [[-d, d, 0], [d, d, 0], [d, -d, 0], [-d, -d, 0]], dtype=np.float32
    )
    corners_target = []
    for point in base_corners:
        transformed = avg_rot_matrix @ point + avg_translation
        corners_target.append(transformed.tolist())

    # 7. Print Metrics
    print("\n" + "=" * 50)
    print(" VALIDATION METRICS")
    print("=" * 50)
    print(f"Frames Processed:   {len(captured_frames)}")
    print(f"Translation (XYZ):  {avg_translation} meters")
    print(f"Translation StdDev: {std_translation * 1000} mm")
    print(f"Rotation Jitter:    {rmse_rotation:.4f} degrees (RMSE)")

    if np.any(std_translation > 0.01):  # Warning if > 1cm jitter
        print(
            "\nWARNING: High translation variance! Check camera calibration or hold steady."
        )
    else:
        print("\nSUCCESS: Calibration looks stable.")

    # 8. Save to YAML
    output_data = {
        "board_description": "Custom Angled Aruco Board",
        "ids": [ID_ANCHOR, ID_TARGET],
        "tag_size": TAG_SIZE,
        "metrics": {
            "translation_std_mm": (std_translation * 1000).tolist(),
            "rotation_rmse_deg": float(rmse_rotation),
        },
        "corners": [corners_anchor, corners_target],
    }

    with open(OUTPUT_BOARD_YAML, "w") as outfile:
        yaml.dump(output_data, outfile, default_flow_style=None)

    print(f"\nModel saved to {OUTPUT_BOARD_YAML}")


if __name__ == "__main__":
    main()