import cv2
import argparse
import os
import time
import sys


def main():
    # 1. Setup Argument Parser
    parser = argparse.ArgumentParser(description="Camera Calibration Image Capturer")

    parser.add_argument(
        "--camera",
        type=str,
        default="0",
        help="Camera input device index (e.g., '0') or path to video file.",
    )
    parser.add_argument(
        "--width", type=int, default=None, help="Desired camera width resolution."
    )
    parser.add_argument(
        "--height", type=int, default=None, help="Desired camera height resolution."
    )
    parser.add_argument(
        "--checkerboard_width",
        type=int,
        required=True,
        help="Number of inner corners along the width of the checkerboard.",
    )
    parser.add_argument(
        "--checkerboard_height",
        type=int,
        required=True,
        help="Number of inner corners along the height of the checkerboard.",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        required=True,
        help="Root directory to save the captured images.",
    )
    parser.add_argument(
        "--name",
        type=str,
        default="frame",
        help="Base name for the saved files and subdirectory.",
    )
    parser.add_argument(
        "--processing_interval",
        type=float,
        default=1.0,
        help="Time in seconds between attempting to detect and save a checkerboard.",
    )

    args = parser.parse_args()

    # 2. Setup Camera Variables
    camera_input_device = args.camera
    if camera_input_device.isdigit():
        camera_input_device = int(camera_input_device)

    # 3. Initialize Camera
    print(f"Opening camera: {camera_input_device}...")
    cap = cv2.VideoCapture(camera_input_device, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("Error: Could not open video device.")
        return

    if args.width is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height is not None:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Camera resolution: {actual_width}x{actual_height}")
    print(f"Processing interval: {args.processing_interval} seconds")

    # 4. Setup Output Directory
    folder_name = (
        f"{args.name}-"
        f"{args.checkerboard_width}x{args.checkerboard_height}-"
        f"{actual_width}x{actual_height}"
    )

    full_output_path = os.path.join(args.output_dir, folder_name)

    if not os.path.exists(full_output_path):
        os.makedirs(full_output_path)
        print(f"Created output directory: {full_output_path}")
    else:
        print(f"Saving to existing directory: {full_output_path}")

    # 5. Main Loop
    last_process_time = 0
    saved_count = 0
    pattern_size = (args.checkerboard_width, args.checkerboard_height)

    # Store the last successfully detected corners here
    last_corners = None

    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        display_frame = frame.copy()
        current_time = time.time()

        # Check if enough time has passed to attempt processing
        if (current_time - last_process_time) >= args.processing_interval:
            last_process_time = current_time

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

            if found:
                saved_count += 1

                # Update the persistent variable with the new corners
                last_corners = corners

                filename = (
                    f"{args.name}{saved_count:05d}-"
                    f"{args.checkerboard_width}x{args.checkerboard_height}-"
                    f"{actual_width}x{actual_height}.png"
                )

                filepath = os.path.join(full_output_path, filename)

                # Save the CLEAN frame
                cv2.imwrite(filepath, frame)
                print(f"Saved: {filename} ({saved_count} total)")

        # Draw the last known corners on the current frame
        if last_corners is not None:
            cv2.drawChessboardCorners(display_frame, pattern_size, last_corners, True)

        cv2.imshow("Calibration Capture", display_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
