import subprocess
import time
import itertools

# List of your specific rover services
SERVICES = [
    "/Left_back_wheel_arm_joint/clear_errors",
    "/Left_back_wheel_joint/clear_errors",
    "/Left_front_wheel_arm_joint/clear_errors",
    "/Left_front_wheel_joint/clear_errors",
    "/Right_back_wheel_arm_joint/clear_errors",
    "/Right_back_wheel_joint/clear_errors",
    "/Right_front_wheel_arm_joint/clear_errors",
    "/Right_front_wheel_joint/clear_errors",
]

SERVICE_TYPE = "std_srvs/srv/Trigger"
PAYLOAD = "{}"  # Trigger services require an empty YAML dictionary
INTERVAL = 2.0  # Seconds to wait between each call


def main():
    print(f"Starting service rotation. Calling one service every {INTERVAL} seconds.")
    print("Press Ctrl+C to stop.\n")

    try:
        # itertools.cycle creates an infinite loop over the list
        for service in itertools.cycle(SERVICES):
            print(f"Calling: {service}")

            # Construct the terminal command exactly as you would type it
            command = ["ros2", "service", "call", service, SERVICE_TYPE, PAYLOAD]

            # Execute the command
            # capture_output=True prevents the raw ROS 2 stdout from cluttering your terminal
            result = subprocess.run(command, capture_output=True, text=True)

            # Check if the command was successful
            if result.returncode == 0:
                print("  -> Success")
            else:
                # If the service isn't available, it will print the error
                print(f"  -> Failed: {result.stderr.strip()}")

            # Sleep before the next command
            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\nScript stopped by user. Exiting cleanly.")


if __name__ == "__main__":
    main()
