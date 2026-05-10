#!/usr/bin/env python3
import argparse
import subprocess
import sys


def main():
    # The valid mission types matching cprt_commander.py configs
    missions = [
        "gps",
        "aruco10m",
        "aruco20m",
        "mallet",
        "pick",
        "bottle",
        "indoor_spiral",
    ]

    parser = argparse.ArgumentParser(
        description="Easily set the mission type for cprt_commander."
    )
    group = parser.add_mutually_exclusive_group(required=True)

    # Create a flag for each mission type
    for m in missions:
        group.add_argument(
            f"--{m}",
            action="store_const",
            dest="mission",
            const=m,
            help=f"Set mission to {m}",
        )

    args = parser.parse_args()

    print(f"Setting mission type to: {args.mission}")

    # Run the ros2 topic pub command
    cmd = [
        "ros2",
        "topic",
        "pub",
        "--once",
        "/commander/mission_type",
        "std_msgs/msg/String",
        f"{{data: '{args.mission}'}}",
    ]

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to publish mission type: {e}", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print(
            "Error: 'ros2' command not found. Make sure you have sourced your ROS 2 environment.",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
