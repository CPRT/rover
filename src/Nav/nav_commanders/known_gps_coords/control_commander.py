#!/usr/bin/env python3
import argparse
import os
import yaml
import subprocess
import sys


def read_coordinates_from_file(file_path):
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"File '{file_path}' not found.")
    with open(file_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)
        if "lat" not in data or "lon" not in data:
            raise ValueError("YAML file must contain 'lat' and 'lon' keys.")
        return data["lat"], data["lon"]


def construct_gps_command(lat, lon):
    return f"ros2 service call /commander/nav_to_gps_geopose interfaces/srv/NavToGPSGeopose '{{goal: {{position: {{latitude: {lat}, longitude: {lon}}}}}}}'"


def handle_mission(args):
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


def handle_gps(args):
    if args.file:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, args.file)
        lat, lon = read_coordinates_from_file(file_path)
    else:
        lat, lon = args.coords

    bash_command = construct_gps_command(lat, lon)
    if args.print:
        print(f"Command to execute: {bash_command}")
    else:
        print(f"Executing command: {bash_command}")
        subprocess.run(bash_command, shell=True, check=True)


def handle_cancel(args):
    bash_command = "ros2 service call /commander/cancel_nav std_srvs/srv/Trigger '{}'"
    print("Canceling navigation...")
    subprocess.run(bash_command, shell=True, check=True)


def main():
    parser = argparse.ArgumentParser(
        description="Control commander for setting mission types and sending GPS coordinates."
    )
    
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Subparser for 'mission'
    mission_parser = subparsers.add_parser("mission", help="Set the mission type for cprt_commander")
    mission_group = mission_parser.add_mutually_exclusive_group(required=True)
    missions = [
        "gps",
        "aruco10m",
        "aruco20m",
        "mallet",
        "pick",
        "bottle",
        "indoor_spiral",
    ]
    for m in missions:
        mission_group.add_argument(
            f"--{m}",
            action="store_const",
            dest="mission",
            const=m,
            help=f"Set mission to {m}",
        )
    mission_parser.set_defaults(func=handle_mission)

    # Subparser for 'gps'
    gps_parser = subparsers.add_parser("gps", help="Send GPS coordinates to ROS2 service")
    gps_group = gps_parser.add_mutually_exclusive_group(required=True)
    gps_group.add_argument(
        "--file", type=str, help="YAML file containing GPS coordinates (lat, lon)."
    )
    gps_group.add_argument(
        "--coords",
        nargs=2,
        type=float,
        metavar=("LAT", "LON"),
        help="Latitude and Longitude as a pair of doubles.",
    )
    gps_parser.add_argument(
        "--print",
        action="store_true",
        help="Print the command instead of executing it.",
    )
    gps_parser.set_defaults(func=handle_gps)

    # Subparser for 'cancel'
    cancel_parser = subparsers.add_parser("cancel", help="Cancel current navigation")
    cancel_parser.set_defaults(func=handle_cancel)

    args = parser.parse_args()
    
    # Execute the associated function for the chosen subparser
    args.func(args)


if __name__ == "__main__":
    main()
