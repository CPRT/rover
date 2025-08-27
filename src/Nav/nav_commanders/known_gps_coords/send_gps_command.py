import argparse
import os
import yaml
import subprocess


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Send GPS coordinates to ROS2 service."
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--file", type=str, help="YAML file containing GPS coordinates (lat, lon)."
    )
    group.add_argument(
        "--coords",
        nargs=2,
        type=float,
        metavar=("LAT", "LON"),
        help="Latitude and Longitude as a pair of doubles.",
    )
    parser.add_argument(
        "--print",
        action="store_true",
        help="Print the command instead of executing it.",
    )
    return parser.parse_args()


def read_coordinates_from_file(file_path):
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"File '{file_path}' not found.")
    with open(file_path, "r") as file:
        data = yaml.safe_load(file)
        if "lat" not in data or "lon" not in data:
            raise ValueError("YAML file must contain 'lat' and 'lon' keys.")
        return data["lat"], data["lon"]


def construct_bash_command(lat, lon):
    return f"ros2 service call /commander/nav_to_gps_geopose interfaces/srv/NavToGPSGeopose '{{goal: {{position: {{latitude: {lat}, longitude: {lon}}}}}}}'"


def main():
    args = parse_arguments()

    if args.file:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, args.file)
        lat, lon = read_coordinates_from_file(file_path)
    else:
        lat, lon = args.coords

    bash_command = construct_bash_command(lat, lon)
    if args.print:
        print(f"Command to execute: {bash_command}")
    else:
        print(f"Executing command: {bash_command}")
        subprocess.run(bash_command, shell=True, check=True)


if __name__ == "__main__":
    main()
