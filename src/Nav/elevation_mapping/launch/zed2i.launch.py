import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    share_dir = get_package_share_directory("elevation_mapping")
    config_dir = os.path.join(share_dir, "config")
    list_params = []
    list_files = [
        "robots/zed2i_robot.yaml",
        "elevation_maps/zed2i_map.yaml",
        "sensor_processors/zed2i_processor.yaml",
        "postprocessing/traversability_pipeline.yaml",
    ]
    for file in list_files:
        if not os.path.isfile(os.path.join(config_dir, file)):
            raise FileNotFoundError("File not found: " + os.path.join(config_dir, file))
        list_params.append(os.path.join(config_dir, file))

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="elevation_mapping",
                executable="elevation_mapping",
                name="elevation_mapping",
                output="screen",
                parameters=list_params,
            ),
        ]
    )
