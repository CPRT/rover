# Copyright (c) 2024 Angsa Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_type = LaunchConfiguration("type")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "type",
                default_value="gps",
                description="Mission type (gps, aruco10m, aruco20m, mallet, pick, bottle, indoor_spiral)",
            ),
            Node(
                package="nav_commanders",
                name="unified_nav_commander",
                executable="cprt_commander_node",
                parameters=[{"type": mission_type}],
            ),
        ]
    )
