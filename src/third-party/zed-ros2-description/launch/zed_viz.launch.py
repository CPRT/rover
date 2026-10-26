# Copyright (c) 2026 StereoLabs 
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

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command
)
from launch_ros.actions import (
    Node
)

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def launch_setup(context, *args, **kwargs):
    return_array = []
    
    camera_model = LaunchConfiguration('camera_model')    
    camera_model_val = camera_model.perform(context)  
    camera_name_val = 'zed'
    
    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory('zed_description'),
        'urdf',
        'zed_descr.urdf.xacro'
)
    
    # Xacro command with options
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(xacro_path)
    xacro_command.append(' ')
    xacro_command.append('camera_name:=')
    xacro_command.append(camera_name_val)
    xacro_command.append(' ')
    xacro_command.append('camera_model:=')
    xacro_command.append(camera_model_val)

    # Robot State Publisher node
    rsp_name = camera_model_val + '_state_publisher'
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=rsp_name,
        parameters=[{
            'robot_description': Command(xacro_command)
        }],
        remappings=[('robot_description', camera_name_val+'_description')]
    )
    return_array.append(rsp_node) 
    
    # Rviz2 Configurations
    config_rviz2 = os.path.join(
        get_package_share_directory('zed_description'),
        'rviz2',
        'zed_description.rviz'
    )
    
    # Rviz2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name_val,
        executable='rviz2',
        name=camera_name_val +'_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
        parameters=[]
    )
    return_array.append(rviz2_node)

    return return_array

def generate_launch_description():
    return LaunchDescription(
        [
            # Declare launch arguments            
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=[
                	'zed', 'zedm', 'zed2', 'zed2i', 
                	'zedx', 'zedxm', 'zedxnano',
                	'zedxhdr', 'zedxhdrmini', 'zedxhdrmax', 
                	'virtual', 
                	'zedxonegs', 'zedxones', 'zedxone4k', 'zedxonehdr', 'zedxonecore'
                ]),
            OpaqueFunction(function=launch_setup)
        ]
    )
