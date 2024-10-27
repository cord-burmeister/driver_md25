# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    Command,
    PythonExpression,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
)

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro
from launch.actions import OpaqueFunction


# Configure ROS nodes for launch
def generate_launch_description():

    # args that can be set from the command line or a default will be used
    #mecanum_launch_arg = DeclareLaunchArgument(
    #    "mecanum", default_value="True", description="Flag indicating to use mecanum wheels; skid drive otherwise"
    #)
    #mecanum_launch_value = LaunchConfiguration('mecanum')

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('driver_md25')


    # Bridge ROS topics and Gazebo messages for establishing communication
    busmaster = Node(
        package='driver_md25',
        executable='bus_master',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'bus_master.yaml'),
        }],
        output='screen'
    )

    return LaunchDescription([
        busmaster,
    ])
