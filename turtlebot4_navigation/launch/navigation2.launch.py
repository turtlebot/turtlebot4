# Copyright 2021 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('slam', default_value='True',
                          choices=['True', 'False'],
                          description='Use SLAM'),
]


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_bringup_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'bringup_launch.py']
    )
    params_file = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'config', 'nav2.yaml']
    )
    map_file = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'maps', 'turtlebot4_map.yaml']
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch]),
        launch_arguments=[('slam', LaunchConfiguration('slam')),
                          ('params_file', params_file),
                          ('map', map_file)
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_bringup)
    return ld
