# Copyright 2022 Clearpath Robotics, Inc.
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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace')
]


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    localization_params_arg = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_navigation, 'config', 'localization.yaml']),
        description='Localization parameters')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_navigation, 'maps', 'warehouse.yaml']),
        description='Full path to map yaml file to load')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    localization = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
            launch_arguments={'namespace': namespace,
                              'map': LaunchConfiguration('map'),
                              'use_sim_time': use_sim_time,
                              'params_file': LaunchConfiguration('params')}.items()),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(localization_params_arg)
    ld.add_action(map_arg)
    ld.add_action(localization)
    return ld
