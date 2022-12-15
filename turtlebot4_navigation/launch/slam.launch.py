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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace

from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Use synchronous SLAM'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace')
]


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')

    namespace = LaunchConfiguration('namespace')
    sync = LaunchConfiguration('sync')

    slam_params_arg = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_navigation, 'config', 'slam.yaml']),
        description='Robot namespace')

    slam_params = RewrittenYaml(
        source_file=LaunchConfiguration('params'),
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/scan', 'scan'),
        ('/map', 'map'),
        ('/map_metadata', 'map_metadata'),
    ]

    slam = GroupAction([
        PushRosNamespace(namespace),

        Node(package='slam_toolbox',
             executable='sync_slam_toolbox_node',
             name='slam_toolbox',
             output='screen',
             parameters=[
               slam_params,
               {'use_sim_time': LaunchConfiguration('use_sim_time')}
             ],
             remappings=remappings,
             condition=IfCondition(sync)),

        Node(package='slam_toolbox',
             executable='async_slam_toolbox_node',
             name='slam_toolbox',
             output='screen',
             parameters=[
               slam_params,
               {'use_sim_time': LaunchConfiguration('use_sim_time')}
             ],
             remappings=remappings,
             condition=UnlessCondition(sync))
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam_params_arg)
    ld.add_action(slam)
    return ld
