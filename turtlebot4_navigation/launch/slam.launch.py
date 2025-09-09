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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap

from nav2_common.launch import RewrittenYaml

pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Use synchronous SLAM'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.'),  # noqa: E501
    DeclareLaunchArgument('use_lifecycle_manager', default_value='false',
                          choices=['true', 'false'],
                          description='Enable bond connection during node activation'),
    DeclareLaunchArgument('params',
                          default_value=PathJoinSubstitution([pkg_turtlebot4_navigation, 'config', 'slam.yaml']),  # noqa: E501
                          description='Path to the SLAM Toolbox configuration file')
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    sync = LaunchConfiguration('sync')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    slam_params = LaunchConfiguration('params')

    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str

    launch_slam_sync = PathJoinSubstitution(
        [pkg_slam_toolbox, 'launch', 'online_sync_launch.py'])

    launch_slam_async = PathJoinSubstitution(
        [pkg_slam_toolbox, 'launch', 'online_async_launch.py'])

    rewritten_slam_params = RewrittenYaml(
        source_file=slam_params,
        root_key=namespace_str,
        param_rewrites={
            'map_name': namespace_str + '/map',
            'scan_topic': namespace_str + '/scan',
        },
        convert_types=True,
    )

    slam = GroupAction([
        PushRosNamespace(namespace),

        SetRemap('/tf', namespace_str + '/tf'),
        SetRemap('/tf_static', namespace_str + '/tf_static'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam_sync),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('autostart', autostart),
                ('use_lifecycle_manager', use_lifecycle_manager),
                ('slam_params_file', rewritten_slam_params)
            ],
            condition=IfCondition(sync)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam_async),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('autostart', autostart),
                ('use_lifecycle_manager', use_lifecycle_manager),
                ('slam_params_file', rewritten_slam_params)
            ],
            condition=UnlessCondition(sync)
        )
    ])

    return [slam]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
