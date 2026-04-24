#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share          = get_package_share_directory('autonomous_tb3')
    pkg_gazebo_ros         = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_share     = get_package_share_directory('nav2_bringup')

    # World file — loaded directly by gzserver so the full environment
    # (maze geometry, physics, floor tiles) matches what the map was built from.
    world_file            = os.path.join(package_share, 'worlds', 'tb3_maze_world.world')
    params_config_file    = os.path.join(package_share, 'config', 'tb3_nav_params.yaml')
    maze_map_config_file  = os.path.join(package_share, 'config', 'maze_map.yaml')
    slam_params_file      = os.path.join(package_share, 'config', 'slam_params_maze.yaml')
    # super_nav.rviz is configured for SLAM mode (live /map topic, no AMCL particle cloud)
    rviz_config_file      = os.path.join(package_share, 'config', 'super_nav.rviz')
    # Directory that contains the tb3_maze_world Gazebo model folder
    model_path            = os.path.join(package_share, 'worlds')

    # Extend GAZEBO_MODEL_PATH so gzserver can resolve <uri>model://tb3_maze_world</uri>
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = (
        f'{model_path}:{existing_model_path}' if existing_model_path else model_path
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    entity_name = LaunchConfiguration('entity_name', default='waffle')
    # Spawn inside the 1.58 m gap between Wall_7 (ends x=-4.646) and Wall_17
    # (starts x=-3.067) at y≈-7.4.  The corridor between Wall_8 (x=-4.671)
    # and Wall_18 (x=-2.992) is 1.68 m wide; x=-3.83 centres the robot.
    x_pose = LaunchConfiguration('x_pose', default='-3.83')
    y_pose = LaunchConfiguration('y_pose', default='-8.5')

    # --- Gazebo -----------------------------------------------------------

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        # FIX: pass the world file so Gazebo loads the full maze environment,
        # matching the coordinate frame that the map was built in.
        launch_arguments={'world': world_file}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # --- TurtleBot3 -------------------------------------------------------

    # Run robot_state_publisher explicitly so frame IDs stay canonical
    # (base_link/base_scan without a prefix mismatch in RViz/TF).
    urdf_file = os.path.join(turtlebot3_gazebo_share, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as urdf_in:
        robot_description = urdf_in.read()

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'frame_prefix': '',
        }],
    )

    # spawn_entity waits 10 s (wall clock) for Gazebo to be ready, then uses
    # -timeout 60 so the script keeps retrying the /spawn_entity service for
    # up to 60 s rather than failing immediately if Gazebo is still loading.
    spawn_turtlebot_cmd = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                output='screen',
                arguments=[
                    '-entity', entity_name,
                    '-file', os.path.join(
                        turtlebot3_gazebo_share,
                        'models',
                        'turtlebot3_waffle',
                        'model.sdf',
                    ),
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', '0.01',
                    '-timeout', '60',
                ],
            )
        ],
    )

    # --- Nav2 + SLAM ------------------------------------------------------
    # Starts at t=18s: robot spawns at t=10s + up to a few seconds for the
    # diff_drive plugin to start publishing odom→base_footprint.  Giving SLAM
    # a clean TF tree from the start prevents the "No transform" warnings.
    navigation = TimerAction(
        period=18.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'slam':            'True',
                    'map':              maze_map_config_file,
                    'use_sim_time':     use_sim_time,
                    'params_file':      params_config_file,
                    'slam_params_file': slam_params_file,
                    'autostart':        'True',
                    'use_composition':  'False',
                }.items(),
            ),
        ],
    )

    # Frontier explorer starts at t=35s so Nav2 lifecycle nodes have fully
    # activated and the first SLAM map tiles are available before any goals
    # are sent.
    frontier_explorer = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='autonomous_tb3',
                executable='frontier_explorer.py',
                name='frontier_explorer',
                output='screen',
                parameters=[{
                    'use_sim_time':    True,
                    'initial_pose_x':  0.0,
                    'initial_pose_y':  0.0,
                }],
            ),
        ],
    )

    # RViz starts at t=20s — after the robot and SLAM are both up so users
    # never see the transient "No transform" warnings during startup.
    rviz_launching = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_node',
                arguments=['-d', rviz_config_file],
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    # --- Launch description -----------------------------------------------
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('entity_name', default_value='waffle'))
    ld.add_action(SetEnvironmentVariable(name='TURTLEBOT3_MODEL',   value='waffle'))
    # FIX: expose the worlds/ directory to Gazebo so model://tb3_maze_world resolves
    ld.add_action(SetEnvironmentVariable(name='GAZEBO_MODEL_PATH',  value=gazebo_model_path))

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(navigation)
    ld.add_action(frontier_explorer)
    ld.add_action(rviz_launching)

    return ld
