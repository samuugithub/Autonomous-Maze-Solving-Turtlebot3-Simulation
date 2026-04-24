from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share           = get_package_share_directory('autonomous_tb3')
    gazebo_ros_share        = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_share      = get_package_share_directory('nav2_bringup')

    world_file       = os.path.join(package_share, 'worlds', 'tb3_maze_world.world')
    map_file         = os.path.join(package_share, 'config', 'maze_map.yaml')
    params_file      = os.path.join(package_share, 'config', 'tb3_nav_params.yaml')
    slam_params_file = os.path.join(package_share, 'config', 'slam_params_maze.yaml')
    rviz_file        = os.path.join(package_share, 'config', 'super_nav.rviz')
    model_path       = os.path.join(package_share, 'worlds')

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = (
        f'{model_path}:{existing_model_path}' if existing_model_path else model_path
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    x_pose       = LaunchConfiguration('x_pose', default='-3.83')
    y_pose       = LaunchConfiguration('y_pose', default='-8.5')

    return LaunchDescription([
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL',  value='waffle'),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_model_path),

        # --- Gazebo (starts immediately) ------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
            ),
        ),

        # --- robot_state_publisher (starts immediately; no Gazebo dependency)
        # frame_prefix is intentionally left at the turtlebot3 default ('/')
        # because TF2 strips leading slashes on every lookup, so the published
        # frames /base_link, /base_scan etc. are resolved correctly.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_share, 'launch',
                             'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # --- Spawn robot at t=10s with a 60s service timeout ----------------
        # Gazebo can take 10-20 s to fully start on slower machines.
        # -timeout 60 tells spawn_entity to keep retrying the /spawn_entity
        # service for up to 60 s so a slow Gazebo start never causes a silent
        # spawn failure and missing odom→base_footprint.
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(turtlebot3_gazebo_share, 'launch',
                                     'spawn_turtlebot3.launch.py')
                    ),
                    launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
                ),
            ],
        ),

        # --- Nav2 + SLAM at t=18s ------------------------------------------
        # Robot spawns at t=10s; the extra 8s lets the diff_drive plugin fully
        # initialise and publish a stable stream of odom→base_footprint
        # transforms before SLAM attempts its first TF lookup.
        TimerAction(
            period=18.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={
                        'slam':            'True',
                        'map':              map_file,
                        'use_sim_time':     use_sim_time,
                        'params_file':      params_file,
                        'slam_params_file': slam_params_file,
                        'autostart':        'True',
                        'use_composition':  'False',
                    }.items(),
                ),
            ],
        ),

        # --- Frontier explorer at t=35s ------------------------------------
        # Nav2 lifecycle nodes need ~10-14s to activate after bringup starts.
        TimerAction(
            period=35.0,
            actions=[
                Node(
                    package='autonomous_tb3',
                    executable='frontier_explorer.py',
                    name='frontier_explorer',
                    output='screen',
                    parameters=[{
                        'use_sim_time':   True,
                        'initial_pose_x': 0.0,
                        'initial_pose_y': 0.0,
                    }],
                ),
            ],
        ),

        # --- RViz2 at t=20s ------------------------------------------------
        # Opens after the robot and SLAM are both running so users see a clean
        # TF tree on first launch — no transient "No transform" warnings.
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_file],
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        ),
    ])
