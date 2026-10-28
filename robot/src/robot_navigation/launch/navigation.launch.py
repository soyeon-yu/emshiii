from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav2_params = "/home/soyeon/M_J_T/robot/ros2_ws/src/robot_navigation/config/nav2_params.yaml"

    # -------------------------------
    # map 인자 선언 (기본값 지정 가능)
    # -------------------------------
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/soyeon/M_J_T/robot/maps/test2.yaml',
        description='Full path to map yaml file to load'
    )
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        map_arg,

        # -------------------------------
        # Map Server
        # -------------------------------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': False
            }]
        ),
        # -------------------------------
        # AMCL
        # -------------------------------
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params]
        ),
        # -------------------------------
        # Planner
        # -------------------------------
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),
        # -------------------------------
        # Controller
        # -------------------------------
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),
        # -------------------------------
        # Recovery Behaviors
        # -------------------------------
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params]
        ),
        # -------------------------------
        # Behavior Tree Navigator
        # -------------------------------
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),
        # -------------------------------
        # Lifecycle Manager
        # -------------------------------
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),
    ])
