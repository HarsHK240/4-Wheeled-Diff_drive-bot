import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_bot_description = get_package_share_directory('my_bot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_robot_localization = get_package_share_directory('robot_localization')

    # Path to the Nav2 parameters file for mapless navigation
    nav2_params_file = os.path.join(pkg_my_bot_description, 'config', 'nav2_param.yaml')
    
    # Path to the EKF config file for sensor fusion
    ekf_config_file = os.path.join(pkg_my_bot_description, 'config', 'ekf.yaml')

    # --- Actions ---

    # 1. Include your robot's main launch file (starts Gazebo, ros2_control, etc.)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_bot_description, 'launch', 'display.launch.py')
        )
    )

    # 2. Start the robot_localization EKF node for sensor fusion
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}]
    )

    # 3. Start the navsat_transform_node (converts GPS lat/lon to the map frame)
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}],
        remappings=[('/imu', '/imu'),
                    ('/gps/fix', '/gps'),
                    ('/odometry/filtered', '/odometry/global'),
                    ('/odometry/gps', '/odometry/gps')]
    )

    # 4. Include the Nav2 bringup launch file in mapless mode
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': '', # Pass an empty string for mapless navigation
            'params_file': nav2_params_file,
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([
        robot_launch,
        start_robot_localization_cmd,
        start_navsat_transform_cmd,
        nav2_launch,
    ])
