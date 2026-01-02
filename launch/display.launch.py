# display.launch.py
import os
import yaml

from launch_ros.actions import SetRemap
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Use FindPackageShare to correctly locate the package
    pkg_share = FindPackageShare('my_bot_description')
    
    # Path to the new top-level XACRO file
    urdf_xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'my_bot_description.urdf.xacro'])
    
    # Path to the controller config file
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'my_bot_controllers.yaml'])

    # --- DEFINE PATH TO THE WORLD FILE ---
    # This points to config/gps_world.world
    world_file_path = PathJoinSubstitution([pkg_share, 'config', 'gps_world.world'])

    # -------------- build robot_description (xacro) at runtime --------------
    # This Command runs "xacro <urdf_xacro_file>" and returns the expanded URDF string
    robot_description = {
            'robot_description': Command([
                'xacro ', ' ', 
                 urdf_xacro_file,
                ' use_sim_time:=true' 
            ])
        }

    # -------------- Gazebo Classic (server + client) --------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'world': world_file_path   # <--- PASS THE WORLD FILE HERE
        }.items()
    )
    
    # -------------- Spawn robot into Gazebo --------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # -------------- robot_state_publisher --------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # -------------- Spawners for controllers --------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        # remappings=[
        #     ('/diff_drive_controller/odom', '/odom_garbage'), # You already added this
        #     # ADD THIS LINE: Connect standard cmd_vel to the controller
        #     ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel') 
        # ]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        SetRemap(src='/diff_drive_controller/cmd_vel_unstamped', dst='/cmd_vel'),
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
    ])
