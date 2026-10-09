#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='false',
        description='Enable OpenCV HUD preview window (Set true only with active VNC/X11)'
    )
    
    altitude_arg = DeclareLaunchArgument(
        'search_altitude',
        default_value='3.0',
        description='Cruise and search altitude in meters'
    )

    competition_mode_arg = DeclareLaunchArgument(
        'competition_mode',
        default_value='false',
        description='Enable Competition Mode (Reduces ROS 2 log spam, only prints critical phases)'
    )

    qr_vision_node = Node(
        package='addc_autonomy',
        executable='qr_ros',
        name='vision_control_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': False,
            'enable_debug_window': LaunchConfiguration('enable_gui'),
        }]
    )

    search_planner_node = Node(
        package='addc_autonomy',
        executable='search_node',
        name='search_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': False,
            'search_altitude': LaunchConfiguration('search_altitude'),
            'lane_spacing': 2.0
        }]
    )

    mission_control_node = Node(
        package='addc_autonomy',
        executable='mission_control',
        name='mission_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'target_altitude': LaunchConfiguration('search_altitude')
        }]
    )

    precision_landing_node = Node(
        package='addc_autonomy',
        executable='precision_landing',
        name='precision_landing_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    hmi_bridge_node = Node(
        package='addc_autonomy',
        executable='hmi_bridge',
        name='hmi_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'http_port': 5000
        }]
    )

    return LaunchDescription([
        enable_gui_arg,
        altitude_arg,
        competition_mode_arg,
        qr_vision_node,
        search_planner_node,
        mission_control_node,
        precision_landing_node,
        hmi_bridge_node,
    ])
