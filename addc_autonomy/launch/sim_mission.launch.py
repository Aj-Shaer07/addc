#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='Enable OpenCV HUD preview window'
    )
    
    arena_config_arg = DeclareLaunchArgument(
        'search_altitude',
        default_value='3.0',
        description='Search altitude in meters'
    )

    qr_vision_node = Node(
        package='addc_autonomy',
        executable='qr_ros',
        name='vision_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'enable_debug_window': LaunchConfiguration('enable_gui'),
            'camera_topic': '/camera/image_raw'
        }]
    )

    search_planner_node = Node(
        package='addc_autonomy',
        executable='search_node',
        name='search_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'recon_x_min': 8.0,
            'recon_x_max': 20.0,
            'recon_y_min': -6.0,
            'recon_y_max': 6.0,
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
            'use_sim_time': True,
            'target_altitude': LaunchConfiguration('search_altitude')
        }]
    )

    precision_landing_node = Node(
        package='addc_autonomy',
        executable='precision_landing',
        name='precision_landing_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'camera_topic': '/camera/image_raw'
        }]
    )

    hmi_bridge_node = Node(
        package='addc_autonomy',
        executable='hmi_bridge',
        name='hmi_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'http_port': 5000
        }]
    )

    return LaunchDescription([
        enable_gui_arg,
        arena_config_arg,
        qr_vision_node,
        search_planner_node,
        mission_control_node,
        precision_landing_node,
        hmi_bridge_node,
    ])
