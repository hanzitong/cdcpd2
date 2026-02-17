#!/usr/bin/env python3
"""
Launch file for CDCPD offline processing node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for CDCPD offline node"""
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    # CDCPD offline node
    cdcpd_offline_node = Node(
        package='cdcpd',
        executable='cdcpd_offline',
        name='cdcpd_offline',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        cdcpd_offline_node,
    ])
