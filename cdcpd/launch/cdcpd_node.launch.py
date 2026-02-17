#!/usr/bin/env python3
"""
Launch file for CDCPD node with OSQP optimization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for CDCPD node"""
    
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
    
    # CDCPD node
    cdcpd_node = Node(
        package='cdcpd',
        executable='cdcpd_node',
        name='cdcpd_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Add topic remappings here if needed
            # ('input_topic', 'remapped_topic'),
        ],
        # IMPORTANT: Enable proper shutdown handling
        sigterm_timeout='5',  # Wait 5 seconds for SIGTERM before SIGKILL
        sigkill_timeout='5',  # Wait 5 seconds for SIGKILL  
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        cdcpd_node,
    ])
