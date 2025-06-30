#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # HSV Range Tool Node Only
    hsv_range_tool_node = Node(
        package='hsv_range_tool',
        executable='hsv_range_tool',
        name='hsv_range_tool',
        output='screen'
    )

    return LaunchDescription([
        hsv_range_tool_node,
    ])