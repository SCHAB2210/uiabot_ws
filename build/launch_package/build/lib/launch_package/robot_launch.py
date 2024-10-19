import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Node to publish cmd_vel commands (from my_publisher_package)
    cmd_vel_publisher = Node(
        package='my_publisher_package',
        executable='cmd_vel_publisher.py',
        name='cmd_vel_publisher',
        output='screen'
    )

    # Node for LIDAR data (from lidar_package)
    lidar_node = Node(
        package='lidar_package',
        executable='lidar_node.py',
        name='lidar_node',
        output='screen'
    )

    # Return the LaunchDescription containing all nodes
    return LaunchDescription([
        cmd_vel_publisher,
        lidar_node,
    ])
