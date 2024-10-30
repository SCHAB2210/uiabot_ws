from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set up paths to URDF
    pkg_share = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    # Nodes to include
    nodes = [
        # Load and publish URDF with robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),
        
        # Use the standard joint_state_publisher node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Lidar Node for sensor data
        Node(
            package='lidar_package',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        )
    ]

    return LaunchDescription(nodes)

