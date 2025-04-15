from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_patrol'),
        'rviz',
        'config.rviz'
    )

    return LaunchDescription([
        # Server
        Node(
            package='robot_patrol',
            executable='go_to_pose_node',
            output='screen'),

        # RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])