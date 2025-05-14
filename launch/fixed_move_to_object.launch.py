from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecharm_skills_ROS2',
            executable='fixed_move_to_object',
            name='fixed_move_to_object',
            output='screen',
        ),
    ])