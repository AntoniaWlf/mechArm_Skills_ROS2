from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecharm_skills',
            executable='mqtt_client',
            name='mqtt_client'
        ),
        Node(
            package='mecharm_skills',
            executable='skill_server',
            name='skill_server'
        ),
    ])

    