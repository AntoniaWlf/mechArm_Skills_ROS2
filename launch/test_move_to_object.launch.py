from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='mecharm_skills',      # muss exakt so in package.xml stehen
            executable='main_node',        # wie in setup.py registriert
            name='test_move_to_object',
            output='screen',
            parameters=[{
                # Nur diesen einen Skill laden
                'skills': ['MoveToObject'],
                # optional: Geschwindigkeit anpassen
                'move_speed': 60,
            }],
        ),
    ])