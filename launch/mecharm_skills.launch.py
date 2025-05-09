from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # MQTT-Client bleibt unverändert
        Node(
            package='mecharm_skills',
            executable='mqtt_client',
            name='mqtt_client',
            output='screen',
        ),

        # Skill-Orchestrator statt skill_server
        Node(
            package='mecharm_skills',
            executable='main_node',       # so wie in setup.py registriert
            name='skill_orchestrator',
            output='screen',
            parameters=[{
                # Hier übergibst du alle Skills in der Reihenfolge:
                'skills': ['LocateObject', 'MoveToObject']
            }],
        ),
    ])
