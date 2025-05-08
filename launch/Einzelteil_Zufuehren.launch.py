from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecharm_skills',
            executable='main_node.py',
            name='Einzelteil_Zufuehren',
            parameters=[
                {"skills": ["MoveToObject", "LocateObject", "PickUpObject", "MoveToRessource", "PlaceObject"]},
            ]
        )
    ])