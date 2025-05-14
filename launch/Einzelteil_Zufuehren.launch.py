from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Argument für Skill-Liste (als kommaseparierter String)
    declare_skills_arg = DeclareLaunchArgument(
        'skills',
        default_value='MoveToObject',
        description='Komma-separierte Liste der Skill-Namen, die main_node ausführen soll'
    )

    # 2) Launch-Konfiguration aus dem Argument
    skills_cfg = LaunchConfiguration('skills')

    return LaunchDescription([
        declare_skills_arg,

        # 3) Node-Definition
        Node(
            package='mecharm_skills_ROS2',      
            executable='main_node',        
            name='einzelteil_zufuehren',    
            output='screen',               
            parameters=[{
                'skills': [skills_cfg],
            }],
        ),
    ])
