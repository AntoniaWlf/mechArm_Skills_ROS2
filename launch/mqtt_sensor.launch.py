from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecharm_skills_ROS2',
            executable='mqtt_sensor.py',
            name='mqtt_sensor_node',
            output='screen',
            parameters=[
                {"mqtt_broker": "141.19.44.65"},
                {"mqtt_port": 18443},
                {"mqtt_topic_position": "sensor/position"},
                {"mqtt_topic_move": "robot/move_to_pose"},
                {"mqtt_user": "suedzucker"},
                {"mqtt_password": "isomalt"},
                {"mqtt_client_id": "pi_robot_arm"}
            ]
        )
    ])