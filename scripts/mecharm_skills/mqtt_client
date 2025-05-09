#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import paho.mqtt.client as mqtt
import json
import time
from pymycobot.mycobot import MyCobot
import os

class MqttCapabilityClient(Node):
    def __init__(self):
        super().__init__('mqtt_capability_client')
        self.publisher = self.create_publisher(Point, '/object_position', 10)
        self.get_logger().info("MQTT-Capability-Client gestartet.")

        # MQTT-Parameter
        self.declare_parameter("mqtt_broker", "141.19.44.65")
        self.declare_parameter("mqtt_port", 18443)
        self.declare_parameter("mqtt_topic_position", "sensor/position")
        self.declare_parameter("mqtt_topic_move", "robot/move_to_pose")
        self.declare_parameter("mqtt_topic_capability", "capabilities/execute")  # Neues Topic für Capabilities
        self.declare_parameter("mqtt_user", "suedzucker")
        self.declare_parameter("mqtt_password", "isomalt")
        self.declare_parameter("mqtt_client_id", "pi_robot_arm")

        self.mqtt_broker = self.get_parameter("mqtt_broker").get_parameter_value().string_value
        self.mqtt_port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        self.mqtt_topic_position = self.get_parameter("mqtt_topic_position").get_parameter_value().string_value
        self.mqtt_topic_move = self.get_parameter("mqtt_topic_move").get_parameter_value().string_value
        self.mqtt_topic_capability = self.get_parameter("mqtt_topic_capability").get_parameter_value().string_value
        self.mqtt_user = self.get_parameter("mqtt_user").get_parameter_value().string_value
        self.mqtt_password = self.get_parameter("mqtt_password").get_parameter_value().string_value
        self.mqtt_client_id = self.get_parameter("mqtt_client_id").get_parameter_value().string_value

        # MQTT-Client einrichten
        self.client = mqtt.Client(client_id=self.mqtt_client_id)
        self.client.username_pw_set(self.mqtt_user, self.mqtt_password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # Verbindung zum MQTT-Broker herstellen
        self.client.connect(self.mqtt_broker, self.mqtt_port, 60)
        self.get_logger().info(f"Verbunden mit MQTT-Broker: {self.mqtt_broker}:{self.mqtt_port}")

        # Roboter initialisieren
        self.robot = MyCobot('/dev/ttyAMA0', 1000000)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Erfolgreich mit MQTT-Broker verbunden.")
            client.subscribe(self.mqtt_topic_position)
            client.subscribe(self.mqtt_topic_move)
            client.subscribe(self.mqtt_topic_capability)  # Capability-Topic abonnieren
            self.get_logger().info(f"Abonniere MQTT-Topics: {self.mqtt_topic_position}, {self.mqtt_topic_move}, {self.mqtt_topic_capability}")
        else:
            self.get_logger().error(f"Fehler beim Verbinden mit MQTT-Broker. Code: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            self.get_logger().info(f"MQTT-Nachricht empfangen auf Topic {msg.topic}: {msg.payload.decode('utf-8')}")
            if msg.topic == self.mqtt_topic_position:
                self.process_position_message(msg)
            elif msg.topic == self.mqtt_topic_move:
                self.process_move_message(msg)
            elif msg.topic == self.mqtt_topic_capability:  # Capability-Befehle verarbeiten
                self.process_capability_message(msg)
        except Exception as e:
            self.get_logger().error(f"Fehler beim Verarbeiten der MQTT-Nachricht: {e}")

    def process_position_message(self, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            self.get_logger().info(f"MQTT-Daten empfangen: {data}")

            # ROS-Nachricht erstellen und veröffentlichen
            position = Point()
            position.x = data.get('x', 0.0)
            position.y = data.get('y', 0.0)
            position.z = data.get('z', 0.0)
            self.publisher.publish(position)
            self.get_logger().info(f"ROS-Nachricht veröffentlicht: {position}")
        except Exception as e:
            self.get_logger().error(f"Fehler beim Verarbeiten der Sensordaten: {e}")

    def process_move_message(self, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            if data.get("command") == "move_to_pose":
                pose = [150, 0, 150, 0, 0, 0]
                speed = data.get("speed", 50)
                delay = data.get("delay", 3)

                self.get_logger().info(f"Bewege Roboter zu voreingestellter Pose: {pose} mit Geschwindigkeit: {speed}")
                self.robot.send_coords(pose, speed, 0)
                time.sleep(delay)
                self.get_logger().info("Bewegung abgeschlossen.")
            else:
                self.get_logger().warn("Unbekannter Befehl empfangen.")
        except Exception as e:
            self.get_logger().error(f"Fehler beim Verarbeiten des Bewegungsbefehls: {e}")

    def process_capability_message(self, msg):
        """
        Verarbeitet Capability-Befehle und startet die entsprechende Launch-Datei.
        """
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            capability_name = data.get("capability", None)
            if capability_name:
                self.get_logger().info(f"Capability-Befehl empfangen: {capability_name}")
                # Starten Sie die Launch-Datei für die Capability
                os.system(f"ros2 launch mecharm_skills {capability_name}.launch.py")
            else:
                self.get_logger().warn("Ungültiger Capability-Befehl empfangen.")
        except Exception as e:
            self.get_logger().error(f"Fehler beim Verarbeiten des Capability-Befehls: {e}")

    def start(self):
        self.client.loop_start()
        rclpy.spin(self)
        self.client.loop_stop()

def main(args=None):
    rclpy.init(args=args)
    node = MqttCapabilityClient()
    try:
        node.start()
    except KeyboardInterrupt:
        node.get_logger().info("MQTT-Capability-Client beendet.")
    finally:
        node.destroy_node()
        rclpy.shutdown()