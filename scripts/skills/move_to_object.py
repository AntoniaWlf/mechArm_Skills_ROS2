#!/usr/bin/env python3

from geometry_msgs.msg import Point

class MoveToObject:
    def execute(self, node):
        node.get_logger().info("Warte auf Sensordaten, um Roboter zu Zielposition zu bewegen...")

        # Callback-Funktion für Sensordaten
        def sensor_data_callback(msg):
            try:
                # Sensordaten aus ROS2-Nachricht extrahieren
                target_position = Point()
                target_position.x = msg.x
                target_position.y = msg.y
                target_position.z = msg.z

                node.get_logger().info(f"Empfangene Zielposition: x={target_position.x}, y={target_position.y}, z={target_position.z}")

                # Bewegung ausführen
                pose = [target_position.x, target_position.y, target_position.z, 0, 0, 0]
                speed = 50
                node.robot.send_coords(pose, speed, 0)
                node.get_logger().info("Bewegung abgeschlossen.")

                # Nächsten Skill ausführen
                node.execute_next_skill()

            except Exception as e:
                node.get_logger().error(f"Fehler beim Verarbeiten der Sensordaten: {e}")

        # Abonnieren des ROS2-Topics für Sensordaten
        subscription = node.create_subscription(Point, '/object_position', sensor_data_callback, 10)
        node.get_logger().info("Abonniere Sensordaten-Topic: /object_position")




