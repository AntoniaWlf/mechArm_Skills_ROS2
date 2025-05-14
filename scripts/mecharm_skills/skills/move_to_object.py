#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MoveToObject:
    """
    Skill: Bewegt den Roboter basierend auf Sensordaten zur Zielposition.
    Nach Erhalt der ersten Position wird automatisch die Subscription beendet
    und der nächste Skill ausgeführt.
    """

    def __init__(self):
        self.subscription = None

    def execute(self, node):
        """
        Einstiegspunkt für den Skill.
        Wird vom main_node aufgerufen und erhält die Node-Instanz.
        """
        node.get_logger().info("Starte 'MoveToObject' Skill – abonniere '/object_position'.")

        # QoS-Konfiguration: zuverlässig, letzte 10 Nachrichten behalten
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Vorhandene Subscription entfernen (falls execute mehrfach aufgerufen wird)
        if self.subscription:
            node.destroy_subscription(self.subscription)
            self.subscription = None

        # Subscription erstellen und Callback registrieren
        self.subscription = node.create_subscription(
            Point,
            '/object_position',
            lambda msg: self._sensor_data_callback(msg, node),
            qos
        )

    def _sensor_data_callback(self, msg: Point, node):
        """
        Callback für eingehende Sensordaten.
        Führt die Roboterbewegung aus und startet den nächsten Skill.
        """
        try:
            # Loggen der Zielkoordinate
            node.get_logger().info(
                f"Empfangene Zielposition – x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}"
            )

            # Geschwindigkeit aus Parameter (falls gesetzt), sonst Default=50
            speed = 50
            if node.has_parameter('move_speed'):
                speed = node.get_parameter('move_speed').value

            # Pose für den Roboter (x, y, z, Rx, Ry, Rz)
            pose = [msg.x, msg.y, msg.z, 0.0, 0.0, 0.0]
            node.get_logger().info(f"Bewege Roboter zu {pose} mit speed={speed}")

            # Roboter-Befehl ausführen
            node.robot.send_coords(pose, speed, 0)
            node.get_logger().info("Bewegung abgeschlossen.")

        except Exception as e:
            node.get_logger().error(f"Fehler in MoveToObject: {e}")

        finally:
            # Subscription auflösen, da wir nur eine Position benötigen
            if self.subscription:
                node.destroy_subscription(self.subscription)
                self.subscription = None

            # Nächsten Skill starten
            node.execute_next_skill()
            node.get_logger().info("Starte nächsten Skill.")