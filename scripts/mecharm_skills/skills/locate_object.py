#!/usr/bin/env python3

class LocateObject:
    def execute(self, node):
        node.get_logger().info("Lokalisierung des Objekts...")
        # Beispiel: Sensordaten verarbeiten
        position = node.get_sensor_data()
        node.get_logger().info(f"Objektposition: {position}")
        node.execute_next_skill()  # Nächsten Skill ausführen
