#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymycobot.mycobot import MyCobot
import time

class FixedMoveToObject(Node):
    def __init__(self):
        super().__init__('fixed_move_to_object')
        self.robot = MyCobot('/dev/ttyAMA0', 1000000)
        self.get_logger().info("Roboter verbunden.")

        # Vorprogrammierte Positionen
        self.positions = [
            [150, 0, 150, 0, 0, 0],
            [200, 50, 150, 0, 0, 0],
            [100, -50, 150, 0, 0, 0]
        ]

        # Bewegungen ausführen
        for pos in self.positions:
            self.move_to(pos)

        self.get_logger().info("Alle Bewegungen abgeschlossen.")

    def move_to(self, pose):
        speed = 50
        self.robot.send_coords(pose, speed, 0)
        self.get_logger().info(f"Roboter bewegt zu Position: {pose}")
        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = FixedMoveToObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()