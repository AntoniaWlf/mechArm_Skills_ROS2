#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class SkillServerNode(Node):
    def __init__(self):
        super().__init__('skill_server')
        self.create_service(Trigger, 'execute_skill', self.execute_skill_callback)

    def execute_skill_callback(self, request, response):
        self.get_logger().info('Executing skill...')
        # Implement skill logic here
        response.success = True
        response.message = 'Skill executed successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SkillServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()