#!/usr/bin/env python3

from rclpy.node import Node
from skills.move_to_object import MoveToObject
from skills.locate_object import LocateObject

class SkillOrchestrator(Node):
    def __init__(self):
        super().__init__('skill_orchestrator')
        self.skills = []  # Liste der Skills
        self.current_skill_index = 0

    def add_skill(self, skill):
        self.skills.append(skill)

    def execute_next_skill(self):
        if self.current_skill_index < len(self.skills):
            skill = self.skills[self.current_skill_index]
            self.get_logger().info(f"Führe Skill aus: {skill.__class__.__name__}")
            skill.execute(self)
            self.current_skill_index += 1
        else:
            self.get_logger().info("Alle Skills abgeschlossen.")

    def start(self):
        self.execute_next_skill()