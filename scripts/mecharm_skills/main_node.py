#!/usr/bin/env python3

import importlib
import rclpy
from rclpy.node import Node

class SkillOrchestrator(Node):
    """
    Liest beim Start einen Parameter 'skills' (Liste von Klassennamen),
    lädt diese Klassen aus dem Paket 'mecharm_skills_ROS2.skills', instanziiert
    sie und führt sie nacheinander aus.
    """

    def __init__(self):
        super().__init__('skill_orchestrator')

        # 1) Parameter deklarieren und einlesen
        self.declare_parameter('skills', [])
        skill_names = self.get_parameter('skills').get_parameter_value().string_array_value

        # 2) Skills dynamisch laden und instanziieren
        self.skills = []
        for name in skill_names:
            try:
                module = importlib.import_module(f'mecharm_skills_ROS2.skills.{name.lower()}')
                cls    = getattr(module, name)
                self.add_skill(cls())
                self.get_logger().info(f"Skill geladen: {name}")
            except Exception as e:
                self.get_logger().error(f"Fehler beim Laden von Skill '{name}': {e}")

        self.current_skill_index = 0

        # 3) Starte die Skill-Ausführung nach einer kleinen Verzögerung, damit
        #    alle ROS-Infrastruktur (Publisher/Subscribers etc.) angelegt ist.
        #    Hier verwenden wir einen Timer mit null Delay.
        self.create_timer(0.01, self.execute_next_skill, once=True)

    def add_skill(self, skill):
        """Fügt einen Skill zur internen Liste hinzu."""
        self.skills.append(skill)

    def execute_next_skill(self):
        """Führt den nächsten Skill aus oder beendet, wenn alle durch sind."""
        if self.current_skill_index < len(self.skills):
            skill = self.skills[self.current_skill_index]
            self.get_logger().info(f"Führe Skill aus: {skill.__class__.__name__}")
            try:
                skill.execute(self)
            except Exception as e:
                self.get_logger().error(f"Ausnahme in Skill {skill.__class__.__name__}: {e}")
            self.current_skill_index += 1

            # Wenn dieser Skill asynchron arbeitet und sich selbst weiterschaltet,
            # dann passiert der nächste Start dort. Sonst hier:
            if not hasattr(skill, 'subscription'):
                # sofort weitermachen, falls der Skill synchron ist
                self.execute_next_skill()

        else:
            self.get_logger().info("🎉 Alle Skills abgeschlossen. Shutting down...")
            rclpy.shutdown()  # sauber beenden

def main(args=None):
    rclpy.init(args=args)
    node = SkillOrchestrator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
