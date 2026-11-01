#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select

class SwarmTeleop(Node):
    def __init__(self):
        super().__init__('swarm_teleop_node')
        
        # Define your active swarm namespaces based on your topic list
        self.robots = ['TB3_1', 'TB3_2', 'TB3_3', 'TB3_4']
        self.robot_pubs = {}
        
        # Create a cmd_vel publisher for each namespaced robot
        for robot in self.robots:
            topic_name = f'/{robot}/cmd_vel'
            self.robot_pubs[robot] = self.create_publisher(Twist, topic_name, 10)
        
        self.current_target = 'TB3_1' # Default target
        self.get_logger().info("=== Swarm Controller Initialized ===")
        self.print_instructions()
        
        self.timer = self.create_timer(0.1, self.check_input)

    def print_instructions(self):
        print("\n========================================")
        print(f"Active Target: [ {self.current_target} ]")
        print("----------------------------------------")
        print("Switch Target: Type '1', '2', '3', '4', or 'all'")
        print("Movement:      'forward', 'backward', 'left', 'right', 'stop'")
        print("Quit:          'exit'")
        print("========================================\n")

    def check_input(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            user_input = sys.stdin.readline().strip().lower()
            self.process_command(user_input)

    def process_command(self, cmd):
        # Handle switching targets
        if cmd in ['1', '2', '3', '4']:
            idx = int(cmd) - 1
            self.current_target = self.robots[idx]
            print(f"--> Switched target to: {self.current_target}")
            return
        elif cmd == 'all':
            self.current_target = 'ALL'
            print("--> Switched target to: ALL ROBOTS (Swarm mode)")
            return

        twist = Twist()
        matched = True

        if 'forward' in cmd or 'straight' in cmd:
            twist.linear.x = 0.15
            twist.angular.z = 0.0
        elif 'backward' in cmd or 'reverse' in cmd:
            twist.linear.x = -0.15
            twist.angular.z = 0.0
        elif 'left' in cmd:
            twist.linear.x = 0.0
            twist.angular.z = 0.8
        elif 'right' in cmd:
            twist.linear.x = 0.0
            twist.angular.z = -0.8
        elif 'stop' in cmd or 'halt' in cmd:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif 'exit' in cmd:
            print("Exiting swarm teleop...")
            rclpy.shutdown()
            sys.exit(0)
        else:
            print(f"Unknown command: '{cmd}'")
            matched = False

        if matched:
            if self.current_target == 'ALL':
                for robot, pub in self.robot_pubs.items():
                    pub.publish(twist)
                print(f"-> Sent command to ALL robots")
            else:
                self.robot_pubs[self.current_target].publish(twist)
                print(f"-> Sent command to {self.current_target}")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
