#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SwarmController(Node):

    def __init__(self):

        super().__init__('swarm_controller')

        self.robots = ['TB3_1', 'TB3_2', 'TB3_3', 'TB3_4']

        self.pose = {}

        self.cmd_publishers = {}

        for robot in self.robots:

            self.pose[robot] = None

            self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, r=robot: self.odom_callback(msg, r),
                10)

            self.cmd_publishers[robot] = self.create_publisher(
                Twist,
                f'/{robot}/cmd_vel',
                10)

        self.timer = self.create_timer(
            0.1,
            self.control_loop)

        self.get_logger().info("Swarm controller started")

    def odom_callback(self, msg, robot):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.pose[robot] = (x, y, yaw)

    def control_loop(self):

        for robot in self.robots:

            if self.pose[robot] is None:
                continue

            x, y, yaw = self.pose[robot]

            sep_x = 0.0
            sep_y = 0.0

            coh_x = 0.0
            coh_y = 0.0

            align_x = 0.0
            align_y = 0.0

            neighbours = 0

            for other in self.robots:

                if other == robot:
                    continue

                if self.pose[other] is None:
                    continue

                ox, oy, oyaw = self.pose[other]

                dx = ox - x
                dy = oy - y

                d = math.sqrt(dx * dx + dy * dy)

                if d > 2.5:
                    continue

                neighbours += 1

                # Separation
                if d < 0.5:
                    sep_x -= dx / (d + 0.01)
                    sep_y -= dy / (d + 0.01)

                # Cohesion
                coh_x += ox
                coh_y += oy

                # Alignment
                align_x += math.cos(oyaw)
                align_y += math.sin(oyaw)

            cmd = Twist()

            if neighbours == 0:

                #
                # Explore until another robot is found
                #
                cmd.linear.x = 0.10
                cmd.angular.z = 0.25

            else:

                coh_x = coh_x / neighbours - x
                coh_y = coh_y / neighbours - y

                align_heading = math.atan2(
                    align_y,
                    align_x)

                vx = (
                    2.0 * sep_x +
                    0.4 * coh_x +
                    0.3 * math.cos(align_heading)
                )

                vy = (
                    2.0 * sep_y +
                    0.4 * coh_y +
                    0.3 * math.sin(align_heading)
                )

                desired_heading = math.atan2(vy, vx)

                heading_error = desired_heading - yaw

                while heading_error > math.pi:
                    heading_error -= 2 * math.pi

                while heading_error < -math.pi:
                    heading_error += 2 * math.pi

                speed = math.sqrt(vx * vx + vy * vy)

                cmd.linear.x = min(speed, 0.18)
                cmd.angular.z = 1.5 * heading_error

            self.cmd_publishers[robot].publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = SwarmController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

