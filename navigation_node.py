#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Bool
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # ── Publishers ──────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_status_pub = self.create_publisher(String, '/nav_status', 10)

        # ── Subscribers ─────────────────────────────────────────
        self.detected_colour_sub = self.create_subscription(
            String, '/detected_colour', self.colour_callback, 10)
        self.colour_angle_sub = self.create_subscription(
            Float32, '/colour_angle', self.angle_callback, 10)
        self.colour_distance_sub = self.create_subscription(
            Float32, '/colour_distance', self.distance_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/nav_command', self.command_callback, 10)

        # ── State ────────────────────────────────────────────────
        self.detected_colour = 'none'
        self.colour_offset = 0.0
        self.colour_distance = 999.0
        self.current_command = 'IDLE'
        self.target_colour = 'green'
        self.stop_distance = 1.0
        self.spin_speed = 1.2
        self.last_detection_time = 0.0

        # ── Control loop ─────────────────────────────────────────
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Navigation node started!')

    def colour_callback(self, msg):
        self.detected_colour = msg.data

    def angle_callback(self, msg):
        self.colour_offset = msg.data

    def distance_callback(self, msg):
        self.colour_distance = msg.data

    def command_callback(self, msg):
        parts = msg.data.split(':')
        command = parts[0]

        if command == 'STOP':
            self.current_command = 'IDLE'
            self.stop_robot()
            return

        if command == 'SEARCH':
            new_colour = parts[1] if len(parts) > 1 else 'green'
            new_distance = float(parts[2]) if len(parts) > 2 else 1.0

            # Only reset if this is a NEW colour target
            if new_colour != self.target_colour:
                self.target_colour = new_colour
                self.stop_distance = new_distance
                self.current_command = 'SEARCH'
                self.get_logger().info(
                    f'New target: {self.target_colour}, '
                    f'stop at {self.stop_distance}'
                )
            elif self.current_command == 'IDLE':
                # Same colour but we were idle - start searching
                self.target_colour = new_colour
                self.stop_distance = new_distance
                self.current_command = 'SEARCH'
                self.get_logger().info(
                    f'Resuming search: {self.target_colour}'
                )
            # If same colour and already searching - ignore, don't reset
    def control_loop(self):
        cmd = Twist()

        if self.current_command == 'SEARCH':
            if self.detected_colour == self.target_colour:
                # Colour detected!
                if abs(self.colour_offset) > 0.1:
                    # Not centred - rotate to centre it
                    cmd.linear.x = 0.0
                    cmd.angular.z = -self.colour_offset * 0.8
                    self.get_logger().info(
                        f'Centering on {self.target_colour}: '
                        f'offset={self.colour_offset:.2f}'
                    )
                elif self.colour_distance > self.stop_distance:
                    # Centred but not close enough - drive forward
                    cmd.linear.x = 0.6
                    cmd.angular.z = -self.colour_offset * 0.5
                    self.get_logger().info(
                        f'Driving to {self.target_colour}: '
                        f'dist={self.colour_distance:.1f}'
                    )
                else:
                    # Reached target!
                    self.stop_robot()
                    self.current_command = 'IDLE'
                    status = String()
                    status.data = 'REACHED'
                    self.nav_status_pub.publish(status)
                    self.get_logger().info(
                        f'Reached {self.target_colour}!'
                    )
                    return
            else:
                # Colour not detected - spin to search
                cmd.linear.x = 0.0
                cmd.angular.z = self.spin_speed
                self.get_logger().info(
                    f'Spinning to find {self.target_colour}...'
                )

        elif self.current_command == 'IDLE':
            self.stop_robot()
            return

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()