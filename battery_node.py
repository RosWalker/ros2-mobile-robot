#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        # ── Publishers ──────────────────────────────────────────
        self.battery_status_pub = self.create_publisher(
            String, '/battery_status', 10)

        # ── Subscribers ─────────────────────────────────────────
        # This is the topic the assignment says will publish true/false
        self.battery_low_sub = self.create_subscription(
            Bool, '/battery_level_low', self.battery_callback, 10)

        # ── State ────────────────────────────────────────────────
        self.battery_is_low = False
        self.is_charging = False

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('charge_wait_time', 5.0)

        # ── Timer to publish battery status ──────────────────────
        self.timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info('Battery node started!')

    def battery_callback(self, msg):
        """Called when battery_level_low topic publishes"""
        if msg.data and not self.battery_is_low:
            self.get_logger().warn('Battery low! Must return to dock!')
            self.battery_is_low = True
            self.is_charging = False

        elif not msg.data and self.battery_is_low:
            self.get_logger().info('Battery charged! Resuming mission.')
            self.battery_is_low = False
            self.is_charging = False

    def publish_status(self):
        """Regularly publish battery status for the behaviour tree"""
        status = String()
        if self.battery_is_low and not self.is_charging:
            status.data = 'LOW'
        elif self.battery_is_low and self.is_charging:
            status.data = 'CHARGING'
        else:
            status.data = 'OK'

        self.battery_status_pub.publish(status)

    def set_charging(self, charging: bool):
        self.is_charging = charging

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()