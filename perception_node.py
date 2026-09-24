#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # ── CV Bridge ────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Publishers ──────────────────────────────────────────
        self.detected_colour_pub = self.create_publisher(String, '/detected_colour', 10)
        self.colour_angle_pub = self.create_publisher(Float32, '/colour_angle', 10)
        self.colour_distance_pub = self.create_publisher(Float32, '/colour_distance', 10)

        # ── Subscribers ─────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # ── Colour ranges in HSV ─────────────────────────────────
        self.colour_ranges = {
            'green':  ([35, 50, 50],  [85, 255, 255]),   # survivor
            'blue':   ([100, 50, 50], [130, 255, 255]),   # dam
            'red':    ([0, 100, 100], [10, 255, 255]),    # fire
            'yellow': ([20, 100, 100],[35, 255, 255]),    # medical kit
            'purple': ([130, 50, 50], [160, 255, 255]),   # exit
            'black':  ([0, 0, 0],     [180, 255, 50]),    # docking station
        }

        # ── State ────────────────────────────────────────────────
        self.target_colour = 'green'
        self.image_width = 640

        # ── Target colour subscriber ─────────────────────────────
        self.target_sub = self.create_subscription(
            String, '/target_colour', self.target_callback, 10)

        self.get_logger().info('Perception node started!')

    def target_callback(self, msg):
        self.target_colour = msg.data
        self.get_logger().info(f'New target colour: {self.target_colour}')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_width = cv_image.shape[1]
            image_height = cv_image.shape[0]

            # Convert to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Get colour range for target
            if self.target_colour not in self.colour_ranges:
                return

            lower, upper = self.colour_ranges[self.target_colour]
            lower = np.array(lower)
            upper = np.array(upper)

            # Create mask
            mask = cv2.inRange(hsv, lower, upper)

            # For red - also check upper range
            if self.target_colour == 'red':
                lower2 = np.array([170, 100, 100])
                upper2 = np.array([180, 255, 255])
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask, mask2)

            # Find contours
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Get largest contour
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)

                if area > 100:
                    # Get centre of contour
                    M = cv2.moments(largest)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])

                        # Calculate angle offset from centre
                        # Positive = target is to the right
                        # Negative = target is to the left
                        offset = (cx - self.image_width / 2) / (self.image_width / 2)

                        # Estimate distance from object size
                        # Larger area = closer
                        distance = 100000.0 / area

                        # Publish results
                        colour_msg = String()
                        colour_msg.data = self.target_colour
                        self.detected_colour_pub.publish(colour_msg)

                        angle_msg = Float32()
                        angle_msg.data = float(offset)
                        self.colour_angle_pub.publish(angle_msg)

                        dist_msg = Float32()
                        dist_msg.data = float(distance)
                        self.colour_distance_pub.publish(dist_msg)

                        self.get_logger().info(
                            f'Detected {self.target_colour}: '
                            f'offset={offset:.2f} dist={distance:.1f}'
                        )
                else:
                    # Nothing detected - publish empty
                    msg = String()
                    msg.data = 'none'
                    self.detected_colour_pub.publish(msg)
            else:
                msg = String()
                msg.data = 'none'
                self.detected_colour_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()