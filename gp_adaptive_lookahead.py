#!/usr/bin/env python3
# gp_adaptive_lookahead.py - Gap follower using adaptive speed from /adaptive_speed_from_angle

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import time
import math

class GPAdaptiveLookahead(Node):
    def __init__(self):
        super().__init__('gp_adaptive_lookahead')

        # Parameters
        self.safety_radius = 2.0
        self.steering_sensitivity = 0.7
        self.max_steering_angle = 0.52
        self.default_speed = 0.5
        self.adaptive_speed = self.default_speed
        self.is_running = True

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Float32, '/reference_speed', self.adaptive_speed_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_marker_pub = self.create_publisher(MarkerArray, '/debug_marker', 1)
        self.control_state_pub = self.create_publisher(String, 'control_state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('🚀 GP Adaptive Lookahead Node Initialized')

    def adaptive_speed_callback(self, msg):
        self.adaptive_speed = msg.data

    def scan_callback(self, scan_data):
        ranges = np.array(scan_data.ranges)
        max_range = scan_data.range_max
        ranges[np.isinf(ranges)] = max_range

        safe_ranges = np.where(ranges > self.safety_radius, ranges, 0)
        best_direction = self.find_best_gap(safe_ranges, scan_data.angle_min, scan_data.angle_increment)
        self.publish_drive_command(best_direction)

    def find_best_gap(self, ranges, angle_min, angle_increment):
        safe_indices = np.where(ranges > self.safety_radius)[0]
        if len(safe_indices) == 0:
            return 0.0

        def normalize_angle(angle):
            return (angle + np.pi) % (2 * np.pi) - np.pi

        safe_indices = [idx for idx in safe_indices if -np.pi/2 <= normalize_angle(angle_min + idx * angle_increment) <= np.pi/2]
        if len(safe_indices) == 0:
            return 0.0

        gaps = []
        gap_start = safe_indices[0]
        for i in range(1, len(safe_indices)):
            if safe_indices[i] - safe_indices[i - 1] > 1:
                gaps.append((gap_start, safe_indices[i - 1]))
                gap_start = safe_indices[i]
        gaps.append((gap_start, safe_indices[-1]))

        largest_gap = max(gaps, key=lambda gap: gap[1] - gap[0])
        mid_index = (largest_gap[0] + largest_gap[1]) // 2
        best_angle = normalize_angle(angle_min + mid_index * angle_increment)
        best_angle = (best_angle + np.pi) % (2 * np.pi) - np.pi

        print(f"Largest gap: Start {largest_gap[0]}, End {largest_gap[1]}, Best Angle: {np.degrees(best_angle):.1f}°")
        return best_angle

    def publish_drive_command(self, best_angle):
        twist_cmd = Twist()
        twist_cmd.linear.x = self.adaptive_speed
        twist_cmd.angular.z = best_angle * self.steering_sensitivity
        if self.is_running:
            self.cmd_pub.publish(twist_cmd)

    def timer_callback(self):
        pass

    def shutdown_node(self):
        self.get_logger().info('🔻 Shutting down GP Adaptive Lookahead node')
        self.is_running = False
        try:
            twist_cmd = Twist()
            twist_cmd.linear.x = 0.0
            twist_cmd.angular.z = 0.0
            self.cmd_pub.publish(twist_cmd)
            time.sleep(0.5)
            self.cmd_pub.publish(twist_cmd)
        except Exception as e:
            print(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions(2))
    node = GPAdaptiveLookahead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_node()
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception as e:
            print(f"An error occurred: {e}")
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
