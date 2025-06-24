#!/usr/bin/env python3
# adaptive_lookahead_feature.py - Adjust lookahead and speed based on LiDAR gap angle (exponential model)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import numpy as np

class AdaptiveLookaheadFeature(Node):
    def __init__(self):
        super().__init__('adaptive_lookahead_feature')

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.lookahead_pub = self.create_publisher(Float32, '/adaptive_lookahead', 10)
        self.speed_pub = self.create_publisher(Float32, '/reference_speed', 10)

        # Parameters
        self.min_lookahead = 0.8
        self.max_lookahead = 2.0
        self.k_lookahead = 0.05  # sensitivity for exponential drop

        self.min_speed = 0.5
        self.max_speed = 1.0

        self.get_logger().info("🚀 Adaptive Lookahead Feature Node (exponential) Initialized")

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        valid_indices = np.where((ranges > 0.1) & (ranges < 10.0))[0]
        if len(valid_indices) == 0:
            self.get_logger().warn("No valid LiDAR data.")
            return

        # Find widest gap direction
        best_index = valid_indices[np.argmax(ranges[valid_indices])]
        gap_angle_rad = angle_min + best_index * angle_increment
        gap_angle_deg = abs(gap_angle_rad * 180.0 / math.pi)

        # Exponential dropoff for lookahead based on gap angle
        lookahead = self.min_lookahead + (self.max_lookahead - self.min_lookahead) * math.exp(-self.k_lookahead * gap_angle_deg)
        lookahead = max(self.min_lookahead, min(self.max_lookahead, lookahead))

        # Map lookahead to speed (nonlinear)
        speed = self.min_speed + ((lookahead - self.min_lookahead) / (self.max_lookahead - self.min_lookahead))**2 * (self.max_speed - self.min_speed)
        speed = max(self.min_speed, min(self.max_speed, speed))

        self.lookahead_pub.publish(Float32(data=lookahead))
        self.speed_pub.publish(Float32(data=speed))

        self.get_logger().info(f"🔁 Gap angle: {gap_angle_deg:.1f}°, Lookahead: {lookahead:.2f} m, Speed: {speed:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveLookaheadFeature()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
