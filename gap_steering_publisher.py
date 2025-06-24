#!/usr/bin/env python3
# gap_steering_publisher.py – Robust Gap Steering for Follow-the-Gap

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np

class GapSteeringPublisher(Node):
    def __init__(self):
        super().__init__('gap_steering_node')

        # تنظیمات ثابت
        self.safety_radius = 1.0   # حداقل فاصله امن
        self.max_steering_angle = 0.52  # محدودیت فیزیکی ربات
        self.min_gap_width = 8     # حداقل نقاط لازم برای یک gap واقعی

        # Publisher و Subscriber
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.steering_pub = self.create_publisher(Float64, '/raw_steering', 10)

        self.get_logger().info('✅ Robust Gap Steering Node started')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges[np.isinf(ranges)] = msg.range_max

        # فقط نقاط امن
        safe_ranges = np.where(ranges > self.safety_radius, ranges, 0)

        best_angle = self.find_best_gap(safe_ranges, angle_min, angle_increment)

        steering_angle = np.clip(best_angle, -self.max_steering_angle, self.max_steering_angle)

        self.steering_pub.publish(Float64(data=steering_angle))

        self.get_logger().info(f"📐 Steering: {steering_angle:.3f} rad | Best Gap Center: {np.degrees(best_angle):.1f}°")

    def find_best_gap(self, ranges, angle_min, angle_increment):
        safe_indices = np.where(ranges > self.safety_radius)[0]
        if len(safe_indices) == 0:
            return 0.0

        # فقط زاویه‌های روبروی ربات (±60 درجه ≈ ±1.05 رادیان)
        def normalize_angle(angle):
            return (angle + np.pi) % (2 * np.pi) - np.pi

        safe_indices = [
            i for i in safe_indices
            if -1.05 <= normalize_angle(angle_min + i * angle_increment) <= 1.05
        ]

        if len(safe_indices) == 0:
            return 0.0

        # پیدا کردن gap های متوالی
        gaps = []
        start_idx = safe_indices[0]
        for i in range(1, len(safe_indices)):
            if safe_indices[i] - safe_indices[i-1] > 1:
                gaps.append((start_idx, safe_indices[i-1]))
                start_idx = safe_indices[i]
        gaps.append((start_idx, safe_indices[-1]))

        # فقط gapهای کافی بزرگ
        gaps = [gap for gap in gaps if (gap[1] - gap[0]) >= self.min_gap_width]

        if len(gaps) == 0:
            return 0.0

        # انتخاب بزرگ‌ترین gap
        largest_gap = max(gaps, key=lambda g: g[1] - g[0])

        # انتخاب مرکز gap به عنوان بهترین زاویه
        mid_idx = (largest_gap[0] + largest_gap[1]) // 2
        best_angle = normalize_angle(angle_min + mid_idx * angle_increment)

        return best_angle

def main(args=None):
    rclpy.init(args=args)
    node = GapSteeringPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
