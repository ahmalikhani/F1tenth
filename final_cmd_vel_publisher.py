# final_cmd_vel_publisher.py - FTG with Nonlinear Weighted Adaptive Speed + /reference_speed Publisher

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import math

class AdaptiveGapFollower(Node):
    def __init__(self):
        super().__init__('final_cmd_vel_publisher')

        # Parameters
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('min_lookahead', 0.8)
        self.declare_parameter('max_lookahead', 2.0)
        self.declare_parameter('lookahead_sensitivity', 0.05)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('steering_sensitivity', 1.0)
        self.declare_parameter('safety_radius', 2.0)

        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_lookahead = self.get_parameter('min_lookahead').value
        self.max_lookahead = self.get_parameter('max_lookahead').value
        self.lookahead_sensitivity = self.get_parameter('lookahead_sensitivity').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.steering_sensitivity = self.get_parameter('steering_sensitivity').value
        self.safety_radius = self.get_parameter('safety_radius').value

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ref_speed_pub = self.create_publisher(Float32, '/reference_speed', 10)

        self.get_logger().info('✅ Node Initialized: Nonlinear Weighted Adaptive Speed')

    def lidar_callback(self, msg):
        ranges = np.nan_to_num(np.array(msg.ranges), nan=0.0, posinf=msg.range_max)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        safe_ranges = np.where(ranges > self.safety_radius, ranges, 0)
        best_angle = self.find_best_gap(safe_ranges, angle_min, angle_increment)

        if best_angle is None:
            self.get_logger().warn("❌ No valid gap found.")
            return

        # --- Adaptive Lookahead based on gap angle
        gap_angle_deg = abs(math.degrees(best_angle))
        lookahead = self.min_lookahead + (self.max_lookahead - self.min_lookahead) * math.exp(-self.lookahead_sensitivity * gap_angle_deg)
        lookahead = np.clip(lookahead, self.min_lookahead, self.max_lookahead)

        # --- Steering
        steering = best_angle * self.steering_sensitivity
        steering = np.clip(steering, -self.max_steering_angle, self.max_steering_angle)

        # --- Nonlinear Weighted Penalty (steering > lookahead)
        steering_factor = (abs(steering) / self.max_steering_angle) ** 1.5
        lookahead_factor = (1.0 - (lookahead - self.min_lookahead) / (self.max_lookahead - self.min_lookahead)) ** 1.2
        penalty = 0.7 * steering_factor + 0.3 * lookahead_factor

        adaptive_speed = self.max_speed * (1.0 - penalty)
        adaptive_speed = np.clip(adaptive_speed, self.min_speed, self.max_speed)

        # Publish command
        twist = Twist()
        twist.linear.x = adaptive_speed
        twist.angular.z = steering
        self.cmd_pub.publish(twist)
        self.ref_speed_pub.publish(Float32(data=adaptive_speed))

        self.get_logger().info(
            f"[FTG] Steering: {steering:.2f} rad | Lookahead: {lookahead:.2f} m | Speed: {adaptive_speed:.2f} m/s"
        )

    def find_best_gap(self, ranges, angle_min, angle_increment):
        safe_indices = np.where(ranges > self.safety_radius)[0]
        if len(safe_indices) == 0:
            return None

        def normalize_angle(angle):
            return (angle + np.pi) % (2 * np.pi) - np.pi

        safe_indices = [idx for idx in safe_indices if -np.pi/2 <= normalize_angle(angle_min + idx * angle_increment) <= np.pi/2]
        if len(safe_indices) == 0:
            return None

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

        return best_angle

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveGapFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
