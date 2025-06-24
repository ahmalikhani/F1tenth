import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import numpy as np
import time

class FollowTheGapAdaptiveSpeedNode(Node):
    def __init__(self):
        super().__init__('gp_adaptive_speed')

        # Parameters
        self.safety_radius = 2.0   # Minimum safe distance from obstacles
        self.max_steering_angle = 0.52
        self.steering_sensitivity = 0.7
        self.is_running = True

        # Initial adaptive speed (default)
        self.adaptive_speed = 0.5  

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.speed_sub = self.create_subscription(Float32, '/reference_speed', self.speed_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.control_state_pub = self.create_publisher(String, 'control_state', 10)

        self.timer1 = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Adaptive Follow the Gap node has started.')

    def speed_callback(self, msg):
        self.adaptive_speed = msg.data

    def scan_callback(self, scan_data):
        ranges = np.array(scan_data.ranges)
        max_range = scan_data.range_max
        ranges[np.isinf(ranges)] = max_range

        safe_ranges = np.where(ranges > self.safety_radius, ranges, 0)

        best_angle = self.find_best_gap(safe_ranges, scan_data.angle_min, scan_data.angle_increment)

        self.publish_drive_command(best_angle)

    def find_best_gap(self, ranges, angle_min, angle_increment):
        safe_indices = np.where(ranges > self.safety_radius)[0]

        if len(safe_indices) == 0:
            return 0.0  # Keep driving straight

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

        self.get_logger().info(f"Largest gap angle: {np.degrees(best_angle):.1f}°")
        return best_angle

    def publish_drive_command(self, best_angle):
        steering_value = best_angle * self.steering_sensitivity
        steering_value = np.clip(steering_value, -self.max_steering_angle, self.max_steering_angle)

        # Adaptive speed based on received reference
        adaptive_throttle = np.clip(self.adaptive_speed * (1 - (abs(steering_value)/self.max_steering_angle)**2),
                                    0.1, self.adaptive_speed)

        twist_cmd = Twist()
        twist_cmd.linear.x = adaptive_throttle
        twist_cmd.angular.z = steering_value

        if self.is_running:
            self.cmd_pub.publish(twist_cmd)

        state_msg = String()
        state_msg.data = f"Throttle: {adaptive_throttle:.2f}, Steering: {steering_value:.2f}"
        self.control_state_pub.publish(state_msg)

    def timer_callback(self):
        pass

    def shutdown_node(self):
        self.get_logger().info('Shutting down Adaptive Follow the Gap...')
        self.is_running = False
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.0
        twist_cmd.angular.z = 0.0
        try:
            self.cmd_pub.publish(twist_cmd)
            time.sleep(0.5)
            self.cmd_pub.publish(twist_cmd)
            self.get_logger().info('Adaptive Follow the Gap stopped.')
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapAdaptiveSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
