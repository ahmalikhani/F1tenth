import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class AdaptiveSpeedFeatureBalanced(Node):
    def __init__(self):
        super().__init__('adaptive_speed_feature_balanced')

        # 📌 پارامترهای تنظیمی
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('max_lidar_distance', 5.0)

        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.max_lidar_distance = self.get_parameter('max_lidar_distance').get_parameter_value().double_value

        # 📡 Subscriber به داده‌های LiDAR
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # 📤 Publisher ها
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reference_speed_pub = self.create_publisher(Float32, '/reference_speed', 10)

        self.get_logger().info("🚀 Adaptive Speed Feature (Balanced Version) started")

    def lidar_callback(self, msg):
        # تبدیل داده‌های LiDAR به numpy
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        valid_indices = np.where((ranges > 0.1) & (ranges < self.max_lidar_distance))[0]
        if len(valid_indices) == 0:
            self.get_logger().warn("⚠️ No valid LiDAR data.")
            return

        # پیدا کردن بهترین gap
        best_index = valid_indices[np.argmax(ranges[valid_indices])]
        best_distance = ranges[best_index]
        best_angle = angle_min + best_index * angle_increment

        # ⚙️ نرمال‌سازی
        normalized_angle = abs(best_angle) / self.max_steering_angle
        normalized_distance = best_distance / self.max_lidar_distance

        # ✅ فرمول جدید adaptive speed
        speed = self.min_speed + (self.max_speed - self.min_speed) * (1 - normalized_angle)**2 * normalized_distance
        speed = np.clip(speed, self.min_speed, self.max_speed)

        # ارسال فرمان cmd_vel
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = best_angle
        self.cmd_pub.publish(twist)

        # انتشار /reference_speed
        self.reference_speed_pub.publish(Float32(data=speed))

        # 🔎 لاگ برای تحلیل
        self.get_logger().info(
            f"Gap angle: {np.degrees(best_angle):.1f}°, Distance: {best_distance:.2f} m | Speed: {speed:.2f} m/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSpeedFeatureBalanced()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
