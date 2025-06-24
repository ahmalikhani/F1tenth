import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class FollowTheGapBaselineSpeedPublisher(Node):
    def __init__(self):
        super().__init__('follow_the_gap_baseline_reference_speed')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.publisher_ = self.create_publisher(Float32, '/reference_speed', 10)
        self.get_logger().info("📡 follow_the_gap_baseline_reference_speed node started")

    def cmd_callback(self, msg):
        speed = Float32()
        speed.data = msg.linear.x
        self.publisher_.publish(speed)

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapBaselineSpeedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
