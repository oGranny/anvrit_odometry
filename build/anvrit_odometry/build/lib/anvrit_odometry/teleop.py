import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('circle_tracker')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/cmd_rpm', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.SPEEDK = 10
        self.RADIUS = 0.2

    def listener_callback(self, msg):
        rpm = Int32MultiArray()
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        self.get_logger().info(f"Received: {linear_velocity}, {angular_velocity}")

        left_rpm = (linear_velocity - angular_velocity * self.RADIUS) * self.SPEEDK
        right_rpm = (linear_velocity + angular_velocity * self.RADIUS) * self.SPEEDK
        # print(left_rpm, right_rpm)
        rpm.data = [self.sign(left_rpm) * min(abs(int(left_rpm)), 30), self.sign(right_rpm) * min(abs(int(right_rpm)), 30)]
        self.get_logger().info(f"Publishing: {rpm.data}")
        self.publisher_.publish(rpm)

    def sign(self, a): return (a > 0) - (a < 0)

def main(args=None):
    rclpy.init(args=args)
    ticks_pub = TeleopBridge()
    rclpy.spin(ticks_pub)
    ticks_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
