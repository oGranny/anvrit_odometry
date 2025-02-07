import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class RpmPublisher(Node):

    def __init__(self):
        super().__init__('circle_tracker')
        
        self.TICKS_PER_REVOLUTION = 23000
        self.WHEEL_RADIUS = 0.2
        self.WHEEL_BASE = 0.85
        self.TICKS_PER_METER = 18312
        self.TIME_PERIOD = 0.05
        self.current_ticks = [0, 0]
        self.prev_ticks = [0, 0]

        self.rpm_msg = Int32MultiArray()

        self.ticks_subscription = self.create_subscription(Int32MultiArray, '/wheel_ticks', self.ticks_callback, 10)
        self.publisher_ = self.create_publisher(Int32MultiArray, '/rpm', 10)
        self.rpm_pub_timer = self.create_timer(self.TIME_PERIOD, self.calculate_rpm)

    def calculate_rpm(self):
        ticks_diff = [0, 0]
        rpm = []
        for i in range(2):
            ticks_diff[i] = self.current_ticks[i] - self.prev_ticks[i]
            # self.get_logger().info(f"Ticks diff: {ticks_diff[i]}")
            # self.get_logger().info(f"prev_ticks: {self.prev_ticks[i]}")
            # self.get_logger().info(f"current_ticks: {self.current_ticks[i]}")
            _rpm = (ticks_diff[i] / self.TICKS_PER_REVOLUTION) * (60.0 / self.TIME_PERIOD)
            rpm.append(_rpm)

        self.rpm_msg.data = [-1* int(r) for r in rpm]
        self.publisher_.publish(self.rpm_msg)

    def ticks_callback(self, ticks_data):
        # self.get_logger().info(f"Ticks data: {ticks_data.data}")
        self.prev_ticks = self.current_ticks[:]
        self.current_ticks[0] = ticks_data.data[0]
        self.current_ticks[1] = ticks_data.data[1]


def main(args=None):
    rclpy.init(args=args)
    odom_generator = RpmPublisher()    
    rclpy.spin(odom_generator)
    odom_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
