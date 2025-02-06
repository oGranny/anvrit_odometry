import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
    
class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Controller node has been started.')
        self.target_rpm_subscriber = self.create_subscription(Int32MultiArray,'/rpm_vel',self.target_rpm_callback,10)
        self.actual_rpm_subscriber = self.create_subscription(Int32MultiArray,'/rpm',self.actual_rpm_callback,10)

        self.target_rpm_left = 0
        self.target_rpm_right = 0
        self.actual_rpm_left = 0
        self.actual_rpm_right = 0
        self.kp = 1.0  # Proportional gain for the PID controller

        self.baud_rate = 9600
        self.serial_port = "/dev/ttyUSB0"

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None



    def target_rpm_callback(self, msg):
        self.target_rpm_left = msg.data[0]
        self.target_rpm_right = msg.data[1]
        self.control_loop()

    def actual_rpm_callback(self, msg):
        self.actual_rpm_left = msg.data[0]
        self.actual_rpm_right = msg.data[1]

    def control_loop(self):
        error_left = self.target_rpm_left - self.actual_rpm_left
        error_right = self.target_rpm_right - self.actual_rpm_right

        control_signal_left = self.kp * error_left
        control_signal_right = self.kp * error_right

        self.actual_rpm_left += control_signal_left
        self.actual_rpm_right += control_signal_right

        if self.ser:
            try:
                self.ser.write(f'{int(self.actual_rpm_left)} {int(self.actual_rpm_right)}\n'.encode())
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()