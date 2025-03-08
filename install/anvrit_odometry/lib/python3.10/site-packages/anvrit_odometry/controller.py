import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
    
class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Controller node has been started.')
        self.target_rpm_subscriber = self.create_subscription(Int32MultiArray,'/cmd_rpm',self.target_rpm_callback,10)
        self.actual_rpm_subscriber = self.create_subscription(Int32MultiArray,'/rpm',self.actual_rpm_callback,10)

        self.target_rpm_left = 0
        self.target_rpm_right = 0
        self.actual_rpm_left = 0
        self.actual_rpm_right = 0
        self.kp = 10.0  # Proportional gain for the PID controller

        self.baud_rate = 9600
        self.serial_port = "/dev/ttyUSB0"

        self.mapping = [ # Mapping from RPM to PWM
            (10, 3),
            (20, 3),
            (30, 4),
            (40, 6),
            (50, 8),
            (60, 10),
            (70, 12),
            (80, 14),
            (90, 16),
            (100, 18),
            (110, 19),
            (120, 20)
        ]

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
        print(f"Target RPM: {self.target_rpm_left}, {self.target_rpm_right}")
        error_left = self.target_rpm_left - self.actual_rpm_left
        error_right = self.target_rpm_right - self.actual_rpm_right

        control_signal_left = self.kp * error_left
        control_signal_right = self.kp * error_right

        self.actual_rpm_left += control_signal_left
        self.actual_rpm_right += control_signal_right

        if self.ser:
            try:
                print(f"Sending: {int(self.rpm_to_pwm(self.actual_rpm_left))}, {int(self.rpm_to_pwm(self.actual_rpm_right))}")
                self.ser.write(f'{int(self.rpm_to_pwm(self.actual_rpm_left))} {int(self.rpm_to_pwm(self.actual_rpm_right))}\n'.encode())
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial port: {e}")

    def rpm_to_pwm(self, rpm):
        
        if rpm < self.mapping[0][0]:
            return self.mapping[0][1]
        for i in range(len(self.mapping) - 1):
            if self.mapping[i][0] <= rpm < self.mapping[i+1][0]:
                x0, y0 = self.mapping[i]
                x1, y1 = self.mapping[i+1]
                return int(y0 + (y1 - y0) * (rpm - x0) / (x1 - x0))
        return self.mapping[-1][1]


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()