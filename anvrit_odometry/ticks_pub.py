import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class TicksPublisher(Node):
    def __init__(self):
        super().__init__('circle_tracker')
        self.baud_rate = 9600
        self.serial_port = "/dev/ttyUSB0"
        self.data = Int32MultiArray()
        self.publisher_ = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

    def publish_ticks(self) -> list:
        if self.ser is None:
            return

        arr = [0, 0]
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    data = line.split(' ')
                    if len(data) >= 2:
                        arr[0] = int(data[0])
                        arr[1] = int(data[1])
                        self.get_logger().info(f"Received values: {arr}")
                        self.data.data = arr
                        self.publisher_.publish(self.data)
        except Exception as e:
            self.get_logger().error(f"Error reading from serial port: {e}")


def main(args=None):
    rclpy.init(args=args)
    ticks_pub = TicksPublisher()
    timer_period = 0.05 
    ticks_pub.create_timer(timer_period, ticks_pub.publish_ticks)
    rclpy.spin(ticks_pub)
    ticks_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
