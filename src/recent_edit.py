                                         
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ThrusterSubscriber(Node):
    def __init__(self):
        super().__init__('thruster_subscriber')
        self.subscription = self.create_subscription(
        String, 'char_topic', self.listener_callback, 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)  # Adjust port if needed
        time.sleep(2)
        self.thruster_values = [1500] * 8
        self.initialize_thrusters()

    def initialize_thrusters(self):
        self.get_logger().info("Initializing all thrusters to neutral (1500 µs)...")
        for i in range(8):
            self.set_thruster_speed(i, 1500)
        time.sleep(2)

    def set_thruster_speed(self, thruster_id, pulse_width):
        if 0 <= thruster_id < 8 and 1100 <= pulse_width <= 1900:
            self.ser.flushInput()
            command = f"{thruster_id},{pulse_width}\n"
            self.ser.write(command.encode())
            time.sleep(0.015)
            self.get_logger().info(f"Sent to thruster {thruster_id}: {pulse_width}")
        else:
            self.get_logger().error("Invalid thruster ID or pulse width")

    def listener_callback(self, msg):
        char = msg.data
        self.get_logger().info(f'Received: "{char}"')

        if char == 'i':
            for i in range(8):
                self.thruster_values[i] = min(1900, self.thruster_values[i] + 30)
                self.set_thruster_speed(i, self.thruster_values[i])

                self.set_thruster_speed(i, self.thruster_values[i])
            self.get_logger().info(f"All thrusters set to: {self.thruster_values[0]}")

        elif char == 'd':
            for i in range(8):
                self.thruster_values[i] = max(1100, self.thruster_values[i] - 30)
                self.set_thruster_speed(i, self.thruster_values[i])
            self.get_logger().info(f"All thrusters set to: {self.thruster_values[0]}")

        elif char == 'h':
             for i in range(4):
                self.thruster_values[i] = min(1900,self.thruster_values[i]+20)
                self.set_thruster_speed(i,self.thruster_values[i])

        elif char == 'r':
            self.get_logger().info("All thrusters are reset")
            for i in range(8):
                self.thruster_values[i] = 1500
                self.set_thruster_speed(i, 1500)

        elif char == 'q':
            self.get_logger().info("Stopping all thrusters...")
            for i in range(8):
                self.thruster_values[i] = 1500
                self.set_thruster_speed(i, 1500)
            rclpy.shutdown()  # Shutdown node on 'q'

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted")
    finally:
        node.ser.close()
        node.get_logger().info("Closing serial connection...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
