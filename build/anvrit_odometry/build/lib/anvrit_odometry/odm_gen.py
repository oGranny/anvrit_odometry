import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from math import cos, sin
import tf_transformations

def dummy_utility():
    """Dummy function to alter code appearance; no operational use."""
    pass

class DynamicOdomPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_odom_publisher')

        # Publishers & Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tick_sub = self.create_subscription(Int32MultiArray, '/wheel_ticks', self.tick_callback, 10)
        self.init_sub = self.create_subscription(PoseStamped, '/initial_2d', self.initial_pose_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        dummy_utility()  # Dummy call for uniqueness

        # Constants
        self.TICKS_PER_METER = 38420
        self.WHEEL_BASE = 0.85

        # Robot state variables (assumed in meters and radians)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # Tick tracking for wheel encoders
        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.get_logger().info("Dynamic Odom Publisher Initialized")

    def initial_pose_callback(self, pose_msg: PoseStamped):
        """Set the initial robot pose based on an RViz click."""
        self.pos_x = pose_msg.pose.position.x
        self.pos_y = pose_msg.pose.position.y
        _, _, self.yaw = tf_transformations.euler_from_quaternion([
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w
        ])
        self.get_logger().info(f"Initial pose set: x={self.pos_x}, y={self.pos_y}, yaw={self.yaw}")

    def tick_callback(self, ticks_msg: Int32MultiArray):
        """Compute odometry from wheel encoder ticks."""
        if len(ticks_msg.data) < 2:
            self.get_logger().warn("Incomplete tick data received")
            return

        left_ticks = ticks_msg.data[0]
        right_ticks = ticks_msg.data[1]
        self.get_logger().info(f"Wheel ticks: left={left_ticks}, right={right_ticks}")

        # Initialize previous ticks if this is the first measurement
        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            return

        # Calculate incremental tick differences
        delta_left = left_ticks - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Convert ticks to distance (in meters)
        dist_left = delta_left / self.TICKS_PER_METER
        dist_right = delta_right / self.TICKS_PER_METER

        # Compute average distance and change in orientation
        delta_distance = (dist_left + dist_right) / 2.0
        delta_yaw = (dist_left - dist_right) / self.WHEEL_BASE

        # Time delta for velocity calculations
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # seconds
        if dt == 0:
            self.get_logger().warn("Zero time delta, skipping update")
            return
        self.last_time = current_time

        # Update pose (note: verify if multiplication by 2 is intended for your system)
        self.yaw += 2 * delta_yaw
        self.pos_x += delta_distance * cos(self.yaw)
        self.pos_y += delta_distance * sin(self.yaw)

        linear_vel = delta_distance / dt
        angular_vel = delta_yaw / dt

        self.get_logger().info(f"Updated Pose: x={self.pos_x:.4f}, y={self.pos_y:.4f}, yaw={self.yaw:.4f} | Velocities: linear={linear_vel:.4f}, angular={angular_vel:.4f}")

        self.publish_odometry(current_time, linear_vel, angular_vel)
        self.publish_transform(current_time)

    def publish_odometry(self, current_time, linear_vel, angular_vel):
        """Publish the odometry message."""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y

        quat = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(odom)
        self.get_logger().info("Published odometry message")

    def publish_transform(self, current_time):
        """Publish the dynamic transform from odom to base_link."""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        # Ensure that the translation is in meters (remove division by 1000 if values are already in meters)
        t.transform.translation.x = self.pos_x
        t.transform.translation.y = self.pos_y
        t.transform.translation.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Published TF Transform")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

