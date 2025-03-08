import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import atan, cos, sin


class OdometryGen(Node):

    def __init__(self):
        super().__init__('ticks_counter')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.ticks_subscription = self.create_subscription(Int32MultiArray, '/wheel_ticks', self.ticks_callback, 10)
        self.rviz_click_subscription = self.create_subscription(PoseStamped, '/initial_2d', self.set_initial_2d, 10)
        
        self.new_odom = Odometry()
        self.old_odom = Odometry()
        self.tf_broadcaster = TransformBroadcaster(self)


        self.TICKS_PER_REVOLUTION = 47000
        self.WHEEL_RADIUS = 0.2
        self.WHEEL_BASE = 0.85
        self.TICKS_PER_METER = 38420

        self.distance_left = 0
        self.distance_right = 0
        self.prev_distance_left = 0
        self.prev_distance_right = 0
        
        self.initialX = 0.0
        self.initialY = 0.0
        self.initialTheta = 0.00000000001

    def set_initial_2d(self, rviz_click: PoseStamped):
        self.old_odom.pose.pose.position.x = rviz_click.pose.position.x
        self.old_odom.pose.pose.position.y = rviz_click.pose.position.y
        self.old_odom.pose.pose.orientation.z = rviz_click.pose.orientation.z

    def ticks_callback(self, ticks_data):
        # Convert wheel ticks to distance
        self.distance_left = ticks_data.data[0] / self.TICKS_PER_METER
        self.distance_right = ticks_data.data[1] / self.TICKS_PER_METER

        # Average distance traveled by the robot
        cycleDistance = (self.distance_right + self.distance_left) / 2
        cycleDistance /= 10  # scale factor for the cycle, if required

        # Calculate angular displacement (theta change)
        cycleAngle = atan((2 * self.distance_right - self.distance_left) / self.WHEEL_BASE)
        avgAngle = (cycleAngle + self.old_odom.pose.pose.orientation.z) / 2

        # Update position (x, y)
        self.new_odom.pose.pose.position.x = self.old_odom.pose.pose.position.x + cos(avgAngle) * cycleDistance
        self.new_odom.pose.pose.position.y = self.old_odom.pose.pose.position.y + sin(avgAngle) * cycleDistance
        self.new_odom.pose.pose.orientation.z = cycleAngle + self.old_odom.pose.pose.orientation.z

        # Update velocities
        self.new_odom.twist.twist.linear.x = cycleDistance / self.get_clock().now().to_msg().sec  # time per cycle, adjust as needed
        self.new_odom.twist.twist.angular.z = cycleAngle / self.get_clock().now().to_msg().sec  # angular change

        # Publish the updated Odometry
        self.publisher_.publish(self.new_odom)

        # Publish TF transformation
        t = TransformStamped()
        t.header.stamp = self.new_odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.new_odom.pose.pose.position.x
        t.transform.translation.y = self.new_odom.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.new_odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    odom_generator = OdometryGen()
    odom_generator.new_odom.pose.pose.position.z = 0.0
    odom_generator.new_odom.pose.pose.orientation.x = 0.0
    odom_generator.new_odom.pose.pose.orientation.y = 0.0
    odom_generator.new_odom.twist.twist.linear.x = 0.0
    odom_generator.new_odom.twist.twist.linear.y = 0.0
    odom_generator.new_odom.twist.twist.linear.z = 0.0
    odom_generator.new_odom.twist.twist.angular.x = 0.0
    odom_generator.new_odom.twist.twist.angular.y = 0.0
    odom_generator.new_odom.twist.twist.angular.z = 0.0
    odom_generator.old_odom.pose.pose.position.x = odom_generator.initialX
    odom_generator.old_odom.pose.pose.position.y = odom_generator.initialY
    odom_generator.old_odom.pose.pose.orientation.z = odom_generator.initialTheta

    rclpy.spin(odom_generator)
    odom_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

