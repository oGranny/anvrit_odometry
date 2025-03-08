import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder")
        self.bridge = CvBridge()
        self.output_file = "Output.mp4"
        self.fps = 30
        self.video_writer = None
        self.subscription = self.create_subscription(
            Image,
            "/flir_camera/image_raw",  # Replace with your topic
            self.callback,
            10
        )

    def callback(self, msg):
        # Convert ROS 2 Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Initialize video writer with first frame
        if self.video_writer is None:
            height, width = cv_image.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(self.output_file, fourcc, self.fps, (width, height))

        # Write frame
        self.video_writer.write(cv_image)

    def __del__(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info("Video saved as Output.mp4")

def main(args=None):
    rclpy.init(args=args)
    recorder = VideoRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()