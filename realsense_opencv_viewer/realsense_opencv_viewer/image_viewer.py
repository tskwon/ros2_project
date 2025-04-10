import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RealsenseViewer(Node):
    def __init__(self):
        super().__init__('realsense_opencv_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw/compressedDepth',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("Received image frame!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("RealSense Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

def main(args=None):
    rclpy.init(args=args)
    viewer = RealsenseViewer()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()