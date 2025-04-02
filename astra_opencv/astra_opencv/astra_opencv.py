import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class AstraOpenCV(Node):

    def __init__(self):
        super().__init__('astra_opencv')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile=qos_profile
        )
        self.bridge = CvBridge()
        # OpenCL 활성화
        try:
            cv2.ocl.setUseOpenCL(True)
            self.get_logger().info("OpenCL is enabled")
        except:
            self.get_logger().warn("OpenCL is not supported or failed to enable")
        self.gpu_enabled = cv2.ocl.useOpenCL()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # GPU 가속 적용
            if self.gpu_enabled:
                gpu_image = cv2.UMat(cv_image)
                cv2.imshow('Astra+ Camera', gpu_image)
            else:
                cv2.imshow('Astra+ Camera', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    astra_opencv = AstraOpenCV()
    rclpy.spin(astra_opencv)
    astra_opencv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()