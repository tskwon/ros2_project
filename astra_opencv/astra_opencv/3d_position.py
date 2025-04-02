import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MultiBox3DLocator(Node):
    def __init__(self):
        super().__init__('multi_box_3d_locator')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None

        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, qos_profile)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, qos_profile)
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, qos_profile)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f"Depth image conversion error: {e}")

    def rgb_callback(self, msg):
        if self.camera_info is None or self.depth_image is None:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # 윤곽선 추출 (네모 박스 검출)
            _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)

                # 너무 작은 박스는 무시
                if w < 30 or h < 30:
                    continue

                # 중심점 좌표
                cx = x + w // 2
                cy = y + h // 2

                # Depth 가져오기 (중심점 기반)
                depth = self.depth_image[cy, cx]

                # Depth 유효하지 않으면 skip
                if depth == 0:
                    continue

                # Camera Intrinsics
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                cx_k = self.camera_info.k[2]
                cy_k = self.camera_info.k[5]

                z = float(depth)
                x3d = (cx - cx_k) * z / fx
                y3d = (cy - cy_k) * z / fy

                # 정보 출력
                self.get_logger().info(f"[Box @ ({cx},{cy})] 3D Position: x={x3d:.1f}mm, y={y3d:.1f}mm, z={z:.1f}mm")

                # 영상에 표시
                cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(color_image, f"Z={z/1000:.2f}m", (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow("MultiBox RGB + Depth", color_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"RGB image error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiBox3DLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
