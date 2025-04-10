import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from .ggcnn_model import GGCNN  # GGCNN 모델은 같은 디렉토리에 있어야 합니다

class GGCNNGraspNode(Node):
    def __init__(self):
        super().__init__('ggcnn_depth_only_grasp_node')

        self.bridge = CvBridge()
        self.model = GGCNN()
        self.model.load_state_dict(torch.load(
            '/home/xotn/ros2_ws/src/ggcnn_grasp_detector/model/ggcnn/ggcnn_weights_cornell/ggcnn_epoch_23_cornell_statedict.pt',
            map_location=torch.device('cpu')))
        self.model.eval()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.image_counter = 0

        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, qos_profile)

        self.get_logger().info("✅ GGCNN Depth-only Grasp Node Initialized!")

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # 모델 입력 전처리
            depth_resized = cv2.resize(depth_image, (300, 300))
            depth_tensor = torch.from_numpy(depth_resized).unsqueeze(0).unsqueeze(0).float()

            with torch.no_grad():
                pos_out, cos_out, sin_out, width_out = self.model(depth_tensor)

            q_img = pos_out.squeeze().numpy()
            cos_img = cos_out.squeeze().numpy()
            sin_img = sin_out.squeeze().numpy()
            width_img = width_out.squeeze().numpy()

            ang_img = 0.5 * np.arctan2(sin_img, cos_img)

            max_idx = np.unravel_index(np.argmax(q_img), q_img.shape)
            grasp_y, grasp_x = max_idx
            grasp_z = depth_resized[grasp_y, grasp_x]
            grasp_angle = ang_img[grasp_y, grasp_x]

            # 시각화용 흑백 이미지 생성
            norm_depth = cv2.normalize(depth_resized, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            vis_image = cv2.cvtColor(norm_depth, cv2.COLOR_GRAY2BGR)

            cv2.circle(vis_image, (grasp_x, grasp_y), 8, (0, 255, 0), 2)
            cv2.putText(vis_image, f"x={grasp_x}, y={grasp_y}", (grasp_x + 10, grasp_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(vis_image, f"z={grasp_z / 1000:.2f}m", (grasp_x + 10, grasp_y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(vis_image, f"angle={np.degrees(grasp_angle):.1f}°", (grasp_x + 10, grasp_y + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if os.environ.get('DISPLAY', '') == '':
                filename = f'/tmp/ggcnn_grasp_depth_{self.image_counter:04d}.jpg'
                cv2.imwrite(filename, vis_image)
                self.get_logger().warn(f"🖼️ No display found. Saved to {filename}")
                self.image_counter += 1
            else:
                cv2.imshow("GGCNN Grasp (Depth Only)", vis_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Depth callback error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GGCNNGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
