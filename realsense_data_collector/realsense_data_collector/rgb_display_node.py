# realsense_data_collector/realsense_data_collector/rgb_display_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RealSenseRGBDisplay(Node):

    def __init__(self):
        super().__init__('realsense_rgb_display_node_py')
        self.get_logger().info('RealSense RGB Display Node (Python) has been started.')

        # CvBridge 인스턴스 생성
        self.br = CvBridge()

        # 이미지 토픽 구독 설정
        # RealSense에서 컬러 이미지가 발행되는 토픽 이름 사용
        # '/camera/camera/color/image_raw' 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1
            ) 
        )
        self.subscription  # prevent unused variable warning

        # OpenCV 윈도우 생성
        cv2.namedWindow("RealSense RGB Image", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info('OpenCV window created. Waiting for images...')

    def __del__(self):
        # 노드 소멸 시 OpenCV 윈도우 닫기
        cv2.destroyAllWindows()
        self.get_logger().info('OpenCV window destroyed.')

    def image_callback(self, msg):
        # self.get_logger().info('Receiving video frame')
        try:
            # ROS 이미지 메시지를 OpenCV 이미지 (NumPy 배열)로 변환
            # 인코딩은 'bgr8' 또는 'rgb8' 사용. OpenCV는 'bgr8'을 주로 사용.
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 이미지가 비어있지 않으면 화면에 표시
            if cv_image is not None and not cv_image.size == 0:
                display_image = cv2.resize(cv_image, (320, 240))
                cv2.imshow("RealSense RGB Image", display_image)
                # 윈도우 이벤트를 처리하고 이미지를 업데이트하기 위해 필요
                # 1ms 대기하며 키 입력을 확인
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error('Error converting or displaying image: %s' % str(e))


def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)

    # 노드 생성
    image_display_node = RealSenseRGBDisplay()

    try:
        # 노드 스핀 (콜백 함수 실행)
        rclpy.spin(image_display_node)
    except KeyboardInterrupt:
        pass # Ctrl+C 시 종료

    # 노드 정리 및 종료
    image_display_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows() # 혹시 남은 윈도우가 있다면 모두 닫기


if __name__ == '__main__':
    main()