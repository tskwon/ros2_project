import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import subprocess
import time
import socket
import os

class MQTTListener(Node):
    def __init__(self):
        super().__init__('mqtt_listener')
        self.publisher_ = self.create_publisher(String, 'order', 10)
        
        # MQTT 브로커 자동 시작
        self.ensure_broker_running()
        
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # 브로커 시작 대기
        time.sleep(2)
        
        # 로컬 브로커에 연결
        self.client.connect("localhost", 1883, 60)
        self.client.loop_start()

    def ensure_broker_running(self):
        """MQTT 브로커가 실행 중인지 확인하고 필요시 시작"""
        
        # 브로커가 이미 실행 중인지 확인
        if self.is_broker_running():
            self.get_logger().info("MQTT 브로커가 이미 실행 중입니다.")
            return
        
        # 브로커 시작
        self.get_logger().info("MQTT 브로커를 시작합니다...")
        try:
            # 백그라운드에서 브로커 실행
            subprocess.Popen([
                'sudo', 'mosquitto', 
                '-c', '/etc/mosquitto/mosquitto.conf', 
                '-d'
            ])
            
            # 브로커 시작 대기
            for i in range(10):
                time.sleep(1)
                if self.is_broker_running():
                    self.get_logger().info("✅ MQTT 브로커 시작 완료")
                    return
            
            self.get_logger().error("MQTT 브로커 시작 실패")
            
        except Exception as e:
            self.get_logger().error(f"브로커 시작 중 오류: {e}")

    def is_broker_running(self):
        """브로커가 실행 중인지 확인"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex(('localhost', 1883))
            sock.close()
            return result == 0
        except:
            return False

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("✅ MQTT 브로커에 연결됨")
            client.subscribe("ros2/order")
            self.get_logger().info("📡 ros2/order 토픽 구독 시작")
        else:
            self.get_logger().error(f"연결 실패, 코드: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            self.get_logger().info(f"📱 핸드폰에서 수신: {payload}")
            
            ros_msg = String()
            ros_msg.data = payload
            self.publisher_.publish(ros_msg)
            
            self.get_logger().info(f"🤖 로봇으로 전송: {payload}")
        except Exception as e:
            self.get_logger().error(f"메시지 처리 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MQTTListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()