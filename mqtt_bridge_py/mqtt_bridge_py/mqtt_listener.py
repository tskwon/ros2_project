import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json

class MQTTListener(Node):
    def __init__(self):
        super().__init__('mqtt_listener')
        self.publisher_ = self.create_publisher(String, 'order', 10)

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # MQTT 브로커 주소 설정 (PC 주소 사용)
        self.client.connect("192.168.137.56", 1883, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("ros2/order")

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            self.get_logger().info(f"Received MQTT: {payload}")
            ros_msg = String()
            ros_msg.data = payload
            self.publisher_.publish(ros_msg)
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MQTTListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
