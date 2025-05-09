import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import gpiod
import time

class IRSensorReader(Node):
    def __init__(self):
        super().__init__('ir_sensor_reader_node')
        
        # Publisher 설정
        self.ir_publisher = self.create_publisher(
            Int32MultiArray,
            '/ir_sensor_data',
            10
        )
        
        # GPIO 핀 번호 설정 (BCM 번호 그대로 사용)
        self.ir_pins = [17, 18, 27, 22, 23]
        
        # GPIO 초기화 - gpiochip4 사용
        self.chip = None
        self.lines = []
        
        try:
            # 라즈베리파이 5에서는 gpiochip4 사용
            self.chip = gpiod.Chip('/dev/gpiochip4')
            self.get_logger().info(f"Successfully opened chip: {self.chip.name()}")
            
            # 각 핀을 입력으로 설정
            for pin in self.ir_pins:
                try:
                    line = self.chip.get_line(pin)
                    # 핀을 입력으로 설정 (v1 API 사용)
                    line.request(consumer="ir_sensor", type=gpiod.LINE_REQ_DIR_IN, flags=0)
                    self.lines.append(line)
                    self.get_logger().info(f"Successfully configured pin {pin}")
                except Exception as e:
                    self.get_logger().error(f"Error setting up pin {pin}: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Error initializing GPIO: {str(e)}")
            self.lines = []
        
        # 타이머 설정 (15ms마다 센서 값 읽기)
        self.timer = self.create_timer(0.011, self.publish_sensor_data)
        
        self.get_logger().info(f'IR Sensor Reader Node Started with {len(self.lines)} pins configured')
    
    def publish_sensor_data(self):
        msg = Int32MultiArray()
        
        if not self.lines:
            msg.data = [0, 0, 0, 0, 0]
            self.ir_publisher.publish(msg)
            return
        
        # 각 핀에서 센서 값 읽기
        sensor_values = []
        for i, line in enumerate(self.lines):
            try:
                # 0이 감지, 1이 비감지이므로 1에서 빼줌
                value = 1 - line.get_value()
                sensor_values.append(value)
            except Exception as e:
                self.get_logger().error(f"Error reading pin {self.ir_pins[i]}: {str(e)}")
                sensor_values.append(0)
        
        # 구성된 핀이 5개보다 적으면 나머지는 0으로 채움
        while len(sensor_values) < 5:
            sensor_values.append(0)
        
        msg.data = sensor_values
        
        # 메시지 발행
        self.ir_publisher.publish(msg)
        
        # 로그 출력
        self.get_logger().info(f'Published IR Sensor Data: {msg.data}')
    
    def destroy_node(self):
        # 노드 종료 시 GPIO 정리
        for line in self.lines:
            try:
                line.release()
            except Exception as e:
                self.get_logger().error(f"Error releasing line: {str(e)}")
        
        # 칩 닫기
        if hasattr(self, 'chip') and self.chip:
            try:
                self.chip.close()
            except:
                pass
                
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IRSensorReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()