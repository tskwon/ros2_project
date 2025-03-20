#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <wiringPi.h>

class IRSensorReader : public rclcpp::Node {
public:
    IRSensorReader() : Node("ir_sensor_reader_node") {
        // Publisher 설정 (5개 센서 값을 배열로 퍼블리시)
        ir_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/ir_sensor_data", 10);
        
        // 타이머 설정 (100ms마다 센서 값 읽기)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                         std::bind(&IRSensorReader::publish_sensor_data, this));

        // GPIO 핀 번호 설정
        ir_pins_ = {17,18,27,22,23};  // Raspberry Pi 기준 GPIO 핀 번호

        // GPIO 초기화
        wiringPiSetupGpio();
        for (int pin : ir_pins_) {
            pinMode(pin, INPUT);
        }

        RCLCPP_INFO(this->get_logger(), "IR Sensor Reader Node Started!");
    }

private:
    void publish_sensor_data() {
        std_msgs::msg::Int32MultiArray msg;
        for (int pin : ir_pins_) {
            msg.data.push_back(1-digitalRead(pin)); // 디지털 입력 읽기
        }

        ir_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published IR Sensor Data: [%d, %d, %d, %d, %d]", 
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]);
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ir_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<int> ir_pins_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRSensorReader>());
    rclcpp::shutdown();
    return 0;
}
