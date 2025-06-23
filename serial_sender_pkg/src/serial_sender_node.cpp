#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <libserial/SerialPort.h>
#include <string>

class SerialSenderNode : public rclcpp::Node {
public:
    SerialSenderNode() : Node("serial_sender_node") {
    try {
        // 시리얼 포트 설정
        serialPort_.Open("/dev/ttyACM0");
        serialPort_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
        serialPort_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serialPort_.SetParity(LibSerial::Parity::PARITY_NONE);
        serialPort_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serialPort_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        
        // 타임아웃 설정 (libserial에서는 다른 방식 사용)
        serialPort_.SetVTime(10); // 1초 타임아웃

        if (serialPort_.IsOpen()) {
            RCLCPP_INFO(this->get_logger(), "시리얼 포트 열림!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "시리얼 포트를 열 수 없습니다.");
        }
    } catch (const LibSerial::OpenFailed &e) {
        RCLCPP_FATAL(this->get_logger(), "예외 발생: %s", e.what());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(this->get_logger(), "일반 예외 발생: %s", e.what());
    }
    
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/number_data", 10,
        std::bind(&SerialSenderNode::topic_callback, this, std::placeholders::_1));
}

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // 개행 문자 없이 숫자만 전송
        std::string data = std::to_string(msg->data);
        
        if (serialPort_.IsOpen()) {
            // LibSerial에서는 문자열 형태로 전송
            serialPort_.Write(data);
            RCLCPP_INFO(this->get_logger(), "전송됨: '%s'", data.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "시리얼 포트가 닫혀있음");
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    LibSerial::SerialPort serialPort_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialSenderNode>());
    rclcpp::shutdown();
    return 0;
}