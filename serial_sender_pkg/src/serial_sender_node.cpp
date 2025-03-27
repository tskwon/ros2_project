#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <serial/serial.h>

class SerialSenderNode : public rclcpp::Node {
public:
    SerialSenderNode() : Node("serial_sender_node") {
    try {
        serial_.setPort("/dev/ttyACM0");
        serial_.setBaudrate(9600);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);  // 🔧 수정
        serial_.setTimeout(timeout);  // ✅ 에러 해결
        serial_.open();
        if (serial_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "시리얼 포트 열림!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "시리얼 포트를 열 수 없습니다.");
        }
    } catch (const std::exception &e) {
        RCLCPP_FATAL(this->get_logger(), "예외 발생: %s", e.what());
    }

    sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/number_data", 10,
        std::bind(&SerialSenderNode::topic_callback, this, std::placeholders::_1));
}


private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        std::string data = std::to_string(msg->data) + "\n";
        if (serial_.isOpen()) {
            serial_.write(data);
            RCLCPP_INFO(this->get_logger(), "전송됨: '%s'", data.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "시리얼 포트가 닫혀있음");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    serial::Serial serial_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialSenderNode>());
    rclcpp::shutdown();
    return 0;
}
