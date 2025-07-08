#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mqtt/async_client.h>

class MQTTListener : public rclcpp::Node {
public:
    MQTTListener()
    : Node("mqtt_listener"),
      mqtt_client_("tcp://192.168.137.56:1883", "mqtt_cpp_listener") // MQTT 브로커 주소
    {
        // ROS 2 퍼블리셔 생성
        ros_publisher_ = this->create_publisher<std_msgs::msg::String>("order", 10);

        // MQTT 콜백 클래스 연결
        callback_ = std::make_shared<MqttCallback>(ros_publisher_, this->get_logger());
        mqtt_client_.set_callback(*callback_);

        // MQTT 연결 옵션
        mqtt::connect_options connOpts;
        try {
            mqtt_client_.connect(connOpts)->wait();
            mqtt_client_.subscribe("ros2/order", 1);  // MQTT 토픽 구독
            RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker and subscribed to 'ros2/order'");
        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "MQTT connection failed: %s", e.what());
        }
    }

private:
    class MqttCallback : public virtual mqtt::callback {
    public:
        MqttCallback(
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
            rclcpp::Logger logger)
        : publisher_(pub), logger_(logger) {}

        void message_arrived(mqtt::const_message_ptr msg) override {
            std_msgs::msg::String ros_msg;
            ros_msg.data = msg->to_string();
            publisher_->publish(ros_msg);
            RCLCPP_INFO(logger_, "Received MQTT: %s", ros_msg.data.c_str());
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Logger logger_;
    };

    mqtt::async_client mqtt_client_;
    std::shared_ptr<MqttCallback> callback_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MQTTListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
