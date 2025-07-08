#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mqtt/async_client.h>

class MqttCallback : public virtual mqtt::callback {
public:
    MqttCallback(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher, rclcpp::Logger logger)
        : ros_pub_(publisher), logger_(logger) {}

    void message_arrived(mqtt::const_message_ptr msg) override {
        std_msgs::msg::String ros_msg;
        ros_msg.data = msg->to_string();
        ros_pub_->publish(ros_msg);
        RCLCPP_INFO(logger_, "MQTT → ROS: %s", ros_msg.data.c_str());
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_pub_;
    rclcpp::Logger logger_;
};

class MqttBridgeNode : public rclcpp::Node {
public:
    MqttBridgeNode() : Node("mqtt_bridge_node"),
        mqtt_client_("tcp://localhost:1883", "ros2_mqtt_bridge")
    {
        ros_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_task", 10);

        mqtt::connect_options connOpts;
        mqtt_client_.connect(connOpts)->wait();

        // 콜백 설정
        callback_ = std::make_shared<MqttCallback>(ros_pub_, this->get_logger());
        mqtt_client_.set_callback(*callback_);

        mqtt_client_.subscribe("robot/task", 1);
        RCLCPP_INFO(this->get_logger(), "MQTT Bridge Node started. Listening on 'robot/task'");
    }

private:
    mqtt::async_client mqtt_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_pub_;
    std::shared_ptr<MqttCallback> callback_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MqttBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




