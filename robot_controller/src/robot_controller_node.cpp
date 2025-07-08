#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <thread>

using json = nlohmann::json;

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller_node") {
        // ROS publishers
        target_position_pub_ = this->create_publisher<std_msgs::msg::Int32>("/target_position", 10);
        lift_pub_ = this->create_publisher<std_msgs::msg::Int32>("/number_data", 10);

        // Subscribe to robot task command (from mqtt_client bridge)
        task_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_task", 10,
            std::bind(&RobotController::task_callback, this, std::placeholders::_1));

        // Subscribe to line arrival confirmation
        line_arrived_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/line_arrived", 10,
            std::bind(&RobotController::line_arrived_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Robot Controller Node started (ROS ↔ MQTT bridge mode)");
    }

private:
    void task_callback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            auto payload = json::parse(msg->data);
            int position = payload["position"];
            int floor = payload["floor"];

            // 1. 목적지 전송
            std_msgs::msg::Int32 pos_msg;
            pos_msg.data = position;
            target_position_pub_->publish(pos_msg);
            RCLCPP_INFO(this->get_logger(), "Published target_position: %d", position);

            // 2. 도착 대기
            line_arrived_ = false;
            while (!line_arrived_ && rclcpp::ok()) {
                rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // 3. 리프트 동작
            std_msgs::msg::Int32 lift_msg;
            lift_msg.data = floor;
            lift_pub_->publish(lift_msg);
            RCLCPP_INFO(this->get_logger(), "Arrived. Lifting to floor: %d", floor);

            std::this_thread::sleep_for(std::chrono::seconds(3));

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse task JSON: %s", e.what());
        }
    }

    void line_arrived_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            line_arrived_ = true;
            RCLCPP_INFO(this->get_logger(), "Line arrived signal received.");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lift_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr line_arrived_sub_;

    bool line_arrived_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
