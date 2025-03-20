#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <csignal>

class LineTracingNode : public rclcpp::Node {
public:
    LineTracingNode() : Node("line_tracing_node") {
        auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        rpm_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/cmd_rpm", qos);
        sensor_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/ir_sensor_data", qos,
            std::bind(&LineTracingNode::sensor_callback, this, std::placeholders::_1)
        );
        stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/stop_signal", qos,
            std::bind(&LineTracingNode::stop_callback, this, std::placeholders::_1)
        );
        left_rpm_ = -50;
        right_rpm_ = 50;
        RCLCPP_INFO(this->get_logger(), "Line Tracing Node started");
    }

    void stop_motors() {
        left_rpm_ = 0;
        right_rpm_ = 0;
        auto rpm_msg = std_msgs::msg::Int32MultiArray();
        rpm_msg.data = {0, 0};
        rpm_pub_->publish(rpm_msg);
        RCLCPP_INFO(this->get_logger(), "Motors stopped before shutdown");
    }

private:
    void sensor_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        auto sensor_values = msg->data;
        auto rpm_msg = std_msgs::msg::Int32MultiArray();
        rpm_msg.data = {left_rpm_, right_rpm_};
        rpm_pub_->publish(rpm_msg);

        std::string sensor_str = "[";
        for (size_t i = 0; i < sensor_values.size(); ++i) {
            sensor_str += std::to_string(sensor_values[i]);
            if (i < sensor_values.size() - 1) sensor_str += ", ";
        }
        sensor_str += "]";
        RCLCPP_INFO(this->get_logger(), "IR Sensor: %s, Left RPM: %d, Right RPM: %d",
                    sensor_str.c_str(), left_rpm_, right_rpm_);
    }

    void stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            stop_motors();
            RCLCPP_INFO(this->get_logger(), "Received stop signal, motors stopped");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    int32_t left_rpm_;
    int32_t right_rpm_;
};

// 전역 노드 포인터
static std::shared_ptr<LineTracingNode> global_node;

void signal_handler(int signum) {
    if (global_node && rclcpp::ok()) {
        global_node->stop_motors();
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    global_node = std::make_shared<LineTracingNode>();
    signal(SIGINT, signal_handler);  // Ctrl+C 처리 등록

    try {
        rclcpp::spin(global_node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(global_node->get_logger(), "Exception occurred: %s", e.what());
        if (rclcpp::ok()) {
            global_node->stop_motors();
        }
    }

    rclcpp::shutdown();
    global_node.reset();
    return 0;
}