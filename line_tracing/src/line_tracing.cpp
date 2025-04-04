#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

class LineTracingNode : public rclcpp::Node {
public:
    LineTracingNode() : Node("line_tracing_node") {
        auto qos = rclcpp::QoS(10).reliable();

        rpm_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/cmd_rpm", qos);
        arrived_pub_ = create_publisher<std_msgs::msg::Bool>("/line_arrived", qos);

        sensor_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/ir_sensor_data", qos, std::bind(&LineTracingNode::sensor_callback, this, std::placeholders::_1));

        target_position_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/target_position", qos, std::bind(&LineTracingNode::target_callback, this, std::placeholders::_1));

        declare_parameter<double>("Kp", 1.2);
        declare_parameter<double>("Ki", 0.0);
        declare_parameter<double>("Kd", 2.65);
        get_parameter("Kp", Kp_);
        get_parameter("Ki", Ki_);
        get_parameter("Kd", Kd_);

        base_speed_ = 30.0;
        current_speed_ = 0.0;
        error_ = prev_error_ = integral_ = 0.0;
        is_running_ = false;
        speed_ramp_rate_ = 2.0;
        edge_detect_ = false;
        target_position_ = -1;
        rising_edge_count_ = 0;

        RCLCPP_INFO(get_logger(), "Line Tracing Node started");
    }

private:
    void target_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        target_position_ = msg->data;
        rising_edge_count_ = 0;
        is_running_ = true;
        edge_detect_ = false;
        RCLCPP_INFO(get_logger(), "Received new target position: %d", target_position_);
    }

    void stop_motors() {
        std_msgs::msg::Int32MultiArray rpm_msg;
        rpm_msg.data = {0, 0};
        rpm_pub_->publish(rpm_msg);

        is_running_ = false;
        current_speed_ = 0.0;

        std_msgs::msg::Bool arrived_msg;
        arrived_msg.data = true;
        arrived_pub_->publish(arrived_msg);

        RCLCPP_INFO(get_logger(), "Motors stopped. Line arrived published.");
    }

    void sensor_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 5) return;

        double position = 0.0;
        int active_sensors = 0;
        for (int i = 0; i < 5; ++i) {
            if (msg->data[i] == 1) {
                position += sensor_weight_[i];
                active_sensors++;
            }
        }

        if (active_sensors == 5 && prev_active_sensors_ < 5 && !edge_detect_ && is_running_) {
            edge_detect_ = true;
            rising_edge_count_++;
            RCLCPP_INFO(get_logger(), "Rising edge #%d detected", rising_edge_count_);

            if (rising_edge_count_ == target_position_) {
                stop_motors();
                prev_active_sensors_ = active_sensors;
                return;
            }
        }

        if (active_sensors == 0) {
            edge_detect_ = false;
        }

        prev_active_sensors_ = active_sensors;

        if (!is_running_) return;

        error_ = position;
        integral_ += error_;
        double derivative = error_ - prev_error_;
        double correction = Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative;

        if (current_speed_ < base_speed_) {
            current_speed_ += speed_ramp_rate_;
            if (current_speed_ > base_speed_) current_speed_ = base_speed_;
        }

        double left_rpm = std::clamp(current_speed_ + correction, -70.0, 70.0);
        double right_rpm = std::clamp(current_speed_ - correction, -70.0, 70.0);
        prev_error_ = error_;

        std_msgs::msg::Int32MultiArray rpm_msg;
        rpm_msg.data = {static_cast<int32_t>(left_rpm), -static_cast<int32_t>(right_rpm)};
        rpm_pub_->publish(rpm_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_position_sub_;

    double sensor_weight_[5] = {-2.5, -1.5, 0, 1.5, 2.5};
    double base_speed_, current_speed_, error_, prev_error_, integral_;
    double Kp_, Ki_, Kd_, speed_ramp_rate_;
    bool is_running_, edge_detect_;
    int prev_active_sensors_ = 0;
    int target_position_;
    int rising_edge_count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineTracingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
