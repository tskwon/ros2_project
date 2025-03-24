#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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

        this->declare_parameter<double>("Kp", 1.2);
        this->declare_parameter<double>("Ki", 0.0);
        this->declare_parameter<double>("Kd", 2.65);

        this->get_parameter("Kp", Kp_);
        this->get_parameter("Ki", Ki_);
        this->get_parameter("Kd", Kd_);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&LineTracingNode::param_callback, this, std::placeholders::_1)
        );

        base_speed_ = 20.0;
        current_speed_ = 0.0;
        left_rpm_ = 0.0;
        right_rpm_ = 0.0;
        error_ = 0.0;
        prev_error_ = 0.0;
        integral_ = 0.0;
        is_stopped_ = true;
        is_run_ = true;
        speed_ramp_rate_ = 2.0;

        RCLCPP_INFO(this->get_logger(), "Line Tracing Node started (PID dynamic: ON)");
        RCLCPP_INFO(this->get_logger(), "Initial PID: Kp=%.2f, Ki=%.2f, Kd=%.2f", Kp_, Ki_, Kd_);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "PID parameters updated";

        for (const auto& param : params) {
            if (param.get_name() == "Kp") {
                Kp_ = param.as_double();
            } else if (param.get_name() == "Ki") {
                Ki_ = param.as_double();
            } else if (param.get_name() == "Kd") {
                Kd_ = param.as_double();
            }
        }
        RCLCPP_INFO(this->get_logger(), "Updated PID params: Kp=%.2f, Ki=%.2f, Kd=%.2f", Kp_, Ki_, Kd_);
        return result;
    }

    void stop_motors() {
        left_rpm_ = 0.0;
        right_rpm_ = 0.0;
        current_speed_ = 0.0;
        is_stopped_ = true;
        auto rpm_msg = std_msgs::msg::Int32MultiArray();
        rpm_msg.data = {0, 0};
        rpm_pub_->publish(rpm_msg);
        RCLCPP_INFO(this->get_logger(), "Motors stopped before shutdown");
    }

private:
    void sensor_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        auto sensor_values = msg->data;

        if (sensor_values.size() != 5) {
            RCLCPP_WARN(this->get_logger(), "Invalid sensor data size: %zu", sensor_values.size());
            return;
        }

        double position = 0.0;

        
        int active_sensors = 0;
        for (int i = 0; i < 5; ++i) {
            if (sensor_values[i] == 1) {
                position += sensor_weight[i];
                active_sensors++;
            }
        }

        if (active_sensors > 0) {
            is_run_ = true;
            error_ = position;
        } else {
            is_run_ = false;
            is_stopped_ = true;
            stop_motors();
            return;
        }

        if (is_stopped_) {
            current_speed_ += speed_ramp_rate_;
            if (current_speed_ >= base_speed_) {
                current_speed_ = base_speed_;
                is_stopped_ = false;
            }
        }

        integral_ += error_;
        double derivative = error_ - prev_error_;
        double correction = Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative;

        left_rpm_ = current_speed_ + correction;
        right_rpm_ = current_speed_ - correction;

        left_rpm_ = std::clamp(left_rpm_, 0.0, 50.0);
        right_rpm_ = std::clamp(right_rpm_, 0.0, 50.0);

        prev_error_ = error_;

        if (!is_run_) return;

        auto rpm_msg = std_msgs::msg::Int32MultiArray();
        rpm_msg.data = {static_cast<int32_t>(left_rpm_), -static_cast<int32_t>(right_rpm_)};
        rpm_pub_->publish(rpm_msg);

        std::string sensor_str = "[";
        for (size_t i = 0; i < sensor_values.size(); ++i) {
            sensor_str += std::to_string(sensor_values[i]);
            if (i < sensor_values.size() - 1) sensor_str += ", ";
        }
        sensor_str += "]";
        RCLCPP_INFO(this->get_logger(), "IR Sensor: %s, Error: %.2f, Speed: %.1f, Left RPM: %.1f, Right RPM: %.1f",
                    sensor_str.c_str(), error_, current_speed_, left_rpm_, right_rpm_);
    }

    void stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        is_run_ = !msg->data;
        is_stopped_ = true;
        stop_motors();
        RCLCPP_INFO(this->get_logger(), "Received stop signal, motors stopped");
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    double sensor_weight[5] = {-2.5,-1.5,0,1.5,2.5};
    double base_speed_;
    double current_speed_;
    double left_rpm_;
    double right_rpm_;
    double error_;
    double prev_error_;
    double integral_;
    double Kp_, Ki_, Kd_;
    bool is_stopped_;
    bool is_run_;
    double speed_ramp_rate_;
};

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
    signal(SIGINT, signal_handler);

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