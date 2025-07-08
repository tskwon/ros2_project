#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <algorithm>

class LineTracingNode : public rclcpp::Node {
public:
    LineTracingNode() : Node("line_tracing_node") {
        auto cmd_vel_qos = rclcpp::QoS(2).reliable();
        auto signal_qos = rclcpp::QoS(1).reliable();
        auto sensor_qos = rclcpp::QoS(1)
        .keep_last(1)        // 최신 메시지만 보관 정책
        .reliable()         // 신뢰성 있는 통신
        .durability_volatile();

        arrived_pub_ = create_publisher<std_msgs::msg::Bool>("/line_arrived", signal_qos);

        target_position_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/target_position", signal_qos, std::bind(&LineTracingNode::target_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos);

        sensor_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/ir_sensor_data", sensor_qos, std::bind(&LineTracingNode::sensor_callback, this, std::placeholders::_1));

        declare_parameter("wheel_radius", 0.103);
        declare_parameter("wheel_separation", 0.503);
        declare_parameter("reverse_direction", true);
        declare_parameter("max_rpm", 500);
        declare_parameter<double>("Kp", 0.9);
        declare_parameter<double>("Ki", 0.001);
        declare_parameter<double>("Kd", 0.4);
        declare_parameter<double>("max_linear_speed", 0.3);
        declare_parameter<double>("min_linear_speed", 0.0);
        declare_parameter<double>("normal_linear_speed", 0.3);
        declare_parameter<double>("max_angular_speed", 20.0);
        declare_parameter<double>("curve_threshold", 0.4);
        declare_parameter<double>("filter_coefficient", 0.88);
        declare_parameter<double>("acceleration", 0.008);
        declare_parameter<double>("deceleration", 0.006);
        declare_parameter<int>("timeout_count", 500);

        update_parameters();

        params_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&LineTracingNode::parameters_callback, this, std::placeholders::_1));

        rclcpp::on_shutdown([this]() {
            RCLCPP_WARN(get_logger(), "Shutdown detected. Stopping motors.");
            stop_motors();
        });

        current_linear_speed_ = 0.0;
        error_ = 0.0;
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_angular_velocity_ = 0.0;
        is_running_ = false;
        edge_detect_ = false;
        target_position_ = -1;
        rising_edge_count_ = 0;
        error_history_[0] = error_history_[1] = error_history_[2] = 0.0;
        error_history_index_ = 0;
        cnt_ = 0;
        last_derivative_ = 0.0;
        time_step_ = 0.01;

        max_theoretical_linear_speed_ = wheel_radius_ * max_rpm_ * 2 * M_PI / 60.0;
        max_theoretical_angular_speed_ = max_theoretical_linear_speed_ * 2 / wheel_separation_;

        sensor_weight_[0] = -0.4;
        sensor_weight_[1] = -0.08;
        sensor_weight_[2] = 0.0;
        sensor_weight_[3] = 0.08;
        sensor_weight_[4] = 0.4;
    }

private:
    void update_parameters() {
        get_parameter("wheel_radius", wheel_radius_);
        get_parameter("wheel_separation", wheel_separation_);
        get_parameter("reverse_direction", reverse_direction_);
        get_parameter("max_rpm", max_rpm_);
        get_parameter("Kp", Kp_);
        get_parameter("Ki", Ki_);
        get_parameter("Kd", Kd_);
        get_parameter("max_linear_speed", max_linear_speed_);
        get_parameter("min_linear_speed", min_linear_speed_);
        get_parameter("normal_linear_speed", normal_linear_speed_);
        get_parameter("max_angular_speed", max_angular_speed_);
        get_parameter("curve_threshold", curve_threshold_);
        get_parameter("filter_coefficient", filter_coefficient_);
        get_parameter("acceleration", acceleration_);
        get_parameter("deceleration", deceleration_);
        get_parameter("timeout_count", timeout_count_);
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters) {
        update_parameters();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void target_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        target_position_ = msg->data;
        rising_edge_count_ = 0;
        is_running_ = true;
        edge_detect_ = false;
    }

    void stop_motors() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg);

        is_running_ = false;
        current_linear_speed_ = 0.0;
        integral_ = 0.0;

        std_msgs::msg::Bool arrived_msg;
        arrived_msg.data = true;
        arrived_pub_->publish(arrived_msg);
    }

    double update_moving_average(double new_value) {
        error_history_[error_history_index_] = new_value;
        error_history_index_ = (error_history_index_ + 1) % 3;
        return (error_history_[0] + error_history_[1] + error_history_[2]) / 3.0;
    }

    void sensor_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!is_running_ || msg->data.size() != 5) return;

        double raw_position = 0.0;
        int active_sensors = 0;
        for (int i = 0; i < 5; ++i) {
            if (msg->data[i] == 1) {
                raw_position += sensor_weight_[i];
                active_sensors++;
            }
        }

        if (active_sensors == 0) {
            cnt_++;
            if (cnt_ >= timeout_count_) {
                stop_motors();
                return;
            }
        } 
        // else {
        //     raw_position /= active_sensors;
        //     double filtered_position = update_moving_average(raw_position);
        //     error_ = filtered_position;
        //     cnt_ = 0;
        // }
        else {
            raw_position /= active_sensors;
            static double filtered_error = 0.0;
            const double alpha = 0.4;
            filtered_error = alpha * raw_position + (1.0 - alpha) * filtered_error;
            error_ = filtered_error;
            cnt_ = 0;
        }

        if (active_sensors == 5 && prev_active_sensors_ < 5 && !edge_detect_) {
            edge_detect_ = true;
            rising_edge_count_++;
            if (target_position_ != -1 && rising_edge_count_ == target_position_) {
                stop_motors();
                return;
            }
        }

        if (active_sensors == 0) edge_detect_ = false;
        prev_active_sensors_ = active_sensors;

        integral_ += error_ * time_step_;
        integral_ = std::clamp(integral_, -0.8, 0.8);

        double derivative = (error_ - prev_error_) / time_step_;
        derivative = 0.8 * derivative + 0.2 * last_derivative_;
        last_derivative_ = derivative;

        double angular_correction = Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative;
        if (reverse_direction_) angular_correction = -angular_correction;
        double angular_velocity = std::clamp(angular_correction, -max_angular_speed_, max_angular_speed_);
        angular_velocity = filter_coefficient_ * prev_angular_velocity_ + (1.0 - filter_coefficient_) * angular_velocity;
        prev_angular_velocity_ = angular_velocity;

        double error_magnitude = std::abs(error_);
        double target_linear_speed = (error_magnitude > curve_threshold_) ? 0.15 : 0.23;

        if (error_magnitude > curve_threshold_) {
            double angular_magnitude = std::abs(angular_velocity);
            if (angular_magnitude > 1.0) {
                double reduction_factor = std::max(0.75, 1.0 - (angular_magnitude - 1.0) / 3.0);
                target_linear_speed *= reduction_factor;
            }
        }

        target_linear_speed = std::clamp(target_linear_speed, min_linear_speed_, max_linear_speed_);

        if (current_linear_speed_ < target_linear_speed) {
            current_linear_speed_ += acceleration_;
            current_linear_speed_ = std::min(current_linear_speed_, target_linear_speed);
        } else {
            current_linear_speed_ -= deceleration_;
            current_linear_speed_ = std::max(current_linear_speed_, target_linear_speed);
        }

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = current_linear_speed_;
        cmd_vel_msg.angular.z = angular_velocity;
        cmd_vel_pub_->publish(cmd_vel_msg);

        prev_error_ = error_;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_position_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    double wheel_radius_, wheel_separation_;
    bool reverse_direction_;
    int max_rpm_;
    double max_theoretical_linear_speed_, max_theoretical_angular_speed_;

    double sensor_weight_[5];
    double error_, integral_, prev_error_;
    double last_derivative_, error_history_[3];
    int error_history_index_;
    double Kp_, Ki_, Kd_;
    double max_linear_speed_, min_linear_speed_, normal_linear_speed_;
    double max_angular_speed_, current_linear_speed_;
    double curve_threshold_, filter_coefficient_;
    double acceleration_, deceleration_;
    double prev_angular_velocity_, time_step_;
    int timeout_count_;
    bool is_running_, edge_detect_;
    int prev_active_sensors_, target_position_, rising_edge_count_, cnt_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineTracingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}