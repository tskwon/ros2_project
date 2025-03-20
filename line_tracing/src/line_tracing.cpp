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

        // 초기 설정
        base_speed_ = 40.0;  // 목표 속도
        current_speed_ = 0.0;  // 현재 속도 (멈춘 상태에서 시작)
        left_rpm_ = 0.0;
        right_rpm_ = 0.0;
        error_ = 0.0;
        prev_error_ = 0.0;
        integral_ = 0.0;
        is_stopped_ = true;  // 초기 상태: 멈춤

        // PID 상수
        Kp_ = 5.0;
        Ki_ = 0.2;
        Kd_ = 5.0;
        use_pid_ = false;  // PID 끔
        is_run_ = true;

        // 속도 증가율 (부드러운 시작)
        speed_ramp_rate_ = 5.0;  // 초당 증가 속도 (조정 가능)

        RCLCPP_INFO(this->get_logger(), "Line Tracing Node started (PID: %s)", use_pid_ ? "ON" : "OFF");
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
        if(!is_run_) return;

        auto sensor_values = msg->data;

        if (sensor_values.size() != 5) {
            RCLCPP_WARN(this->get_logger(), "Invalid sensor data size: %zu", sensor_values.size());
            return;
        }

        // 센서 값으로 에러 계산
        double position = 0.0;
        int active_sensors = 0;
        for (int i = 0; i < 5; ++i) {
            if (sensor_values[i] == 1) {
                position += (i - 2);  // -2, -1, 0, 1, 2
                active_sensors++;
            }
        }
        if (active_sensors > 0) {
            error_ = position / active_sensors;
        } else {
            error_ = prev_error_;
        }

        // 속도 점진적 증가
        if (is_stopped_) {
            current_speed_ += speed_ramp_rate_;  // 속도 증가
            if (current_speed_ >= base_speed_) {
                current_speed_ = base_speed_;
                is_stopped_ = false;  // 주행 상태로 전환
            }
        }

        integral_ += error_;
        double derivative = error_ - prev_error_;
        double correction = Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative;

        left_rpm_ = current_speed_ + correction;
        right_rpm_ = current_speed_ - correction;

        left_rpm_ = std::max(-100.0, std::min(100.0, left_rpm_));
        right_rpm_ = std::max(-100.0, std::min(100.0, right_rpm_));

        prev_error_ = error_;
        

        // RPM 발행
        auto rpm_msg = std_msgs::msg::Int32MultiArray();
        rpm_msg.data = {-static_cast<int32_t>(left_rpm_), static_cast<int32_t>(right_rpm_)};
        rpm_pub_->publish(rpm_msg);

        // 디버깅 출력: %d → %f로 변경
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

    double base_speed_;       // 목표 속도
    double current_speed_;    // 현재 속도
    double left_rpm_;
    double right_rpm_;
    double error_;
    double prev_error_;
    double integral_;
    double Kp_, Ki_, Kd_;
    bool use_pid_;
    bool is_stopped_;         // 멈춤 상태 플래그
    bool is_run_;
    double speed_ramp_rate_;  // 속도 증가율
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