#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp> // IMU 메시지 헤더 추가
#include <cmath>
#include <algorithm> // std::clamp 사용

class LineTracingNode : public rclcpp::Node {
public:
    LineTracingNode() : Node("line_tracing_node") {
        auto cmd_vel_qos = rclcpp::QoS(2).reliable();
        auto signal_qos = rclcpp::QoS(1).reliable();
        auto sensor_qos = rclcpp::QoS(10).reliable(); // IR 및 IMU 센서에 동일 QoS 적용

        arrived_pub_ = create_publisher<std_msgs::msg::Bool>("/line_arrived", signal_qos);

        target_position_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/target_position", signal_qos, std::bind(&LineTracingNode::target_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos);

        sensor_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/ir_sensor_data", sensor_qos, std::bind(&LineTracingNode::sensor_callback, this, std::placeholders::_1));

        // IMU 데이터 구독 추가
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sensor_qos, std::bind(&LineTracingNode::imu_callback, this, std::placeholders::_1)); // imu/data 토픽 구독

        // PID 및 제어 파라미터 선언
        declare_parameter<double>("Kp", 0.7);
        declare_parameter<double>("Ki", 0.0);
        // 기존 Kd 대신 Yaw rate 게인 Kg 사용
        declare_parameter<double>("Kg", 0); // IMU Yaw Rate 제어 게인 추가

        // 속도 조절 파라미터 추가
        declare_parameter<double>("max_linear_speed", 0.4);
        declare_parameter<double>("min_linear_speed", 0.1);
        declare_parameter<double>("max_angular_speed", 0.95);
        declare_parameter<double>("curve_threshold", 1.0);

        // 현재 파라미터 값 가져오기
        update_parameters();

        // 파라미터 변경 콜백 설정
        params_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&LineTracingNode::parameters_callback, this, std::placeholders::_1));

        rclcpp::on_shutdown([this]() {
            RCLCPP_WARN(get_logger(), "Shutdown detected. Stopping motors.");
            stop_motors();
        });

        current_linear_speed_ = 0.0;
        error_ = integral_ = 0.0; // prev_error_는 더 이상 사용하지 않음
        last_yaw_rate_ = 0.0; // IMU Yaw Rate 초기화
        is_running_ = false;
        speed_ramp_rate_ = 0.015;
        edge_detect_ = false;
        target_position_ = -1;
        rising_edge_count_ = 0;

        RCLCPP_INFO(get_logger(), "Line Tracing Node started (velocity control mode with IMU Yaw Rate)");
        RCLCPP_INFO(get_logger(), "Control gains (Kp, Ki, Kg) and speed parameters can be changed using ROS2 param set");
        RCLCPP_INFO(get_logger(), "Example: ros2 param set /line_tracing_node Kp 5.0");
    }

private:
    // 파라미터 업데이트 함수
    void update_parameters() {
        get_parameter("Kp", Kp_);
        get_parameter("Ki", Ki_);
        get_parameter("Kg", Kg_); // Kd 대신 Kg 파라미터 사용
        get_parameter("max_linear_speed", max_linear_speed_);
        get_parameter("min_linear_speed", min_linear_speed_);
        get_parameter("max_angular_speed", max_angular_speed_);
        get_parameter("curve_threshold", curve_threshold_);
    }

    // 파라미터 변경 콜백
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "Kp") {
                Kp_ = param.as_double();
                RCLCPP_INFO(get_logger(), "Kp changed to %.2f", Kp_);
            } else if (param.get_name() == "Ki") {
                Ki_ = param.as_double();
                RCLCPP_INFO(get_logger(), "Ki changed to %.2f", Ki_);
            } else if (param.get_name() == "Kg") { // Kd 대신 Kg 처리
                Kg_ = param.as_double();
                RCLCPP_INFO(get_logger(), "Kg changed to %.2f", Kg_);
            } else if (param.get_name() == "max_linear_speed") {
                max_linear_speed_ = param.as_double();
                RCLCPP_INFO(get_logger(), "max_linear_speed changed to %.2f", max_linear_speed_);
            } else if (param.get_name() == "min_linear_speed") {
                min_linear_speed_ = param.as_double();
                RCLCPP_INFO(get_logger(), "min_linear_speed changed to %.2f", min_linear_speed_);
            } else if (param.get_name() == "max_angular_speed") {
                max_angular_speed_ = param.as_double();
                RCLCPP_INFO(get_logger(), "max_angular_speed changed to %.2f", max_angular_speed_);
            } else if (param.get_name() == "curve_threshold") {
                curve_threshold_ = param.as_double();
                RCLCPP_INFO(get_logger(), "curve_threshold changed to %.2f", curve_threshold_);
            }
        }
        return result;
    }

    // IMU 데이터 콜백 함수 추가
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // myAHRS+ 드라이버의 imu/data 토픽은 ROS REP 103 기준으로 angular_velocity.z에 Yaw rate를 담고 있습니다.
        last_yaw_rate_ = msg->angular_velocity.z;
    }

    // 기존 콜백 함수들...
    void target_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        target_position_ = msg->data;
        rising_edge_count_ = 0;
        is_running_ = true;
        edge_detect_ = false;
        RCLCPP_INFO(get_logger(), "Received new target position: %d. Starting run.", target_position_);
    }

    void stop_motors() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg);

        is_running_ = false;
        current_linear_speed_ = 0.0;
        integral_ = 0.0; // 정지 시 적분항 초기화

        std_msgs::msg::Bool arrived_msg;
        arrived_msg.data = true;
        arrived_pub_->publish(arrived_msg);

        RCLCPP_INFO(get_logger(), "Motors stopped. Target position reached or lost line.");
    }

    void sensor_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!is_running_) {
             // 주행 중이 아닐 때는 센서 데이터 무시 (정지 상태 유지)
             // 필요시 정지 상태에서의 특정 동작 추가 가능
             return;
        }

        if (msg->data.size() != 5) {
             RCLCPP_WARN(get_logger(), "Invalid sensor data size: %zu", msg->data.size());
             return;
        }

        double position = 0.0;
        int active_sensors = 0;
        int zero_sensors = 0;
        for (int i = 0; i < 5; ++i) {
            if (msg->data[i] == 1) {
                position += sensor_weight_[i];
                active_sensors++;
            }
            else if (msg->data[i] == 0) {
                zero_sensors++;
            }
        };

        if (zero_sensors == 5) {
        cnt_++; // 모든 센서가 0인 상태 카운트 증가
        RCLCPP_INFO(get_logger(), "All sensors are zero. Count: %d", cnt_);
        
        // 연속으로 10번 이상 모든 센서가 0이면 정지
        if (cnt_ >= 200) {
            RCLCPP_WARN(get_logger(), "All sensors have been zero for %d consecutive readings. Line lost. Stopping motors.", cnt_);
            stop_motors();
            return;
        }
        } else {
            // 센서가 하나라도 1이면 카운터 초기화
            cnt_ = 0;
        }

        // if (zero_sensors == 5) {
        //     RCLCPP_WARN(get_logger(), "All sensors are zero. Line lost. Stopping motors.");
        //     stop_motors();
        //     return;
        // }

        // 정지 지점 감지 로직 (전체 센서 ON 상태의 Rising Edge)
        if (active_sensors == 5 && prev_active_sensors_ < 5 && !edge_detect_ && is_running_) {
            edge_detect_ = true;
            rising_edge_count_++;
            RCLCPP_INFO(get_logger(), "Rising edge #%d detected", rising_edge_count_);

            if (target_position_ != -1 && rising_edge_count_ == target_position_) { // target_position_이 -1이 아닐 때만 정지
                RCLCPP_INFO(get_logger(), "Target position %d reached. Stopping motors.", target_position_);
                stop_motors();
                //prev_active_sensors_ = active_sensors; // 정지했으므로 이후 콜백은 무시될 것임
                return;
            }
        }


        // edge_detect_ 플래그 초기화 (모든 센서 OFF 상태 진입 시)
        if (active_sensors == 0) {
            edge_detect_ = false;
        }

        prev_active_sensors_ = active_sensors;

        // 현재 라인 위치 오류 업데이트
        error_ = position;

        // PID 계산 (IMU Yaw Rate 활용)
        integral_ += error_;
        integral_ = std::clamp(integral_, -15.0, 15.0); // 적분항 제한

        // 기존 D항 (오류 미분) 대신 IMU Yaw Rate 사용
        // angular_correction = Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative; // <-- 기존 코드
        double angular_correction = Kp_ * error_ + Ki_ * integral_ - Kg_ * last_yaw_rate_; // <-- 수정된 코드 (Yaw Rate 항 추가 및 부호 조정)

        // 각속도 계산 및 제한
        double angular_velocity = std::clamp(angular_correction, -max_angular_speed_, max_angular_speed_);

        // 선형 속도 계산 (오류 크기에 따라 속도 조절)
        double error_magnitude = std::abs(error_);
        
        double target_linear_speed;
        if (error_magnitude > curve_threshold_) {
            // 곡선 구간 (error가 클 때) - 속도 감소
            // curve_threshold_와 error_magnitude 비율에 따라 최소~최대 속도 사이 값을 계산
            //  double curve_factor = std::max(0.0, std::min(1.0, (3.0 - error_magnitude) / (3.0 - curve_threshold_))); // error가 5.0일 때 0, curve_threshold_일 때 1
            // target_linear_speed = min_linear_speed_ +
            //                      (max_linear_speed_ - min_linear_speed_) * curve_factor;
            target_linear_speed = 0.2;
        } else {
            // 직선 구간 (error가 작을 때) - 속도 증가
            // curve_threshold_일 때 0
            target_linear_speed = 0.2;
        }

         // 계산된 target_linear_speed가 min_linear_speed와 max_linear_speed 범위를 벗어나지 않도록 강제
         target_linear_speed = std::clamp(target_linear_speed, min_linear_speed_, max_linear_speed_);


        // 점진적 속도 조절 (Ramping)
        if (current_linear_speed_ < target_linear_speed) {
            current_linear_speed_ += speed_ramp_rate_;
            if (current_linear_speed_ > target_linear_speed)
                current_linear_speed_ = target_linear_speed;
        } else if (current_linear_speed_ > target_linear_speed) {
            current_linear_speed_ -= speed_ramp_rate_;
            if (current_linear_speed_ < target_linear_speed)
                current_linear_speed_ = target_linear_speed;
        }

        // 속도가 너무 낮아지지 않도록 최소값 보장 (ramp down 시)
        if (current_linear_speed_ < min_linear_speed_ && target_linear_speed < min_linear_speed_) // ramp down 중 목표가 최소값보다 작을 때만 적용
             current_linear_speed_ = min_linear_speed_;


        // Twist 메시지 발행
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = current_linear_speed_;
        cmd_vel_msg.angular.z = angular_velocity;
        cmd_vel_pub_->publish(cmd_vel_msg);

        RCLCPP_INFO(get_logger(), "Linear: %.2f m/s, Angular: %.2f rad/s, Error: %.2f, YawRate: %.2f",
            current_linear_speed_, angular_velocity, error_, last_yaw_rate_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_position_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // IMU 구독자 추가

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    double sensor_weight_[5] = {0.8, 0.2, 0, -0.2, -0.8}; // 센서 위치 가중치 예시
    double error_, integral_; // prev_error_ 제거
    double Kp_, Ki_, Kg_, speed_ramp_rate_; // Kd_ 대신 Kg_
    double max_linear_speed_, min_linear_speed_, max_angular_speed_;
    double current_linear_speed_;
    double curve_threshold_;
    double last_yaw_rate_; // 최신 IMU Yaw Rate 저장

    bool is_running_, edge_detect_;
    int prev_active_sensors_ = 0;
    int target_position_;
    int rising_edge_count_;
    int cnt_ = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineTracingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}