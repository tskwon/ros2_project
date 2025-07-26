#include "md_controller/com.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
// Nav2 연동을 위한 추가 헤더
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>

#define VELOCITY_CONSTANT_VALUE   9.5492743  // m/sec를 RPM으로 변환하는 상수
#define constrain(amt,low,high)   ((amt)<=(low)?(low):((amt)>=(high)?(high):(amt)))
#define LEFT  0
#define RIGHT 1

class MdControllerNode : public rclcpp::Node {
public:
    MdControllerNode() : Node("md_controller_node") {
        using std::placeholders::_1;

        // 파라미터 선언 및 할당
        this->declare_parameter("MDUI", 184);
        this->declare_parameter("MDT", 183);
        this->declare_parameter("Port", "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG016MF2-if00-port0");
        this->declare_parameter("Baudrate", 115200);
        this->declare_parameter("ID", 1);
        this->declare_parameter("GearRatio", 1);
        this->declare_parameter("poles", 30);

        this->declare_parameter("wheel_radius", 0.103);     // 바퀴 반지름 (미터)
        this->declare_parameter("wheel_separation", 0.503);  // 바퀴 간 거리 (미터)
        this->declare_parameter("reverse_direction", true); // 방향 반전 여부
        this->declare_parameter("max_rpm", 200);            // 최대 RPM

        this->get_parameter("MDUI", Com.nIDMDUI);
        this->get_parameter("MDT", Com.nIDMDT);
        this->get_parameter("Port", Com.nPort);
        this->get_parameter("Baudrate", Com.nBaudrate);
        this->get_parameter("ID", Motor.ID);
        this->get_parameter("GearRatio", Motor.GearRatio);
        this->get_parameter("poles", Motor.poles);

        this->get_parameter("wheel_radius", Motor.wheel_radius);
        this->get_parameter("wheel_separation", Motor.wheel_separation);
        this->get_parameter("reverse_direction", Motor.reverse_direction);
        this->get_parameter("max_rpm", max_rpm_);

        auto qos = rclcpp::QoS(2).reliable();
        cmd_vel_sub_  = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", qos,
            std::bind(&MdControllerNode::cmdVelCallback, this, _1));

        // 기존 엔코더 퍼블리셔
        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/encoder_values", qos);

        // Nav2 연동을 위한 추가 퍼블리셔
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom/robot", qos);

        // TF 브로드캐스터
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        Motor.left_rpm = 0;
        Motor.right_rpm = 0;
        Motor.left_position = 0;
        Motor.right_position = 0;
        Motor.InitMotor = ON;
        Motor.InitError = 0;
        fgInitsetting = OFF;
        byCntInitStep = 1;
        byCntStartDelay = 0;
        byCntControl = 0;
        SendCmdRpm = ON; // 초기 RPM 0을 보내 모터를 정지시키기 위해 ON으로 설정
        
        // 이전 엔코더 값 초기화
        prev_left_position_ = 0;
        prev_right_position_ = 0;
        
        // 오도메트리 초기화
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        last_time_ = this->now();
        
        last_cmd_time_ = this->now();

        InitSerial(); // 통신 초기화

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MdControllerNode::timerCallback, this)
        );
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 마지막 명령 시간 업데이트
        last_cmd_time_ = this->now();
        
        // 선속도와 각속도
        double linear = msg->linear.x;
        double angular = msg->angular.z;
        
        // RPM으로 변환
        int16_t* goal_rpm = RobotSpeedToRPMSpeed(linear, angular);
        
        // 모터 RPM 업데이트
        if (Motor.left_rpm != goal_rpm[LEFT] || Motor.right_rpm != goal_rpm[RIGHT]) {
            Motor.left_rpm = -goal_rpm[LEFT]; // 참고: 원본 코드에는 왼쪽 바퀴에 음수 부호가 있었습니다.
            Motor.right_rpm = goal_rpm[RIGHT];
            SendCmdRpm = ON;
        }
    }

    int16_t* RobotSpeedToRPMSpeed(double linear, double angular) {
        static int16_t goal_rpm_speed[2];
        double wheel_velocity_cmd[2];
        
        // 왼쪽/오른쪽 바퀴 선형 속도 계산 (Motor 구조체의 멤버 사용)
        wheel_velocity_cmd[LEFT]  = linear - (angular * Motor.wheel_separation / 2);
        wheel_velocity_cmd[RIGHT] = linear + (angular * Motor.wheel_separation / 2);
        
        // 선형 속도를 RPM으로 변환 (Motor 구조체의 멤버 사용)
        wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / Motor.wheel_radius * Motor.GearRatio, -max_rpm_, max_rpm_);
        wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / Motor.wheel_radius * Motor.GearRatio, -max_rpm_, max_rpm_);
        
        // 방향 반전이 필요한 경우 (Motor 구조체의 멤버 사용)
        if (Motor.reverse_direction) {
            wheel_velocity_cmd[LEFT] = -wheel_velocity_cmd[LEFT];
            wheel_velocity_cmd[RIGHT] = -wheel_velocity_cmd[RIGHT];
        }
        
        // int16_t로 변환하여 반환
        goal_rpm_speed[LEFT] = static_cast<int16_t>(wheel_velocity_cmd[LEFT]);
        goal_rpm_speed[RIGHT] = static_cast<int16_t>(wheel_velocity_cmd[RIGHT]);
        
        return goal_rpm_speed;
    }

    void calculateOdometry() {
        // 현재 시간
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        
        if (dt <= 0.0) return;
        
        // 엔코더 차이 계산
        int32_t current_left = Motor.left_position;
        int32_t current_right = -Motor.right_position;  // 기존 코드에서 right는 음수로 처리
        
        int32_t delta_left = current_left - prev_left_position_;
        int32_t delta_right = current_right - prev_right_position_;
        
        // 엔코더 틱을 거리로 변환
        // 엔코더 PPR (Pulses Per Revolution) 계산이 필요 - 모터 사양에 따라 조정
        double encoder_ppr = Motor.poles * Motor.GearRatio * 4.0; // 4배수 인코딩 가정
        double left_distance = (delta_left / encoder_ppr) * 2.0 * M_PI * Motor.wheel_radius;
        double right_distance = (delta_right / encoder_ppr) * 2.0 * M_PI * Motor.wheel_radius;
        
        // 로봇 속도 계산
        double linear_velocity = (left_distance + right_distance) / (2.0 * dt);
        double angular_velocity = (right_distance - left_distance) / (Motor.wheel_separation * dt);
        
        // 위치 업데이트
        double delta_distance = (left_distance + right_distance) / 2.0;
        double delta_theta = (right_distance - left_distance) / Motor.wheel_separation;
        
        x_ += delta_distance * cos(theta_ + delta_theta / 2.0);
        y_ += delta_distance * sin(theta_ + delta_theta / 2.0);
        theta_ += delta_theta;
        
        // 각도 정규화
        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
        
        // 오도메트리 메시지 발행
        publishOdometry(current_time, linear_velocity, angular_velocity);
        
        // TF 발행
        publishTF(current_time);
        
        // 이전 값 업데이트
        prev_left_position_ = current_left;
        prev_right_position_ = current_right;
        last_time_ = current_time;
    }

    void publishOdometry(const rclcpp::Time& current_time, double linear_vel, double angular_vel) {
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        
        odom_msg->header.stamp = current_time;
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";
        
        // 위치
        odom_msg->pose.pose.position.x = x_;
        odom_msg->pose.pose.position.y = y_;
        odom_msg->pose.pose.position.z = 0.0;
        
        // 방향 (quaternion)
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta_);
        odom_msg->pose.pose.orientation.x = quat.x();
        odom_msg->pose.pose.orientation.y = quat.y();
        odom_msg->pose.pose.orientation.z = quat.z();
        odom_msg->pose.pose.orientation.w = quat.w();
        
        // 속도
        odom_msg->twist.twist.linear.x = linear_vel;
        odom_msg->twist.twist.linear.y = 0.0;
        odom_msg->twist.twist.angular.z = angular_vel;
        
        // 공분산 설정 (적절한 값으로 조정 필요)
        std::fill(odom_msg->pose.covariance.begin(), odom_msg->pose.covariance.end(), 0.0);
        std::fill(odom_msg->twist.covariance.begin(), odom_msg->twist.covariance.end(), 0.0);
        
        // 위치 공분산
        odom_msg->pose.covariance[0] = 0.1;   // x
        odom_msg->pose.covariance[7] = 0.1;   // y
        odom_msg->pose.covariance[35] = 0.1;  // theta
        
        // 속도 공분산
        odom_msg->twist.covariance[0] = 0.1;   // vx
        odom_msg->twist.covariance[35] = 0.1;  // vtheta
        
        odom_pub_->publish(std::move(odom_msg));
    }

    void publishTF(const rclcpp::Time& current_time) {
        geometry_msgs::msg::TransformStamped t;
        
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta_);
        t.transform.rotation.x = quat.x();
        t.transform.rotation.y = quat.y();
        t.transform.rotation.z = quat.z();
        t.transform.rotation.w = quat.w();
        
        tf_broadcaster_->sendTransform(t);
    }

    void timerCallback() {
        ReceiveDataFromController(Motor.InitMotor);
    
        static int position_request_counter = 0;
    
        // 모든 경우에 주기 카운터 증가 (10ms마다 실행되므로 30 → 30ms)
        position_request_counter++;
    
        if (fgInitsetting == ON) {
            // 오도메트리 계산 및 발행 (Nav2 연동을 위해 추가)
            calculateOdometry();
            
            // 기존 엔코더 값 발행 (그대로 유지)
            int32_t current_left = Motor.left_position;
            int32_t current_right = -Motor.right_position;
            
            auto encoder_msg = std::make_unique<std_msgs::msg::Int32MultiArray>();
            encoder_msg->data.push_back(current_left);
            encoder_msg->data.push_back(current_right);
            encoder_pub_->publish(std::move(encoder_msg));
            
            if (SendCmdRpm) {
                IByte left = Short2Byte(Motor.left_rpm);
                IByte right = Short2Byte(Motor.right_rpm);
                int nArray[4] = {right.byLow, right.byHigh, left.byLow, left.byHigh};
    
                if (Motor.left_rpm == 0 && Motor.right_rpm == 0) {
                    // PutMdData(PID_PNT_TQ_OFF, Com.nIDMDT, Motor.ID, nArray);
                    PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);
                } else {
                    PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);
                }
    
                SendCmdRpm = OFF;
            }
    
            // 엔코더 값 요청: SendCmdRpm 여부와 무관하게 주기적으로 실행
            if (position_request_counter >= 3) {
                int monitorArray[1] = {PID_PNT_MONITOR}; // 요청 데이터 종류
                PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, monitorArray);
                position_request_counter = 0;
            }
    
        } else {
            if (byCntStartDelay <= 30) {
                byCntStartDelay++;
            } else {
                InitMotorSequence();
            }
        }
    }
    
    void InitMotorSequence() {
        int nArray[4] = {0, 0, 0, 0};
        static int step_delay_counter = 0;  // 각 단계별 딜레이 카운터
        const int STEP_DELAY = 100;         // 1초 딜레이 (10ms * 100)
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "Init step: %d, delay counter: %d", byCntInitStep, step_delay_counter);
        
        switch (byCntInitStep) {
            case 1:
                if(step_delay_counter == 0) {
                    nArray[0] = PID_MAIN_DATA;
                    RCLCPP_INFO(this->get_logger(), "Step 1: Requesting MAIN_DATA for motor ID %d", Motor.ID);
                    PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);
                    step_delay_counter++;
                }
                else if(step_delay_counter < STEP_DELAY) {
                    step_delay_counter++;
                    
                    if (Motor.InitMotor == ON) {
                        Motor.InitError++;
                        if(Motor.InitError % 20 == 0) {  // 0.2초마다 로그 출력
                            RCLCPP_WARN(this->get_logger(), "Motor still not responding, error count: %d", Motor.InitError);
                        }
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Step 1 SUCCESS: Motor responded!");
                        byCntInitStep++;
                        step_delay_counter = 0;
                    }
                    
                    if (Motor.InitError > 200) {  // 2초 대기
                        RCLCPP_ERROR(this->get_logger(), "ID %d MOTOR INIT ERROR!!", Motor.ID);
                        rclcpp::shutdown();
                    }
                }
                else {
                    // 타임아웃
                    RCLCPP_WARN(this->get_logger(), "Step 1 timeout, proceeding to next step");
                    byCntInitStep++;
                    step_delay_counter = 0;
                }
                break;
    
            case 2:
                if(step_delay_counter == 0) {
                    RCLCPP_INFO(this->get_logger(), "Step 2: Waiting...");
                    step_delay_counter++;
                }
                else if(step_delay_counter >= STEP_DELAY) {
                    RCLCPP_INFO(this->get_logger(), "Step 2 COMPLETE: Moving to step 3");
                    byCntInitStep++;
                    step_delay_counter = 0;
                }
                else {
                    step_delay_counter++;
                }
                break;
    
            case 3:
                if(step_delay_counter == 0) {
                    RCLCPP_INFO(this->get_logger(), "Step 3: Resetting encoder positions to 0");
                    PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor.ID, nArray);
                    step_delay_counter++;
                }
                else if(step_delay_counter >= STEP_DELAY) {
                    RCLCPP_INFO(this->get_logger(), "Step 3 COMPLETE: Encoder reset done");
                    byCntInitStep++;
                    step_delay_counter = 0;
                }
                else {
                    step_delay_counter++;
                }
                break;
    
            case 4:
                if(step_delay_counter == 0) {
                    RCLCPP_INFO(this->get_logger(), "Step 4: Sending VEL_CMD");
                    PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);
                    step_delay_counter++;
                }
                else if(step_delay_counter >= STEP_DELAY) {
                    RCLCPP_INFO(this->get_logger(), "Step 4 COMPLETE: Moving to encoder reset");
                    byCntInitStep++;
                    step_delay_counter = 0;
                }
                else {
                    step_delay_counter++;
                }
                break;
    
            case 5:
                if(step_delay_counter == 0) {
                    RCLCPP_INFO(this->get_logger(), "Step 5: Final initialization");
                    
                    // 오도메트리 초기화
                    x_ = 0.0;
                    y_ = 0.0;
                    theta_ = 0.0;
                    last_time_ = this->now();
                    
                    step_delay_counter++;
                }
                else if(step_delay_counter >= STEP_DELAY) {
                    RCLCPP_INFO(this->get_logger(), "MOTOR INIT COMPLETE - All steps finished successfully!");             
                    fgInitsetting = ON;
                    step_delay_counter = 0;  // 리셋
                }
                else {
                    step_delay_counter++;
                }
                break;
        }
    }

    // 기존 멤버 변수들
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    
    // Nav2 연동을 위한 추가 멤버 변수
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    BYTE SendCmdRpm;
    BYTE fgInitsetting;
    BYTE byCntInitStep;
    BYTE byCntStartDelay;
    BYTE byCntControl;

    int max_rpm_;
    rclcpp::Time last_cmd_time_;
    
    // 이전 엔코더 값 저장을 위한 멤버 변수
    int32_t prev_left_position_;
    int32_t prev_right_position_;
    
    // 오도메트리를 위한 추가 멤버 변수
    double x_, y_, theta_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MdControllerNode>());
    rclcpp::shutdown();
    return 0;
}