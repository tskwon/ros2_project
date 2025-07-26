#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <memory>
#include <dynamixel_sdk/dynamixel_sdk.h>


// === 링크 길이 (단위: cm) ===
const double L1 = 20.0;
const double L2 = 16.0;

// === 다이나믹셀 주소값 ===
const int ADDR_OPERATING_MODE = 11;
const int ADDR_TORQUE_ENABLE = 64;
const int ADDR_GOAL_POSITION = 116;
const int ADDR_ACCELERATION = 108;
const int ADDR_VELOCITY = 112;
const int ADDR_PROFILE_VELOCITY = 112;
const int ADDR_PRESENT_POSITION = 132;
const int ADDR_PRESENT_CURRENT = 126;

// === 파라미터 ===
const int TORQUE_ENABLE = 1;
const int INIT_POSITION = 2048;
const int SLOW_SPEED = 120;
const int OPEN_POSITION = 3500;
const int CLOSE_POSITION = 2500;
const int CURRENT_THRESHOLD = 500;
const int DELAY_MS = 50;

class DynamixelRobotArmNode : public rclcpp::Node {
private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mission_complete_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_rotate_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr logistics_command_sub_;
    
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr gripper_monitor_timer_;
    rclcpp::TimerBase::SharedPtr position_complete_timer_;
    rclcpp::TimerBase::SharedPtr logistics_complete_timer_;
    
    // 현재 상태
    struct JointAngles {
        double theta1 = 0.0;
        double theta2 = 0.0;
        double theta3 = 0.0;
    };
    
    JointAngles current_angles_;
    bool is_initialized_ = false;
    bool is_gripper_closing_ = false;
    
    // 사전 정의된 위치들
    struct Position {
        double x, y;
        std::string name;
    };
    
    std::vector<Position> predefined_positions_ = {
        {0.0, 25.0, "home"},
        {15.0, 15.0, "pickup_1"},
        {-15.0, 15.0, "pickup_2"},
        {20.0, 10.0, "delivery_1"},
        {-20.0, 10.0, "delivery_2"}
    };

public:
    DynamixelRobotArmNode() : Node("dynamixel_robot_arm_node"), portHandler(nullptr), packetHandler(nullptr) {
        // Publishers
        status_pub_ = this->create_publisher<std_msgs::msg::String>("robot_arm/status", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("robot_arm/joint_states", 10);
        mission_complete_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robot_arm/mission_complete", 10);
        
        // Subscribers
        position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "robot_arm/target_position", 10,
            std::bind(&DynamixelRobotArmNode::positionCallback, this, std::placeholders::_1));
        
        gripper_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "robot_arm/gripper_control", 10,
            std::bind(&DynamixelRobotArmNode::gripperCallback, this, std::placeholders::_1));
        
        gripper_rotate_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "robot_arm/gripper_rotate", 10,
            std::bind(&DynamixelRobotArmNode::gripperRotateCallback, this, std::placeholders::_1));
        
        // logistics_command_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        //     "/logistics/command", 10,
        //     std::bind(&DynamixelRobotArmNode::logisticsCommandCallback, this, std::placeholders::_1));
        
        // Services
        init_service_ = this->create_service<std_srvs::srv::Trigger>(
            "robot_arm/initialize",
            std::bind(&DynamixelRobotArmNode::initializeService, this, std::placeholders::_1, std::placeholders::_2));
        
        home_service_ = this->create_service<std_srvs::srv::Trigger>(
            "robot_arm/home",
            std::bind(&DynamixelRobotArmNode::homeService, this, std::placeholders::_1, std::placeholders::_2));
        
        // Timer for status publishing
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&DynamixelRobotArmNode::publishStatus, this));
        
        // 초기화 시도
        try {
            motorInit();
            publishStatusMessage("🤖 다이나믹셀 로봇 팔이 초기화되었습니다.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "초기화 실패: %s", e.what());
            publishStatusMessage("❌ 초기화 실패: " + std::string(e.what()));
        }
    }
    
    ~DynamixelRobotArmNode() {
        if (portHandler && portHandler->is_using_) {
            portHandler->closePort();
        }
        if (portHandler) delete portHandler;
        if (packetHandler) delete packetHandler;
    }

private:
    int angleToPositionJoint1(double theta1_deg) {
        return static_cast<int>(2048 - theta1_deg * (1024.0 / 90.0));
    }

    int angleToPositionJoint2(double theta2_deg) {
        if (theta2_deg < 180) {
            return static_cast<int>(2048 - theta2_deg * (2048.0 / 180.0));
        } else {
            theta2_deg -= 180;
            return static_cast<int>(2048 + theta2_deg * (2048.0 / 180.0));
        }
    }

    int angleToPositionJoint3(double theta3_deg) {
        return static_cast<int>(2048 + theta3_deg * (1024.0 / 90.0));
    }

    int16_t toSigned(uint16_t val) {
        if (val & 0x8000) {
            return static_cast<int16_t>(val - 0x10000);
        }
        return static_cast<int16_t>(val);
    }

    JointAngles inverseKinematics(double x, double y) {
        double x_2 = x;
        x = std::abs(x);
        y = std::abs(y);
        
        double D = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
        
        if (std::abs(D) > 1) {
            throw std::runtime_error("도달불가");
        }
        
        double theta2_rad = std::acos(D);
        double theta2_deg = 180 - theta2_rad * 180.0 / M_PI;
        
        double theta1_rad = std::atan2(y, x) - std::atan2(L2*std::sin(theta2_rad), L1 + L2*std::cos(theta2_rad));
        double theta1_deg = theta1_rad * 180.0 / M_PI;
        
        double theta3_deg = theta2_deg - theta1_deg;

        if (x_2 < 0) {
            theta1_deg *= -1;
            theta2_deg = 180 + theta2_deg;
            theta3_deg = 180 - theta3_deg;
        }
        
        if (theta1_deg < -90 || theta1_deg > 90) {
            throw std::runtime_error("Joint1 범위 초과: θ1 = " + std::to_string(theta1_deg) + "°");
        }
        
        return {theta1_deg, theta2_deg, theta3_deg};
    }

    void motorInit() {
        portHandler = dynamixel::PortHandler::getPortHandler("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U3VA-if00-port0");
        packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
        
        if (!portHandler->openPort()) {
            throw std::runtime_error("포트 열기 실패");
        }
        
        if (!portHandler->setBaudRate(57600)) {
            throw std::runtime_error("보레이트 설정 실패");
        }
        
        uint8_t dxl_error = 0;
        
        // Joint1, Joint2, Joint3 초기화
        std::vector<int> motor_ids = {0, 1, 3};
        for (int motor_id : motor_ids) {
            packetHandler->write4ByteTxRx(portHandler, motor_id, ADDR_ACCELERATION, 30, &dxl_error);
            packetHandler->write4ByteTxRx(portHandler, motor_id, ADDR_VELOCITY, 100, &dxl_error);
            packetHandler->write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            packetHandler->write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, INIT_POSITION, &dxl_error);
        }

        // 그리퍼 초기화
        packetHandler->write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, 4, ADDR_OPERATING_MODE, 5, &dxl_error);
        packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PROFILE_VELOCITY, SLOW_SPEED, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, OPEN_POSITION, &dxl_error);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        is_initialized_ = true;
    }

    void moveTo(double x, double y, double z = 0.0) {
        if (!is_initialized_) {
            RCLCPP_ERROR(this->get_logger(), "모터가 초기화되지 않았습니다.");
            return;
        }
        
        try {
            JointAngles angles = inverseKinematics(x, y);
            angles.theta3 += z;

            
            int pos1 = angleToPositionJoint1(angles.theta1);
            int pos2 = angleToPositionJoint2(angles.theta2);
            int pos3 = angleToPositionJoint3(angles.theta3);
            
            uint8_t dxl_error = 0;
            packetHandler->write4ByteTxRx(portHandler, 0, ADDR_GOAL_POSITION, pos1, &dxl_error);
            packetHandler->write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, pos2, &dxl_error);
            packetHandler->write4ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, pos3, &dxl_error);
            
            current_angles_ = angles;
            
            RCLCPP_INFO(this->get_logger(), "이동 좌표: (%.1f, %.1f)", x, y);
            RCLCPP_INFO(this->get_logger(), "→ θ1 = %.2f°, θ2 = %.2f°, θ3 = %.2f°", 
                       angles.theta1, angles.theta2, angles.theta3);
            
            publishStatusMessage("🎯 위치 이동: (" + std::to_string(x) + ", " + std::to_string(y) + ")");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "이동 실패: %s", e.what());
            publishStatusMessage("❌ 이동 실패: " + std::string(e.what()));
        }
    }

    void rotateGripper(double delta_deg) {
        if (!is_initialized_) return;
        
        uint32_t present_pos;
        uint8_t dxl_error = 0;
        packetHandler->read4ByteTxRx(portHandler, 3, ADDR_PRESENT_POSITION, &present_pos, &dxl_error);
        
        double current_deg = (static_cast<int>(present_pos) - 2048) * (90.0 / 1024.0);
        double target_deg = current_deg + delta_deg;
        int target_pos = angleToPositionJoint3(target_deg);
        
        packetHandler->write4ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, target_pos, &dxl_error);
        
        RCLCPP_INFO(this->get_logger(), "그리퍼 회전: %.1f° → %.1f°", current_deg, target_deg);
        publishStatusMessage("🔄 그리퍼 회전: " + std::to_string(delta_deg) + "°");
    }

    void openGripper() {
        if (!is_initialized_) return;
        
        uint8_t dxl_error = 0;
        packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, OPEN_POSITION, &dxl_error);
        
        RCLCPP_INFO(this->get_logger(), "그리퍼 열기");
        publishStatusMessage("✋ 그리퍼 열기");
        
        // 그리퍼 열기 완료 확인을 위한 타이머 시작
        gripper_monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(DELAY_MS),
            std::bind(&DynamixelRobotArmNode::checkGripperOpenComplete, this));
    }

    void closeGripper() {
        if (!is_initialized_) return;
        
        uint8_t dxl_error = 0;
        packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, CLOSE_POSITION, &dxl_error);
        
        RCLCPP_INFO(this->get_logger(), "그리퍼 닫기 (전류 감지 중)");
        publishStatusMessage("👋 그리퍼 닫기 중...");
        
        is_gripper_closing_ = true;
        
        // 그리퍼 닫기 완료 확인을 위한 타이머 시작
        gripper_monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(DELAY_MS),
            std::bind(&DynamixelRobotArmNode::checkGripperCloseComplete, this));
    }

    void checkGripperOpenComplete() {
        if (!is_initialized_) {
            if (gripper_monitor_timer_) {
                gripper_monitor_timer_->cancel();
                gripper_monitor_timer_ = nullptr;
            }
            return;
        }
        
        uint32_t pos;
        uint8_t dxl_error = 0;
        packetHandler->read4ByteTxRx(portHandler, 4, ADDR_PRESENT_POSITION, &pos, &dxl_error);
        
        // 그리퍼가 열림 위치에 도달했는지 확인 (허용 오차 50)
        if (pos > OPEN_POSITION - 50) {
            RCLCPP_INFO(this->get_logger(), "그리퍼 열기 완료");
            publishStatusMessage("✅ 그리퍼 열기 완료");
            
            // 완료 신호 발행
            auto complete_msg = std_msgs::msg::Int32();
            complete_msg.data = 1;
            mission_complete_pub_->publish(complete_msg);
            
            // 타이머 정지
            if (gripper_monitor_timer_) {
                gripper_monitor_timer_->cancel();
                gripper_monitor_timer_ = nullptr;
            }
        }
    }

    void checkGripperCloseComplete() {
        if (!is_initialized_) {
            if (gripper_monitor_timer_) {
                gripper_monitor_timer_->cancel();
                gripper_monitor_timer_ = nullptr;
            }
            return;
        }
        
        uint16_t raw_current;
        uint32_t pos;
        uint8_t dxl_error = 0;
        
        packetHandler->read2ByteTxRx(portHandler, 4, ADDR_PRESENT_CURRENT, &raw_current, &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, 4, ADDR_PRESENT_POSITION, &pos, &dxl_error);
        
        int16_t current = toSigned(raw_current);
        
        // 전류 임계값 초과 시 (물체 감지)
        if (std::abs(current) > CURRENT_THRESHOLD) {
            RCLCPP_INFO(this->get_logger(), "물체 감지됨! 전류: %d mA", current);
            packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, pos, &dxl_error);
            publishStatusMessage("🎯 물체 감지 완료!");
            
            // 완료 신호 발행
            auto complete_msg = std_msgs::msg::Int32();
            complete_msg.data = 1;
            mission_complete_pub_->publish(complete_msg);
            
            is_gripper_closing_ = false;
            
            // 타이머 정지
            if (gripper_monitor_timer_) {
                gripper_monitor_timer_->cancel();
                gripper_monitor_timer_ = nullptr;
            }
        }
        // 닫힘 위치에 도달했는지 확인 (허용 오차 50)
        else if (pos < CLOSE_POSITION + 50) {
            RCLCPP_INFO(this->get_logger(), "그리퍼 닫기 완료 (물체 없음)");
            publishStatusMessage("✅ 그리퍼 닫기 완료");
            
            // 완료 신호 발행
            auto complete_msg = std_msgs::msg::Int32();
            complete_msg.data = 1;
            mission_complete_pub_->publish(complete_msg);
            
            is_gripper_closing_ = false;
            
            // 타이머 정지
            if (gripper_monitor_timer_) {
                gripper_monitor_timer_->cancel();
                gripper_monitor_timer_ = nullptr;
            }
        }
    }

    void publishStatusMessage(const std::string& message) {
        auto msg = std_msgs::msg::String();
        msg.data = message;
        status_pub_->publish(msg);
    }

    void publishStatus() {
        if (!is_initialized_) return;
        
        // Joint state 발행
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();
        joint_msg.name = {"joint1", "joint2", "joint3", "gripper"};
        joint_msg.position = {
            current_angles_.theta1 * M_PI / 180.0,
            current_angles_.theta2 * M_PI / 180.0,
            current_angles_.theta3 * M_PI / 180.0,
            0.0  // 그리퍼 위치 (필요시 실제 값으로 업데이트)
        };
        joint_state_pub_->publish(joint_msg);
    }

    void publishPositionComplete() {
        auto complete_msg = std_msgs::msg::Int32();
        complete_msg.data = 1;
        mission_complete_pub_->publish(complete_msg);
        
        // 타이머 정지
        if (position_complete_timer_) {
            position_complete_timer_->cancel();
            position_complete_timer_ = nullptr;
        }
    }

    void publishLogisticsComplete() {
        auto complete_msg = std_msgs::msg::Int32();
        complete_msg.data = 1;
        mission_complete_pub_->publish(complete_msg);
        
        // 타이머 정지
        if (logistics_complete_timer_) {
            logistics_complete_timer_->cancel();
            logistics_complete_timer_ = nullptr;
        }
    }

    // 콜백 함수들
        void positionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "위치+회전 명령 수신: (%.1f, %.1f, %.1f°)", msg->x, msg->y, msg->z);
        
        // 1. 위치 이동 시작
        moveTo(msg->x, msg->y, msg->z);
        

        // 2. 회전 실행 (z 값이 0이 아닌 경우)
        if (std::abs(msg->z) > 0.01) {
        RCLCPP_INFO(this->get_logger(), "회전 포함: %.1f°", msg->z);
        publishStatusMessage("🔄 이동+회전 포함: (" + std::to_string(msg->x) + ", " +
                            std::to_string(msg->y) + ") + " + std::to_string(msg->z) + "°");
        } 
        else {
            publishStatusMessage("🎯 위치 이동: (" + std::to_string(msg->x) + ", " + std::to_string(msg->y) + ")");
        }
                
        // 위치 이동 완료 타이머 시작 (0.5초 후)
        position_complete_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&DynamixelRobotArmNode::publishPositionComplete, this));
    }


    void gripperCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            closeGripper();
        } else {
            openGripper();
        }
    }

    void gripperRotateCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        rotateGripper(msg->data);
    }

    // void logisticsCommandCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    //     int position_id = msg->data;
        
    //     if (position_id > 0 && position_id <= static_cast<int>(predefined_positions_.size())) {
    //         auto pos = predefined_positions_[position_id - 1];
    //         RCLCPP_INFO(this->get_logger(), "물류 명령 수신: 위치 %d (%s)", position_id, pos.name.c_str());
    //         moveTo(pos.x, pos.y);
            
    //         // 완료 신호 발행 타이머 시작 (1초 후)
    //         logistics_complete_timer_ = this->create_wall_timer(
    //             std::chrono::seconds(1),
    //             std::bind(&DynamixelRobotArmNode::publishLogisticsComplete, this));
    //     } else {
    //         RCLCPP_ERROR(this->get_logger(), "잘못된 위치 ID: %d", position_id);
    //     }
    // }

    void initializeService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
            if (portHandler && portHandler->is_using_) {
                portHandler->closePort();
            }
            motorInit();
            response->success = true;
            response->message = "모터가 성공적으로 초기화되었습니다.";
            publishStatusMessage("🔄 모터 재초기화 완료");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "초기화 실패: " + std::string(e.what());
            publishStatusMessage("❌ 재초기화 실패");
        }
    }

    void homeService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
            moveTo(0.0, 25.0);  // 홈 위치
            response->success = true;
            response->message = "홈 위치로 이동했습니다.";
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "홈 이동 실패: " + std::string(e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DynamixelRobotArmNode>();
    
    RCLCPP_INFO(node->get_logger(), "🤖 다이나믹셀 로봇 팔 ROS2 노드가 시작되었습니다.");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "노드 실행 중 오류: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}