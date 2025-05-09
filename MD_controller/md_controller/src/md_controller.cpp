#include "md_controller/com.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

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
        this->declare_parameter("Port", "/dev/ttyUSB0");
        this->declare_parameter("Baudrate", 57600);
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
        SendCmdRpm = OFF;
        
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
            Motor.left_rpm = -goal_rpm[LEFT];
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

    void timerCallback() {
        ReceiveDataFromController(Motor.InitMotor);

        if (fgInitsetting == ON) {
            if (SendCmdRpm) {
                IByte left = Short2Byte(Motor.left_rpm);
                IByte right = Short2Byte(Motor.right_rpm);
                int nArray[4] = {right.byLow, right.byHigh, left.byLow, left.byHigh};

                if (Motor.left_rpm == 0 && Motor.right_rpm == 0) {
                    PutMdData(PID_PNT_TQ_OFF, Com.nIDMDT, Motor.ID, nArray);
                } else {
                    PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);
                }

                SendCmdRpm = OFF;
            }

            // 상태 요청
            int nArray[4] = {PID_MAIN_DATA, 0, 0, 0};
            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);
        

        } else {
            if (byCntStartDelay <= 200) {
                byCntStartDelay++;
            } else {
                InitMotorSequence();
            }
        }
    }

    void InitMotorSequence() {
        int nArray[4] = {0, 0, 0, 0};

        switch (byCntInitStep) {
            case 1:
                nArray[0] = PID_MAIN_DATA;
                PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);

                if (Motor.InitMotor == ON) {
                    Motor.InitError++;
                } else {
                    byCntInitStep++;
                }

                if (Motor.InitError > 10) {
                    RCLCPP_ERROR(this->get_logger(), "ID %d MOTOR INIT ERROR!!", Motor.ID);
                    rclcpp::shutdown();
                }
                break;

            case 2:
                byCntInitStep++;
                break;

            case 3:
                PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);
                byCntInitStep++;
                break;

            case 4:
                PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor.ID, nArray);
                byCntInitStep++;
                break;

            case 5:
                RCLCPP_INFO(this->get_logger(), "MOTOR INIT END");
                fgInitsetting = ON;
                break;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    BYTE SendCmdRpm;
    BYTE fgInitsetting;
    BYTE byCntInitStep;
    BYTE byCntStartDelay;
    BYTE byCntControl;

    int max_rpm_;                // 최대 RPM
    rclcpp::Time last_cmd_time_; // 마지막 명령 수신 시간
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MdControllerNode>());
    rclcpp::shutdown();
    return 0;
}
