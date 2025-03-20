#include "md_controller/com.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>

Communication Com;
MotorVar Motor;

BYTE SendCmdRpm = OFF;

void CmdRpmCallBack(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    // RPM 범위 체크 (-32768 ~ 32767)
    Motor.left_rpm = msg->data[0] ;
    Motor.right_rpm = msg->data[1] ;
    
    SendCmdRpm = ON;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("md_controller_node");

    // Subscriber: /cmd_rpm 토픽 구독
    auto rpm_sub = node->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/cmd_rpm", 1000, CmdRpmCallBack);

    // Motor driver setup
    node->declare_parameter("MDUI", 184);
    node->declare_parameter("MDT", 183);
    node->declare_parameter("Port", "/dev/ttyUSB0");
    node->declare_parameter("Baudrate", 19200);
    node->declare_parameter("ID", 1);
    node->declare_parameter("GearRatio", 10);
    node->declare_parameter("poles", 30);

    node->get_parameter("MDUI", Com.nIDMDUI);
    node->get_parameter("MDT", Com.nIDMDT);
    node->get_parameter("Port", Com.nPort);
    node->get_parameter("Baudrate", Com.nBaudrate);
    node->get_parameter("ID", Motor.ID);
    node->get_parameter("GearRatio", Motor.GearRatio);
    node->get_parameter("poles", Motor.poles);

    // MotorVar 초기화
    Motor.left_rpm = 0;
    Motor.right_rpm = 0;
    Motor.left_position = 0;
    Motor.right_position = 0;

    IByte left, right;
    int nArray[4];
    static BYTE fgInitsetting = OFF, byCntInitStep = 1, byCnt2500us = 0, byCntStartDelay = 0, byCntControl = 0;

    Motor.InitMotor = ON;
    Motor.InitError = 0;

    InitSerial(); // Communication initialization in com.cpp

    while (rclcpp::ok()) {
        ReceiveDataFromController(Motor.InitMotor);

        if (++byCnt2500us == 10) {
            byCnt2500us = 0;

            if (fgInitsetting == ON) {
                if (++byCntControl == TIME_100MS) {
                    byCntControl = 0;

                    if (SendCmdRpm) {
                        left = Short2Byte(Motor.left_rpm);
                        right = Short2Byte(Motor.right_rpm);
                        nArray[0] = left.byLow;
                        nArray[1] = left.byHigh;
                        nArray[2] = right.byLow;
                        nArray[3] = right.byHigh;

                        if (Motor.left_rpm == 0 && Motor.right_rpm == 0) {
                            PutMdData(PID_PNT_TQ_OFF, Com.nIDMDT, Motor.ID, nArray);
                        } else {
                            PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);
                        }

                        SendCmdRpm = OFF;
                    }

                    // 상태 요청
                    nArray[0] = PID_MAIN_DATA;
                    PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);
                }
            } else {
                if (byCntStartDelay <= 200) {
                    byCntStartDelay++;
                } else {
                    switch (byCntInitStep) {
                        case 1: // Motor connect check
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);

                            if (Motor.InitMotor == ON) {
                                Motor.InitError++;
                            } else {
                                byCntInitStep++;
                            }

                            if (Motor.InitError > 10) {
                                RCLCPP_ERROR(node->get_logger(), "ID %d MOTOR INIT ERROR!!", Motor.ID);
                                rclcpp::shutdown();
                                return 0;
                            }
                            break;

                        case 2:
                            byCntInitStep++;
                            break;

                        case 3: // Motor torque ON
                            nArray[0] = 0;
                            nArray[1] = 0;
                            nArray[2] = 0;
                            nArray[3] = 0;
                            PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray); // 속도 0으로 설정
                            byCntInitStep++;
                            break;

                        case 4: // Motor position reset (필요 시)
                            PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor.ID, nArray);
                            byCntInitStep++;
                            break;

                        case 5:
                            RCLCPP_INFO(node->get_logger(), "MOTOR INIT END");
                            fgInitsetting = ON;
                            break;
                    }
                }
            }
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}