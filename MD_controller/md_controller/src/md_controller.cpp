#include "md_controller/com.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>


class MdControllerNode : public rclcpp::Node {
public:
    MdControllerNode() : Node("md_controller_node") {
        using std::placeholders::_1;

        // 파라미터 선언 및 할당
        this->declare_parameter("MDUI", 184);
        this->declare_parameter("MDT", 183);
        this->declare_parameter("Port", "/dev/ttyUSB0");
        this->declare_parameter("Baudrate", 19200);
        this->declare_parameter("ID", 1);
        this->declare_parameter("GearRatio", 30);
        this->declare_parameter("poles", 30);

        this->get_parameter("MDUI", Com.nIDMDUI);
        this->get_parameter("MDT", Com.nIDMDT);
        this->get_parameter("Port", Com.nPort);
        this->get_parameter("Baudrate", Com.nBaudrate);
        this->get_parameter("ID", Motor.ID);
        this->get_parameter("GearRatio", Motor.GearRatio);
        this->get_parameter("poles", Motor.poles);

        rpm_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/cmd_rpm", 10,
            std::bind(&MdControllerNode::cmdRpmCallback, this, _1));

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

        InitSerial(); // 통신 초기화

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MdControllerNode::timerCallback, this)
        );
    }

private:
    void cmdRpmCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        Motor.left_rpm = msg->data[0];
        Motor.right_rpm = msg->data[1];
        SendCmdRpm = ON;
    }

    void timerCallback() {
        ReceiveDataFromController(Motor.InitMotor);

        if (fgInitsetting == ON) {
            if (++byCntControl >= 1) {  
                byCntControl = 0;

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
            }

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

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr rpm_sub_;
    rclcpp::TimerBase::SharedPtr timer_;


    BYTE SendCmdRpm;
    BYTE fgInitsetting;
    BYTE byCntInitStep;
    BYTE byCntStartDelay;
    BYTE byCntControl;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MdControllerNode>());
    rclcpp::shutdown();
    return 0;
}
