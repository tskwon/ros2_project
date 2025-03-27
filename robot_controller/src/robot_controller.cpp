#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller_node") {
        RCLCPP_INFO(this->get_logger(), "로봇 컨트롤러 노드가 시작되었습니다.");

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
            std::bind(&RobotController::control_loop, this));
    }

private:
    void control_loop() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.5;  // 전진 속도
        msg.angular.z = 0.1; // 회전 속도
        cmd_vel_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "속도 명령을 퍼블리시 했습니다.");
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
