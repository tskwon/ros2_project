#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_arm_controll/robot_arm.hpp"

#include <cmath>
#include <memory>

#define L1 0.20
#define L2 0.16
#define DEG2POS_JOINT1(deg) (2048 - static_cast<int>((deg) * (1024.0 / 90.0)))
#define DEG2POS_JOINT2(deg) (2048 + static_cast<int>((deg) * (3050.0 / 270.0)))

class RobotArmNode : public rclcpp::Node
{
public:
  RobotArmNode()
  : Node("robot_arm_node")
  {
    robot_arm_ = std::make_shared<RobotArm>("/dev/ttyUSB0", 57600);
    if (!robot_arm_->initMotors()) {
      RCLCPP_ERROR(this->get_logger(), "모터 초기화 실패");
      rclcpp::shutdown();
    }

    robot_arm_->setJointPosition(0, 2048);
    robot_arm_->setJointPosition(1, 2048);
    RCLCPP_INFO(this->get_logger(), "모터 초기화 완료");

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "target_position", 10,
      std::bind(&RobotArmNode::target_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RobotArmNode::publish_joint_states, this));
  }

private:
  std::shared_ptr<RobotArm> robot_arm_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pos_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    double x = msg->x ;  // 기준점 (0, 0)에서 L1 만큼 오른쪽으로 이동
    double y = msg->y ;  // 기준점 (0, 0)에서 L1 만큼 위로 이동

    double x_abs = std::abs(x);
    double D = ((x_abs * x_abs )  + y * y    - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (std::abs(D) > 1.0) {
      RCLCPP_WARN(this->get_logger(), "도달할 수 없는 위치입니다.");
      return;
    }

    // 기본 자세 (Elbow Down 기준)
    double theta2_rad = std::acos(D);
    double theta1_rad = std::atan2(y, x_abs) - std::atan2(L2 * std::sin(theta2_rad), L1 + L2 * std::cos(theta2_rad));

    // 좌측일 경우 대칭 처리
    if (x < 0) {
      theta1_rad *= -1.0;
      theta2_rad *= -1.0;
    }

    // 각도(도) 변환
    double theta1_deg = theta1_rad * 180.0 / M_PI;
    double theta2_deg = theta2_rad * 180.0 / M_PI;

    if (theta1_deg < -90.0 || theta1_deg > 90.0) {
      RCLCPP_WARN(this->get_logger(), "θ1 범위 초과: %.2f도", theta1_deg);
      return;
    }

    if (theta2_deg < -135.0 || theta2_deg > 135.0) {
      RCLCPP_WARN(this->get_logger(), "θ2 범위 초과: %.2f도", theta2_deg);
      return;
    }

    int goal1 = DEG2POS_JOINT1(theta1_deg);
    int goal2 = DEG2POS_JOINT2(theta2_deg);

    robot_arm_->setJointPosition(0, goal1);
    robot_arm_->setJointPosition(1, goal2);

    RCLCPP_INFO(this->get_logger(),
      "목표 (%.2f, %.2f) → θ1=%.2f°, θ2=%.2f° → pos1=%d, pos2=%d",
      x, y, theta1_deg, theta2_deg, goal1, goal2);
  }

  void publish_joint_states()
  {
    std::vector<int> raw_pos = robot_arm_->getJointPositions();

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = {"joint1", "joint2"};
    msg.position.resize(2);
    msg.position[0] = (raw_pos[0] - 2048) * (M_PI / 2048.0);
    msg.position[1] = (raw_pos[1] - 2048) * (M_PI / 2048.0);
    joint_pub_->publish(msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotArmNode>());
  rclcpp::shutdown();
  return 0;
}

