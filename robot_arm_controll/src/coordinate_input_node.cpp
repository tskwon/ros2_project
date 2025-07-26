#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <iostream>

class CoordinateInputNode : public rclcpp::Node
{
public:
  CoordinateInputNode()
  : Node("coordinate_input_node")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Point>("target_position", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&CoordinateInputNode::publish_point, this));
  }

private:
  void publish_point()
  {
    geometry_msgs::msg::Point point;
    std::cout << "\n 목표 좌표 입력 (단위: 미터)\n";
    std::cout << "x: ";
    std::cin >> point.x;
    std::cout << "y: ";
    std::cin >> point.y;
    point.z = 0.0;

    pub_->publish(point);
    RCLCPP_INFO(this->get_logger(), "Published target (x: %.2f, y: %.2f)", point.x, point.y);
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoordinateInputNode>());
  rclcpp::shutdown();
  return 0;
}

