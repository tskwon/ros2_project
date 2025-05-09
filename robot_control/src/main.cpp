#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class ImuMonitor : public rclcpp::Node {
public:
  ImuMonitor() : Node("imu_monitor")
  {
    // IMU 데이터 구독 (imu/data 토픽은 드라이버의 융합된 오리엔테이션 포함)
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10,
      std::bind(&ImuMonitor::imu_callback, this, std::placeholders::_1));

    // 20ms 주기로 출력 타이머 생성
    print_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ImuMonitor::print_imu_data, this));

    RCLCPP_INFO(this->get_logger(), "IMU Monitor initialized. Ready to receive data from imu/data topic.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::TimerBase::SharedPtr print_timer_;
  sensor_msgs::msg::Imu latest_imu_msg_;
  bool data_received_ = false;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 최신 IMU 메시지를 저장합니다.
    // 이 메시지에는 myAHRS+ 드라이버가 계산한 융합된 오리엔테이션 값이 담겨 있습니다.
    latest_imu_msg_ = *msg;
    data_received_ = true;

    // 여기서는 드라이버가 제공하는 융합된 오리엔테이션을 그대로 사용하므로
    // 별도의 상보 필터 로직은 필요 없습니다.
    // print_imu_data() 함수에서 이 저장된 메시지의 오리엔테이션을 활용합니다.
  }

  void print_imu_data()
  {
    if (!data_received_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "아직 IMU 데이터가 수신되지 않았습니다.");
      return;
    }

    // 수신된 최신 메시지에서 쿼터니언 오리엔테이션 정보를 가져옵니다.
    // 이 값은 myAHRS+ 드라이버가 자체적으로 가속도, 자이로, 지자기 데이터를 융합하여 계산한 결과입니다.
    tf2::Quaternion q(
      latest_imu_msg_.orientation.x,
      latest_imu_msg_.orientation.y,
      latest_imu_msg_.orientation.z,
      latest_imu_msg_.orientation.w
    );

    // 쿼터니언을 RPY 오일러 각으로 변환합니다.
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 라디안을 도(degree) 단위로 변환합니다.
    double roll_deg = roll * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;
    double yaw_deg = yaw * 180.0 / M_PI;

    // IMU 데이터 출력
    RCLCPP_INFO(this->get_logger(), "==== IMU 데이터 ====");
    RCLCPP_INFO(this->get_logger(), "오리엔테이션 (RPY): [Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°]",
                roll_deg, pitch_deg, yaw_deg);

    RCLCPP_INFO(this->get_logger(), "선형 가속도: [x: %.3f, y: %.3f, z: %.3f] m/s²",
                latest_imu_msg_.linear_acceleration.x,
                latest_imu_msg_.linear_acceleration.y,
                latest_imu_msg_.linear_acceleration.z);

    RCLCPP_INFO(this->get_logger(), "각속도: [x: %.3f, y: %.3f, z: %.3f] rad/s",
                latest_imu_msg_.angular_velocity.x,
                latest_imu_msg_.angular_velocity.y,
                latest_imu_msg_.angular_velocity.z);

    RCLCPP_INFO(this->get_logger(), "====================");
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}