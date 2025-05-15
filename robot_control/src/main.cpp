#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <memory>

class ImuRawMonitor : public rclcpp::Node
{
public:
  ImuRawMonitor()
      : Node("imu_raw_monitor"),
        is_first_gyro_(true),
        gyro_bias_x_(0.0), gyro_bias_y_(0.0), gyro_bias_z_(0.0),
        calibration_samples_(0),
        current_roll_(0.0), current_pitch_(0.0), current_yaw_(0.0),
        integrated_roll_(0.0), integrated_pitch_(0.0), integrated_yaw_(0.0)
  {
    // Raw IMU 데이터 구독
    raw_imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10,
        std::bind(&ImuRawMonitor::raw_imu_callback, this, std::placeholders::_1));

    // 자기계 데이터 구독
    mag_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "imu/mag", 10,
        std::bind(&ImuRawMonitor::mag_callback, this, std::placeholders::_1));

    // 비교를 위한 융합된 IMU 데이터 구독
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10,
        std::bind(&ImuRawMonitor::imu_callback, this, std::placeholders::_1));

    // 10ms 주기로 출력 및 처리 타이머 생성
    process_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ImuRawMonitor::process_data, this));

    // 파라미터 설정
    this->declare_parameter("calibration_duration", 3.0); // 초 단위
    this->declare_parameter("motion_threshold", 0.03);    // rad/s

    calibration_duration_ = this->get_parameter("calibration_duration").as_double();
    motion_threshold_ = this->get_parameter("motion_threshold").as_double();

    RCLCPP_INFO(this->get_logger(), "IMU Raw Monitor initialized.");
    RCLCPP_INFO(this->get_logger(), "지자기 보정 및 자이로 바이어스 추정 중... %d초 동안 로봇을 움직이지 마세요.",
                static_cast<int>(calibration_duration_));

    // 보정 시작 시간 설정
    calibration_start_time_ = this->now();
    last_process_time_ = this->now();
  }

private:
  // 구독자
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::TimerBase::SharedPtr process_timer_;

  // 데이터 저장 변수
  sensor_msgs::msg::Imu latest_raw_imu_msg_;
  sensor_msgs::msg::MagneticField latest_mag_msg_;
  sensor_msgs::msg::Imu latest_imu_msg_;

  // 데이터 수신 플래그
  bool raw_data_received_ = false;
  bool mag_data_received_ = false;
  bool imu_data_received_ = false;

  // 시간 관리 (모두 같은 시간 소스인 this->now()를 사용)
  rclcpp::Time calibration_start_time_;
  rclcpp::Time last_process_time_;

  // Raw 데이터 처리를 위한 변수
  double current_roll_, current_pitch_, current_yaw_;
  double integrated_roll_, integrated_pitch_, integrated_yaw_;
  bool is_first_gyro_;

  // 자이로 바이어스 추정
  double gyro_bias_x_, gyro_bias_y_, gyro_bias_z_;
  int calibration_samples_;
  double calibration_duration_;

  // 움직임 감지
  double motion_threshold_;
  bool is_moving_ = false;

  void raw_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_raw_imu_msg_ = *msg;
    raw_data_received_ = true;

    // 보정 기간 동안 자이로 바이어스 추정
    rclcpp::Time current_time = this->now();
    double elapsed = (current_time - calibration_start_time_).seconds();

    if (elapsed < calibration_duration_)
    {
      // 보정 기간 동안 자이로 값 누적
      gyro_bias_x_ += msg->angular_velocity.x;
      gyro_bias_y_ += msg->angular_velocity.y;
      gyro_bias_z_ += msg->angular_velocity.z;
      calibration_samples_++;

      if (calibration_samples_ % 100 == 0)
      {
        RCLCPP_INFO(this->get_logger(), "보정 중... %.1f초 남음",
                    calibration_duration_ - elapsed);
      }
    }
    else if (calibration_samples_ > 0 && elapsed >= calibration_duration_ && !is_first_gyro_)
    {
      // 보정 완료 후 한 번만 실행
      gyro_bias_x_ /= calibration_samples_;
      gyro_bias_y_ /= calibration_samples_;
      gyro_bias_z_ /= calibration_samples_;
      RCLCPP_INFO(this->get_logger(), "자이로 바이어스 추정 완료: [%.6f, %.6f, %.6f]",
                  gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
      is_first_gyro_ = true; // 이미 보정했음을 표시
    }
  }

  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    latest_mag_msg_ = *msg;
    mag_data_received_ = true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_msg_ = *msg;
    imu_data_received_ = true;
  }

  void process_data()
  {
    if (!raw_data_received_)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Raw IMU 데이터가 수신되지 않았습니다.");
      return;
    }

    // 현재 시간 가져오기
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_process_time_).seconds();
    last_process_time_ = current_time;

    // 너무 작거나 큰 dt는 무시
    if (dt <= 0.0 || dt > 0.1)
    {
      return;
    }

    // 보정 기간이 지났을 때만 처리
    double elapsed = (current_time - calibration_start_time_).seconds();
    if (elapsed < calibration_duration_)
    {
      return;
    }

    // 자이로스코프 데이터 추출 및 바이어스 보정
    double gyro_x = latest_raw_imu_msg_.angular_velocity.x;
    double gyro_y = latest_raw_imu_msg_.angular_velocity.y;
    double gyro_z = latest_raw_imu_msg_.angular_velocity.z;

    double unbiased_gyro_x = gyro_x - gyro_bias_x_;
    double unbiased_gyro_y = gyro_y - gyro_bias_y_;
    double unbiased_gyro_z = gyro_z - gyro_bias_z_;

    // 움직임 감지
    double angular_velocity_magnitude = std::sqrt(
        std::pow(unbiased_gyro_x, 2) +
        std::pow(unbiased_gyro_y, 2) +
        std::pow(unbiased_gyro_z, 2));

    // 히스테리시스로 움직임 상태 전환
    if (is_moving_ && angular_velocity_magnitude < motion_threshold_ * 0.8)
    {
      is_moving_ = false;
      RCLCPP_INFO(this->get_logger(), "로봇 정지 상태 감지");
    }
    else if (!is_moving_ && angular_velocity_magnitude > motion_threshold_ * 1.2)
    {
      is_moving_ = true;
      RCLCPP_INFO(this->get_logger(), "로봇 이동 상태 감지");
    }

    // 가속도계 데이터 추출
    double acc_x = latest_raw_imu_msg_.linear_acceleration.x;
    double acc_y = latest_raw_imu_msg_.linear_acceleration.y;
    double acc_z = latest_raw_imu_msg_.linear_acceleration.z;

    // 가속도계 데이터로 롤과 피치 계산
    double acc_roll = std::atan2(acc_y, std::sqrt(acc_x * acc_x + acc_z * acc_z));
    double acc_pitch = std::atan2(-acc_x, std::sqrt(acc_y * acc_y + acc_z * acc_z));

    // 자기계 데이터가 있을 경우 처리
    double mag_yaw = 0.0;
    if (mag_data_received_)
    {
      double mag_x = latest_mag_msg_.magnetic_field.x;
      double mag_y = latest_mag_msg_.magnetic_field.y;
      double mag_z = latest_mag_msg_.magnetic_field.z;

      // 자기계 데이터를 수평면으로 보정
      double cos_roll = std::cos(acc_roll);
      double sin_roll = std::sin(acc_roll);
      double cos_pitch = std::cos(acc_pitch);
      double sin_pitch = std::sin(acc_pitch);

      // 지자기 데이터 틸트 보정
      double mag_x_comp = mag_x * cos_pitch + mag_z * sin_pitch;
      double mag_y_comp = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch;

      // 보정된 지자기 데이터로 yaw 계산
      mag_yaw = std::atan2(-mag_y_comp, mag_x_comp);
    }

    // 자이로스코프 적분으로 각도 추정
    // 자이로 적분
    integrated_roll_ += unbiased_gyro_x * dt;
    integrated_pitch_ += unbiased_gyro_y * dt;
    integrated_yaw_ += unbiased_gyro_z * dt;

    // -π ~ π 범위로 정규화
    while (integrated_roll_ > M_PI)
      integrated_roll_ -= 2.0 * M_PI;
    while (integrated_roll_ < -M_PI)
      integrated_roll_ += 2.0 * M_PI;
    while (integrated_pitch_ > M_PI)
      integrated_pitch_ -= 2.0 * M_PI;
    while (integrated_pitch_ < -M_PI)
      integrated_pitch_ += 2.0 * M_PI;
    while (integrated_yaw_ > M_PI)
      integrated_yaw_ -= 2.0 * M_PI;
    while (integrated_yaw_ < -M_PI)
      integrated_yaw_ += 2.0 * M_PI;

    // 상보 필터: 움직임 상태에 따라 가중치 조정
    double alpha_roll_pitch = is_moving_ ? 0.01 : 0.1;
    double alpha_yaw = is_moving_ ? 0.01 : 0.05;

    // 롤과 피치는 가속도계와 자이로스코프 융합
    current_roll_ = (1.0 - alpha_roll_pitch) * (current_roll_ + unbiased_gyro_x * dt) + alpha_roll_pitch * acc_roll;
    current_pitch_ = (1.0 - alpha_roll_pitch) * (current_pitch_ + unbiased_gyro_y * dt) + alpha_roll_pitch * acc_pitch;

    // 요는 자기계와 자이로스코프 융합 (자기계 데이터가 있을 경우)
    if (mag_data_received_)
    {
      current_yaw_ = (1.0 - alpha_yaw) * (current_yaw_ + unbiased_gyro_z * dt) + alpha_yaw * mag_yaw;
    }
    else
    {
      current_yaw_ += unbiased_gyro_z * dt;
    }

    // -π ~ π 범위로 정규화
    while (current_roll_ > M_PI)
      current_roll_ -= 2.0 * M_PI;
    while (current_roll_ < -M_PI)
      current_roll_ += 2.0 * M_PI;
    while (current_pitch_ > M_PI)
      current_pitch_ -= 2.0 * M_PI;
    while (current_pitch_ < -M_PI)
      current_pitch_ += 2.0 * M_PI;
    while (current_yaw_ > M_PI)
      current_yaw_ -= 2.0 * M_PI;
    while (current_yaw_ < -M_PI)
      current_yaw_ += 2.0 * M_PI;

    // 융합된 IMU 데이터에서 쿼터니언을 오일러 각으로 변환 (비교용)
    double fusion_roll = 0.0, fusion_pitch = 0.0, fusion_yaw = 0.0;
    if (imu_data_received_)
    {
      tf2::Quaternion q(
          latest_imu_msg_.orientation.x,
          latest_imu_msg_.orientation.y,
          latest_imu_msg_.orientation.z,
          latest_imu_msg_.orientation.w);
      tf2::Matrix3x3(q).getRPY(fusion_roll, fusion_pitch, fusion_yaw);
    }

    // 라디안을 도(degree) 단위로 변환
    double acc_roll_deg = acc_roll * 180.0 / M_PI;
    double acc_pitch_deg = acc_pitch * 180.0 / M_PI;
    double mag_yaw_deg = mag_yaw * 180.0 / M_PI;

    double integrated_roll_deg = integrated_roll_ * 180.0 / M_PI;
    double integrated_pitch_deg = integrated_pitch_ * 180.0 / M_PI;
    double integrated_yaw_deg = integrated_yaw_ * 180.0 / M_PI;

    double current_roll_deg = current_roll_ * 180.0 / M_PI;
    double current_pitch_deg = current_pitch_ * 180.0 / M_PI;
    double current_yaw_deg = current_yaw_ * 180.0 / M_PI;

    double fusion_roll_deg = fusion_roll * 180.0 / M_PI;
    double fusion_pitch_deg = fusion_pitch * 180.0 / M_PI;
    double fusion_yaw_deg = fusion_yaw * 180.0 / M_PI;

    // 데이터 출력
    RCLCPP_INFO(this->get_logger(), "=== Raw 센서 데이터 ===");
    RCLCPP_INFO(this->get_logger(), "가속도계 RPY: [Roll: %.2f°, Pitch: %.2f°]",
                acc_roll_deg, acc_pitch_deg);

    if (mag_data_received_)
    {
      RCLCPP_INFO(this->get_logger(), "자기계 Yaw: %.2f°", mag_yaw_deg);
    }

    RCLCPP_INFO(this->get_logger(), "자이로스코프: [x: %.3f, y: %.3f, z: %.3f] rad/s",
                unbiased_gyro_x, unbiased_gyro_y, unbiased_gyro_z);

    RCLCPP_INFO(this->get_logger(), "=== 계산된 자세 ===");
    RCLCPP_INFO(this->get_logger(), "자이로 적분 RPY: [Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°]",
                integrated_roll_deg, integrated_pitch_deg, integrated_yaw_deg);

    RCLCPP_INFO(this->get_logger(), "상보 필터 RPY: [Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°]",
                current_roll_deg, current_pitch_deg, current_yaw_deg);

    if (imu_data_received_)
    {
      RCLCPP_INFO(this->get_logger(), "드라이버 융합 RPY: [Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°]",
                  fusion_roll_deg, fusion_pitch_deg, fusion_yaw_deg);
    }

    RCLCPP_INFO(this->get_logger(), "움직임 상태: %s", is_moving_ ? "이동 중" : "정지 상태");
    RCLCPP_INFO(this->get_logger(), "====================");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuRawMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}