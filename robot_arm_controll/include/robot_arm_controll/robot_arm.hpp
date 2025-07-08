#ifndef ROBOT_ARM_HPP
#define ROBOT_ARM_HPP

#include <string>
#include <vector>

class RobotArm
{
public:
  RobotArm(const std::string &device_name, int baudrate);
  bool initMotors();
  std::vector<int> getJointPositions();
  void setJointPosition(int id, int goal_position);

private:
  std::string device_;
  int baudrate_;
};

#endif  // ROBOT_ARM_HPP

