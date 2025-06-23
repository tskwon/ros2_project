#include "robot_arm_controll/robot_arm.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <iostream>

using namespace dynamixel;

PortHandler *portHandler;
PacketHandler *packetHandler;

RobotArm::RobotArm(const std::string &device_name, int baudrate)
: device_(device_name), baudrate_(baudrate)
{
  portHandler = PortHandler::getPortHandler(device_.c_str());
  packetHandler = PacketHandler::getPacketHandler(2.0);
}

bool RobotArm::initMotors()
{
  if (!portHandler->openPort()) {
    std::cerr << "포트 열기 실패!" << std::endl;
    return false;
  }

  if (!portHandler->setBaudRate(baudrate_)) {
    std::cerr << "보레이트 설정 실패!" << std::endl;
    return false;
  }

  for (int id = 0; id <= 1; ++id) {
  uint8_t dxl_error = 0;

  // 1. Position Mode
  int result = packetHandler->write1ByteTxRx(portHandler, id, 11, 3, &dxl_error);
  if (result != COMM_SUCCESS) return false;

  // 2. 속도 설정 (느리게)
  uint32_t velocity = 100;  // 낮을수록 느림
  result = packetHandler->write4ByteTxRx(portHandler, id, 112, velocity, &dxl_error);
  if (result != COMM_SUCCESS) return false;

  // 3. Torque Enable
  result = packetHandler->write1ByteTxRx(portHandler, id, 64, 1, &dxl_error);
  if (result != COMM_SUCCESS) return false;
}


  std::cout << "모터 초기화 완료" << std::endl;
  return true;
}

void RobotArm::setJointPosition(int id, int goal_position)
{
  uint8_t dxl_error = 0;
  uint32_t pos = static_cast<uint32_t>(goal_position);

  int result = packetHandler->write4ByteTxRx(portHandler, id, 116, pos, &dxl_error);
  if (result != COMM_SUCCESS) {
    std::cerr << "Goal Position 설정 실패 (ID " << id << "): "
              << packetHandler->getTxRxResult(result) << std::endl;
  }
}

std::vector<int> RobotArm::getJointPositions()
{
  std::vector<int> positions;

  for (int id = 0; id <= 1; ++id) {
    uint8_t dxl_error = 0;
    uint32_t pos = 0;
    int result = packetHandler->read4ByteTxRx(portHandler, id, 132, &pos, &dxl_error);
    if (result != COMM_SUCCESS) {
      std::cerr << "Present Position 읽기 실패 (ID " << id << "): "
                << packetHandler->getTxRxResult(result) << std::endl;
      positions.push_back(2048);  // 기본값
    } else {
      positions.push_back(static_cast<int>(pos));
    }
  }

  return positions;
}

