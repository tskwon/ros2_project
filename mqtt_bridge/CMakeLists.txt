cmake_minimum_required(VERSION 3.8)
project(mqtt_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# MQTT C++ 라이브러리 설정
find_package(PkgConfig REQUIRED)
pkg_check_modules(PAHO_MQTT REQUIRED paho-mqttpp3)

include_directories(
  ${PAHO_MQTT_INCLUDE_DIRS}
)

link_directories(
  ${PAHO_MQTT_LIBRARY_DIRS}
)

# 실행 파일 1: mqtt_bridge_node
add_executable(mqtt_bridge_node src/mqtt_bridge_node.cpp)
ament_target_dependencies(mqtt_bridge_node
  rclcpp
  std_msgs
)
target_link_libraries(mqtt_bridge_node
  ${PAHO_MQTT_LIBRARIES}
)

# 실행 파일 2: mqtt_listener
add_executable(mqtt_listener src/mqtt_listener.cpp)
ament_target_dependencies(mqtt_listener
  rclcpp
  std_msgs
)
target_link_libraries(mqtt_listener
  ${PAHO_MQTT_LIBRARIES}
)

# 설치 설정
install(TARGETS
  mqtt_listener
  mqtt_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
