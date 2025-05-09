cmake_minimum_required(VERSION 3.8)
project(mqtt_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# MQTT 라이브러리 수동 링크
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(mqtt_bridge_node src/mqtt_bridge_node.cpp)

ament_target_dependencies(mqtt_bridge_node
  rclcpp
  std_msgs
)

target_link_libraries(mqtt_bridge_node
  paho-mqttpp3
  paho-mqtt3a
)

install(TARGETS
  mqtt_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
