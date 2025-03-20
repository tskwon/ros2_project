cmake_minimum_required(VERSION 3.8)
project(ir_sensor_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add wiringPi library
find_library(WIRINGPI_LIB wiringPi)
include_directories(/usr/include)

# Add executable
add_executable(ir_sensor_reader src/ir_sensor_reader.cpp)
ament_target_dependencies(ir_sensor_reader rclcpp std_msgs)

# Link wiringPi
target_link_libraries(ir_sensor_reader ${WIRINGPI_LIB})

# Install target
install(TARGETS
  ir_sensor_reader
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
