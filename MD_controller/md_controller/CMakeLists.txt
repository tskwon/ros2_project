cmake_minimum_required(VERSION 3.8)
project(md_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# libserial 설정
pkg_check_modules(SERIAL libserial REQUIRED)

# include 디렉토리 설정
include_directories(
  include
  ${SERIAL_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
 
add_executable(md_controller src/md_controller.cpp src/com.cpp)

ament_target_dependencies(md_controller
              rclcpp
              std_msgs
              geometry_msgs
              tf2_ros
              tf2
              sensor_msgs
             )

# libserial 라이브러리 링크
target_link_libraries(md_controller
  ${SERIAL_LIBRARIES}
)

install(TARGETS
  md_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()


