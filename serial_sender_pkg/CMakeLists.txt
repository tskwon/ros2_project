cmake_minimum_required(VERSION 3.8)
project(serial_sender_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# serial 패키지 대신 libserial 사용
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL libserial REQUIRED)

add_executable(serial_sender_node src/serial_sender_node.cpp)
ament_target_dependencies(serial_sender_node rclcpp std_msgs)

# libserial 라이브러리 추가
target_include_directories(serial_sender_node PRIVATE ${SERIAL_INCLUDE_DIRS})
target_link_libraries(serial_sender_node ${SERIAL_LIBRARIES})

# 실행 파일 설치 설정
install(TARGETS
  serial_sender_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()