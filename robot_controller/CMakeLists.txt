cmake_minimum_required(VERSION 3.8)
project(robot_controller)

# 컴파일러 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# 실행 파일 추가
add_executable(robot_controller_node src/robot_controller_node.cpp)

# 타겟에 의존성 연결
ament_target_dependencies(robot_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  nlohmann_json
)

# 실행 파일 설치 경로 지정
install(TARGETS
  robot_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 테스트 관련 설정 (선택)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 패키지 선언
ament_package()
