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
find_package(sensor_msgs REQUIRED)
# Nav2 연동을 위한 추가 패키지
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 인클루드 디렉토리 설정 (기존 헤더 파일들)
include_directories(include)

# 실행 파일 생성
add_executable(md_controller_node 
  src/md_controller.cpp
  src/com.cpp
)

# 의존성 연결
ament_target_dependencies(md_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

# libserial 라이브러리 링크 (시리얼 통신용)
find_library(SERIAL_LIB serial)
if(SERIAL_LIB)
  target_link_libraries(md_controller_node ${SERIAL_LIB})
else()
  message(WARNING "libserial not found, using alternative serial implementation")
endif()

# 설치
install(TARGETS
  md_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# 설정 파일들 설치 (있다면)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 중복 제거를 위해 다음 라인들 추가
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()