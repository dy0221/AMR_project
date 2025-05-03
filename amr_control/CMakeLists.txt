################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(amr_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  realtime_tools
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_msgs

)

################################################################################
# Build
################################################################################

# 헤더 포함 경로 설정
include_directories(
  include
)

# 라이브러리 정의
add_library(${PROJECT_NAME}_lib
  src/amr_controller.cpp
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

# 실행 파일 정의
add_executable(${PROJECT_NAME} src/amr_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

################################################################################
# Install
################################################################################

# 파라미터 폴더 설치 (없다면 생략해도 됨)
# install(DIRECTORY param
#   DESTINATION share/${PROJECT_NAME}
# )

# 실행 파일 설치
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Export
################################################################################
ament_export_include_directories(include)
ament_package()
