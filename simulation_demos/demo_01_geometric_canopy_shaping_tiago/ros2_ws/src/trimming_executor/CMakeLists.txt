cmake_minimum_required(VERSION 3.8)
project(trimming_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Required by the PAL/MoveIt dependency chain in the tested Humble setup.
find_package(tl-expected CONFIG REQUIRED)

add_executable(trimming_executor src/trimming_executor.cpp)
ament_target_dependencies(
  trimming_executor
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
)

install(TARGETS
  trimming_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
