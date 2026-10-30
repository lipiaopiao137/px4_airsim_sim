cmake_minimum_required(VERSION 3.8)
project(px4_keyboard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(keyboard_velocity src/keyboard_velocity.cpp)
ament_target_dependencies(keyboard_velocity
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(velocity_control src/velocity_control.cpp)
ament_target_dependencies(velocity_control
  rclcpp
  geometry_msgs
  std_msgs
  px4_msgs
)

install(TARGETS
  keyboard_velocity
  velocity_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
