cmake_minimum_required(VERSION 3.8)
project(px4_offboard_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(offboard_square src/offboard_square.cpp)
ament_target_dependencies(offboard_square
  rclcpp
  px4_msgs
)

install(TARGETS offboard_square
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
