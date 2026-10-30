cmake_minimum_required(VERSION 3.8)
project(airsim_ros2_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_library(airsim_bridge_core STATIC
  src/frames.cpp
  src/rpc.cpp
)
target_include_directories(airsim_bridge_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(airsim_sensors src/sensors.cpp)
target_link_libraries(airsim_sensors airsim_bridge_core ${OpenCV_LIBS})
target_include_directories(airsim_sensors PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(airsim_sensors
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

add_executable(topic_rates src/topic_rates.cpp)
ament_target_dependencies(topic_rates
  rclcpp
  nav_msgs
  sensor_msgs
)

install(TARGETS
  airsim_sensors
  topic_rates
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
