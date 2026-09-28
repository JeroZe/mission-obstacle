cmake_minimum_required(VERSION 3.8)
project(mid360_obstacle_stop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(obstacle_stop_node
  src/obstacle_stop_node.cpp
  src/obstacle_detector.cpp
)

target_include_directories(obstacle_stop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(obstacle_stop_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  px4_msgs
)

install(TARGETS obstacle_stop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_obstacle_publisher.py
  scripts/fake_px4_stub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
