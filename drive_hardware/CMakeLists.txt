cmake_minimum_required(VERSION 3.8)
project(drive_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rover_serial_driver
  src/rover_serial_driver.cpp
)

ament_target_dependencies(rover_serial_driver
  rclcpp
  std_msgs
)

install(TARGETS
  rover_serial_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

