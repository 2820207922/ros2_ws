cmake_minimum_required(VERSION 3.8)
project(communication)

set(CMAKE_C_STANDARD 99)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ADD_SUBDIRECTORY(include)
include_directories(include)

add_executable(communication src/communication.cpp)
target_link_libraries(communication common)
ament_target_dependencies(communication rclcpp std_msgs)

install(TARGETS
  communication
  DESTINATION lib/${PROJECT_NAME})

ament_package()
