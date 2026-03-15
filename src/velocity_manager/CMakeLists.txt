cmake_minimum_required(VERSION 3.8)
project(velocity_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(velocity_manager_node src/velocity_manager_node.cpp)

ament_target_dependencies(velocity_manager_node
  rclcpp
  geometry_msgs
)

install(TARGETS
  velocity_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()