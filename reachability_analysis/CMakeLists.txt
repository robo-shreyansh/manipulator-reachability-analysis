cmake_minimum_required(VERSION 3.8)
project(reachability_analysis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(robot_state_publisher REQUIRED)

# You might need to modify this, depending on your python version
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter Development)

include_directories(include /usr/include/python3.10 ${Python3_INCLUDE_DIRS})



add_executable(reach_node src/reach_node.cpp)

ament_target_dependencies(reach_node
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  robot_state_publisher
)

add_library(grid_lib src/grid.cpp src/cell.cpp)

target_link_libraries(reach_node grid_lib ${Python3_LIBRARIES})

install (TARGETS reach_node grid_lib
  DESTINATION lib/${PROJECT_NAME})


ament_package()
