cmake_minimum_required(VERSION 3.8)
project(assembly_parametric_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(srv_files
  "srv/ManageExperimentConfig.srv"
  "srv/ValidatePlan.srv"
  "srv/PreviewPlan.srv"
  "srv/ExecuteCachedPlan.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

add_executable(parametric_motion_server src/parametric_motion_server.cpp)
ament_target_dependencies(parametric_motion_server
  ament_index_cpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
)
target_link_libraries(parametric_motion_server yaml-cpp)
rosidl_target_interfaces(parametric_motion_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  parametric_motion_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
