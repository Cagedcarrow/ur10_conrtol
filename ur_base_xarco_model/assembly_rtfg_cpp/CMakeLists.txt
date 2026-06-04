cmake_minimum_required(VERSION 3.8)
project(assembly_rtfg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(FCL REQUIRED fcl)

set(srv_files
  "srv/LoadConfig.srv"
  "srv/FitPreview.srv"
  "srv/ExecuteCached.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs trajectory_msgs
)

add_library(rtfg_core
  src/utils.cpp
  src/robot_model.cpp
  src/ik_backend.cpp
  src/ik_solver.cpp
  src/collision_checker.cpp
  src/collision_pipeline.cpp
  src/continuous_trajectory_solver.cpp
  src/rolling_planner.cpp
  src/trajectory_solver.cpp
  src/trajectory_generator.cpp
)
target_include_directories(rtfg_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/assembly_rtfg_cpp>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${FCL_INCLUDE_DIRS}
)
target_link_directories(rtfg_core PUBLIC ${FCL_LIBRARY_DIRS})
target_link_libraries(rtfg_core
  Eigen3::Eigen
  yaml-cpp
  ${FCL_LIBRARIES}
  orocos-kdl
)
ament_target_dependencies(rtfg_core
  kdl_parser
  urdf
)

add_executable(rtfg_solver_node src/rtfg_solver_node.cpp)
target_link_libraries(rtfg_solver_node rtfg_core)
ament_target_dependencies(rtfg_solver_node
  ament_index_cpp
  control_msgs
  geometry_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  trajectory_msgs
  visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rtfg_solver_node "${cpp_typesupport_target}")

add_executable(rtfg_geometry_check test/rtfg_geometry_check.cpp)
target_link_libraries(rtfg_geometry_check rtfg_core)
ament_target_dependencies(rtfg_geometry_check ament_index_cpp)

install(TARGETS rtfg_core rtfg_solver_node rtfg_geometry_check
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf rviz docs benchmarks
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
