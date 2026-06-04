cmake_minimum_required(VERSION 3.8)
project(assembly_rtfg_cuda)

# CUDA 13.3 support
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# CUDA architecture: Ada Lovelace (RTX 4060 Laptop: sm_89)
set(CMAKE_CUDA_ARCHITECTURES 89)

# CompileIQ AI compiler auto-tuning (CUDA 13.3)
# Uncomment to enable: add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-fopenmp>)

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

# Reuse service definitions from cpp package
set(srv_files
  "../assembly_rtfg_cpp/srv/LoadConfig.srv"
  "../assembly_rtfg_cpp/srv/FitPreview.srv"
  "../assembly_rtfg_cpp/srv/ExecuteCached.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs trajectory_msgs
)

# ---- CUDA core library ----
add_library(rtfg_cuda_core STATIC
  # CPU side: reuse from cpp package
  ../assembly_rtfg_cpp/src/utils.cpp
  ../assembly_rtfg_cpp/src/robot_model.cpp
  ../assembly_rtfg_cpp/src/collision_checker.cpp
  ../assembly_rtfg_cpp/src/collision_pipeline.cpp
  ../assembly_rtfg_cpp/src/continuous_trajectory_solver.cpp
  ../assembly_rtfg_cpp/src/rolling_planner.cpp
  ../assembly_rtfg_cpp/src/trajectory_generator.cpp
  # Modified: CUDA-aware versions
  src/ik_backend.cpp
  src/ik_solver.cpp
  src/trajectory_solver.cpp
  # CUDA sources
  src/cuda/cuda_ik_solver.cu
  src/cuda/cuda_kernels.cu
  src/cuda/cuda_memory.cu
)

target_include_directories(rtfg_cuda_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/assembly_rtfg_cuda>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../assembly_rtfg_cpp/include/assembly_rtfg_cpp>
    ${FCL_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_directories(rtfg_cuda_core PUBLIC ${FCL_LIBRARY_DIRS})
target_link_libraries(rtfg_cuda_core
  Eigen3::Eigen
  yaml-cpp
  ${FCL_LIBRARIES}
  orocos-kdl
  CUDA::cudart
)

ament_target_dependencies(rtfg_cuda_core
  kdl_parser
  urdf
)

# ---- Executable ----
add_executable(rtfg_cuda_solver_node src/rtfg_solver_node.cpp)
target_link_libraries(rtfg_cuda_solver_node rtfg_cuda_core)
ament_target_dependencies(rtfg_cuda_solver_node
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
target_link_libraries(rtfg_cuda_solver_node "${cpp_typesupport_target}")

# ---- Install ----
install(TARGETS rtfg_cuda_core rtfg_cuda_solver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)


# Install shared data from cpp package (config, urdf, rviz, gui)
install(DIRECTORY ../assembly_rtfg_cpp/config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../assembly_rtfg_cpp/urdf
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../assembly_rtfg_cpp/rviz
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../assembly_rtfg_cpp/gui
  DESTINATION share/${PROJECT_NAME})
# Install cuda-specific launch files (modified from cpp version)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
