cmake_minimum_required(VERSION 3.8)
project(assembly_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf config worlds scripts replay_data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
