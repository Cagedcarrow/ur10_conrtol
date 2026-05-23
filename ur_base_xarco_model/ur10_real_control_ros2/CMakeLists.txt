cmake_minimum_required(VERSION 3.8)
project(ur10_real_control_ros2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch docs urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_real_ur10_ready.sh
  scripts/cleanup_real_control_processes.sh
  scripts/real_control_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
