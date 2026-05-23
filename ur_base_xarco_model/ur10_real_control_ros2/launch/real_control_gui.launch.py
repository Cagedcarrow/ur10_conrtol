import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    workspace = LaunchConfiguration("workspace")
    package_prefix = get_package_prefix("ur10_real_control_ros2")
    gui_executable = os.path.join(package_prefix, "lib", "ur10_real_control_ros2", "real_control_gui.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="10.160.9.21"),
            DeclareLaunchArgument("reverse_ip", default_value="10.160.9.100"),
            DeclareLaunchArgument("workspace", default_value="/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model"),
            ExecuteProcess(
                cmd=[
                    gui_executable,
                    "--robot-ip",
                    robot_ip,
                    "--reverse-ip",
                    reverse_ip,
                    "--workspace",
                    workspace,
                ],
                output="screen",
            ),
        ]
    )
