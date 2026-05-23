from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    reverse_ip = LaunchConfiguration("reverse_ip")

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_fake_hardware": "false",
            "headless_mode": headless_mode,
            "launch_rviz": "false",
            "description_package": "ur10_real_control_ros2",
            "description_file": "assembly_real.urdf.xacro",
            "kinematics_params_file": PathJoinSubstitution(
                [
                    FindPackageShare("ur10_real_control_ros2"),
                    "config",
                    "ur10",
                    "default_kinematics.yaml",
                ]
            ),
            "launch_dashboard_client": launch_dashboard_client,
            "reverse_ip": reverse_ip,
            "reverse_port": "50001",
            "script_sender_port": "50002",
            "trajectory_port": "50003",
            "script_command_port": "50004",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "activate_joint_controller": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur10"),
            DeclareLaunchArgument("robot_ip", default_value="10.160.9.21"),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("launch_dashboard_client", default_value="true"),
            DeclareLaunchArgument("reverse_ip", default_value="10.160.9.100"),
            ur_control,
        ]
    )
