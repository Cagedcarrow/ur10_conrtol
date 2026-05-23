import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _load_text(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def launch_setup(context, *args, **kwargs):
    pkg = "ur10_real_control_ros2"

    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_driver = LaunchConfiguration("launch_driver")
    launch_rsp = LaunchConfiguration("launch_rsp")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    reverse_ip = LaunchConfiguration("reverse_ip")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(pkg), "urdf", "assembly_real.urdf.xacro"]),
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "reverse_ip:=",
            reverse_ip,
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    pkg_share = FindPackageShare(pkg).perform(context)
    robot_description_semantic = {
        "robot_description_semantic": _load_text(os.path.join(pkg_share, "config", "assembly_real.srdf"))
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(os.path.join(pkg_share, "config", "kinematics.yaml"))[
            "/**"
        ]["ros__parameters"]["robot_description_kinematics"]
    }
    robot_description_planning = {
        "robot_description_planning": _load_yaml(os.path.join(pkg_share, "config", "joint_limits.yaml"))[
            "joint_limits"
        ]
    }

    ompl_planning = _load_yaml(os.path.join(pkg_share, "config", "ompl_planning.yaml"))
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }
    planning_pipeline["ompl"].update(ompl_planning)

    moveit_controllers = _load_yaml(os.path.join(pkg_share, "config", "moveit_controllers.yaml"))

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline,
            moveit_controllers,
            {
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "trajectory_execution.execution_duration_monitoring": False,
                "trajectory_execution.allowed_start_tolerance": 0.05,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "allow_trajectory_execution": True,
            },
        ],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        condition=IfCondition(launch_rsp),
        parameters=[robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            robot_description_planning,
        ],
    )

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", "include", "ur_driver_bringup.launch.py"])
        ),
        condition=IfCondition(launch_driver),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "headless_mode": headless_mode,
            "launch_dashboard_client": launch_dashboard_client,
            "reverse_ip": reverse_ip,
        }.items(),
    )

    return [ur_driver, rsp, move_group, rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="10.160.9.21"),
            DeclareLaunchArgument("reverse_ip", default_value="10.160.9.100"),
            DeclareLaunchArgument("ur_type", default_value="ur10"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_driver", default_value="true"),
            DeclareLaunchArgument(
                "launch_rsp",
                default_value="false",
                description="Start a standalone robot_state_publisher. Keep false when ur_robot_driver is running.",
            ),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("launch_dashboard_client", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([FindPackageShare("ur10_real_control_ros2"), "config", "moveit.rviz"]),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
