import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_moveit = LaunchConfiguration("launch_moveit")
    rviz_config = LaunchConfiguration("rviz_config")
    solver_mode = LaunchConfiguration("solver_mode")
    solver_backend = LaunchConfiguration("solver_backend")
    pkg_share = FindPackageShare("assembly_rtfg_cpp").perform(context)

    xacro_path = PathJoinSubstitution([FindPackageShare("assembly_rtfg_cpp"), "urdf", "assembly_rtfg.urdf.xacro"])
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_path,
        " ",
        "mesh_root:=package://assembly_rtfg_cpp/urdf/meshes",
        " ",
        "ros_profile:=ros2",
        " ",
        "ros_hardware_interface:=position",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic = {
        "robot_description_semantic": _load_text(os.path.join(pkg_share, "config", "assembly_rtfg.srdf"))
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(
            os.path.join(pkg_share, "config", "kinematics.yaml")
        )["/**"]["ros__parameters"]["robot_description_kinematics"]
    }
    robot_description_planning = {
        "robot_description_planning": _load_yaml(
            os.path.join(pkg_share, "config", "joint_limits.yaml")
        )["joint_limits"]
    }
    ompl_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_config["ompl"].update(_load_yaml(os.path.join(pkg_share, "config", "ompl_planning.yaml")))
    moveit_controllers = {
        "moveit_simple_controller_manager": _load_yaml(os.path.join(pkg_share, "config", "moveit_controllers.yaml")),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.03,
        "trajectory_execution.execution_duration_monitoring": False,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_assembly_rtfg.sqlite"),
    }
    controllers_yaml = PathJoinSubstitution([FindPackageShare("assembly_rtfg_cpp"), "config", "ros2_controllers.yaml"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="rtfg",
        output="screen",
        parameters=[robot_description, controllers_yaml, {"use_sim_time": use_sim_time}],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/rtfg/controller_manager", "--param-file", controllers_yaml],
        output="screen",
    )
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/rtfg/controller_manager", "--param-file", controllers_yaml],
        output="screen",
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        condition=IfCondition(launch_moveit),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )
    solver_node = Node(
        package="assembly_rtfg_cpp",
        executable="rtfg_solver_node",
        output="screen",
        parameters=[
            {
                "config_path": os.path.join(pkg_share, "config", "environment_runtime_config.yaml"),
                "solver_urdf_path": os.path.join(pkg_share, "urdf", "assembly_rtfg_solver.urdf"),
                "base_link": "base_jizuo",
                "tip_link": "sensor_shovel_tcp",
                "clearance_threshold": 0.002,
                "solver_mode": solver_mode,
                "solver_backend": solver_backend,
                "publish_sparse_posearray_realtime": True,
                "posearray_stride_realtime": 10,
                "controller_action": "/rtfg/joint_trajectory_controller/follow_joint_trajectory",
            }
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_rtfg",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_config,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster, on_exit=[joint_trajectory_controller])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_trajectory_controller, on_exit=[move_group_node, solver_node, rviz_node])
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_moveit", default_value="true"),
        DeclareLaunchArgument("solver_mode", default_value="full"),
        DeclareLaunchArgument("solver_backend", default_value="numeric"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([FindPackageShare("assembly_rtfg_cpp"), "rviz", "rtfg_view.rviz"]),
        ),
        OpaqueFunction(function=launch_setup),
    ])
