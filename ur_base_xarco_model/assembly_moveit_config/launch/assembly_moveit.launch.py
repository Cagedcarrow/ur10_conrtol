import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _load_text(path):
    with open(path, 'r', encoding='utf-8') as f:
        return f.read()


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    hardware_backend = LaunchConfiguration('hardware_backend')

    moveit_pkg_share = FindPackageShare('assembly_moveit_config').perform(context)

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([FindPackageShare('assembly_description'), 'urdf', 'assembly.urdf.xacro']),
        ' ',
        'ros_profile:=ros2',
        ' ',
        'ros_hardware_interface:=position',
        ' ',
        'hardware_backend:=',
        hardware_backend,
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic = {
        'robot_description_semantic': _load_text(os.path.join(moveit_pkg_share, 'srdf', 'assembly.srdf'))
    }

    robot_description_kinematics = {
        'robot_description_kinematics': _load_yaml(
            os.path.join(moveit_pkg_share, 'config', 'kinematics.yaml')
        )['/**']['ros__parameters']['robot_description_kinematics']
    }

    robot_description_planning = {
        'robot_description_planning': _load_yaml(
            os.path.join(moveit_pkg_share, 'config', 'joint_limits.yaml')
        )['joint_limits']
    }

    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_pipeline_config['ompl'].update(
        _load_yaml(os.path.join(moveit_pkg_share, 'config', 'ompl_planning.yaml'))
    )

    moveit_controllers = {
        'moveit_simple_controller_manager': _load_yaml(
            os.path.join(moveit_pkg_share, 'config', 'moveit_controllers.yaml')
        ),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.03,
        'trajectory_execution.execution_duration_monitoring': False,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': os.path.expanduser('~/.ros/warehouse_assembly.sqlite'),
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
            warehouse_ros_config,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            warehouse_ros_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('hardware_backend', default_value='gazebo'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('assembly_moveit_config'), 'rviz', 'view_robot.rviz']
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
