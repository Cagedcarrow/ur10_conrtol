from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([FindPackageShare('assembly_description'), 'urdf', 'assembly.urdf.xacro']),
        ' ',
        'ros_profile:=ros2',
        ' ',
        'ros_hardware_interface:=position'
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('assembly_description'), 'config', 'ros2_controllers.yaml'
    ])

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gazebo_gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'assembly_robot'],
    )

    wait_cm = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('assembly_description'),
                'scripts',
                'wait_controller_manager.py',
            ]),
            '--ros-args',
            '-p', 'service_name:=/controller_manager/list_controllers',
            '-p', 'timeout_sec:=60.0',
            '-p', 'poll_interval_sec:=0.5',
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--param-file', controllers_yaml],
        output='screen',
    )

    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager', '--param-file', controllers_yaml],
        output='screen',
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('assembly_moveit_config'), 'launch', 'assembly_moveit.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': launch_rviz,
            'rviz_config': rviz_config,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('assembly_moveit_config'), 'rviz', 'view_robot.rviz'
            ]),
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('assembly_description'), 'worlds', 'empty.world'
            ]),
        ),
        gzserver,
        gzclient,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[wait_cm])),
        RegisterEventHandler(OnProcessExit(target_action=wait_cm, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[joint_trajectory_controller])),
        RegisterEventHandler(OnProcessExit(target_action=joint_trajectory_controller, on_exit=[moveit_launch])),
    ])
