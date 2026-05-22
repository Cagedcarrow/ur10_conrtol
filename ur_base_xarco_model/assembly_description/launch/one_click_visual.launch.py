from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_joint_gui = LaunchConfiguration('use_joint_gui')
    enable_gazebo_joint_sync = LaunchConfiguration('enable_gazebo_joint_sync')

    world = LaunchConfiguration('world')
    entity_name = LaunchConfiguration('entity_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

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

    rviz_config = PathJoinSubstitution([
        FindPackageShare('assembly_description'), 'rviz', 'assembly.rviz'
    ])

    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
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
        arguments=[
            '-topic', 'robot_description',
            '-entity', entity_name,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_joint_gui),
        parameters=[
            robot_description,
            {
                'use_sim_time': use_sim_time,
                'zeros.ur10_shoulder_pan': 2.0474984645843506,
                'zeros.ur10_shoulder_lift': 0.21928656101226807,
                'zeros.ur10_elbow': -1.9548214117633265,
                'zeros.ur10_wrist_1': -0.35923797289003545,
                'zeros.ur10_wrist_2': 2.0502676963806152,
                'zeros.ur10_wrist_3': 1.0330829620361328,
            }
        ],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    joint_sync_bridge = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('assembly_description'),
                'scripts',
                'joint_state_to_gazebo.py',
            ]),
            '--ros-args',
            '-p', ['model_name:=', entity_name],
        ],
        output='screen',
        condition=IfCondition(enable_gazebo_joint_sync),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_joint_gui', default_value='true'),
        DeclareLaunchArgument('enable_gazebo_joint_sync', default_value='false'),
        DeclareLaunchArgument('entity_name', default_value='assembly_robot_visual'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='1.00'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
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
        joint_state_publisher_gui,
        joint_sync_bridge,
        rviz2,
    ])
