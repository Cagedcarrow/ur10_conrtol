from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    speed_scale = LaunchConfiguration('speed_scale')
    loop = LaunchConfiguration('loop')
    csv_path = LaunchConfiguration('csv_path')
    publish_hz = LaunchConfiguration('publish_hz')
    output_mode = LaunchConfiguration('output_mode')
    action_name = LaunchConfiguration('follow_joint_trajectory_action')
    wait_action_timeout_sec = LaunchConfiguration('wait_action_timeout_sec')
    send_full_trajectory = LaunchConfiguration('send_full_trajectory')
    start_delay_sec = LaunchConfiguration('start_delay_sec')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('assembly_description'),
        'urdf',
        'assembly.urdf.xacro'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
        ' ',
        'ros_profile:=ros2',
        ' ',
        'ros_hardware_interface:=position'
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    rviz_config = PathJoinSubstitution([
        FindPackageShare('assembly_description'),
        'rviz',
        'assembly.rviz'
    ])

    replay_script = PathJoinSubstitution([
        FindPackageShare('assembly_description'),
        'scripts',
        'csv_joint_replay.py'
    ])

    default_csv = PathJoinSubstitution([
        FindPackageShare('assembly_description'),
        'replay_data',
        'session_2026-05-01_20-24-59',
        'ur10_ft300_realtime_data.csv'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('speed_scale', default_value='1.0'),
        DeclareLaunchArgument('publish_hz', default_value='125.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('csv_path', default_value=default_csv),
        DeclareLaunchArgument('output_mode', default_value='joint_states'),
        DeclareLaunchArgument('follow_joint_trajectory_action', default_value='/joint_trajectory_controller/follow_joint_trajectory'),
        DeclareLaunchArgument('wait_action_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('send_full_trajectory', default_value='true'),
        DeclareLaunchArgument('start_delay_sec', default_value='0.2'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        ExecuteProcess(
            cmd=[
                'python3',
                replay_script,
                '--ros-args',
                '-p', ['csv_path:=', csv_path],
                '-p', ['speed_scale:=', speed_scale],
                '-p', ['publish_hz:=', publish_hz],
                '-p', ['loop:=', loop],
                '-p', 'time_column:=Time',
                '-p', 'publish_topic:=/joint_states',
                '-p', ['output_mode:=', output_mode],
                '-p', ['follow_joint_trajectory_action:=', action_name],
                '-p', ['wait_action_timeout_sec:=', wait_action_timeout_sec],
                '-p', ['send_full_trajectory:=', send_full_trajectory],
                '-p', ['start_delay_sec:=', start_delay_sec],
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz)
        ),
    ])
