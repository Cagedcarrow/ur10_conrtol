from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    csv_path = LaunchConfiguration('csv_path')
    speed_scale = LaunchConfiguration('speed_scale')
    loop = LaunchConfiguration('loop')
    wait_action_timeout_sec = LaunchConfiguration('wait_action_timeout_sec')
    send_full_trajectory = LaunchConfiguration('send_full_trajectory')
    start_delay_sec = LaunchConfiguration('start_delay_sec')

    default_csv = PathJoinSubstitution([
        FindPackageShare('assembly_description'),
        'replay_data',
        'session_2026-05-01_20-24-59',
        'ur10_ft300_realtime_data.csv',
    ])

    base_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('assembly_description'), 'launch', 'gazebo_moveit.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gazebo_gui': gazebo_gui,
            'launch_rviz': use_rviz,
            'world': world,
        }.items(),
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
            '-p', 'timeout_sec:=90.0',
            '-p', 'poll_interval_sec:=0.5',
        ],
        output='screen',
    )

    replay_script = PathJoinSubstitution([
        FindPackageShare('assembly_description'),
        'scripts',
        'csv_joint_replay.py',
    ])

    replay_csv = ExecuteProcess(
        cmd=[
            'python3',
            replay_script,
            '--ros-args',
            '-p', ['csv_path:=', csv_path],
            '-p', ['speed_scale:=', speed_scale],
            '-p', ['loop:=', loop],
            '-p', 'time_column:=Time',
            '-p', 'output_mode:=trajectory_action',
            '-p', 'follow_joint_trajectory_action:=/joint_trajectory_controller/follow_joint_trajectory',
            '-p', ['wait_action_timeout_sec:=', wait_action_timeout_sec],
            '-p', ['send_full_trajectory:=', send_full_trajectory],
            '-p', ['start_delay_sec:=', start_delay_sec],
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('csv_path', default_value=default_csv),
        DeclareLaunchArgument('speed_scale', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('wait_action_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('send_full_trajectory', default_value='true'),
        DeclareLaunchArgument('start_delay_sec', default_value='0.2'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('assembly_description'),
                'worlds',
                'empty.world',
            ]),
        ),
        base_gazebo,
        TimerAction(period=8.0, actions=[wait_cm]),
        RegisterEventHandler(OnProcessExit(target_action=wait_cm, on_exit=[replay_csv])),
    ])
