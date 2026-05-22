from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    world = LaunchConfiguration('world')
    config_path = LaunchConfiguration('config_path')
    rviz_config = LaunchConfiguration('rviz_config')

    gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('assembly_description'), 'launch', 'gazebo_moveit.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': launch_rviz,
            'gazebo_gui': gazebo_gui,
            'world': world,
            'rviz_config': rviz_config,
        }.items(),
    )

    parametric_server = Node(
        package='assembly_parametric_motion',
        executable='parametric_motion_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_path': config_path,
            'planning_group': 'assembly_manipulator',
            'eef_link': 'sensor_shovel_shovel_tcp',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('assembly_description'), 'worlds', 'empty.world'
            ]),
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('assembly_description'), 'config', 'parametric_experiment.yaml'
            ]),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('assembly_rviz_param_panel'), 'rviz', 'parametric_view.rviz'
            ]),
        ),
        gazebo_moveit,
        TimerAction(period=5.0, actions=[parametric_server]),
    ])
