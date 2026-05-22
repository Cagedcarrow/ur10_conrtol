from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    visual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('assembly_description'), 'launch', 'one_click_visual.launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'visual'"])),
        launch_arguments={
            'gazebo_gui': gazebo_gui,
            'use_rviz': launch_rviz,
            'use_joint_gui': 'true',
            'enable_gazebo_joint_sync': 'false',
            'use_sim_time': use_sim_time,
            'world': world,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('assembly_description'), 'launch', 'gazebo_moveit.launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'moveit'"])),
        launch_arguments={
            'gazebo_gui': gazebo_gui,
            'launch_rviz': launch_rviz,
            'use_sim_time': use_sim_time,
            'world': world,
        }.items(),
    )

    csv_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('assembly_description'), 'launch', 'replay_csv_gazebo.launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'csv_gazebo'"])),
        launch_arguments={
            'gazebo_gui': gazebo_gui,
            'use_rviz': launch_rviz,
            'use_sim_time': use_sim_time,
            'world': world,
        }.items(),
    )

    parametric_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('assembly_description'), 'launch', 'parametric_gui.launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'parametric_gui'"])),
        launch_arguments={
            'gazebo_gui': gazebo_gui,
            'launch_rviz': launch_rviz,
            'use_sim_time': use_sim_time,
            'world': world,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='moveit'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('assembly_description'), 'worlds', 'empty.world'
            ]),
        ),
        visual_launch,
        moveit_launch,
        csv_gazebo_launch,
        parametric_gui_launch,
    ])
