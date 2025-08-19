import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    gazebo_log = LogInfo(msg="Launching Gazebo...")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rocker_description"),
                "launch",
                "gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
    )

    controller_log = LogInfo(msg="Launching Controller...")
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rocker_controller"),
                "launch",
                "controller.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    joystick_log = LogInfo(msg="Launching Joystick Teleop...")
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rocker_controller"),
                "launch",
                "joystick_teleop.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
    )

    rviz_log = LogInfo(msg="Launching RViz (after delay)...")
    rviz = TimerAction(
        period=5.0,  
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution([
                        FindPackageShare("rocker_description"),
                        "rviz",
                        "rviz.rviz"
                    ])
                ],
                output="screen",
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                }],
            )
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo_log,
        gazebo,
        controller_log,
        controller,
        joystick_log,
        joystick,
        rviz_log,
        rviz
    ])
