from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{'use_sim_time': True}],
    )

    simple_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller", 
                   "--controller-manager", 
                   "/controller_manager"],
        parameters=[{'use_sim_time': True}],
    )

    swerve_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_steering_controller", 
                   "--controller-manager", 
                   "/controller_manager"],
        parameters=[{'use_sim_time': True}],
    )

    rocker_right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rocker_right_controller", 
                   "--controller-manager", 
                   "/controller_manager"],
        parameters=[{'use_sim_time': True}],
    )

    rocker_left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rocker_left_controller", 
                   "--controller-manager", 
                   "/controller_manager"],
        parameters=[{'use_sim_time': True}],
    )

    simple_controller = Node(
        package='rocker_controller',
        executable='controller.py',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'wheelbase': 0.4},
            {'track_width': 0.22},
            {'wheel_radius': 0.05},
        ],
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        simple_velocity_controller_spawner,
        swerve_steering_controller_spawner,
        rocker_right_controller_spawner,
        rocker_left_controller_spawner,
        simple_controller,
    ])
