#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ]),
        launch_arguments={
            "ur_type": "ur16e",
            "robot_ip": "localhost",
            "use_fake_hardware": "true",
            "initial_joint_controller": "joint_trajectory_controller",
            "launch_rviz":"false",
            "description_file": PathJoinSubstitution([
                FindPackageShare('ur16e_with_gripper_description'), 'urdf', 'ur16e_with_suction_gripper.xacro'
            ]),
        }.items()
    )

    moveit = TimerAction(period=2.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("ur_moveit_rws_gripper_config"), "launch", "ur_moveit.launch.py"])
            ]),
            launch_arguments={
                "ur_type": "ur16e",
                "launch_rviz": "true",
                "description_file": PathJoinSubstitution([
                    FindPackageShare('ur16e_with_gripper_description'), 'urdf', 'ur16e_with_suction_gripper.xacro'
                ]),
            }.items()
        )
    ])

    action_server = TimerAction(period=2.0, actions=[
        Node(
            package="ur16e_move_server",
            executable="move_to_pose_server",
            output="screen",
            parameters=[{
                "planning_group": "ur_manipulator",
                "end_effector_link": "rws_gripper_tcp",
                "approach_height": 0.10,
                "eef_step": 0.005,
            }],
        )
    ])

    return LaunchDescription([ur_control, moveit, action_server])
