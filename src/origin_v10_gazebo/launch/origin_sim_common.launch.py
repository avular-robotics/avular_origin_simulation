# Copyright 2024 Avular B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("headless", default_value="false"))
    ld.add_action(DeclareLaunchArgument("spawn", default_value="true"))
    ld.add_action(DeclareLaunchArgument("world", default_value="TY_test_area.world"))
    ld.add_action(
        DeclareLaunchArgument(
            "bridge_config", default_value="gazebo_ros_bridge_topics.yaml"
        )
    )
    ld.add_action(
        DeclareLaunchArgument("use_cmd_vel_controller", default_value="False")
    )
    ld.add_action(DeclareLaunchArgument("robot_pose_x", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("robot_pose_y", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("robot_pose_yaw", default_value="0.0"))

    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_dir = get_package_share_directory("origin_v10_gazebo")
    pkg_dir_origin_description = get_package_share_directory("origin_v10_description")

    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (
        os.path.join(pkg_dir, "models")
        + os.pathsep
        + os.path.join(pkg_dir_origin_description, "..")
    )

    world = PathJoinSubstitution([pkg_dir, "worlds", LaunchConfiguration("world")])

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": ["-r -s -v1 ", world],
                "on_exit_shutdown": "true",
            }.items(),
        )
    )

    ld.add_action(
        ExecuteProcess(
            cmd=["ign", "gazebo", "-g", "-v1"],
            condition=UnlessCondition(LaunchConfiguration("headless")),
        )
    )

    sdf_file_path = PathJoinSubstitution(
        [pkg_dir_origin_description, "urdf", "origin_v10.urdf"]
    )

    ld.add_action(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                "Robot",
                "-file",
                sdf_file_path,
                "-x",
                LaunchConfiguration("robot_pose_x"),
                "-y",
                LaunchConfiguration("robot_pose_y"),
                "-z",
                "1.0",
                "-Y",
                LaunchConfiguration("robot_pose_yaw"),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("spawn")),
        )
    )

    bridge_config_file = PathJoinSubstitution(
        [
            pkg_dir,
            "config",
            LaunchConfiguration("bridge_config"),
        ]
    )

    ld.add_action(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[
                {
                    "config_file": bridge_config_file,
                    "qos_overrides./robot/cmd_vel.subscription.reliability": "best_effort",
                }
            ],
        )
    )

    ld.add_action(
        Node(
            package="origin_v10_gazebo",
            executable="robot_specifics",
            arguments=[],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.0",
                "0.0",
                "0.0",
                "0.0",
                "0.0",
                "0.0",
                "camera_link",
                "camera_color_frame",
            ],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.0",
                "0.0",
                "0.0",
                "-1.5707",
                "0.0",
                "-1.5707",
                "camera_color_frame",
                "camera_color_optical_frame",
            ],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.0",
                "0.0",
                "0.0",
                "0.0",
                "0.0",
                "0.0",
                "camera_link",
                "camera_depth_frame",
            ],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.0",
                "0.0",
                "0.0",
                "-1.5707",
                "0.0",
                "-1.5707",
                "camera_color_frame",
                "camera_depth_optical_frame",
            ],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.006253",
                "-0.011775",
                "0.007645",
                "0.0",
                "0.0",
                "0.0",
                "os_sensor",
                "os_imu",
            ],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.0",
                "0.0",
                "0.03618",
                "0.0",
                "0.0",
                "0.0",
                "os_sensor",
                "os_lidar",
            ],
            output="screen",
        )
    )

    # cmd_vel_controller to prioritize control commands
    ld.add_action(
        Node(
            condition=IfCondition(LaunchConfiguration("use_cmd_vel_controller")),
            package="cmd_vel_controller",
            executable="cmd_vel_controller",
            name="cmd_vel_controller",
            namespace="robot",
            parameters=[
                os.path.join(pkg_dir_origin_description, "config", "cmd_vel_controller.yaml"),
                {
                    "use_sim_time": True,
                },
            ],
            respawn=True,
            respawn_delay=1.0,
        )
    )

    return ld
