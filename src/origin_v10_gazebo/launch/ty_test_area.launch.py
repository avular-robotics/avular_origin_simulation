# Copyright 2024 Avular B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    set_world_file_name = SetLaunchConfiguration(
        "world_file_name", "TY_test_area.world"
    )
    robot_pose_x = SetLaunchConfiguration("robot_pose_x", "-18.75")
    robot_pose_y = SetLaunchConfiguration("robot_pose_y", "-34.8")
    robot_pose_yaw = SetLaunchConfiguration("robot_pose_yaw", "1.57")

    launch_origin_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("origin_v10_gazebo"),
                        "launch",
                        "origin_sim_common.launch.py",
                    ]
                )
            ]
        )
    )

    return LaunchDescription(
        [
            set_world_file_name,
            robot_pose_x,
            robot_pose_y,
            robot_pose_yaw,
            launch_origin_common,
        ]
    )
