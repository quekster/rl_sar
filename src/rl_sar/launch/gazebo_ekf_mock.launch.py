# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    rl_sar_share = get_package_share_directory("rl_sar")
    base_launch_path = os.path.join(rl_sar_share, "launch", "gazebo.launch.py")
    default_ekf_params = os.path.join(rl_sar_share, "config", "ekf_go2.yaml")

    rname = LaunchConfiguration("rname")
    gazebo_model_name = LaunchConfiguration("gazebo_model_name")
    ekf_params = LaunchConfiguration("ekf_params")

    gazebo_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments={"rname": rname}.items(),
    )

    mock_odom_bridge = Node(
        package="rl_sar",
        executable="gazebo_mock_odom_bridge.py",
        name="gazebo_mock_odom_bridge",
        output="screen",
        parameters=[
            {
                "gazebo_model_name": gazebo_model_name,
                "input_topic": "/gazebo/model_states",
                "output_topic": "/ekf/input/odom",
                "odom_frame": "odom",
                "base_frame": "base",
            }
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rname",
                default_value=TextSubstitution(text="go2"),
                description="Robot name (e.g., a1, go2)",
            ),
            DeclareLaunchArgument(
                "gazebo_model_name",
                default_value=[rname, TextSubstitution(text="_gazebo")],
                description="Model name in /gazebo/model_states",
            ),
            DeclareLaunchArgument(
                "ekf_params",
                default_value=TextSubstitution(text=default_ekf_params),
                description="Path to EKF parameters YAML file",
            ),
            gazebo_stack,
            mock_odom_bridge,
            ekf_node,
        ]
    )
