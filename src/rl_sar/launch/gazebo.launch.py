# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rname = LaunchConfiguration("rname")
    # Hardcode world file name here (without ".world")
    wname = "go2_stairs_18cm"

    robot_name_sub = Command(["echo -n ", rname])
    gazebo_model_name_sub = Command(["echo -n ", rname, "_gazebo"])

    robot_name = ParameterValue(robot_name_sub, value_type=str)
    gazebo_model_name = ParameterValue(gazebo_model_name_sub, value_type=str)

    robot_description = ParameterValue(
        Command([
            "xacro ",
            Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
            "/share/", rname, "_description/xacro/robot.xacro"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(get_package_share_directory("rl_sar"), "worlds", wname + ".world"),
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", gazebo_model_name_sub,
            "-x", "-0.6",
            "-y", "0.0",
            "-z", "0.42",
            # "-R", "0.0",
            # "-P", "0.0",
            # "-Y", "-1.0472",  # 30 deg yaw
            "-timeout", "120",
        ],
        output="screen",
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
        }],
    )

    lidar_preprocessor_node = Node(
        package="rl_sar",
        executable="lidar_obs_preprocessor.py",
        name="lidar_obs_preprocessor",
        output="screen",
        parameters=[{
            "target_frame": "base",
            "expected_points": 45,
            "max_range": 70.0,
        }],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2)",
            default_value=TextSubstitution(text=""),
        ),
        # Avoid remote model DB fetch stalls that can delay /spawn_entity service startup.
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_node,
        joy_node,
        lidar_preprocessor_node,
        param_node,
    ])
