#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
*****************************************************************************************
*  Filename:       task1a.launch.py
*  Description:    Task 1A -- ore perception. Brings up the arena, the UR7e with its mast
*                  RealSense and the ore props, then opens RViz. No eBot: nothing in this
*                  task drives, and the ores are seen from the mast camera.
*
*                      ros2 launch eyantra_kepler_colony task1a.launch.py
*                      ros2 launch eyantra_kepler_colony task1a.launch.py gui:=false
*
*                  It does NOT run a perception node. Detecting the ores and broadcasting
*                  a TF per ore is the task; run your own node alongside this:
*
*                      ros2 run <your_package> <your_perception_node>
*
*                  What the arena gives you:
*
*                      /camera/camera/color/image_raw                   colour, 640x480
*                      /camera/camera/aligned_depth_to_color/image_raw  depth, 32FC1 m
*                      /camera/camera/color/camera_info                 intrinsics
*
*                  Deproject in camera_color_optical_frame: the frame camera_info's
*                  pinhole model is expressed in, +Z along the view axis, +X right,
*                  +Y down. Every camera message is stamped with it, as a physical
*                  D435i stamps them, the point cloud included.
*
*  Target:         ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8)
*****************************************************************************************
'''

import os
import tempfile

import xacro
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("eyantra_kepler_colony")

    declared_arguments = [
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [pkg_share, "worlds", "eyantra_kepler_world.world.xacro"]
            ),
            description="Absolute path of the SDF world to load. A "
                        "\".xacro\" path is expanded at launch time using "
                        "the plugin feature-toggle arguments below.",
        ),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Run the Gazebo GUI. Set false for headless.",
        ),
        DeclareLaunchArgument(
            "verbosity", default_value="1",
            description="gz sim console verbosity (0-4).",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Open RViz.",
        ),
        DeclareLaunchArgument("arm_delay", default_value="5.0"),
        DeclareLaunchArgument("rviz_delay", default_value="12.0"),
        DeclareLaunchArgument(
            "ores_enabled", default_value="true",
            description="Spawn the six ores and run the ore state-check monitor.",
        ),
    ]

    verbosity = LaunchConfiguration("verbosity")

    server_only_flag = PythonExpression(
        ["'-s ' if '", LaunchConfiguration("gui"), "'.lower() == 'false' else ''"]
    )

    resource_paths = [
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_share, "models"])
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([pkg_share, "models", "rocks"])
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_share, "worlds"])
        ),
    ]

    _WORLD_XACRO_ARGS = (
        "ores_enabled",
    )

    def _launch_gz_sim(context, *_args, **_kwargs):
        world_path = LaunchConfiguration("world_file").perform(context)

        if world_path.endswith(".xacro"):
            mappings = {
                name: LaunchConfiguration(name).perform(context)
                for name in _WORLD_XACRO_ARGS
            }
            expanded_sdf = xacro.process_file(world_path, mappings=mappings).toxml()
            fd, resolved_world_path = tempfile.mkstemp(
                prefix="eyantra_kepler_world_", suffix=".world"
            )
            with os.fdopen(fd, "w") as f:
                f.write(expanded_sdf)
        else:
            resolved_world_path = world_path

        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ),
            launch_arguments={
                "gz_args": [
                    server_only_flag, "-r -v ", verbosity, " ", resolved_world_path,
                ],
                "on_exit_shutdown": "true",
            }.items(),
        )
        return [gz_sim]

    gz_sim = OpaqueFunction(function=_launch_gz_sim)

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "launch", "spawn_ur7e.launch.py"]
            )
        ),
    )

    map_to_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_world",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "world",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "task1a.rviz"])],
    )

    return LaunchDescription(
        declared_arguments + resource_paths + [
            gz_sim,
            clock_bridge,
            map_to_world,
            TimerAction(period=LaunchConfiguration("arm_delay"), actions=[arm]),
            TimerAction(period=LaunchConfiguration("rviz_delay"), actions=[rviz]),
        ]
    )
