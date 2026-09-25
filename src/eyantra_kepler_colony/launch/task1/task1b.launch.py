#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
*****************************************************************************************
*  Filename:       task1b.launch.py
*  Description:    Task 1B -- waypoint manipulation. Brings up the arena and the UR7e on a
*                  clear table, then opens RViz.
*
*                      ros2 launch eyantra_kepler_colony task1b.launch.py
*                      ros2 launch eyantra_kepler_colony task1b.launch.py gui:=false
*
*                  No props and no eBot: the arm is driven to given poses, so nothing needs
*                  to be on the table, and leaving both out saves the contact solving and
*                  a second robot's worth of bridges. KeplerArenaPlugin itself stays fully
*                  disabled (plugin_enabled:=false) for the same reason -- there is no ore,
*                  rock or ore_package state to monitor, and no map or ground truth to
*                  publish, on a clear table.
*
*                  It does NOT run a manipulation node. Driving the arm through the
*                  waypoints is the task; run your own node alongside this:
*
*                      ros2 run <your_package> <your_manipulation_node>
*
*                  What the arm gives you:
*
*                      /tcp_pose_raw                        where the tool tip is now
*                      /ur7e/end_effector/force_magnitude   filtered tool force
*                      /arm_status                          the servo's state
*                      /magnet_status                       is the magnet holding
*                      /magnet                              std_srvs/SetBool, grip/release
*
*                  The Cartesian servo is already running and owns the arm; command it
*                  rather than the controller directly. TF base_link -> tcp resolves.
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
            "plugin_enabled", default_value="false",
            description="Master switch for KeplerArenaPlugin. Off for Task 1B.",
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
        "plugin_enabled",
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "task1b.rviz"])],
    )

    return LaunchDescription(
        declared_arguments + resource_paths + [
            gz_sim,
            clock_bridge,
            TimerAction(period=LaunchConfiguration("arm_delay"), actions=[arm]),
            TimerAction(period=LaunchConfiguration("rviz_delay"), actions=[rviz]),
        ]
    )
