#!/usr/bin/env python

"""Launch Webots NAO ROS2 driver - alpha scope (body movement only).

NAO's "vision" comes from recorded video files (see vision_standalone), not a
simulated camera, so this world/URDF has no camera device. Webots drives the
head (HeadYaw/HeadPitch) and both shoulders (LShoulderPitch/LShoulderRoll,
RShoulderPitch/RShoulderRoll) through ros2_control; a camera bridge is a
follow-up once this is confirmed working.

Patches webots_ros2_driver's is_wsl() detection before importing
WebotsLauncher: is_wsl() unconditionally assumes Webots is a Windows install
reached over a shared folder/TCP bridge, which breaks our native Linux Webots
running directly inside WSL2 (it tries to launch a nonexistent webots.exe).
Forcing it False makes WebotsLauncher resolve WEBOTS_HOME and the controller
protocol the same way it would on plain Linux, which matches our setup.
"""

import os

import webots_ros2_driver.utils as webots_ros2_driver_utils
webots_ros2_driver_utils.is_wsl = lambda: False

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('nao_webots_driver')
    robot_description_path = os.path.join(package_dir, 'resource', 'nao_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'nao_ros2.wbt'),
        ros2_supervisor=True
    )

    nao_driver = WebotsController(
        robot_name='NAO',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': True},
            ros2_control_params,
        ],
        respawn=True,
    )

    # Webots can't export NAO's Hinge2Joint-based HeadYaw/HeadPitch to URDF
    # (set_robot_state_publisher's internal auto-export silently never
    # publishes /robot_description for this robot), so controller_manager is
    # fed our own hand-written URDF directly instead.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['cat ', robot_description_path]),
             'use_sim_time': True},
        ],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '500']
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['head_controller'] + controller_manager_timeout,
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['arm_controller'] + controller_manager_timeout,
    )

    # Controller spawners need the driver's ros2_control plugin to be up
    # first, so they're started once WebotsController reports it connected.
    waiting_nodes = WaitForControllerConnection(
        target_driver=nao_driver,
        nodes_to_start=[joint_state_broadcaster_spawner, head_controller_spawner, arm_controller_spawner],
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        robot_state_publisher,
        nao_driver,
        waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])