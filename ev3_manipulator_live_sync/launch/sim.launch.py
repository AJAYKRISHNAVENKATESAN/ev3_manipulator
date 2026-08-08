from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_name = "ev3_manipulator_live_sync"
    pkg_share = get_package_share_directory(pkg_name)

    model_path = os.path.join(
        pkg_share,
        "urdf",
        "manipulator.urdf.xacro",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                model_path,
            ]
        ),
        value_type=str,
    )

    # ------------------------------------------------------------
    # Gazebo
    # ------------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # ------------------------------------------------------------
    # Robot description
    # ------------------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity",
            "ev3_manipulator_live_sync",
            "-topic",
            "robot_description",
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # ------------------------------------------------------------
    # ros2_control
    # ------------------------------------------------------------

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    twin_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "twin_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    # Wait until the robot is spawned before loading controllers.
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster,
                    ],
                )
            ],
        )
    )

    # Start the position controller after the state broadcaster.
    position_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                twin_position_controller,
            ],
        )
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=(
                    pkg_share
                    + ":"
                    + os.path.dirname(pkg_share)
                ),
            ),

            SetEnvironmentVariable(
                name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                value=(
                    "/opt/ros/humble/lib"
                    + ":/usr/lib/x86_64-linux-gnu/"
                    + "ign-gazebo-6/plugins"
                ),
            ),

            gazebo,
            clock_bridge,
            robot_state_publisher,
            spawn_robot,

            controllers_after_spawn,
            position_after_jsb,
        ]
    )