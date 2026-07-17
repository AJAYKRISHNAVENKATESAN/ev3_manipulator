"""Bring up the ev3_manipulator with the working gzfortress conveyor belt.

  ros2 launch ev3_manipulator conveyor.launch.py              # with GUI
  ros2 launch ev3_manipulator conveyor.launch.py headless:=true

Drive the belt:  ros2 service call /CONVEYORPOWER \
    conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 60.0}"

robot_state_publisher is required: gz_ros2_control blocks the sim step until it
can fetch /robot_description, so without rsp the whole world freezes (nothing
falls or moves). rsp here keeps the sim live.

Controllers: gz_ros2_control loads controller_manager from config/controller.yaml
inside the sim. The spawner chain below activates them in dependency order
(joint_state_broadcaster -> conveyor -> arm -> gripper) once the robot is spawned
and controller_manager is up. sorting_node is NOT launched here yet — run it
separately once the controllers are confirmed active.
"""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawner(name):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager-timeout", "60"],
        parameters=[{"use_sim_time": True}],
    )


def launch_setup(context, *args, **kwargs):
    pkg_path = get_package_share_directory("ev3_manipulator")
    model = os.path.join(pkg_path, "urdf", "manipulator.urdf.xacro")
    world = os.path.join(pkg_path, "worlds", "conveyor.sdf")
    plugin_lib = os.path.join(get_package_prefix("ros2_conveyorbelt"), "lib")

    headless = context.launch_configurations.get("headless", "false") == "true"
    gz_args = ("-s -r -v 3 " if headless else "-r -v 3 ") + world

    # Set on os.environ (not SetEnvironmentVariable) so the gz process launched by
    # the included gz_sim.launch.py reliably inherits the conveyor plugin path.
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = ":".join(filter(None, [
        plugin_lib, "/opt/ros/humble/lib",
        "/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins",
        os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")]))
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ":".join(filter(None, [
        pkg_path, os.path.dirname(pkg_path),
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")]))

    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen")
    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-name", "ev3_manipulator", "-topic", "robot_description"],
        parameters=[{"use_sim_time": True}], output="screen")
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        parameters=[{"use_sim_time": True}], output="screen")

    # Controller spawn chain (proven order from gazebo.launch.py). Each spawner
    # runs only after the previous one exits, so controller_manager loads them
    # one at a time. jsb starts on a timer to let the sim + controller_manager
    # come up after the robot is spawned (spawn fires at 3s).
    jsb = _spawner("joint_state_broadcaster")
    conveyor_ctrl = _spawner("conveyor_controller")
    arm_ctrl = _spawner("arm_controller")
    gripper_ctrl = _spawner("gripper_controller")

    return [
        gazebo,
        rsp,
        TimerAction(period=3.0, actions=[spawn]),  # let rsp advertise robot_description first
        clock_bridge,
        TimerAction(period=7.0, actions=[jsb]),    # after spawn + controller_manager up
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[conveyor_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=conveyor_ctrl, on_exit=[arm_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=arm_ctrl, on_exit=[gripper_ctrl])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
