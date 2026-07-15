"""Bring up the ev3_manipulator with the working gzfortress conveyor belt.

  ros2 launch ev3_manipulator conveyor.launch.py              # with GUI
  ros2 launch ev3_manipulator conveyor.launch.py headless:=true

Drive the belt:  ros2 service call /CONVEYORPOWER \
    conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 60.0}"

robot_state_publisher is required: gz_ros2_control blocks the sim step until it
can fetch /robot_description, so without rsp the whole world freezes (nothing
falls or moves). rsp here keeps the sim live.
"""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    return [
        gazebo,
        rsp,
        TimerAction(period=3.0, actions=[spawn]),  # let rsp advertise robot_description first
        clock_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
