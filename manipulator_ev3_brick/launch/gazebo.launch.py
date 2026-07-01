
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess
)
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    pkg_name = "manipulator_ev3_brick"
    pkg_path = get_package_share_directory(pkg_name)
    model_path = os.path.join(pkg_path, "urdf", "manipulator.urdf.xacro")

    robot_description = ParameterValue(
        Command(["xacro ", model_path]),
        value_type=str
    )

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        #  launch_arguments={"gz_args": "-r empty.sdf"}.items(),
       launch_arguments={"gz_args": "-r " + os.path.join(pkg_path, "worlds", "fast_physics.sdf")}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/conveyor_belt_vel@std_msgs/msg/Float64]ignition.msgs.Double",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity", "manipulator_ev3_brick",
            "-topic", "robot_description"
        ],
         parameters=[{"use_sim_time": True}],
         output="screen"
     )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "60"
        ],
        parameters=[{"use_sim_time": True}]
    )

    conveyor_controller= Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'conveyor_controller',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # ── 3. Arm controller (arm_1 + arm_2) ───────────────────
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # ── 4. Gripper controller ────────────────────────────────
    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}]
    )

    sorting_node = Node(
        package='manipulator_ev3_brick',
        executable='sorting_node',
        name='sorting_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
            on_exit=[TimerAction(
                    period=5.0,   # gz_ros2_control needs a moment after spawn
                    actions=[joint_state_broadcaster]
                )
            ]
        )
    )

    conveyor_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[conveyor_controller]
        )
    )

    # Spawn arm ONLY after conveyor finishes
    arm_after_conveyor = RegisterEventHandler(
        OnProcessExit(
            target_action=conveyor_controller,
            on_exit=[arm_controller]
            
        )
    )

    # Spawn gripper ONLY after arm finishes
    gripper_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller,
            on_exit=[gripper_controller]
        )
    )

    # Start sorting ONLY after gripper finishes
    sorting_after_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller,
            on_exit=[sorting_node]
        )
    )
    
    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.path.dirname(pkg_path)
        ),
        SetEnvironmentVariable(                         
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
        ),
        ignition,
        robot_state_publisher,
        gz_bridge,
        spawn_node,
        delay_joint_state_broadcaster,
        conveyor_after_jsb,
        arm_after_conveyor,
        gripper_after_arm,
        sorting_after_gripper,
    ])
