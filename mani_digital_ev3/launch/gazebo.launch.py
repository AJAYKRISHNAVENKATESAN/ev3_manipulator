# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# import xacro
# from os.path import join

# def generate_launch_description():

#     pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
#     pkg_ros_gz_rbot = get_package_share_directory('mani_digital_ev3')


#     robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'manipulator.urdf.xacro')
#     ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'controller.yaml')
    
#     robot_description_config = xacro.process_file(robot_description_file)
#     robot_description = {'robot_description': robot_description_config.toxml()}

   
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description],
#     )

   
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
#         launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items()
#     )

#     spawn_robot = TimerAction(
#         period=5.0,  
#         actions=[Node(
#             package='ros_gz_sim',
#             executable='create',
#             arguments=[
#                 "-topic", "/robot_description",
#                 "-name", "DIGITAL_TWIN_WITH_JOINTS_AJK_ESN_ROBOTICS_FINAL",
#                 "-allow_renaming", "false",  # prevents "_1" duplicate
#                 "-x", "0.0",
#                 "-y", "0.0",
#                 "-z", "0.32",
#                 "-Y", "0.0"
#             ],
#             output='screen'
#         )]
#     )

#     ros_gz_bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         parameters=[{'config_file': ros_gz_bridge_config}],
#         output='screen'
#     )

#     return LaunchDescription([
#         gazebo,
#         spawn_robot,
#         ros_gz_bridge,
#         robot_state_publisher,
#     ])




# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     IncludeLaunchDescription,
#     SetEnvironmentVariable,
#     RegisterEventHandler,
#     TimerAction,
#     ExecuteProcess
# )
# from launch_ros.actions import Node
# from launch.substitutions import Command, LaunchConfiguration
# from launch_ros.parameter_descriptions import ParameterValue
# from ament_index_python.packages import get_package_share_directory
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# import os


# def generate_launch_description():

#     pkg_name = "mani_digital_ev3"
#     pkg_path = get_package_share_directory(pkg_name)
#     model_path = os.path.join(pkg_path, "urdf", "manipulator.urdf.xacro")

#     robot_description = ParameterValue(
#         Command(["xacro ", model_path]),
#         value_type=str
#     )

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory("ros_gz_sim"),
#                 "launch",
#                 "gz_sim.launch.py"
#             )
#         ),
#         launch_arguments={
#             # "-r" runs the simulation immediately on startup
#             "gz_args": "-r empty.sdf"
#             #"gz_args": "-r " + os.path.join(pkg_path, "worlds", "fast_physics.sdf")
#         }.items(),
#     )

#     # ignition = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource(
#     #         os.path.join(
#     #             get_package_share_directory("ros_gz_sim"),
#     #             "launch",
#     #             "gz_sim.launch.py"
#     #         )
#     #     ),
#     #     #  launch_arguments={"gz_args": "-r empty.sdf"}.items(),
#     #    launch_arguments={"gz_args": "-r " + os.path.join(pkg_path, "worlds", "fast_physics.sdf")}.items(),
#     # )


#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[{
#             "robot_description": robot_description,
#             "use_sim_time": True
#         }],
#         output="screen"
#     )

#     spawn_node = Node(
#         package="ros_gz_sim",
#         executable="create",
#         arguments=[
#             "-entity", "mani_digital_ev3", # <-- Change this to "mani_digital_ev3"
#             "-topic", "robot_description"
#         ],
#          parameters=[{"use_sim_time": True}],
#          output="screen"
#     )

#     gz_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
#         ],
#         parameters=[{"use_sim_time": True}],
#         output="screen"
#     )
    
#     joint_state_broadcaster = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster",
#             "--controller-manager-timeout", "60"
#         ],
#         parameters=[{"use_sim_time": True}]
#     )

#     conveyor_controller= Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=[
#             'conveyor_controller',
#             '--controller-manager-timeout', '60'
#         ],
#         parameters=[{'use_sim_time': True}]
#     )

#     # ── 3. Arm controller (arm_1 + arm_2) ───────────────────
#     arm_controller = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=[
#             'arm_controller',
#             '--controller-manager-timeout', '60'
#         ],
#         parameters=[{'use_sim_time': True}]
#     )

#     # ── 4. Gripper controller ────────────────────────────────
#     gripper_controller = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=[
#             'gripper_controller',
#             '--controller-manager-timeout', '60'
#         ],
#         parameters=[{'use_sim_time': True}]
#     )

#     sorting_node = Node(
#         package='mani_digital_ev3',
#         executable='sorting_node',
#         name='sorting_node',
#         parameters=[{'use_sim_time': True}],
#         output='screen'
#     )

#     ev3_hardware_interface = Node(
#             package='mani_digital_ev3',
#             executable='hardware_interface',
#             name='hardware_interface',
#             output='screen'
#     )
    
#     # delay_joint_state_broadcaster = RegisterEventHandler(
#     #     OnProcessExit(
#     #         target_action=spawn_node,
#     #         on_exit=[TimerAction(
#     #                 period=5.0,   # gz_ros2_control needs a moment after spawn
#     #                 actions=[joint_state_broadcaster]
#     #             )
#     #         ]
#     #     )
#     # )

#     # conveyor_after_jsb = RegisterEventHandler(
#     #     OnProcessExit(
#     #         target_action=joint_state_broadcaster,
#     #         on_exit=[conveyor_controller]
#     #     )
#     # )

#     # # Spawn arm ONLY after conveyor finishes
#     # arm_after_conveyor = RegisterEventHandler(
#     #     OnProcessExit(
#     #         target_action=conveyor_controller,
#     #         on_exit=[arm_controller]
            
#     #     )
#     # )

#     # # Spawn gripper ONLY after arm finishes
#     # gripper_after_arm = RegisterEventHandler(
#     #     OnProcessExit(
#     #         target_action=arm_controller,
#     #         on_exit=[gripper_controller]
#     #     )
#     # )

#     # # Start sorting ONLY after gripper finishes
#     # sorting_after_gripper = RegisterEventHandler(
#     #     OnProcessExit(
#     #         target_action=gripper_controller,
#     #         on_exit=[sorting_node]
#     #     )
#     # )

#     # ── Move the broadcasters into a clean timer or clean chain ────
    
#     # Start the joint_state_broadcaster 5 seconds after the launch starts
#     # (Giving Gazebo and the spawn node plenty of time to stabilize)
#     delay_joint_state_broadcaster = TimerAction(
#         period=5.0,
#         actions=[joint_state_broadcaster]
#     )

#     conveyor_after_jsb = RegisterEventHandler(
#         OnProcessExit(
#             target_action=joint_state_broadcaster,
#             on_exit=[conveyor_controller]
#         )
#     )

#     # Spawn arm ONLY after conveyor finishes
#     arm_after_conveyor = RegisterEventHandler(
#         OnProcessExit(
#             target_action=conveyor_controller,
#             on_exit=[arm_controller]
#         )
#     )

#     # Spawn gripper ONLY after arm finishes
#     gripper_after_arm = RegisterEventHandler(
#         OnProcessExit(
#             target_action=arm_controller,
#             on_exit=[gripper_controller]
#         )
#     )

#     # Start sorting ONLY after gripper finishes
#     sorting_after_gripper = RegisterEventHandler(
#         OnProcessExit(
#             target_action=gripper_controller,
#             on_exit=[sorting_node]
#         )
#     )
    
#     return LaunchDescription([
#         SetEnvironmentVariable(
#             name='IGN_GAZEBO_RESOURCE_PATH',
#             value=pkg_path + ":" + os.path.dirname(pkg_path)
#         ),
#         SetEnvironmentVariable(
#         name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
#         value='/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins'
#         ),
#         gazebo,
#         #ignition,
#         robot_state_publisher,
#         spawn_node,
#         gz_bridge,
#         ev3_hardware_interface,
#         delay_joint_state_broadcaster,
#         conveyor_after_jsb,
#         arm_after_conveyor,
#         gripper_after_arm,
#         sorting_after_gripper,
#     ])



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
    pkg_name = "mani_digital_ev3"
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

    # ============================================================
    # Gazebo
    # ============================================================

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

    gz_bridge = Node(
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

    # ============================================================
    # Robot description
    # ============================================================

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

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity",
            "mani_digital_ev3",
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

    # ============================================================
    # EV3 TCP -> ROS 2 bridge
    # ============================================================

    ev3_hardware_interface = Node(
        package=pkg_name,
        executable="hardware_interface",
        name="hardware_interface",
        parameters=[
            {
                "port": 5005,

                # These names must match the URDF and mirror node.
                "base_joint": "arm1_base_link_joint",
                "arm_joint": "arm1_arm2_joint",
                "gripper_joint": "left_gear_arm4_joint",

                "base_gear_ratio": 3.0,
                "arm_gear_ratio": 5.0,

                # Calibrate these against the physical EV3.
                "base_sign": 1.0,
                "arm_sign": 1.0,
                "gripper_sign": 1.0,

                "base_zero_offset_rad": 0.0,
                "arm_zero_offset_rad": 0.0,

                # Replace with measured EV3 gripper encoder values.
                "gripper_motor_open_deg": 0.0,
                "gripper_motor_closed_deg": 90.0,

                # These must match the URDF gripper limits.
                "gripper_sim_open": 0.5,
                "gripper_sim_closed": 0.0,
            }
        ],
        output="screen",
    )

    # ============================================================
    # ros2_control controllers
    # ============================================================

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

    # ============================================================
    # Continuous Gazebo mirror
    # ============================================================

    gazebo_state_mirror = Node(
        package=pkg_name,

        # Use "sorting_node.py" when installed through:
        # install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})
        executable="sorting_node",

        name="gazebo_state_mirror",
        parameters=[
            {
                "use_sim_time": True,

                # Order must match digital_twin_controllers.yaml.
                "position_joints": [
                    "arm1_base_link_joint",
                    "arm1_arm2_joint",
                    "left_gear_arm4_joint",
                ],

                "position_command_topic": (
                    "/twin_position_controller/commands"
                ),

                "command_rate_hz": 50.0,
                "state_timeout_sec": 0.5,
                "low_pass_alpha": 1.0,

                # Simulated ball spawn location.
                "spawn_x": -0.154099,
                "spawn_y": 0.233,
                "spawn_z": 0.042998,
            }
        ],
        output="screen",
    )

    # ============================================================
    # Startup sequence
    # ============================================================

    # Wait for the robot to be spawned before loading ros2_control
    # controllers.
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
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

    # Start the direct position controller after the joint-state
    # broadcaster has been loaded.
    position_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                twin_position_controller,
            ],
        )
    )

    # Start mirroring only after the position controller is active.
    mirror_after_position_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=twin_position_controller,
            on_exit=[
                gazebo_state_mirror,
            ],
        )
    )

    # ============================================================
    # Launch description
    # ============================================================

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

            # The normal Ignition/Gazebo and gz_ros2_control plugin paths.
            # No conveyor plugin path is needed.
            SetEnvironmentVariable(
                name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                value=(
                    "/opt/ros/humble/lib"
                    + ":/usr/lib/x86_64-linux-gnu/"
                    + "ign-gazebo-6/plugins"
                ),
            ),

            gazebo,
            robot_state_publisher,
            spawn_node,
            gz_bridge,
            ev3_hardware_interface,
            controllers_after_spawn,
            position_after_jsb,
            mirror_after_position_controller,
        ]
    )