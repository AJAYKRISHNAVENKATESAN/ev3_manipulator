# from launch import LaunchDescription
# from launch.actions import (
#     IncludeLaunchDescription,
#     RegisterEventHandler,
#     SetEnvironmentVariable,
#     TimerAction,
# )

# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from ament_index_python.packages import (
#     get_package_prefix,
#     get_package_share_directory,
# )

# import os


# def generate_launch_description():
#     pkg_name = "ev3_manipulator_live_twin_sync"
#     pkg_share = get_package_share_directory(pkg_name)
    
#     conveyor_plugin_lib = os.path.join(
#     get_package_prefix("ros2_conveyorbelt"),
#     "lib",
#      )

#     model_path = os.path.join(
#         pkg_share,
#         "urdf",
#         "manipulator.urdf.xacro",
#     )

#     robot_description = ParameterValue(
#         Command(
#             [
#                 "xacro ",
#                 model_path,
#             ]
#         ),
#         value_type=str,
#     )

#     # ============================================================
#     # Gazebo
#     # ============================================================

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory("ros_gz_sim"),
#                 "launch",
#                 "gz_sim.launch.py",
#             )
#         ),
#         launch_arguments={
#             "gz_args": "-r empty.sdf",
#         }.items(),
#     )

#     gz_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
#         ],
#         parameters=[
#             {
#                 "use_sim_time": True,
#             }
#         ],
#         output="screen",
#     )

#     # ============================================================
#     # Robot description
#     # ============================================================

#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         parameters=[
#             {
#                 "robot_description": robot_description,
#                 "use_sim_time": True,
#             }
#         ],
#         output="screen",
#     )

#     spawn_node = Node(
#         package="ros_gz_sim",
#         executable="create",
#         arguments=[
#             "-entity",
#             "ev3_manipulator_live_twin_sync",
#             "-topic",
#             "robot_description",
#         ],
#         parameters=[
#             {
#                 "use_sim_time": True,
#             }
#         ],
#         output="screen",
#     )

#     # ============================================================
#     # EV3 TCP -> ROS 2 bridge
#     # ============================================================

#     ev3_hardware_interface = Node(
#         package=pkg_name,
#         executable="hardware_interface",
#         name="hardware_interface",
#         parameters=[
#             {
#                 "port": 5005,

#                 # These names must match the URDF and mirror node.
#                 "base_joint": "arm1_base_link_joint",
#                 "arm_joint": "arm1_arm2_joint",
#                 "gripper_joint": "left_gear_arm4_joint",

#                 "base_gear_ratio": 3.0,
#                 "arm_gear_ratio": 5.0,

#                 # Calibrate these against the physical EV3.
#                 "base_sign": 1.0,
#                 "arm_sign": 1.0,
#                 "gripper_sign": 1.0,
#                 "arm_scale": 0.249,

#                 "base_zero_offset_rad": 0.0,
#                 "arm_zero_offset_rad": -0.2,

#                 # Replace with measured EV3 gripper encoder values.
#                 "gripper_motor_open_deg": 0.0,
#                 "gripper_motor_closed_deg": 90.0,

#                 # These must match the URDF gripper limits.
#                 "gripper_sim_open": -0.35,
#                 "gripper_sim_closed": 0.35,
#             }
#         ],
#         output="screen",
#     )

#     # ============================================================
#     # ros2_control controllers
#     # ============================================================

#     joint_state_broadcaster = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster",
#             "--controller-manager",
#             "/controller_manager",
#             "--controller-manager-timeout",
#             "60",
#         ],
#         output="screen",
#     )

#     twin_position_controller = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "twin_position_controller",
#             "--controller-manager",
#             "/controller_manager",
#             "--controller-manager-timeout",
#             "60",
#         ],
#         output="screen",
#     )

#     # ============================================================
#     # Continuous Gazebo mirror
#     # ============================================================

#     gazebo_state_mirror = Node(
#         package=pkg_name,

#         # Use "sorting_node.py" when installed through:
#         # install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})
#         executable="sorting_node",

#         name="gazebo_state_mirror",
#         parameters=[
#             {
#                 "use_sim_time": True,

#                 # Order must match digital_twin_controllers.yaml.
#                 "position_joints": [
#                     "arm1_base_link_joint",
#                     "arm1_arm2_joint",
#                     "left_gear_arm4_joint",
#                 ],

#                 "position_command_topic": (
#                     "/twin_position_controller/commands"
#                 ),

#                 "command_rate_hz": 50.0,
#                 "state_timeout_sec": 0.5,
#                 "low_pass_alpha": 1.0,

#                 # Simulated ball spawn location.
#                 "spawn_x": -0.154099,
#                 "spawn_y": 0.233,
#                 "spawn_z": 0.0581,
#             }
#         ],
#         output="screen",
#     )

#     # ============================================================
#     # Startup sequence
#     # ============================================================

#     # Wait for the robot to be spawned before loading ros2_control
#     # controllers.
#     controllers_after_spawn = RegisterEventHandler(
#         OnProcessExit(
#             target_action=spawn_node,
#             on_exit=[
#                 TimerAction(
#                     period=2.0,
#                     actions=[
#                         joint_state_broadcaster,
#                     ],
#                 )
#             ],
#         )
#     )

#     # Start the direct position controller after the joint-state
#     # broadcaster has been loaded.
#     position_after_jsb = RegisterEventHandler(
#         OnProcessExit(
#             target_action=joint_state_broadcaster,
#             on_exit=[
#                 twin_position_controller,
#             ],
#         )
#     )

#     # Start mirroring only after the position controller is active.
#     mirror_after_position_controller = RegisterEventHandler(
#         OnProcessExit(
#             target_action=twin_position_controller,
#             on_exit=[
#                 gazebo_state_mirror,
#             ],
#         )
#     )

#     # ============================================================
#     # Launch description
#     # ============================================================

#     return LaunchDescription(
#         [
#             SetEnvironmentVariable(
#                 name="IGN_GAZEBO_RESOURCE_PATH",
#                 value=(
#                     pkg_share
#                     + ":"
#                     + os.path.dirname(pkg_share)
#                 ),
#             ),

#             # The normal Ignition/Gazebo and gz_ros2_control plugin paths.
#             # No conveyor plugin path is needed.
#             # SetEnvironmentVariable(
#             #     name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
#             #     value=(
#             #         "/opt/ros/humble/lib"
#             #         + ":/usr/lib/x86_64-linux-gnu/"
#             #         + "ign-gazebo-6/plugins"
#             #     ),
#             # ),

#             SetEnvironmentVariable(
#                 name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
#                 value=(
#                     conveyor_plugin_lib
#                     + ":/opt/ros/humble/lib"
#                     + ":/usr/lib/x86_64-linux-gnu/"
#                     + "ign-gazebo-6/plugins"
#                 ),
#             ),

#             gazebo,
#             robot_state_publisher,
#             spawn_node,
#             gz_bridge,
#             ev3_hardware_interface,
#             controllers_after_spawn,
#             position_after_jsb,
#             mirror_after_position_controller,
#         ]
#     )



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

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

import os


def generate_launch_description():
    # ============================================================
    # Package paths
    # ============================================================

    pkg_name = "ev3_manipulator_live_twin_sync"

    pkg_share = get_package_share_directory(pkg_name)

    conveyor_plugin_lib = os.path.join(
        get_package_prefix("ros2_conveyorbelt"),
        "lib",
    )

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

    # ============================================================
    # Gazebo <-> ROS bridge
    # ============================================================

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
    # Robot state publisher
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

    # ============================================================
    # Spawn robot in Gazebo
    # ============================================================

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity",
            "ev3_manipulator_live_twin_sync",
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
    # EV3 physical hardware interface
    #
    # EV3 TCP telemetry
    #       ↓
    # canonical physical JointState
    #       ↓
    # /twin/physical/joint_states
    # ============================================================

    ev3_hardware_interface = Node(
        package=pkg_name,
        executable="hardware_interface",
        name="hardware_interface",
        parameters=[
            {
                # ------------------------------------------------
                # TCP
                # ------------------------------------------------

                "port": 5005,

                # ------------------------------------------------
                # Canonical joint names
                #
                # These match the Gazebo URDF.
                # ------------------------------------------------

                "base_joint": "arm1_base_link_joint",
                "arm_joint": "arm1_arm2_joint",
                "gripper_joint": "left_gear_arm4_joint",
                "conveyor_joint": "conveyor_left_pulley_joint",

                # ------------------------------------------------
                # Gear ratios
                # ------------------------------------------------

                "base_gear_ratio": 3.0,
                "arm_gear_ratio": 5.0,
                "conveyor_gear_ratio": 1.0,

                # ------------------------------------------------
                # Physical -> canonical conversion
                #
                # These are your current calibrated values.
                # ------------------------------------------------

                "base_sign": 1.0,
                "arm_sign": 1.0,
                "gripper_sign": 1.0,
                "conveyor_sign": 1.0,

                "arm_scale": 0.249,

                "base_zero_offset_rad": 0.0,
                "arm_zero_offset_rad": -0.2,

                # ------------------------------------------------
                # Safe base operating range
                #
                # -90° ... +90°
                # ------------------------------------------------

                "base_sim_min_rad": -1.57079632679,
                "base_sim_max_rad": 1.57079632679,

                # ------------------------------------------------
                # Gripper calibration
                # ------------------------------------------------

                "gripper_motor_open_deg": 0.0,
                "gripper_motor_closed_deg": 90.0,

                # Keep your CURRENT calibrated values because the
                # simulation is currently following the EV3.
                "gripper_sim_open": -0.35,
                "gripper_sim_closed": 0.35,

                # ------------------------------------------------
                # Digital-twin physical topics
                # ------------------------------------------------

                "physical_joint_state_topic": (
                    "/twin/physical/joint_states"
                ),

                "physical_event_topic": (
                    "/twin/physical/events"
                ),

                "physical_connected_topic": (
                    "/twin/physical/connected"
                ),

                "physical_conveyor_velocity_topic": (
                    "/twin/physical/conveyor_velocity"
                ),

                "physical_raw_state_topic": (
                    "/twin/physical/raw_state"
                ),
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
    # Gazebo twin interface
    #
    # /twin/physical/joint_states
    #               ↓
    #      gazebo_twin_interface
    #               ↓
    # /twin_position_controller/commands
    #               ↓
    #             Gazebo
    #
    # Gazebo /joint_states
    #               ↓
    # /twin/sim/joint_states
    # ============================================================

    gazebo_twin_interface = Node(
        package=pkg_name,

        # Your existing sorting_node.py now contains
        # GazeboTwinInterface.
        executable="sorting_node",

        name="gazebo_twin_interface",

        parameters=[
            {
                "use_sim_time": True,

                # ------------------------------------------------
                # Physical joints to mirror.
                #
                # Order MUST be:
                # [base, arm, gripper]
                #
                # This is also the order expected by
                # twin_position_controller.
                # ------------------------------------------------

                "source_joints": [
                    "arm1_base_link_joint",
                    "arm1_arm2_joint",
                    "left_gear_arm4_joint",
                ],

                # ------------------------------------------------
                # Physical input
                # ------------------------------------------------

                "physical_joint_state_topic": (
                    "/twin/physical/joint_states"
                ),

                "physical_event_topic": (
                    "/twin/physical/events"
                ),

                # ------------------------------------------------
                # Gazebo command
                # ------------------------------------------------

                "position_command_topic": (
                    "/twin_position_controller/commands"
                ),

                # ------------------------------------------------
                # Gazebo measured state
                # ------------------------------------------------

                "gazebo_joint_state_topic": "/joint_states",

                "sim_joint_state_topic": (
                    "/twin/sim/joint_states"
                ),

                # ------------------------------------------------
                # State mirroring
                # ------------------------------------------------

                "command_rate_hz": 50.0,
                "state_timeout_sec": 0.5,
                "low_pass_alpha": 1.0,

                # ------------------------------------------------
                # Ball
                # ------------------------------------------------

                "ball_radius": 0.014,

                "spawn_x": -0.154099,
                "spawn_y": 0.233,
                "spawn_z": 0.0581,

                "pickup_x": -0.020859,

                # ------------------------------------------------
                # Conveyor plugin
                # ------------------------------------------------

                "conveyor_power_service": "/CONVEYORPOWER",

                "conveyor_pickup_power": 25.0,
                "conveyor_black_power": 100.0,
                "conveyor_green_power": -100.0,

                "conveyor_green_tail_sec": 0.5,
                "conveyor_black_tail_sec": 0.5,
            }
        ],

        output="screen",
    )

    # ============================================================
    # Digital twin coordinator
    #
    #          physical state
    #              ↓
    #       twin_coordinator
    #              ↑
    #          simulated state
    #
    # Calculates physical-vs-simulation tracking error.
    # ============================================================

    twin_coordinator = Node(
        package=pkg_name,
        executable="twin_coordinator",
        name="twin_coordinator",

        parameters=[
            {
                # ------------------------------------------------
                # Inputs
                # ------------------------------------------------

                "physical_joint_state_topic": (
                    "/twin/physical/joint_states"
                ),

                "sim_joint_state_topic": (
                    "/twin/sim/joint_states"
                ),

                # ------------------------------------------------
                # Physical joint names
                # ------------------------------------------------

                "physical_joints": [
                    "arm1_base_link_joint",
                    "arm1_arm2_joint",
                    "left_gear_arm4_joint",
                ],

                # ------------------------------------------------
                # Corresponding Gazebo joints
                # ------------------------------------------------

                "sim_joints": [
                    "arm1_base_link_joint",
                    "arm1_arm2_joint",
                    "left_gear_arm4_joint",
                ],

                # ------------------------------------------------
                # Synchronization tolerances
                # ------------------------------------------------

                "base_tolerance_rad": 0.03,
                "arm_tolerance_rad": 0.03,
                "gripper_tolerance_rad": 0.02,

                # State considered stale after this amount of time.
                "state_timeout_sec": 0.5,
            }
        ],

        output="screen",
    )

    # ============================================================
    # Startup sequence
    # ============================================================

    # ------------------------------------------------------------
    # 1. Robot must first be spawned into Gazebo.
    #
    # Once ros_gz_sim/create exits successfully, wait 2 seconds
    # before starting the JointStateBroadcaster.
    # ------------------------------------------------------------

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

    # ------------------------------------------------------------
    # 2. Start position controller after JointStateBroadcaster
    #    spawner has completed.
    # ------------------------------------------------------------

    position_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                twin_position_controller,
            ],
        )
    )

    # ------------------------------------------------------------
    # 3. Once the position controller is available:
    #
    #       start Gazebo twin interface
    #       start twin coordinator
    #
    # ------------------------------------------------------------

    twin_after_position_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=twin_position_controller,
            on_exit=[
                gazebo_twin_interface,
                twin_coordinator,
            ],
        )
    )

    # ============================================================
    # Launch description
    # ============================================================

    return LaunchDescription(
        [
            # ----------------------------------------------------
            # Gazebo resource paths
            # ----------------------------------------------------

            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=(
                    pkg_share
                    + ":"
                    + os.path.dirname(pkg_share)
                ),
            ),

            # ----------------------------------------------------
            # Gazebo system plugins
            #
            # Includes:
            # - custom conveyor plugin
            # - ROS/Gazebo plugins
            # - Ignition Gazebo plugins
            # ----------------------------------------------------

            SetEnvironmentVariable(
                name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                value=(
                    conveyor_plugin_lib
                    + ":/opt/ros/humble/lib"
                    + ":/usr/lib/x86_64-linux-gnu/"
                    + "ign-gazebo-6/plugins"
                ),
            ),

            # ----------------------------------------------------
            # Main processes
            # ----------------------------------------------------

            gazebo,
            robot_state_publisher,
            spawn_node,
            gz_bridge,

            # Physical EV3 interface can start immediately and wait
            # for the EV3 TCP connection.
            ev3_hardware_interface,

            # ----------------------------------------------------
            # Ordered startup
            # ----------------------------------------------------

            controllers_after_spawn,
            position_after_jsb,
            twin_after_position_controller,
        ]
    )