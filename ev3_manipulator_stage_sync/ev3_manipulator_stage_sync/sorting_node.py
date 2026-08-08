#!/usr/bin/env python3
# """Gazebo follower for the stage-synchronised EV3 sorter.

# - Full initial homing.
# - Full homing after every RED or BLUE pick-and-place.
# - GREEN and BLACK use conveyor routing and centre hold only.
# - Ball colours may arrive in any order.
# - Homing is sent as one multi-point trajectory for smooth interpolation.
# """

# import math
# import queue
# import subprocess
# import threading
# import time

# import rclpy
# from action_msgs.msg import GoalStatus
# from builtin_interfaces.msg import Duration
# from control_msgs.action import FollowJointTrajectory
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from std_msgs.msg import Float64, Float64MultiArray, String
# from trajectory_msgs.msg import JointTrajectoryPoint


# # ==================================================
# # SIMULATION GEOMETRY AND TARGETS
# # ==================================================

# SPAWN_X = -0.14824
# SPAWN_Y = 0.29075
# SPAWN_Z = 0.09232
# PICKUP_Z = SPAWN_Z + 0.022

# PICKUP_X = -0.015
# TRANSPORT_Z = SPAWN_Z + 0.005

# BELT_END_R = 0.17
# BELT_END_L = -0.17

# # Restored exactly from the earlier reliable ball-motion script.
# STEP_SIZE = 0.008
# STEP_DELAY = 0.090

# SIM_BASE_HOME = 0.0
# SIM_BASE_RED_BIN = -1.5708
# SIM_BASE_BLUE_BIN = 1.5708

# THETA1_MIN = -math.pi / 2
# THETA1_MAX = math.pi / 2
# THETA2_MIN = -0.55
# THETA2_MAX = math.pi / 3

# CLEARANCE_PITCH = 0.2

# # Restored from the earlier working simulation script.
# #
# # If the current URDF/controller visibly behaves in the opposite direction,
# # swap only these two constants; no state-machine change is required.
# SIM_GRIPPER_OPEN = 0.5
# SIM_GRIPPER_CLOSE = 0.0


# # ==================================================
# # ORIGINAL SIMULATION TIMINGS
# # ==================================================

# # Smooth homing trajectories use cumulative wall-clock targets.
# #
# # Measured physical EV3 values from previous runs:
# #   initial home       ~13.1 s
# #   home after RED     ~16.2 s
# #   home after BLUE    ~11.0 s
# # Tuned from the most recent synchronized timestamp log.
# #
# # Latest measured gaps:
# #   HOME_INITIAL: sim 0.555 s late
# #   HOME_AFTER_RED: sim 1.771 s late
# #
# # HOME_AFTER_BLUE is based on the earlier valid blue-home measurement.
# # Retuned from the latest complete RED/GREEN/BLUE/BLACK run.
# #
# # Observed EV3 vs simulation homing durations:
# #   initial:    EV3 ~6.90 s, sim ~13.46 s
# #   after red:  EV3 ~9.28 s, sim ~15.35 s
# #   after blue: EV3 ~4.75 s, sim ~10.41 s
# #
# # Controller overhead adds roughly 0.9-1.3 s beyond the final trajectory time.
# SIM_HOME_INITIAL_TIMES = [0.6, 3.4, 5.6]
# SIM_HOME_AFTER_RED_TIMES = [5.4, 7.9]
# SIM_HOME_AFTER_BLUE_TIMES = [1.4, 3.8]

# SIM_PICKUP_READY_TIME = 0.18
# SIM_PICK_DOWN_TIME = 1.40
# SIM_PICK_UP_TIME = 0.85

# SIM_ROTATE_RED_TIME = 0.65
# SIM_ROTATE_BLUE_TIME = 0.50

# SIM_DROP_DOWN_RED_TIME = 0.80
# SIM_DROP_DOWN_BLUE_TIME = 1.10
# SIM_DROP_UP_TIME = 1.60

# SIM_REJECT_CENTER_HOLD_TIME = 0.25

# SIM_GRIPPER_READY_SETTLE = 0.15
# GRIPPER_CLOSE_SETTLE_TIME = 1.25

# GRIPPER_RELEASE_WAIT_1 = 0.45
# GRIPPER_RELEASE_WAIT_2 = 1.00


# # ==================================================
# # BALL MODEL
# # ==================================================

# BALL_RGB = {
#     "red": (1.0, 0.0, 0.0),
#     "blue": (0.0, 0.0, 1.0),
#     "black": (0.05, 0.05, 0.05),
#     "green": (0.0, 0.8, 0.0),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia>
#           <ixx>8e-06</ixx>
#           <iyy>8e-06</iyy>
#           <izz>8e-06</izz>
#         </inertia>
#       </inertial>
#       <collision name='col'>
#         <geometry>
#           <sphere>
#             <radius>0.0185</radius>
#           </sphere>
#         </geometry>
#         <surface>
#           <friction>
#             <ode>
#               <mu>0.7</mu>
#               <mu2>0.7</mu2>
#             </ode>
#           </friction>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry>
#           <sphere>
#             <radius>0.0185</radius>
#           </sphere>
#         </geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# def solve_ik_sim(x, y, z_target):
#     theta1 = math.atan2(x, y)

#     z0 = 0.220995
#     r_arm = 0.226963

#     sin_t2 = (z_target - z0) / r_arm
#     sin_t2 = max(-1.0, min(1.0, sin_t2))

#     theta2 = math.asin(sin_t2)

#     theta1 = max(
#         THETA1_MIN,
#         min(THETA1_MAX, theta1),
#     )

#     theta2 = max(
#         THETA2_MIN,
#         min(THETA2_MAX, theta2),
#     )

#     return theta1, theta2


# class SortingNode(Node):
#     def __init__(self):
#         super().__init__("sorting_node")

#         self.arm_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             "/arm_controller/follow_joint_trajectory",
#         )

#         self.gripper_pub = self.create_publisher(
#             Float64MultiArray,
#             "/gripper_controller/commands",
#             10,
#         )

#         self.belt_vel_pub = self.create_publisher(
#             Float64,
#             "/conveyor_belt_vel",
#             10,
#         )

#         self.stage_event_sub = self.create_subscription(
#             String,
#             "/hw/stage_event",
#             self.stage_event_callback,
#             20,
#         )

#         self.stage_sync_pub = self.create_publisher(
#             String,
#             "/sim/stage_sync",
#             20,
#         )

#         self.event_queue = queue.Queue()
#         self.shutdown_event = threading.Event()

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#         self.current_ball_name = None
#         self.last_bin_target = SIM_BASE_HOME

#         self.worker_thread = threading.Thread(
#             target=self.worker_loop,
#             daemon=True,
#         )

#         self.worker_thread.start()

#     # ==================================================
#     # PROTOCOL HELPERS
#     # ==================================================

#     @staticmethod
#     def parse_stage_line(line, expected_kind):
#         parts = line.split("|")

#         if len(parts) != 4:
#             raise ValueError(
#                 "Invalid stage packet: {}".format(line)
#             )

#         if parts[0] != expected_kind:
#             raise ValueError(
#                 "Expected {}, got {}".format(
#                     expected_kind,
#                     line,
#                 )
#             )

#         return (
#             int(parts[1]),
#             int(parts[2]),
#             parts[3],
#         )

#     @staticmethod
#     def make_stage_line(
#         kind,
#         cycle_id,
#         sequence_id,
#         stage,
#     ):
#         return "{}|{}|{}|{}".format(
#             kind,
#             cycle_id,
#             sequence_id,
#             stage,
#         )

#     def publish_stage_sync(self, text):
#         msg = String()
#         msg.data = text

#         self.stage_sync_pub.publish(msg)

#         self.get_logger().info(
#             "-> EV3 [{}]".format(text)
#         )

#     def stage_event_callback(self, msg):
#         self.event_queue.put(msg.data.strip())

#     # ==================================================
#     # WORKER AND STAGE BARRIER
#     # ==================================================

#     def worker_loop(self):
#         self.get_logger().info(
#             "Waiting for arm action server..."
#         )

#         if not self.arm_client.wait_for_server(
#             timeout_sec=30.0
#         ):
#             self.get_logger().error(
#                 "Arm action server was not available."
#             )
#             return

#         self.get_logger().info(
#             "Arm action server available."
#         )

#         self.start_conveyor_visual(0.05)

#         while (
#             rclpy.ok()
#             and not self.shutdown_event.is_set()
#         ):
#             try:
#                 line = self.event_queue.get(
#                     timeout=0.2
#                 )
#             except queue.Empty:
#                 continue

#             try:
#                 if line.startswith("STAGE_START|"):
#                     self.handle_stage_start(line)

#                 elif line.startswith("STAGE_HW_DONE|"):
#                     self.handle_hardware_done(line)

#                 elif line == "EV3_DONE":
#                     self.get_logger().info(
#                         "EV3 completed the full task."
#                     )

#                     self.start_conveyor_visual(0.0)

#                 else:
#                     self.get_logger().warn(
#                         "Ignoring unknown stage event: {}".format(
#                             line
#                         )
#                     )

#             except Exception as exc:
#                 self.get_logger().error(
#                     "Error while processing [{}]: {}".format(
#                         line,
#                         exc,
#                     )
#                 )

#                 self.publish_failure_for_active_stage(
#                     str(exc)
#                 )

#     def handle_stage_start(self, line):
#         (
#             cycle_id,
#             sequence_id,
#             stage,
#         ) = self.parse_stage_line(
#             line,
#             "STAGE_START",
#         )

#         key = (
#             cycle_id,
#             sequence_id,
#             stage,
#         )

#         if self.active_key is not None:
#             raise RuntimeError(
#                 "Received {}, but {} is active.".format(
#                     key,
#                     self.active_key,
#                 )
#             )

#         self.active_key = key
#         self.sim_done = False
#         self.hardware_done = False

#         self.get_logger().info(
#             "=== START cycle={} seq={} stage={} ===".format(
#                 cycle_id,
#                 sequence_id,
#                 stage,
#             )
#         )

#         self.execute_sim_stage(
#             cycle_id,
#             stage,
#         )

#         self.sim_done = True

#         self.try_complete_active_stage()

#     def handle_hardware_done(self, line):
#         (
#             cycle_id,
#             sequence_id,
#             stage,
#         ) = self.parse_stage_line(
#             line,
#             "STAGE_HW_DONE",
#         )

#         key = (
#             cycle_id,
#             sequence_id,
#             stage,
#         )

#         if self.active_key != key:
#             raise RuntimeError(
#                 "Hardware completed {}, active stage is {}.".format(
#                     key,
#                     self.active_key,
#                 )
#             )

#         self.hardware_done = True

#         self.get_logger().info(
#             "Hardware completed cycle={} seq={} stage={}.".format(
#                 cycle_id,
#                 sequence_id,
#                 stage,
#             )
#         )

#         self.try_complete_active_stage()

#     def try_complete_active_stage(self):
#         if not (
#             self.sim_done
#             and self.hardware_done
#         ):
#             return

#         (
#             cycle_id,
#             sequence_id,
#             stage,
#         ) = self.active_key

#         response = self.make_stage_line(
#             "STAGE_SYNC_DONE",
#             cycle_id,
#             sequence_id,
#             stage,
#         )

#         self.publish_stage_sync(response)

#         self.get_logger().info(
#             "=== DONE cycle={} seq={} stage={} ===".format(
#                 cycle_id,
#                 sequence_id,
#                 stage,
#             )
#         )

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#     def publish_failure_for_active_stage(self, reason):
#         if self.active_key is None:
#             return

#         (
#             cycle_id,
#             sequence_id,
#             stage,
#         ) = self.active_key

#         safe_reason = (
#             reason
#             .replace("|", "/")
#             .replace("\n", " ")
#         )

#         response = (
#             "STAGE_SYNC_FAILED|{}|{}|{}|{}"
#             .format(
#                 cycle_id,
#                 sequence_id,
#                 stage,
#                 safe_reason,
#             )
#         )

#         self.publish_stage_sync(response)

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#     # ==================================================
#     # STAGE IMPLEMENTATION
#     # ==================================================

#     def execute_sim_stage(self, cycle_id, stage):
#         if stage == "HOME_INITIAL":
#             self.execute_sim_homing("initial")
#             return

#         if stage == "HOME_AFTER_RED":
#             self.execute_sim_homing("after_red")
#             return

#         if stage == "HOME_AFTER_BLUE":
#             self.execute_sim_homing("after_blue")
#             return

#         if stage.startswith("SPAWN_"):
#             color = stage.split("_", 1)[1].lower()

#             self.current_ball_name = self.spawn_ball(
#                 color,
#                 cycle_id,
#             )

#             return

#         if stage == "CONVEYOR_TO_PICKUP":
#             self.require_current_ball()

#             self.move_ball(
#                 self.current_ball_name,
#                 SPAWN_X,
#                 PICKUP_X,
#             )

#             return

#         if stage == "PICKUP_READY":
#             self.send_arm_target(
#                 [
#                     SIM_BASE_HOME,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_PICKUP_READY_TIME,
#             )

#             return

#         if stage == "GRIPPER_READY":
#             self.command_gripper(
#                 SIM_GRIPPER_OPEN
#             )

#             time.sleep(SIM_GRIPPER_READY_SETTLE)

#             return

#         if stage == "PICK_DOWN":
#             _theta1, pick_pitch = solve_ik_sim(
#                 PICKUP_X,
#                 SPAWN_Y,
#                 PICKUP_Z,
#             )

#             self.send_arm_target(
#                 [
#                     SIM_BASE_HOME,
#                     pick_pitch,
#                 ],
#                 SIM_PICK_DOWN_TIME,
#             )

#             return

#         if stage == "GRIP_CLOSE":
#             self.command_gripper(
#                 SIM_GRIPPER_CLOSE
#             )

#             time.sleep(
#                 GRIPPER_CLOSE_SETTLE_TIME
#             )

#             return

#         if stage == "PICK_UP":
#             self.send_arm_target(
#                 [
#                     SIM_BASE_HOME,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_PICK_UP_TIME,
#             )

#             return

#         if stage == "ROTATE_RED":
#             self.last_bin_target = (
#                 SIM_BASE_RED_BIN
#             )

#             self.send_arm_target(
#                 [
#                     SIM_BASE_RED_BIN,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_ROTATE_RED_TIME,
#             )

#             return

#         if stage == "ROTATE_BLUE":
#             self.last_bin_target = (
#                 SIM_BASE_BLUE_BIN
#             )

#             self.send_arm_target(
#                 [
#                     SIM_BASE_BLUE_BIN,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_ROTATE_BLUE_TIME,
#             )

#             return

#         if stage == "PLACE_DOWN":
#             if self.last_bin_target == SIM_BASE_RED_BIN:
#                 duration = SIM_DROP_DOWN_RED_TIME
#             else:
#                 duration = SIM_DROP_DOWN_BLUE_TIME

#             self.send_arm_target(
#                 [
#                     self.last_bin_target,
#                     -0.45,
#                 ],
#                 duration,
#             )

#             return

#         if stage == "GRIP_RELEASE":
#             # Original two-step release behaviour.
#             self.command_gripper(0.2)
#             time.sleep(GRIPPER_RELEASE_WAIT_1)

#             self.command_gripper(
#                 SIM_GRIPPER_OPEN
#             )
#             time.sleep(GRIPPER_RELEASE_WAIT_2)

#             return

#         if stage == "PLACE_UP":
#             self.send_arm_target(
#                 [
#                     self.last_bin_target,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_DROP_UP_TIME,
#             )

#             return

#         if stage == "CONVEYOR_BLACK":
#             self.require_current_ball()

#             self.fall_ball(
#                 self.current_ball_name,
#                 direction=1,
#             )

#             return

#         if stage == "CONVEYOR_GREEN":
#             self.require_current_ball()

#             self.fall_ball(
#                 self.current_ball_name,
#                 direction=-1,
#             )

#             return

#         if stage == "CENTER_HOLD":
#             self.get_logger().info(
#                 "Centre hold: maintaining the centred clearance pose."
#             )

#             self.last_bin_target = SIM_BASE_HOME
#             time.sleep(SIM_REJECT_CENTER_HOLD_TIME)

#             return

#         if stage == "CYCLE_COMPLETE":
#             self.current_ball_name = None
#             self.last_bin_target = SIM_BASE_HOME

#             self.get_logger().info(
#                 "Simulation cycle state cleared."
#             )

#             return

#         raise ValueError(
#             "No simulation implementation for stage {}".format(
#                 stage
#             )
#         )

#     # ==================================================
#     # SIMULATED HOMING
#     # ==================================================

#     def execute_sim_homing(self, profile):
#         """Run one continuous multi-point homing trajectory."""

#         if profile == "initial":
#             # Raise once, sweep to the simulated switch, then return to centre.
#             positions = [
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#                 [1.0472, CLEARANCE_PITCH],
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#             ]
#             cumulative_times = SIM_HOME_INITIAL_TIMES

#         elif profile == "after_red":
#             # PLACE_UP already left the pitch at clearance.
#             # Keep it there while the base performs the homing sweep.
#             positions = [
#                 [1.0472, CLEARANCE_PITCH],
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#             ]
#             cumulative_times = SIM_HOME_AFTER_RED_TIMES

#         elif profile == "after_blue":
#             # PLACE_UP already left the pitch at clearance.
#             positions = [
#                 [1.0472, CLEARANCE_PITCH],
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#             ]
#             cumulative_times = SIM_HOME_AFTER_BLUE_TIMES

#         else:
#             raise ValueError(
#                 "Unknown homing profile: {}".format(profile)
#             )

#         self.get_logger().info(
#             "Executing smooth {} homing trajectory; total {:.2f} s."
#             .format(profile, cumulative_times[-1])
#         )

#         self.send_arm_trajectory(
#             positions,
#             cumulative_times,
#         )

#         self.last_bin_target = SIM_BASE_HOME

#         self.get_logger().info(
#             "Simulated {} homing complete."
#             .format(profile)
#         )

#     # ==================================================
#     # CONTROLLER HELPERS
#     # ==================================================

#     def wait_for_future(
#         self,
#         future,
#         timeout_sec,
#     ):
#         event = threading.Event()

#         future.add_done_callback(
#             lambda _future: event.set()
#         )

#         if not event.wait(timeout_sec):
#             raise TimeoutError(
#                 "ROS action future timed out."
#             )

#         return future.result()

#     def send_arm_trajectory(
#         self,
#         positions,
#         cumulative_times,
#     ):
#         if len(positions) != len(cumulative_times):
#             raise ValueError(
#                 "positions and cumulative_times must have equal length."
#             )

#         if not positions:
#             raise ValueError("Trajectory must contain at least one point.")

#         goal = FollowJointTrajectory.Goal()

#         goal.trajectory.joint_names = [
#             "arm_1_base_link_joint",
#             "arm_2_left_arm_linkage_joint",
#         ]

#         previous_time = 0.0

#         for position, cumulative_time in zip(
#             positions,
#             cumulative_times,
#         ):
#             if cumulative_time <= previous_time:
#                 raise ValueError(
#                     "Trajectory times must be strictly increasing."
#                 )

#             point = JointTrajectoryPoint()
#             point.positions = [
#                 float(value)
#                 for value in position
#             ]

#             point.time_from_start = Duration(
#                 sec=int(cumulative_time),
#                 nanosec=int(
#                     (
#                         cumulative_time
#                         - int(cumulative_time)
#                     )
#                     * 1e9
#                 ),
#             )

#             goal.trajectory.points.append(point)
#             previous_time = cumulative_time

#         goal_future = self.arm_client.send_goal_async(goal)

#         goal_handle = self.wait_for_future(
#             goal_future,
#             timeout_sec=3.0,
#         )

#         if (
#             goal_handle is None
#             or not goal_handle.accepted
#         ):
#             raise RuntimeError(
#                 "Trajectory goal was rejected."
#             )

#         result_future = goal_handle.get_result_async()

#         wrapped_result = self.wait_for_future(
#             result_future,
#             timeout_sec=cumulative_times[-1] + 5.0,
#         )

#         if (
#             wrapped_result.status
#             != GoalStatus.STATUS_SUCCEEDED
#         ):
#             raise RuntimeError(
#                 "Trajectory action status was {}."
#                 .format(
#                     wrapped_result.status
#                 )
#             )

#         result = wrapped_result.result

#         if (
#             result.error_code
#             != FollowJointTrajectory.Result.SUCCESSFUL
#         ):
#             raise RuntimeError(
#                 "Trajectory controller error {}: {}"
#                 .format(
#                     result.error_code,
#                     result.error_string,
#                 )
#             )

#     def send_arm_target(
#         self,
#         positions,
#         duration_sec,
#     ):
#         self.send_arm_trajectory(
#             [positions],
#             [duration_sec],
#         )

#     def command_gripper(self, position):
#         msg = Float64MultiArray()

#         msg.data = [float(position)]

#         self.gripper_pub.publish(msg)

#     def start_conveyor_visual(self, speed):
#         msg = Float64()

#         msg.data = float(speed)

#         self.belt_vel_pub.publish(msg)

#     # ==================================================
#     # ORIGINAL BALL SPAWN AND SET-POSE MOVEMENT
#     # ==================================================

#     def require_current_ball(self):
#         if not self.current_ball_name:
#             raise RuntimeError(
#                 "No active simulated ball exists."
#             )

#     def move_ball(
#         self,
#         name,
#         x_start,
#         x_end,
#     ):
#         x = x_start

#         while x < x_end:
#             x = min(
#                 x + STEP_SIZE,
#                 x_end,
#             )

#             self.set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 TRANSPORT_Z,
#             )

#             time.sleep(STEP_DELAY)

#     def fall_ball(
#         self,
#         name,
#         direction,
#     ):
#         if direction < 0:
#             x_end = BELT_END_L - 0.06
#         else:
#             x_end = BELT_END_R + 0.06

#         x = SPAWN_X
#         z = TRANSPORT_Z

#         step = STEP_SIZE * direction

#         while (
#             (
#                 direction > 0
#                 and x < x_end
#             )
#             or
#             (
#                 direction < 0
#                 and x > x_end
#             )
#         ):
#             x += step

#             if (
#                 (
#                     direction > 0
#                     and x > BELT_END_R
#                 )
#                 or
#                 (
#                     direction < 0
#                     and x < BELT_END_L
#                 )
#             ):
#                 z = max(
#                     z - 0.006,
#                     -0.05,
#                 )

#             self.set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 z,
#             )

#             time.sleep(STEP_DELAY)

#     def set_pose(
#         self,
#         name,
#         x,
#         y,
#         z,
#     ):
#         # Kept intentionally identical in behaviour to the old script:
#         # short timeout + asynchronous subprocess.
#         command = [
#             "ign",
#             "service",
#             "-s",
#             "/world/empty/set_pose",
#             "--reqtype",
#             "ignition.msgs.Pose",
#             "--reptype",
#             "ignition.msgs.Boolean",
#             "--timeout",
#             "150",
#             "--req",
#             (
#                 'name: "{}" '
#                 "position: {{x: {:.5f} y: {:.5f} z: {:.5f}}}"
#                 .format(
#                     name,
#                     x,
#                     y,
#                     z,
#                 )
#             ),
#         ]

#         subprocess.Popen(
#             command,
#             stdout=subprocess.DEVNULL,
#             stderr=subprocess.DEVNULL,
#         )

#     def spawn_ball(
#         self,
#         color,
#         cycle_id,
#         retries=3,
#     ):
#         if color not in BALL_RGB:
#             raise ValueError(
#                 "Unsupported ball colour: {}".format(
#                     color
#                 )
#             )

#         name = "{}_ball_{}".format(
#             color,
#             cycle_id,
#         )

#         r, g, b = BALL_RGB[color]

#         sdf = BALL_SDF.format(
#             name=name,
#             r=r,
#             g=g,
#             b=b,
#         )

#         command = [
#             "ros2",
#             "run",
#             "ros_gz_sim",
#             "create",
#             "-name",
#             name,
#             "-x",
#             str(SPAWN_X),
#             "-y",
#             str(SPAWN_Y),
#             "-z",
#             str(SPAWN_Z),
#             "-string",
#             sdf,
#         ]

#         for attempt in range(
#             1,
#             retries + 1,
#         ):
#             try:
#                 result = subprocess.run(
#                     command,
#                     capture_output=True,
#                     text=True,
#                     timeout=10,
#                     check=False,
#                 )

#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     "Spawn attempt {} timed out."
#                     .format(attempt)
#                 )

#                 time.sleep(1.0)

#                 continue

#             if result.returncode == 0:
#                 self.get_logger().info(
#                     "Spawned {}.".format(name)
#                 )

#                 return name

#             self.get_logger().warn(
#                 "Spawn attempt {} failed: {}"
#                 .format(
#                     attempt,
#                     result.stderr.strip(),
#                 )
#             )

#         raise RuntimeError(
#             "Failed to spawn {}.".format(name)
#         )

#     # ==================================================
#     # SHUTDOWN
#     # ==================================================

#     def destroy_node(self):
#         self.shutdown_event.set()

#         if rclpy.ok():
#             try:
#                 self.start_conveyor_visual(0.0)
#             except Exception:
#                 pass

#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)

#     node = SortingNode()

#     executor = rclpy.executors.MultiThreadedExecutor(
#         num_threads=4
#     )

#     executor.add_node(node)

#     try:
#         executor.spin()

#     except KeyboardInterrupt:
#         pass

#     finally:
#         try:
#             node.destroy_node()
#         finally:
#             if rclpy.ok():
#                 rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
# """Gazebo follower for the stage-synchronised EV3 sorter.

# - Full initial homing.
# - Full homing after every RED or BLUE pick-and-place.
# - GREEN and BLACK use conveyor routing and centre hold only.
# - Ball colours may arrive in any order.
# - Homing is sent as one multi-point trajectory for smooth interpolation.
# """

# import math
# import queue
# import subprocess
# import threading
# import time

# import rclpy
# from action_msgs.msg import GoalStatus
# from builtin_interfaces.msg import Duration
# from control_msgs.action import FollowJointTrajectory
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from std_msgs.msg import Float64, Float64MultiArray, String
# from trajectory_msgs.msg import JointTrajectoryPoint


# # ==================================================
# # SIMULATION GEOMETRY AND TARGETS
# # ==================================================

# SPAWN_X = -0.14824
# SPAWN_Y = 0.29075
# SPAWN_Z = 0.09232
# PICKUP_Z = SPAWN_Z + 0.022

# PICKUP_X = -0.015
# TRANSPORT_Z = SPAWN_Z + 0.005

# BELT_END_R = 0.17
# BELT_END_L = -0.17

# # Restored exactly from the earlier reliable ball-motion script.
# STEP_SIZE = 0.008
# STEP_DELAY = 0.090

# SIM_BASE_HOME = 0.0
# SIM_BASE_RED_BIN = -1.5708
# SIM_BASE_BLUE_BIN = 1.5708

# THETA1_MIN = -math.pi / 2
# THETA1_MAX = math.pi / 2
# THETA2_MIN = -0.55
# THETA2_MAX = math.pi / 3

# CLEARANCE_PITCH = 0.2

# # Restored from the earlier working simulation script.
# #
# # If the current URDF/controller visibly behaves in the opposite direction,
# # swap only these two constants; no state-machine change is required.
# SIM_GRIPPER_OPEN = 0.5
# SIM_GRIPPER_CLOSE = 0.0


# # ==================================================
# # ORIGINAL SIMULATION TIMINGS
# # ==================================================

# # Smooth homing trajectories use cumulative wall-clock targets.
# #
# # Measured physical EV3 values from the latest run:
# #   initial home       ~8.94 s
# #   home after RED     ~10.74 s
# #   home after BLUE    ~7.95 s
# #
# # These values are tuned to keep Gazebo roughly aligned with the EV3
# # stage synchronization, instead of letting the simulator run much faster
# # than the physical system.
# SIM_HOME_INITIAL_TIMES = [0.96, 5.43, 8.94]
# SIM_HOME_AFTER_RED_TIMES = [7.35, 10.74]
# SIM_HOME_AFTER_BLUE_TIMES = [2.93, 7.95]

# SIM_PICKUP_READY_TIME = 0.27
# SIM_PICK_DOWN_TIME = 1.65
# SIM_PICK_UP_TIME = 1.65

# SIM_ROTATE_RED_TIME = 1.15
# SIM_ROTATE_BLUE_TIME = 1.15

# SIM_DROP_DOWN_RED_TIME = 1.84
# SIM_DROP_DOWN_BLUE_TIME = 1.84
# SIM_DROP_UP_TIME = 1.82

# SIM_REJECT_CENTER_HOLD_TIME = 0.25

# SIM_GRIPPER_READY_SETTLE = 0.15
# GRIPPER_CLOSE_SETTLE_TIME = 1.25

# GRIPPER_RELEASE_WAIT_1 = 0.45
# GRIPPER_RELEASE_WAIT_2 = 1.00


# # ==================================================
# # BALL MODEL
# # ==================================================

# BALL_RGB = {
#     "red": (1.0, 0.0, 0.0),
#     "blue": (0.0, 0.0, 1.0),
#     "black": (0.05, 0.05, 0.05),
#     "green": (0.0, 0.8, 0.0),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia>
#           <ixx>8e-06</ixx>
#           <iyy>8e-06</iyy>
#           <izz>8e-06</izz>
#         </inertia>
#       </inertial>
#       <collision name='col'>
#         <geometry>
#           <sphere>
#             <radius>0.0185</radius>
#           </sphere>
#         </geometry>
#         <surface>
#           <friction>
#             <ode>
#               <mu>0.7</mu>
#               <mu2>0.7</mu2>
#             </ode>
#           </friction>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry>
#           <sphere>
#             <radius>0.0185</radius>
#           </sphere>
#         </geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# def solve_ik_sim(x, y, z_target):
#     theta1 = math.atan2(x, y)

#     z0 = 0.220995
#     r_arm = 0.226963

#     sin_t2 = (z_target - z0) / r_arm
#     sin_t2 = max(-1.0, min(1.0, sin_t2))

#     theta2 = math.asin(sin_t2)

#     theta1 = max(
#         THETA1_MIN,
#         min(THETA1_MAX, theta1),
#     )

#     theta2 = max(
#         THETA2_MIN,
#         min(THETA2_MAX, theta2),
#     )

#     return theta1, theta2


# class SortingNode(Node):
#     def __init__(self):
#         super().__init__("sorting_node")

#         self.arm_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             "/arm_controller/follow_joint_trajectory",
#         )

#         self.gripper_pub = self.create_publisher(
#             Float64MultiArray,
#             "/gripper_controller/commands",
#             10,
#         )

#         self.belt_vel_pub = self.create_publisher(
#             Float64,
#             "/conveyor_belt_vel",
#             10,
#         )

#         self.stage_event_sub = self.create_subscription(
#             String,
#             "/hw/stage_event",
#             self.stage_event_callback,
#             20,
#         )

#         self.stage_sync_pub = self.create_publisher(
#             String,
#             "/sim/stage_sync",
#             20,
#         )

#         self.event_queue = queue.Queue()
#         self.shutdown_event = threading.Event()

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#         self.current_ball_name = None
#         self.last_bin_target = SIM_BASE_HOME

#         self.worker_thread = threading.Thread(
#             target=self.worker_loop,
#             daemon=True,
#         )

#         self.worker_thread.start()

#     # ==================================================
#     # PROTOCOL HELPERS
#     # ==================================================

#     @staticmethod
#     def parse_stage_line(line, expected_kind):
#         parts = line.split("|")

#         if len(parts) != 4:
#             raise ValueError(
#                 "Invalid stage packet: {}".format(line)
#             )

#         if parts[0] != expected_kind:
#             raise ValueError(
#                 "Expected {}, got {}".format(
#                     expected_kind,
#                     line,
#                 )
#             )

#         return (
#             int(parts[1]),
#             int(parts[2]),
#             parts[3],
#         )

#     @staticmethod
#     def make_stage_line(
#         kind,
#         cycle_id,
#         sequence_id,
#         stage,
#     ):
#         return "{}|{}|{}|{}".format(
#             kind,
#             cycle_id,
#             sequence_id,
#             stage,
#         )

#     def publish_stage_sync(self, text):
#         msg = String()
#         msg.data = text

#         self.stage_sync_pub.publish(msg)

#         self.get_logger().info(
#             "-> EV3 [{}]".format(text)
#         )

#     def stage_event_callback(self, msg):
#         self.event_queue.put(msg.data.strip())

#     # ==================================================
#     # WORKER AND STAGE BARRIER
#     # ==================================================

#     def worker_loop(self):
#         self.get_logger().info(
#             "Waiting for arm action server..."
#         )

#         if not self.arm_client.wait_for_server(
#             timeout_sec=30.0
#         ):
#             self.get_logger().error(
#                 "Arm action server was not available."
#             )
#             return

#         self.get_logger().info(
#             "Arm action server available."
#         )

#         self.start_conveyor_visual(0.05)

#         while (
#             rclpy.ok()
#             and not self.shutdown_event.is_set()
#         ):
#             try:
#                 line = self.event_queue.get(
#                     timeout=0.2
#                 )
#             except queue.Empty:
#                 continue

#             try:
#                 if line.startswith("STAGE_START|"):
#                     self.handle_stage_start(line)

#                 elif line.startswith("STAGE_HW_DONE|"):
#                     self.handle_hardware_done(line)

#                 elif line == "EV3_DONE":
#                     self.get_logger().info(
#                         "EV3 completed the full task."
#                     )

#                     self.start_conveyor_visual(0.0)

#                 else:
#                     self.get_logger().warn(
#                         "Ignoring unknown stage event: {}".format(
#                             line
#                         )
#                     )

#             except Exception as exc:
#                 self.get_logger().error(
#                     "Error while processing [{}]: {}".format(
#                         line,
#                         exc,
#                     )
#                 )

#                 self.publish_failure_for_active_stage(
#                     str(exc)
#                 )

#     def handle_stage_start(self, line):
#         (
#             cycle_id,
#             sequence_id,
#             stage,
#         ) = self.parse_stage_line(
#             line,
#             "STAGE_START",
#         )

#         key = (
#             cycle_id,
#             sequence_id,
#             stage,
#         )

#         if self.active_key is not None:
#             raise RuntimeError(
#                 "Received {}, but {} is active.".format(
#                     key,
#                     self.active_key,
#                 )
#             )

#         self.active_key = key
#         self.sim_done = False
#         self.hardware_done = False

#         self.get_logger().info(
#             "=== START cycle={} seq={} stage={} ===".format(
#                 cycle_id,
#                 sequence_id,
#                 stage,
#             )
#         )

#         self.execute_sim_stage(
#             cycle_id,
#             stage,
#         )

#         self.sim_done = True

#         self.try_complete_active_stage()

#     def handle_hardware_done(self, line):
#         (
#             cycle_id,
#             sequence_id,
#             stage,
#         ) = self.parse_stage_line(
#             line,
#             "STAGE_HW_DONE",
#         )

#         key = (
#             cycle_id,
#             sequence_id,
#             stage,
#         )

#         if self.active_key != key:
#             raise RuntimeError(
#                 "Hardware completed {}, active stage is {}.".format(
#                     key,
#                     self.active_key,
#                 )
#             )

#         self.hardware_done = True

#         self.get_logger().info(
#             "Hardware completed cycle={} seq={} stage={}.".format(
#                 cycle_id,
#                 sequence_id,
#                 stage,
#             )
#         )

#         self.try_complete_active_stage()

#     def try_complete_active_stage(self):
#         if not (
#             self.sim_done
#             and self.hardware_done
#         ):
#             return

#         (
#             cycle_id,
#             sequence_id,
#             stage,
#         ) = self.active_key

#         response = self.make_stage_line(
#             "STAGE_SYNC_DONE",
#             cycle_id,
#             sequence_id,
#             stage,
#         )

#         self.publish_stage_sync(response)

#         self.get_logger().info(
#             "=== DONE cycle={} seq={} stage={} ===".format(
#                 cycle_id,
#                 sequence_id,
#                 stage,
#             )
#         )

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#     def publish_failure_for_active_stage(self, reason):
#         if self.active_key is None:
#             return

#         (
#             cycle_id,
#             sequence_id,
#             stage,
#         ) = self.active_key

#         safe_reason = (
#             reason
#             .replace("|", "/")
#             .replace("\n", " ")
#         )

#         response = (
#             "STAGE_SYNC_FAILED|{}|{}|{}|{}"
#             .format(
#                 cycle_id,
#                 sequence_id,
#                 stage,
#                 safe_reason,
#             )
#         )

#         self.publish_stage_sync(response)

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#     # ==================================================
#     # STAGE IMPLEMENTATION
#     # ==================================================

#     def execute_sim_stage(self, cycle_id, stage):
#         if stage == "HOME_INITIAL":
#             self.execute_sim_homing("initial")
#             return

#         if stage == "HOME_AFTER_RED":
#             self.execute_sim_homing("after_red")
#             return

#         if stage == "HOME_AFTER_BLUE":
#             self.execute_sim_homing("after_blue")
#             return

#         if stage.startswith("SPAWN_"):
#             color = stage.split("_", 1)[1].lower()

#             self.current_ball_name = self.spawn_ball(
#                 color,
#                 cycle_id,
#             )

#             return

#         if stage == "CONVEYOR_TO_PICKUP":
#             self.require_current_ball()

#             self.move_ball(
#                 self.current_ball_name,
#                 SPAWN_X,
#                 PICKUP_X,
#             )

#             return

#         if stage == "PICKUP_READY":
#             self.send_arm_target(
#                 [
#                     SIM_BASE_HOME,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_PICKUP_READY_TIME,
#             )

#             return

#         if stage == "GRIPPER_READY":
#             self.command_gripper(
#                 SIM_GRIPPER_OPEN
#             )

#             time.sleep(SIM_GRIPPER_READY_SETTLE)

#             return

#         if stage == "PICK_DOWN":
#             _theta1, pick_pitch = solve_ik_sim(
#                 PICKUP_X,
#                 SPAWN_Y,
#                 PICKUP_Z,
#             )

#             self.send_arm_target(
#                 [
#                     SIM_BASE_HOME,
#                     pick_pitch,
#                 ],
#                 SIM_PICK_DOWN_TIME,
#             )

#             return

#         if stage == "GRIP_CLOSE":
#             self.command_gripper(
#                 SIM_GRIPPER_CLOSE
#             )

#             time.sleep(
#                 GRIPPER_CLOSE_SETTLE_TIME
#             )

#             return

#         if stage == "PICK_UP":
#             self.send_arm_target(
#                 [
#                     SIM_BASE_HOME,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_PICK_UP_TIME,
#             )

#             return

#         if stage == "ROTATE_RED":
#             self.last_bin_target = (
#                 SIM_BASE_RED_BIN
#             )

#             self.send_arm_target(
#                 [
#                     SIM_BASE_RED_BIN,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_ROTATE_RED_TIME,
#             )

#             return

#         if stage == "ROTATE_BLUE":
#             self.last_bin_target = (
#                 SIM_BASE_BLUE_BIN
#             )

#             self.send_arm_target(
#                 [
#                     SIM_BASE_BLUE_BIN,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_ROTATE_BLUE_TIME,
#             )

#             return

#         if stage == "PLACE_DOWN":
#             if self.last_bin_target == SIM_BASE_RED_BIN:
#                 duration = SIM_DROP_DOWN_RED_TIME
#             else:
#                 duration = SIM_DROP_DOWN_BLUE_TIME

#             self.send_arm_target(
#                 [
#                     self.last_bin_target,
#                     -0.45,
#                 ],
#                 duration,
#             )

#             return

#         if stage == "GRIP_RELEASE":
#             # Original two-step release behaviour.
#             self.command_gripper(0.2)
#             time.sleep(GRIPPER_RELEASE_WAIT_1)

#             self.command_gripper(
#                 SIM_GRIPPER_OPEN
#             )
#             time.sleep(GRIPPER_RELEASE_WAIT_2)

#             return

#         if stage == "PLACE_UP":
#             self.send_arm_target(
#                 [
#                     self.last_bin_target,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_DROP_UP_TIME,
#             )

#             return

#         if stage == "CONVEYOR_BLACK":
#             self.require_current_ball()

#             self.fall_ball(
#                 self.current_ball_name,
#                 direction=1,
#             )

#             return

#         if stage == "CONVEYOR_GREEN":
#             self.require_current_ball()

#             self.fall_ball(
#                 self.current_ball_name,
#                 direction=-1,
#             )

#             return

#         if stage == "CENTER_HOLD":
#             self.get_logger().info(
#                 "Centre hold: maintaining the centred clearance pose."
#             )

#             self.last_bin_target = SIM_BASE_HOME
#             time.sleep(SIM_REJECT_CENTER_HOLD_TIME)

#             return

#         if stage == "CYCLE_COMPLETE":
#             self.current_ball_name = None
#             self.last_bin_target = SIM_BASE_HOME

#             self.get_logger().info(
#                 "Simulation cycle state cleared."
#             )

#             return

#         raise ValueError(
#             "No simulation implementation for stage {}".format(
#                 stage
#             )
#         )

#     # ==================================================
#     # SIMULATED HOMING
#     # ==================================================

#     def execute_sim_homing(self, profile):
#         """Run one continuous multi-point homing trajectory."""

#         if profile == "initial":
#             # Raise once, sweep to the simulated switch, then return to centre.
#             positions = [
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#                 [1.0472, CLEARANCE_PITCH],
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#             ]
#             cumulative_times = SIM_HOME_INITIAL_TIMES

#         elif profile == "after_red":
#             # PLACE_UP already left the pitch at clearance.
#             # Keep it there while the base performs the homing sweep.
#             positions = [
#                 [1.0472, CLEARANCE_PITCH],
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#             ]
#             cumulative_times = SIM_HOME_AFTER_RED_TIMES

#         elif profile == "after_blue":
#             # PLACE_UP already left the pitch at clearance.
#             positions = [
#                 [1.0472, CLEARANCE_PITCH],
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#             ]
#             cumulative_times = SIM_HOME_AFTER_BLUE_TIMES

#         else:
#             raise ValueError(
#                 "Unknown homing profile: {}".format(profile)
#             )

#         self.get_logger().info(
#             "Executing smooth {} homing trajectory; total {:.2f} s."
#             .format(profile, cumulative_times[-1])
#         )

#         self.send_arm_trajectory(
#             positions,
#             cumulative_times,
#         )

#         self.last_bin_target = SIM_BASE_HOME

#         self.get_logger().info(
#             "Simulated {} homing complete."
#             .format(profile)
#         )

#     # ==================================================
#     # CONTROLLER HELPERS
#     # ==================================================

#     def wait_for_future(
#         self,
#         future,
#         timeout_sec,
#     ):
#         event = threading.Event()

#         future.add_done_callback(
#             lambda _future: event.set()
#         )

#         if not event.wait(timeout_sec):
#             raise TimeoutError(
#                 "ROS action future timed out."
#             )

#         return future.result()

#     def send_arm_trajectory(
#         self,
#         positions,
#         cumulative_times,
#     ):
#         if len(positions) != len(cumulative_times):
#             raise ValueError(
#                 "positions and cumulative_times must have equal length."
#             )

#         if not positions:
#             raise ValueError("Trajectory must contain at least one point.")

#         goal = FollowJointTrajectory.Goal()

#         goal.trajectory.joint_names = [
#             "arm_1_base_link_joint",
#             "arm_2_left_arm_linkage_joint",
#         ]

#         previous_time = 0.0

#         for position, cumulative_time in zip(
#             positions,
#             cumulative_times,
#         ):
#             if cumulative_time <= previous_time:
#                 raise ValueError(
#                     "Trajectory times must be strictly increasing."
#                 )

#             point = JointTrajectoryPoint()
#             point.positions = [
#                 float(value)
#                 for value in position
#             ]

#             point.time_from_start = Duration(
#                 sec=int(cumulative_time),
#                 nanosec=int(
#                     (
#                         cumulative_time
#                         - int(cumulative_time)
#                     )
#                     * 1e9
#                 ),
#             )

#             goal.trajectory.points.append(point)
#             previous_time = cumulative_time

#         goal_future = self.arm_client.send_goal_async(goal)

#         goal_handle = self.wait_for_future(
#             goal_future,
#             timeout_sec=3.0,
#         )

#         if (
#             goal_handle is None
#             or not goal_handle.accepted
#         ):
#             raise RuntimeError(
#                 "Trajectory goal was rejected."
#             )

#         result_future = goal_handle.get_result_async()

#         wrapped_result = self.wait_for_future(
#             result_future,
#             timeout_sec=cumulative_times[-1] + 5.0,
#         )

#         if (
#             wrapped_result.status
#             != GoalStatus.STATUS_SUCCEEDED
#         ):
#             raise RuntimeError(
#                 "Trajectory action status was {}."
#                 .format(
#                     wrapped_result.status
#                 )
#             )

#         result = wrapped_result.result

#         if (
#             result.error_code
#             != FollowJointTrajectory.Result.SUCCESSFUL
#         ):
#             raise RuntimeError(
#                 "Trajectory controller error {}: {}"
#                 .format(
#                     result.error_code,
#                     result.error_string,
#                 )
#             )

#     def send_arm_target(
#         self,
#         positions,
#         duration_sec,
#     ):
#         self.send_arm_trajectory(
#             [positions],
#             [duration_sec],
#         )

#     def command_gripper(self, position):
#         msg = Float64MultiArray()

#         msg.data = [float(position)]

#         self.gripper_pub.publish(msg)

#     def start_conveyor_visual(self, speed):
#         msg = Float64()

#         msg.data = float(speed)

#         self.belt_vel_pub.publish(msg)

#     # ==================================================
#     # ORIGINAL BALL SPAWN AND SET-POSE MOVEMENT
#     # ==================================================

#     def require_current_ball(self):
#         if not self.current_ball_name:
#             raise RuntimeError(
#                 "No active simulated ball exists."
#             )

#     def move_ball(
#         self,
#         name,
#         x_start,
#         x_end,
#     ):
#         x = x_start

#         while x < x_end:
#             x = min(
#                 x + STEP_SIZE,
#                 x_end,
#             )

#             self.set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 TRANSPORT_Z,
#             )

#             time.sleep(STEP_DELAY)

#     def fall_ball(
#         self,
#         name,
#         direction,
#     ):
#         if direction < 0:
#             x_end = BELT_END_L - 0.06
#         else:
#             x_end = BELT_END_R + 0.06

#         x = SPAWN_X
#         z = TRANSPORT_Z

#         step = STEP_SIZE * direction

#         while (
#             (
#                 direction > 0
#                 and x < x_end
#             )
#             or
#             (
#                 direction < 0
#                 and x > x_end
#             )
#         ):
#             x += step

#             if (
#                 (
#                     direction > 0
#                     and x > BELT_END_R
#                 )
#                 or
#                 (
#                     direction < 0
#                     and x < BELT_END_L
#                 )
#             ):
#                 z = max(
#                     z - 0.006,
#                     -0.05,
#                 )

#             self.set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 z,
#             )

#             time.sleep(STEP_DELAY)

#     def set_pose(
#         self,
#         name,
#         x,
#         y,
#         z,
#     ):
#         # Kept intentionally identical in behaviour to the old script:
#         # short timeout + asynchronous subprocess.
#         command = [
#             "ign",
#             "service",
#             "-s",
#             "/world/empty/set_pose",
#             "--reqtype",
#             "ignition.msgs.Pose",
#             "--reptype",
#             "ignition.msgs.Boolean",
#             "--timeout",
#             "150",
#             "--req",
#             (
#                 'name: "{}" '
#                 "position: {{x: {:.5f} y: {:.5f} z: {:.5f}}}"
#                 .format(
#                     name,
#                     x,
#                     y,
#                     z,
#                 )
#             ),
#         ]

#         subprocess.Popen(
#             command,
#             stdout=subprocess.DEVNULL,
#             stderr=subprocess.DEVNULL,
#         )

#     def spawn_ball(
#         self,
#         color,
#         cycle_id,
#         retries=3,
#     ):
#         if color not in BALL_RGB:
#             raise ValueError(
#                 "Unsupported ball colour: {}".format(
#                     color
#                 )
#             )

#         name = "{}_ball_{}".format(
#             color,
#             cycle_id,
#         )

#         r, g, b = BALL_RGB[color]

#         sdf = BALL_SDF.format(
#             name=name,
#             r=r,
#             g=g,
#             b=b,
#         )

#         command = [
#             "ros2",
#             "run",
#             "ros_gz_sim",
#             "create",
#             "-name",
#             name,
#             "-x",
#             str(SPAWN_X),
#             "-y",
#             str(SPAWN_Y),
#             "-z",
#             str(SPAWN_Z),
#             "-string",
#             sdf,
#         ]

#         for attempt in range(
#             1,
#             retries + 1,
#         ):
#             try:
#                 result = subprocess.run(
#                     command,
#                     capture_output=True,
#                     text=True,
#                     timeout=10,
#                     check=False,
#                 )

#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     "Spawn attempt {} timed out."
#                     .format(attempt)
#                 )

#                 time.sleep(1.0)

#                 continue

#             if result.returncode == 0:
#                 self.get_logger().info(
#                     "Spawned {}.".format(name)
#                 )

#                 return name

#             self.get_logger().warn(
#                 "Spawn attempt {} failed: {}"
#                 .format(
#                     attempt,
#                     result.stderr.strip(),
#                 )
#             )

#         raise RuntimeError(
#             "Failed to spawn {}.".format(name)
#         )

#     # ==================================================
#     # SHUTDOWN
#     # ==================================================

#     def destroy_node(self):
#         self.shutdown_event.set()

#         if rclpy.ok():
#             try:
#                 self.start_conveyor_visual(0.0)
#             except Exception:
#                 pass

#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)

#     node = SortingNode()

#     executor = rclpy.executors.MultiThreadedExecutor(
#         num_threads=4
#     )

#     executor.add_node(node)

#     try:
#         executor.spin()

#     except KeyboardInterrupt:
#         pass

#     finally:
#         try:
#             node.destroy_node()
#         finally:
#             if rclpy.ok():
#                 rclpy.shutdown()


# if __name__ == "__main__":
#     main()


"""Gazebo follower for the stage-synchronised EV3 sorter.

- Full initial homing.
- Full homing after every RED or BLUE pick-and-place.
- GREEN and BLACK use conveyor routing and centre hold only.
- Ball colours may arrive in any order.
- Homing is sent as one multi-point trajectory for smooth interpolation.
"""

import math
import queue
import subprocess
import threading
import time
import os
import csv

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint


# ==================================================
# SIMULATION GEOMETRY AND TARGETS
# ==================================================

SPAWN_X = -0.14824
SPAWN_Y = 0.29075
SPAWN_Z = 0.09232
PICKUP_Z = SPAWN_Z + 0.022

PICKUP_X = -0.015
TRANSPORT_Z = SPAWN_Z + 0.005

BELT_END_R = 0.17
BELT_END_L = -0.17

# Restored exactly from the earlier reliable ball-motion script.
STEP_SIZE = 0.008
STEP_DELAY = 0.090

SIM_BASE_HOME = 0.0
SIM_BASE_RED_BIN = -1.5708
SIM_BASE_BLUE_BIN = 1.5708

THETA1_MIN = -math.pi / 2
THETA1_MAX = math.pi / 2
THETA2_MIN = -0.55
THETA2_MAX = math.pi / 3

CLEARANCE_PITCH = 0.2

# Restored from the earlier working simulation script.
#
# If the current URDF/controller visibly behaves in the opposite direction,
# swap only these two constants; no state-machine change is required.
SIM_GRIPPER_OPEN = 0.5
SIM_GRIPPER_CLOSE = 0.0

# Enable writing measured trajectory times to a CSV for easier analysis.
SIM_LOG_TO_CSV = True
SIM_TRAJ_CSV = "/tmp/sim_traj_times.csv"


# ==================================================
# ORIGINAL SIMULATION TIMINGS
# ==================================================

# Smooth homing trajectories use cumulative wall-clock targets.
#
# Measured physical EV3 values from the latest run:
#   initial home       ~5.01 s hardware, ~6.3 s sim
#   home after RED     ~6.43 s hardware, ~6.4 s sim
#   home after BLUE    ~3.07 s hardware, ~5.9 s sim
#
# These values are tuned to keep Gazebo roughly aligned with the EV3
# stage synchronization without overshooting the action timeout.
# Initial homing is shaped to start briskly, then return at a more
# natural speed instead of finishing too quickly.
SIM_HOME_INITIAL_TIMES = [0.60, 4.20, 5.40]
SIM_HOME_AFTER_RED_TIMES = [3.20, 6.40]
SIM_HOME_AFTER_BLUE_TIMES = [2.93, 5.90]

SIM_PICKUP_READY_TIME = 0.16
SIM_PICK_DOWN_TIME = 1.40
SIM_PICK_UP_TIME = 1.80

SIM_ROTATE_RED_TIME = 1.75
SIM_ROTATE_BLUE_TIME = 1.65

SIM_DROP_DOWN_RED_TIME = 2.42
SIM_DROP_DOWN_BLUE_TIME = 2.62
SIM_DROP_UP_TIME = 0.35

SIM_REJECT_CENTER_HOLD_TIME = 0.25

SIM_GRIPPER_READY_SETTLE = 0.15
GRIPPER_CLOSE_SETTLE_TIME = 1.25

GRIPPER_RELEASE_WAIT_1 = 0.45
GRIPPER_RELEASE_WAIT_2 = 1.00


# ==================================================
# BALL MODEL
# ==================================================

BALL_RGB = {
    "red": (1.0, 0.0, 0.0),
    "blue": (0.0, 0.0, 1.0),
    "black": (0.05, 0.05, 0.05),
    "green": (0.0, 0.8, 0.0),
}

BALL_SDF = """<sdf version='1.7'>
  <model name='{name}'>
    <link name='link'>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>8e-06</ixx>
          <iyy>8e-06</iyy>
          <izz>8e-06</izz>
        </inertia>
      </inertial>
      <collision name='col'>
        <geometry>
          <sphere>
            <radius>0.0185</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.7</mu>
              <mu2>0.7</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name='vis'>
        <geometry>
          <sphere>
            <radius>0.0185</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def solve_ik_sim(x, y, z_target):
    theta1 = math.atan2(x, y)

    z0 = 0.220995
    r_arm = 0.226963

    sin_t2 = (z_target - z0) / r_arm
    sin_t2 = max(-1.0, min(1.0, sin_t2))

    theta2 = math.asin(sin_t2)

    theta1 = max(
        THETA1_MIN,
        min(THETA1_MAX, theta1),
    )

    theta2 = max(
        THETA2_MIN,
        min(THETA2_MAX, theta2),
    )

    return theta1, theta2


class SortingNode(Node):
    def __init__(self):
        super().__init__("sorting_node")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            "/gripper_controller/commands",
            10,
        )

        self.belt_vel_pub = self.create_publisher(
            Float64,
            "/conveyor_belt_vel",
            10,
        )

        self.stage_event_sub = self.create_subscription(
            String,
            "/hw/stage_event",
            self.stage_event_callback,
            20,
        )

        self.stage_sync_pub = self.create_publisher(
            String,
            "/sim/stage_sync",
            20,
        )

        self.event_queue = queue.Queue()
        self.shutdown_event = threading.Event()

        self.active_key = None
        self.sim_done = False
        self.hardware_done = False

        self.current_ball_name = None
        self.last_bin_target = SIM_BASE_HOME

        self.worker_thread = threading.Thread(
            target=self.worker_loop,
            daemon=True,
        )

        self.worker_thread.start()

    # ==================================================
    # PROTOCOL HELPERS
    # ==================================================

    @staticmethod
    def parse_stage_line(line, expected_kind):
        parts = line.split("|")

        if len(parts) != 4:
            raise ValueError(
                "Invalid stage packet: {}".format(line)
            )

        if parts[0] != expected_kind:
            raise ValueError(
                "Expected {}, got {}".format(
                    expected_kind,
                    line,
                )
            )

        return (
            int(parts[1]),
            int(parts[2]),
            parts[3],
        )

    @staticmethod
    def make_stage_line(
        kind,
        cycle_id,
        sequence_id,
        stage,
    ):
        return "{}|{}|{}|{}".format(
            kind,
            cycle_id,
            sequence_id,
            stage,
        )

    def publish_stage_sync(self, text):
        msg = String()
        msg.data = text

        self.stage_sync_pub.publish(msg)

        self.get_logger().info(
            "-> EV3 [{}]".format(text)
        )

    def stage_event_callback(self, msg):
        self.event_queue.put(msg.data.strip())

    # ==================================================
    # WORKER AND STAGE BARRIER
    # ==================================================

    def worker_loop(self):
        self.get_logger().info(
            "Waiting for arm action server..."
        )

        if not self.arm_client.wait_for_server(
            timeout_sec=30.0
        ):
            self.get_logger().error(
                "Arm action server was not available."
            )
            return

        self.get_logger().info(
            "Arm action server available."
        )

        self.start_conveyor_visual(0.05)

        while (
            rclpy.ok()
            and not self.shutdown_event.is_set()
        ):
            try:
                line = self.event_queue.get(
                    timeout=0.2
                )
            except queue.Empty:
                continue

            try:
                if line.startswith("STAGE_START|"):
                    self.handle_stage_start(line)

                elif line.startswith("STAGE_HW_DONE|"):
                    self.handle_hardware_done(line)

                elif line == "EV3_DONE":
                    self.get_logger().info(
                        "EV3 completed the full task."
                    )

                    self.start_conveyor_visual(0.0)

                else:
                    self.get_logger().warn(
                        "Ignoring unknown stage event: {}".format(
                            line
                        )
                    )

            except Exception as exc:
                self.get_logger().error(
                    "Error while processing [{}]: {}".format(
                        line,
                        exc,
                    )
                )

                self.publish_failure_for_active_stage(
                    str(exc)
                )

    def handle_stage_start(self, line):
        (
            cycle_id,
            sequence_id,
            stage,
        ) = self.parse_stage_line(
            line,
            "STAGE_START",
        )

        key = (
            cycle_id,
            sequence_id,
            stage,
        )

        if self.active_key is not None:
            raise RuntimeError(
                "Received {}, but {} is active.".format(
                    key,
                    self.active_key,
                )
            )

        self.active_key = key
        self.sim_done = False
        self.hardware_done = False

        self.get_logger().info(
            "=== START cycle={} seq={} stage={} ===".format(
                cycle_id,
                sequence_id,
                stage,
            )
        )

        self.execute_sim_stage(
            cycle_id,
            stage,
        )

        self.sim_done = True

        self.try_complete_active_stage()

    def handle_hardware_done(self, line):
        (
            cycle_id,
            sequence_id,
            stage,
        ) = self.parse_stage_line(
            line,
            "STAGE_HW_DONE",
        )

        key = (
            cycle_id,
            sequence_id,
            stage,
        )

        if self.active_key != key:
            raise RuntimeError(
                "Hardware completed {}, active stage is {}.".format(
                    key,
                    self.active_key,
                )
            )

        self.hardware_done = True

        self.get_logger().info(
            "Hardware completed cycle={} seq={} stage={}.".format(
                cycle_id,
                sequence_id,
                stage,
            )
        )

        self.try_complete_active_stage()

    def try_complete_active_stage(self):
        if not (
            self.sim_done
            and self.hardware_done
        ):
            return

        (
            cycle_id,
            sequence_id,
            stage,
        ) = self.active_key

        response = self.make_stage_line(
            "STAGE_SYNC_DONE",
            cycle_id,
            sequence_id,
            stage,
        )

        self.publish_stage_sync(response)

        self.get_logger().info(
            "=== DONE cycle={} seq={} stage={} ===".format(
                cycle_id,
                sequence_id,
                stage,
            )
        )

        self.active_key = None
        self.sim_done = False
        self.hardware_done = False

    def publish_failure_for_active_stage(self, reason):
        if self.active_key is None:
            return

        (
            cycle_id,
            sequence_id,
            stage,
        ) = self.active_key

        safe_reason = (
            reason
            .replace("|", "/")
            .replace("\n", " ")
        )

        response = (
            "STAGE_SYNC_FAILED|{}|{}|{}|{}"
            .format(
                cycle_id,
                sequence_id,
                stage,
                safe_reason,
            )
        )

        self.publish_stage_sync(response)

        self.active_key = None
        self.sim_done = False
        self.hardware_done = False

    # ==================================================
    # STAGE IMPLEMENTATION
    # ==================================================

    def execute_sim_stage(self, cycle_id, stage):
        if stage == "HOME_INITIAL":
            self.execute_sim_homing("initial")
            return

        if stage == "HOME_AFTER_RED":
            self.execute_sim_homing("after_red")
            return

        if stage == "HOME_AFTER_BLUE":
            self.execute_sim_homing("after_blue")
            return

        if stage.startswith("SPAWN_"):
            color = stage.split("_", 1)[1].lower()

            self.current_ball_name = self.spawn_ball(
                color,
                cycle_id,
            )

            return

        if stage == "CONVEYOR_TO_PICKUP":
            self.require_current_ball()

            self.move_ball(
                self.current_ball_name,
                SPAWN_X,
                PICKUP_X,
            )

            return

        if stage == "PICKUP_READY":
            self.send_arm_target(
                [
                    SIM_BASE_HOME,
                    CLEARANCE_PITCH,
                ],
                SIM_PICKUP_READY_TIME,
            )

            return

        if stage == "GRIPPER_READY":
            self.command_gripper(
                SIM_GRIPPER_OPEN
            )

            time.sleep(SIM_GRIPPER_READY_SETTLE)

            return

        if stage == "PICK_DOWN":
            _theta1, pick_pitch = solve_ik_sim(
                PICKUP_X,
                SPAWN_Y,
                PICKUP_Z,
            )

            self.send_arm_target(
                [
                    SIM_BASE_HOME,
                    pick_pitch,
                ],
                SIM_PICK_DOWN_TIME,
            )

            return

        if stage == "GRIP_CLOSE":
            self.command_gripper(
                SIM_GRIPPER_CLOSE
            )

            time.sleep(
                GRIPPER_CLOSE_SETTLE_TIME
            )

            return

        if stage == "PICK_UP":
            self.send_arm_target(
                [
                    SIM_BASE_HOME,
                    CLEARANCE_PITCH,
                ],
                SIM_PICK_UP_TIME,
            )

            return

        if stage == "ROTATE_RED":
            self.last_bin_target = (
                SIM_BASE_RED_BIN
            )

            self.send_arm_target(
                [
                    SIM_BASE_RED_BIN,
                    CLEARANCE_PITCH,
                ],
                SIM_ROTATE_RED_TIME,
            )

            return

        if stage == "ROTATE_BLUE":
            self.last_bin_target = (
                SIM_BASE_BLUE_BIN
            )

            self.send_arm_target(
                [
                    SIM_BASE_BLUE_BIN,
                    CLEARANCE_PITCH,
                ],
                SIM_ROTATE_BLUE_TIME,
            )

            return

        if stage == "PLACE_DOWN":
            if self.last_bin_target == SIM_BASE_RED_BIN:
                duration = SIM_DROP_DOWN_RED_TIME
            else:
                duration = SIM_DROP_DOWN_BLUE_TIME

            self.send_arm_target(
                [
                    self.last_bin_target,
                    -0.45,
                ],
                duration,
            )

            return

        if stage == "GRIP_RELEASE":
            # Original two-step release behaviour.
            self.command_gripper(0.2)
            time.sleep(GRIPPER_RELEASE_WAIT_1)

            self.command_gripper(
                SIM_GRIPPER_OPEN
            )
            time.sleep(GRIPPER_RELEASE_WAIT_2)

            return

        if stage == "PLACE_UP":
            self.send_arm_target(
                [
                    self.last_bin_target,
                    CLEARANCE_PITCH,
                ],
                SIM_DROP_UP_TIME,
            )

            return

        if stage == "CONVEYOR_BLACK":
            self.require_current_ball()

            self.fall_ball(
                self.current_ball_name,
                direction=1,
            )

            return

        if stage == "CONVEYOR_GREEN":
            self.require_current_ball()

            self.fall_ball(
                self.current_ball_name,
                direction=-1,
            )

            return

        if stage == "CENTER_HOLD":
            self.get_logger().info(
                "Centre hold: maintaining the centred clearance pose."
            )

            self.last_bin_target = SIM_BASE_HOME
            time.sleep(SIM_REJECT_CENTER_HOLD_TIME)

            return

        if stage == "CYCLE_COMPLETE":
            self.current_ball_name = None
            self.last_bin_target = SIM_BASE_HOME

            self.get_logger().info(
                "Simulation cycle state cleared."
            )

            return

        raise ValueError(
            "No simulation implementation for stage {}".format(
                stage
            )
        )

    # ==================================================
    # SIMULATED HOMING
    # ==================================================

    def execute_sim_homing(self, profile):
        """Run one continuous multi-point homing trajectory."""

        if profile == "initial":
            # Raise once, sweep to the simulated switch, then return to centre.
            positions = [
                [SIM_BASE_HOME, CLEARANCE_PITCH],
                [1.0472, CLEARANCE_PITCH],
                [SIM_BASE_HOME, CLEARANCE_PITCH],
            ]
            cumulative_times = SIM_HOME_INITIAL_TIMES

        elif profile == "after_red":
            # PLACE_UP already left the pitch at clearance.
            # Keep it there while the base performs the homing sweep.
            positions = [
                [1.0472, CLEARANCE_PITCH],
                [SIM_BASE_HOME, CLEARANCE_PITCH],
            ]
            cumulative_times = SIM_HOME_AFTER_RED_TIMES

        elif profile == "after_blue":
            # PLACE_UP already left the pitch at clearance.
            positions = [
                [1.0472, CLEARANCE_PITCH],
                [SIM_BASE_HOME, CLEARANCE_PITCH],
            ]
            cumulative_times = SIM_HOME_AFTER_BLUE_TIMES

        else:
            raise ValueError(
                "Unknown homing profile: {}".format(profile)
            )

        self.get_logger().info(
            "Executing smooth {} homing trajectory; total {:.2f} s."
            .format(profile, cumulative_times[-1])
        )

        self.send_arm_trajectory(
            positions,
            cumulative_times,
        )

        self.last_bin_target = SIM_BASE_HOME

        self.get_logger().info(
            "Simulated {} homing complete."
            .format(profile)
        )

    # ==================================================
    # CONTROLLER HELPERS
    # ==================================================

    def wait_for_future(
        self,
        future,
        timeout_sec,
    ):
        event = threading.Event()

        future.add_done_callback(
            lambda _future: event.set()
        )

        if not event.wait(timeout_sec):
            raise TimeoutError(
                "ROS action future timed out."
            )

        return future.result()

    def send_arm_trajectory(
        self,
        positions,
        cumulative_times,
    ):
        if len(positions) != len(cumulative_times):
            raise ValueError(
                "positions and cumulative_times must have equal length."
            )

        if not positions:
            raise ValueError("Trajectory must contain at least one point.")

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            "arm_1_base_link_joint",
            "arm_2_left_arm_linkage_joint",
        ]

        previous_time = 0.0

        for position, cumulative_time in zip(
            positions,
            cumulative_times,
        ):
            if cumulative_time <= previous_time:
                raise ValueError(
                    "Trajectory times must be strictly increasing."
                )

            point = JointTrajectoryPoint()
            point.positions = [
                float(value)
                for value in position
            ]

            point.time_from_start = Duration(
                sec=int(cumulative_time),
                nanosec=int(
                    (
                        cumulative_time
                        - int(cumulative_time)
                    )
                    * 1e9
                ),
            )

            goal.trajectory.points.append(point)
            previous_time = cumulative_time

        # Log and timestamp when the trajectory is sent so we can measure
        # how long Gazebo actually takes to complete the requested motion.
        start_ts = time.time()

        self.get_logger().info(
            "Arm trajectory start: points={} expected_total={:.2f}s.".format(
                len(goal.trajectory.points),
                cumulative_times[-1],
            )
        )

        goal_future = self.arm_client.send_goal_async(goal)

        goal_handle = self.wait_for_future(
            goal_future,
            timeout_sec=3.0,
        )

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                "Trajectory goal was rejected."
            )

        result_future = goal_handle.get_result_async()

        wrapped_result = self.wait_for_future(
            result_future,
            timeout_sec=cumulative_times[-1] + 5.0,
        )

        elapsed = time.time() - start_ts

        self.get_logger().info(
            "Arm trajectory finished: elapsed={:.3f}s requested={:.2f}s status={}".format(
                elapsed,
                cumulative_times[-1],
                wrapped_result.status,
            )
        )

        # Optionally append measured trajectory timing to CSV for analysis.
        if SIM_LOG_TO_CSV:
            try:
                write_header = not os.path.exists(SIM_TRAJ_CSV)

                try:
                    cycle_id = ""
                    seq_id = ""
                    stage_name = ""

                    if getattr(self, "active_key", None):
                        try:
                            cycle_id, seq_id, stage_name = self.active_key
                        except Exception:
                            cycle_id = seq_id = stage_name = ""

                    with open(SIM_TRAJ_CSV, "a", newline="") as csvf:
                        writer = csv.writer(csvf)

                        if write_header:
                            writer.writerow([
                                "ts",
                                "cycle",
                                "seq",
                                "stage",
                                "points",
                                "requested_s",
                                "elapsed_s",
                                "status",
                            ])

                        writer.writerow([
                            time.time(),
                            cycle_id,
                            seq_id,
                            stage_name,
                            len(goal.trajectory.points),
                            float(cumulative_times[-1]),
                            float(elapsed),
                            int(wrapped_result.status),
                        ])
                except Exception as e:
                    self.get_logger().warn(
                        "Failed to write sim trajectory CSV row: {}".format(e)
                    )

            except Exception:
                # Never let CSV logging break the main flow.
                pass

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            raise RuntimeError(
                "Trajectory action status was {}."
                .format(
                    wrapped_result.status
                )
            )

        result = wrapped_result.result

        if (
            result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                "Trajectory controller error {}: {}"
                .format(
                    result.error_code,
                    result.error_string,
                )
            )

    def send_arm_target(
        self,
        positions,
        duration_sec,
    ):
        self.send_arm_trajectory(
            [positions],
            [duration_sec],
        )

    def command_gripper(self, position):
        msg = Float64MultiArray()

        msg.data = [float(position)]

        self.gripper_pub.publish(msg)

    def start_conveyor_visual(self, speed):
        msg = Float64()

        msg.data = float(speed)

        self.belt_vel_pub.publish(msg)

    # ==================================================
    # ORIGINAL BALL SPAWN AND SET-POSE MOVEMENT
    # ==================================================

    def require_current_ball(self):
        if not self.current_ball_name:
            raise RuntimeError(
                "No active simulated ball exists."
            )

    def move_ball(
        self,
        name,
        x_start,
        x_end,
    ):
        x = x_start

        while x < x_end:
            x = min(
                x + STEP_SIZE,
                x_end,
            )

            self.set_pose(
                name,
                x,
                SPAWN_Y,
                TRANSPORT_Z,
            )

            time.sleep(STEP_DELAY)

    def fall_ball(
        self,
        name,
        direction,
    ):
        if direction < 0:
            x_end = BELT_END_L - 0.06
        else:
            x_end = BELT_END_R + 0.06

        x = SPAWN_X
        z = TRANSPORT_Z

        step = STEP_SIZE * direction

        while (
            (
                direction > 0
                and x < x_end
            )
            or
            (
                direction < 0
                and x > x_end
            )
        ):
            x += step

            if (
                (
                    direction > 0
                    and x > BELT_END_R
                )
                or
                (
                    direction < 0
                    and x < BELT_END_L
                )
            ):
                z = max(
                    z - 0.006,
                    -0.05,
                )

            self.set_pose(
                name,
                x,
                SPAWN_Y,
                z,
            )

            time.sleep(STEP_DELAY)

    def set_pose(
        self,
        name,
        x,
        y,
        z,
    ):
        # Kept intentionally identical in behaviour to the old script:
        # short timeout + asynchronous subprocess.
        command = [
            "ign",
            "service",
            "-s",
            "/world/empty/set_pose",
            "--reqtype",
            "ignition.msgs.Pose",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "150",
            "--req",
            (
                'name: "{}" '
                "position: {{x: {:.5f} y: {:.5f} z: {:.5f}}}"
                .format(
                    name,
                    x,
                    y,
                    z,
                )
            ),
        ]

        subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def spawn_ball(
        self,
        color,
        cycle_id,
        retries=3,
    ):
        if color not in BALL_RGB:
            raise ValueError(
                "Unsupported ball colour: {}".format(
                    color
                )
            )

        name = "{}_ball_{}".format(
            color,
            cycle_id,
        )

        r, g, b = BALL_RGB[color]

        sdf = BALL_SDF.format(
            name=name,
            r=r,
            g=g,
            b=b,
        )

        command = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-name",
            name,
            "-x",
            str(SPAWN_X),
            "-y",
            str(SPAWN_Y),
            "-z",
            str(SPAWN_Z),
            "-string",
            sdf,
        ]

        for attempt in range(
            1,
            retries + 1,
        ):
            try:
                result = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=10,
                    check=False,
                )

            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    "Spawn attempt {} timed out."
                    .format(attempt)
                )

                time.sleep(1.0)

                continue

            if result.returncode == 0:
                self.get_logger().info(
                    "Spawned {}.".format(name)
                )

                return name

            self.get_logger().warn(
                "Spawn attempt {} failed: {}"
                .format(
                    attempt,
                    result.stderr.strip(),
                )
            )

        raise RuntimeError(
            "Failed to spawn {}.".format(name)
        )

    # ==================================================
    # SHUTDOWN
    # ==================================================

    def destroy_node(self):
        self.shutdown_event.set()

        if rclpy.ok():
            try:
                self.start_conveyor_visual(0.0)
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SortingNode()

    executor = rclpy.executors.MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()