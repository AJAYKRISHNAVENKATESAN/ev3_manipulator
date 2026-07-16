#!/usr/bin/env python3

# import math
# import subprocess
# import threading
# import time

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient

# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64, Float64MultiArray, String


# # ==================================================
# # GEOMETRY & SPATIAL ANCHORS
# # ==================================================

# SPAWN_X = -0.14824
# SPAWN_Y = 0.29075
# SPAWN_Z = 0.09232
# PICKUP_Z = SPAWN_Z + 0.022

# PICKUP_X = -0.015
# TRANSPORT_Z = SPAWN_Z + 0.005

# BELT_END_R = 0.17
# BELT_END_L = -0.17

# STEP_SIZE = 0.008
# STEP_DELAY = 0.090

# SIM_BASE_HOME = 0.0
# SIM_BASE_RED_BIN = -1.5708
# SIM_BASE_BLUE_BIN = 1.5708

# THETA1_MIN = -math.pi / 2
# THETA1_MAX = math.pi / 2
# THETA2_MIN = -0.55
# THETA2_MAX = math.pi / 3

# ARM_HOME = [0.0, 0.2]
# CLEARANCE_PITCH = 0.2

# SIM_GRIPPER_OPEN = 0.5
# SIM_GRIPPER_CLOSE = 0.0

# MAX_BALLS = 4


# # ==================================================
# # TIMING
# # ==================================================

# SIM_SETTLE_DELAY = 0.15

# SIM_HOME_START_TIME = 1.5
# SIM_HOME_START_WAIT = 2.6

# SIM_HOME_PITCH_UP_TIME = 1.0
# SIM_HOME_PITCH_UP_WAIT = 1.1

# SIM_HOME_BASE_RIGHT_TIME = 1.2
# SIM_HOME_BASE_RIGHT_WAIT = 1.3

# SIM_HOME_BASE_CENTER_TIME = 1.2
# SIM_HOME_BASE_CENTER_WAIT = 1.3

# SIM_PICKUP_READY_TIME = 1.0
# SIM_PICKUP_READY_WAIT = 1.1

# SIM_PICK_DOWN_TIME = 0.8
# SIM_PICK_DOWN_WAIT = 0.9

# SIM_PICK_UP_TIME = 0.8
# SIM_PICK_UP_WAIT = 0.9

# SIM_SWIVEL_TO_BIN_TIME = 1.2
# SIM_SWIVEL_TO_BIN_WAIT = 1.3

# SIM_DROP_DOWN_TIME = 0.8
# SIM_DROP_DOWN_WAIT = 0.9

# SIM_RELEASE_WAIT_1 = 0.4
# SIM_RELEASE_WAIT_2 = 0.6

# SIM_DROP_UP_TIME = 0.8
# SIM_DROP_UP_WAIT = 0.9

# SIM_RETURN_HOME_TIME = 1.2
# SIM_RETURN_HOME_WAIT = 1.3

# SIM_FINAL_HOME_TIME = 2.0


# BALL_RGB = {
#     'red': (1, 0, 0),
#     'blue': (0, 0, 1),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0, 0.8, 0),
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

#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

#     return theta1, theta2


# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting_node')

#         # ---------------- Controllers ----------------
#         self._arm_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory'
#         )

#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray,
#             '/gripper_controller/commands',
#             10
#         )

#         self._belt_vel_pub = self.create_publisher(
#             Float64,
#             '/conveyor_belt_vel',
#             10
#         )

#         # ---------------- Hardware interface subscribers ----------------
#         self._start_homing_sub = self.create_subscription(
#             String,
#             '/hw/start_homing',
#             self._start_homing_callback,
#             10
#         )

#         self._ev3_homed_sub = self.create_subscription(
#             String,
#             '/hw/ev3_homed',
#             self._ev3_homed_callback,
#             10
#         )

#         self._ev3_gripper_open_sub = self.create_subscription(
#             String,
#             '/hw/ev3_gripper_open',
#             self._ev3_gripper_open_callback,
#             10
#         )

#         self._color_sub = self.create_subscription(
#             String,
#             '/hw/ball_detected',
#             self._ball_detected_callback,
#             10
#         )

#         self._ready_pickup_sub = self.create_subscription(
#             String,
#             '/hw/ready_pickup',
#             self._ready_pickup_callback,
#             10
#         )

#         self._ready_place_sub = self.create_subscription(
#             String,
#             '/hw/ready_place',
#             self._ready_place_callback,
#             10
#         )

#         self._theta1_sub = self.create_subscription(
#             Float64,
#             '/hw/theta1',
#             self._theta1_callback,
#             10
#         )

#         self._theta2_sub = self.create_subscription(
#             Float64,
#             '/hw/theta2',
#             self._theta2_callback,
#             10
#         )

#         # ---------------- Sim -> hardware interface publishers ----------------
#         self._sim_homed_pub = self.create_publisher(
#             String,
#             '/sim/homed',
#             10
#         )

#         self._sim_gripper_open_pub = self.create_publisher(
#             String,
#             '/sim/gripper_open',
#             10
#         )

#         self._spawn_confirmed_pub = self.create_publisher(
#             String,
#             '/sim/spawn_confirmed',
#             10
#         )

#         self._sim_ready_pickup_pub = self.create_publisher(
#             String,
#             '/sim/ready_pickup',
#             10
#         )

#         self._sim_ready_place_pub = self.create_publisher(
#             String,
#             '/sim/ready_place',
#             10
#         )

#         self._sim_cycle_done_pub = self.create_publisher(
#             String,
#             '/sim/cycle_done',
#             10
#         )

#         # ---------------- Thread synchronization ----------------
#         self._start_homing_event = threading.Event()
#         self._ev3_homed_event = threading.Event()
#         self._ev3_gripper_open_event = threading.Event()

#         self._color_event = threading.Event()
#         self._pickup_event = threading.Event()
#         self._place_event = threading.Event()

#         self._detected_color = None
#         self.ball_count = 0

#         threading.Thread(
#             target=self._start,
#             daemon=True
#         ).start()

#     # ==================================================
#     # ROS utility
#     # ==================================================

#     def _publish_string(self, publisher, text):
#         msg = String()
#         msg.data = text
#         publisher.publish(msg)

#     # ==================================================
#     # Subscriber callbacks
#     # ==================================================

#     def _start_homing_callback(self, msg):
#         self.get_logger().info("Received /hw/start_homing.")
#         self._start_homing_event.set()

#     def _ev3_homed_callback(self, msg):
#         self.get_logger().info("Received /hw/ev3_homed.")
#         self._ev3_homed_event.set()

#     def _ev3_gripper_open_callback(self, msg):
#         self.get_logger().info("Received /hw/ev3_gripper_open.")
#         self._ev3_gripper_open_event.set()

#     def _ball_detected_callback(self, msg):
#         color = msg.data.strip().lower()

#         if color not in ['red', 'blue', 'black', 'green']:
#             self.get_logger().warn(f"Ignoring unknown color: {color}")
#             return

#         self._detected_color = color
#         self._color_event.set()

#         self.get_logger().info(f"Hardware detected ball: {color}")

#     def _ready_pickup_callback(self, msg):
#         self.get_logger().info("Hardware reached pickup-ready pose.")
#         self._pickup_event.set()

#     def _ready_place_callback(self, msg):
#         self.get_logger().info("Hardware reached place-ready pose.")
#         self._place_event.set()

#     def _theta1_callback(self, msg):
#         self.get_logger().info(f"[EV3 telemetry] theta1 = {msg.data:.2f} deg")

#     def _theta2_callback(self, msg):
#         self.get_logger().info(f"[EV3 telemetry] theta2 = {msg.data:.2f} deg")

#     # ==================================================
#     # Main startup thread
#     # ==================================================

#     def _start(self):
#         self.get_logger().info("Waiting for arm action server...")
#         self._arm_client.wait_for_server()
#         self.get_logger().info("Arm action server available.")

#         self.get_logger().info("Starting old stage-sync sorting loop.")
#         self._main_loop()

#     # ==================================================
#     # Homing
#     # ==================================================

#     def _execute_initial_homing(self):
#         """
#         Simulated homing sequence.

#         Important:
#         The gripper is NOT opened here.
#         It opens only after EV3 sends EV3_GRIPPER_OPEN.
#         """

#         self.get_logger().info("Executing simulated homing sequence...")

#         self._send_trajectory(
#             positions=[[0.0, 0.0]],
#             durations=[SIM_HOME_START_TIME]
#         )
#         time.sleep(SIM_HOME_START_WAIT)

#         self.get_logger().info("Homing step 1: arm_2 pitching up to 0.2 rad.")
#         self._send_trajectory(
#             positions=[[0.0, 0.2]],
#             durations=[SIM_HOME_PITCH_UP_TIME]
#         )
#         time.sleep(SIM_HOME_PITCH_UP_WAIT)

#         self.get_logger().info("Homing step 2: arm_1 rotating right by +60 deg.")
#         self._send_trajectory(
#             positions=[[1.0472, 0.2]],
#             durations=[SIM_HOME_BASE_RIGHT_TIME]
#         )
#         time.sleep(SIM_HOME_BASE_RIGHT_WAIT)

#         self.get_logger().info("Homing step 3: arm_1 returning to center/home.")
#         self._send_trajectory(
#             positions=[[0.0, 0.2]],
#             durations=[SIM_HOME_BASE_CENTER_TIME]
#         )
#         time.sleep(SIM_HOME_BASE_CENTER_WAIT)

#         self.get_logger().info("Sim homing motion complete.")

#     def _execute_cycle_homing(self):
#         """
#         Per-ball and final homing cycle.

#         EV3 -> START_HOMING
#         Sim homes.
#         EV3 -> EV3_HOMED
#         Sim -> ROS_HOMED
#         EV3 opens gripper.
#         EV3 -> EV3_GRIPPER_OPEN
#         Sim opens gripper.
#         Sim -> ROS_GRIPPER_OPEN
#         """

#         self.get_logger().info("Waiting for EV3 START_HOMING...")
#         self._start_homing_event.wait()
#         self._start_homing_event.clear()

#         self._ev3_homed_event.clear()
#         self._ev3_gripper_open_event.clear()
#         self._pickup_event.clear()
#         self._place_event.clear()

#         self._execute_initial_homing()

#         self.get_logger().info("Waiting for EV3_HOMED...")
#         self._ev3_homed_event.wait()
#         self._ev3_homed_event.clear()

#         self.get_logger().info("Publishing /sim/homed.")
#         self._publish_string(self._sim_homed_pub, "done")

#         self.get_logger().info("Waiting for EV3_GRIPPER_OPEN...")
#         self._ev3_gripper_open_event.wait()
#         self._ev3_gripper_open_event.clear()

#         self.get_logger().info("Opening simulated gripper.")
#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(1.0)

#         self.get_logger().info("Publishing /sim/gripper_open.")
#         self._publish_string(self._sim_gripper_open_pub, "done")

#     # ==================================================
#     # Main coordinated loop
#     # ==================================================

#     def _main_loop(self):
#         self.start_conveyor(0.05)

#         while self.ball_count < MAX_BALLS:
#             self._execute_cycle_homing()

#             self.get_logger().info("Waiting for hardware ball detection...")

#             self._color_event.wait()
#             color = self._detected_color

#             self._detected_color = None
#             self._color_event.clear()

#             if color is None:
#                 continue

#             self.get_logger().info(
#                 f"=== Ball {self.ball_count + 1}: {color.upper()} ==="
#             )

#             name = self._spawn(color)
#             time.sleep(0.1)

#             self._publish_string(self._spawn_confirmed_pub, color)
#             self.get_logger().info("Published /sim/spawn_confirmed.")

#             if color in ['red', 'blue']:
#                 self._move_ball(name, SPAWN_X, PICKUP_X)

#                 self._pick_place_sequence(left_side=(color == 'red'))

#                 self._publish_string(self._sim_cycle_done_pub, color)
#                 self.get_logger().info("Published /sim/cycle_done.")

#             elif color in ['black', 'green']:
#                 direction = 1 if color == 'black' else -1

#                 self._fall(name, direction=direction)
#                 time.sleep(1.5)

#                 self._publish_string(self._sim_cycle_done_pub, color)
#                 self.get_logger().info("Published /sim/cycle_done.")

#         self.get_logger().info(
#             "All balls sorted. Waiting for final EV3 homing request..."
#         )

#         self._execute_cycle_homing()

#         self.stop_conveyor()

#         self._send_trajectory(
#             positions=[ARM_HOME],
#             durations=[SIM_FINAL_HOME_TIME]
#         )

#         self._grip(SIM_GRIPPER_OPEN)

#         self.get_logger().info("=== Execution cycle complete ===")

#     # ==================================================
#     # Red / Blue pick-place sequence - old coarse sync
#     # ==================================================

#     def _pick_place_sequence(self, left_side=True):
#         bin_yaw = SIM_BASE_RED_BIN if left_side else SIM_BASE_BLUE_BIN

#         _, pick_pitch = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)

#         # ---------------- PICKUP READY ----------------
#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(0.1)

#         self.get_logger().info("Moving sim arm to pickup-ready pose.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
#             durations=[SIM_PICKUP_READY_TIME]
#         )
#         time.sleep(SIM_PICKUP_READY_WAIT)

#         self.get_logger().info("Waiting for EV3 READY_PICKUP...")
#         self._pickup_event.wait()
#         self._pickup_event.clear()

#         self._publish_string(self._sim_ready_pickup_pub, "ready")
#         self.get_logger().info("Published /sim/ready_pickup.")

#         # ---------------- PICK ----------------
#         self.get_logger().info("Sim pitching down to pick ball.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, pick_pitch]],
#             durations=[SIM_PICK_DOWN_TIME]
#         )
#         time.sleep(SIM_PICK_DOWN_WAIT)

#         self.get_logger().info("Closing simulated gripper.")
#         self._grip(SIM_GRIPPER_CLOSE)
#         time.sleep(0.5)

#         self.get_logger().info("Sim pitching up to clearance with ball.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
#             durations=[SIM_PICK_UP_TIME]
#         )
#         time.sleep(SIM_PICK_UP_WAIT)

#         # ---------------- PLACE READY ----------------
#         self.get_logger().info("Waiting for EV3 READY_PLACE...")
#         self._place_event.wait()
#         self._place_event.clear()

#         self._publish_string(self._sim_ready_place_pub, "ready")
#         self.get_logger().info("Published /sim/ready_place.")

#         # ---------------- PLACE ----------------
#         self.get_logger().info(
#             f"Sim swiveling to bin yaw {math.degrees(bin_yaw):.1f} deg."
#         )
#         self._send_trajectory(
#             positions=[[bin_yaw, CLEARANCE_PITCH]],
#             durations=[SIM_SWIVEL_TO_BIN_TIME]
#         )
#         time.sleep(SIM_SWIVEL_TO_BIN_WAIT)

#         self.get_logger().info("Sim pitching down into bin.")
#         self._send_trajectory(
#             positions=[[bin_yaw, -0.45]],
#             durations=[SIM_DROP_DOWN_TIME]
#         )
#         time.sleep(SIM_DROP_DOWN_WAIT)

#         self.get_logger().info("Opening simulated gripper to release ball.")
#         self._grip(0.2)
#         time.sleep(SIM_RELEASE_WAIT_1)

#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(SIM_RELEASE_WAIT_2)

#         self.get_logger().info("Sim pitching up from bin.")
#         self._send_trajectory(
#             positions=[[bin_yaw, CLEARANCE_PITCH]],
#             durations=[SIM_DROP_UP_TIME]
#         )
#         time.sleep(SIM_DROP_UP_WAIT)

#         self.get_logger().info("Sim returning to pickup/home state.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
#             durations=[SIM_RETURN_HOME_TIME]
#         )
#         time.sleep(SIM_RETURN_HOME_WAIT)

#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(0.3)

#     # ==================================================
#     # Controller helpers
#     # ==================================================

#     def _send_trajectory(self, positions, durations):
#         goal = FollowJointTrajectory.Goal()

#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint'
#         ]

#         for pos, t in zip(positions, durations):
#             point = JointTrajectoryPoint()
#             point.positions = [float(v) for v in pos]

#             point.time_from_start = Duration(
#                 sec=int(t),
#                 nanosec=int((t - int(t)) * 1e9)
#             )

#             goal.trajectory.points.append(point)

#         future = self._arm_client.send_goal_async(goal)

#         while rclpy.ok() and not future.done():
#             time.sleep(0.01)

#         goal_handle = future.result()

#         if not goal_handle.accepted:
#             self.get_logger().error("Trajectory goal rejected.")
#             return

#         result_future = goal_handle.get_result_async()

#         while rclpy.ok() and not result_future.done():
#             time.sleep(0.01)

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)

#     def start_conveyor(self, speed=0.05):
#         msg = Float64()
#         msg.data = float(speed)
#         self._belt_vel_pub.publish(msg)

#     def stop_conveyor(self):
#         self.start_conveyor(0.0)

#     # ==================================================
#     # Ball motion helpers
#     # ==================================================

#     def _move_ball(self, name, x_start, x_end):
#         x = x_start

#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)

#             self._set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 TRANSPORT_Z
#             )

#             time.sleep(STEP_DELAY)

#     def _fall(self, name, direction):
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         x = SPAWN_X
#         z = TRANSPORT_Z

#         step = STEP_SIZE * direction

#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step

#             if (
#                 (direction > 0 and x > BELT_END_R)
#                 or
#                 (direction < 0 and x < BELT_END_L)
#             ):
#                 z = max(z - 0.006, -0.05)

#             self._set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 z
#             )

#             time.sleep(STEP_DELAY)

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign',
#             'service',
#             '-s',
#             '/world/empty/set_pose',
#             '--reqtype',
#             'ignition.msgs.Pose',
#             '--reptype',
#             'ignition.msgs.Boolean',
#             '--timeout',
#             '150',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]

#         subprocess.Popen(
#             cmd,
#             stdout=subprocess.DEVNULL,
#             stderr=subprocess.DEVNULL
#         )

#     def _spawn(self, color, retries=3):
#         self.ball_count += 1

#         name = f'{color}_ball_{self.ball_count}'

#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))

#         sdf = BALL_SDF.format(
#             name=name,
#             r=r,
#             g=g,
#             b=b
#         )

#         cmd = [
#             'ros2',
#             'run',
#             'ros_gz_sim',
#             'create',
#             '-name',
#             name,
#             '-x',
#             str(SPAWN_X),
#             '-y',
#             str(SPAWN_Y),
#             '-z',
#             str(SPAWN_Z),
#             '-string',
#             sdf
#         ]

#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(
#                     cmd,
#                     capture_output=True,
#                     text=True,
#                     timeout=10
#                 )

#                 if result.returncode == 0:
#                     self.get_logger().info(f"Spawned {name}.")
#                     return name

#                 self.get_logger().warn(
#                     f"Spawn attempt {attempt + 1} failed: {result.stderr}"
#                 )

#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     f"Spawn attempt {attempt + 1} timed out."
#                 )
#                 time.sleep(1.0)

#         self.get_logger().error(
#             f"Failed to spawn {name}; continuing anyway."
#         )

#         return name


# def main(args=None):
#     rclpy.init(args=args)

#     node = SortingNode()

#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)

#     try:
#         executor.spin()

#     except KeyboardInterrupt:
#         pass

#     finally:
#         node.stop_conveyor()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
# """Gazebo follower for the EV3 stage-synchronised digital twin.

# The simulation never decides the task order. It only executes the stage named
# by the EV3 and acknowledges a stage after both conditions are true:
#   * the corresponding simulation action has completed, and
#   * STAGE_HW_DONE for the same cycle/sequence/stage has arrived.
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
# # SIMULATION-SPECIFIC TARGETS
# # ==================================================

# # These targets intentionally belong only to the larger simulated model.
# # They do not have to match the EV3 model's Cartesian dimensions.
# SIM_BASE_HOME = 0.0
# SIM_BASE_RED_BIN = -math.pi / 2
# SIM_BASE_BLUE_BIN = math.pi / 2
# CLEARANCE_PITCH = 0.2
# SIM_PICK_PITCH = -0.48
# SIM_PLACE_PITCH = -0.45

# SIM_GRIPPER_OPEN = 0.5
# SIM_GRIPPER_CLOSE = 0.0

# TRAJECTORY_TIMES = {
#     'HOME': 1.5,
#     'PICKUP_READY': 1.0,
#     'PICK_DOWN': 1.2,
#     'PICK_UP': 1.2,
#     'ROTATE_RED': 1.5,
#     'ROTATE_BLUE': 1.5,
#     'PLACE_DOWN': 1.2,
#     'PLACE_UP': 1.2,
#     'RETURN_HOME': 1.5,
# }

# GRIPPER_SETTLE_TIME = 0.45


# # ==================================================
# # BALL / CONVEYOR MODEL
# # ==================================================

# WORLD_NAME = 'empty'
# SPAWN_X = -0.14824
# SPAWN_Y = 0.29075
# SPAWN_Z = 0.09232
# TRANSPORT_Z = SPAWN_Z + 0.005
# PICKUP_X = -0.015
# BELT_END_R = 0.17
# BELT_END_L = -0.17

# # This pose-based backend is retained for the first synchronisation test.
# # It is isolated in _move_ball_to_pickup() and _fall_ball(), so it can later
# # be replaced by the IFRA conveyor plugin without changing the state machine.
# STEP_SIZE = 0.012
# STEP_DELAY = 0.03

# BALL_RGB = {
#     'red': (1.0, 0.0, 0.0),
#     'blue': (0.0, 0.0, 1.0),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0.0, 0.8, 0.0),
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
#           <sphere><radius>0.0185</radius></sphere>
#         </geometry>
#         <surface>
#           <friction>
#             <ode><mu>0.7</mu><mu2>0.7</mu2></ode>
#           </friction>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry>
#           <sphere><radius>0.0185</radius></sphere>
#         </geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting_node')

#         self.arm_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory',
#         )

#         self.gripper_pub = self.create_publisher(
#             Float64MultiArray,
#             '/gripper_controller/commands',
#             10,
#         )

#         self.belt_vel_pub = self.create_publisher(
#             Float64,
#             '/conveyor_belt_vel',
#             10,
#         )

#         self.stage_event_sub = self.create_subscription(
#             String,
#             '/hw/stage_event',
#             self.stage_event_callback,
#             20,
#         )

#         self.stage_sync_pub = self.create_publisher(
#             String,
#             '/sim/stage_sync',
#             20,
#         )

#         self.event_queue = queue.Queue()
#         self.shutdown_event = threading.Event()

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False
#         self.current_ball_name = None

#         self.worker_thread = threading.Thread(
#             target=self.worker_loop,
#             daemon=True,
#         )
#         self.worker_thread.start()

#     # ==================================================
#     # Protocol helpers
#     # ==================================================

#     @staticmethod
#     def parse_stage_line(line, expected_kind):
#         parts = line.split('|')

#         if len(parts) != 4 or parts[0] != expected_kind:
#             raise ValueError(f'Invalid {expected_kind} packet: {line}')

#         return int(parts[1]), int(parts[2]), parts[3]

#     @staticmethod
#     def make_stage_line(kind, cycle_id, sequence_id, stage):
#         return f'{kind}|{cycle_id}|{sequence_id}|{stage}'

#     def publish_stage_sync(self, text):
#         msg = String()
#         msg.data = text
#         self.stage_sync_pub.publish(msg)
#         self.get_logger().info(f'-> EV3 [{text}]')

#     def stage_event_callback(self, msg):
#         self.event_queue.put(msg.data.strip())

#     # ==================================================
#     # Event worker and barrier
#     # ==================================================

#     def worker_loop(self):
#         self.get_logger().info('Waiting for arm action server...')

#         if not self.arm_client.wait_for_server(timeout_sec=30.0):
#             self.get_logger().error('Arm action server was not available.')
#             return

#         self.get_logger().info('Arm action server available.')
#         self.start_conveyor_visual(0.05)

#         while rclpy.ok() and not self.shutdown_event.is_set():
#             try:
#                 line = self.event_queue.get(timeout=0.2)
#             except queue.Empty:
#                 continue

#             try:
#                 if line.startswith('STAGE_START|'):
#                     self.handle_stage_start(line)
#                 elif line.startswith('STAGE_HW_DONE|'):
#                     self.handle_hardware_done(line)
#                 elif line == 'EV3_DONE':
#                     self.get_logger().info('EV3 completed the full task.')
#                     self.start_conveyor_visual(0.0)
#                 else:
#                     self.get_logger().warn(
#                         f'Ignoring unknown stage event: {line}'
#                     )
#             except Exception as exc:
#                 self.get_logger().error(
#                     f'Error while processing [{line}]: {exc}'
#                 )
#                 self.publish_failure_for_active_stage(str(exc))

#     def handle_stage_start(self, line):
#         cycle_id, sequence_id, stage = self.parse_stage_line(
#             line,
#             'STAGE_START',
#         )
#         key = (cycle_id, sequence_id, stage)

#         if self.active_key is not None:
#             raise RuntimeError(
#                 f'Received {key}, but stage {self.active_key} is still active.'
#             )

#         self.active_key = key
#         self.sim_done = False
#         self.hardware_done = False

#         self.get_logger().info(
#             f'=== START cycle={cycle_id} seq={sequence_id} stage={stage} ==='
#         )

#         self.execute_sim_stage(cycle_id, stage)
#         self.sim_done = True
#         self.try_complete_active_stage()

#     def handle_hardware_done(self, line):
#         cycle_id, sequence_id, stage = self.parse_stage_line(
#             line,
#             'STAGE_HW_DONE',
#         )
#         key = (cycle_id, sequence_id, stage)

#         if self.active_key != key:
#             raise RuntimeError(
#                 f'Hardware completed {key}, active stage is {self.active_key}.'
#             )

#         self.hardware_done = True
#         self.get_logger().info(
#             f'Hardware completed cycle={cycle_id} seq={sequence_id} '
#             f'stage={stage}.'
#         )
#         self.try_complete_active_stage()

#     def try_complete_active_stage(self):
#         if not (self.sim_done and self.hardware_done):
#             return

#         cycle_id, sequence_id, stage = self.active_key
#         response = self.make_stage_line(
#             'STAGE_SYNC_DONE',
#             cycle_id,
#             sequence_id,
#             stage,
#         )
#         self.publish_stage_sync(response)

#         self.get_logger().info(
#             f'=== DONE cycle={cycle_id} seq={sequence_id} stage={stage} ==='
#         )

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#     def publish_failure_for_active_stage(self, reason):
#         if self.active_key is None:
#             return

#         cycle_id, sequence_id, stage = self.active_key
#         safe_reason = reason.replace('|', '/').replace('\n', ' ')
#         response = (
#             f'STAGE_SYNC_FAILED|{cycle_id}|{sequence_id}|{stage}|{safe_reason}'
#         )
#         self.publish_stage_sync(response)

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#     # ==================================================
#     # Stage-to-simulation mapping
#     # ==================================================

#     def execute_sim_stage(self, cycle_id, stage):
#         if stage == 'HOME':
#             self.send_arm_target(
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['HOME'],
#             )
#             return

#         if stage == 'GRIPPER_OPEN':
#             self.command_gripper(SIM_GRIPPER_OPEN)
#             time.sleep(GRIPPER_SETTLE_TIME)
#             return

#         if stage.startswith('SPAWN_'):
#             color = stage.split('_', 1)[1].lower()
#             self.current_ball_name = self.spawn_ball(color, cycle_id)
#             return

#         if stage == 'CONVEYOR_TO_PICKUP':
#             self.require_current_ball()
#             self.move_ball_to_pickup(self.current_ball_name)
#             return

#         if stage == 'CONVEYOR_BLACK':
#             self.require_current_ball()
#             self.fall_ball(self.current_ball_name, direction=1)
#             return

#         if stage == 'CONVEYOR_GREEN':
#             self.require_current_ball()
#             self.fall_ball(self.current_ball_name, direction=-1)
#             return

#         if stage == 'PICKUP_READY':
#             self.command_gripper(SIM_GRIPPER_OPEN)
#             self.send_arm_target(
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['PICKUP_READY'],
#             )
#             return

#         if stage == 'PICK_DOWN':
#             self.send_arm_target(
#                 [SIM_BASE_HOME, SIM_PICK_PITCH],
#                 TRAJECTORY_TIMES['PICK_DOWN'],
#             )
#             return

#         if stage == 'GRIP_CLOSE':
#             self.command_gripper(SIM_GRIPPER_CLOSE)
#             time.sleep(GRIPPER_SETTLE_TIME)
#             return

#         if stage == 'PICK_UP':
#             self.send_arm_target(
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['PICK_UP'],
#             )
#             return

#         if stage == 'ROTATE_RED':
#             self.send_arm_target(
#                 [SIM_BASE_RED_BIN, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['ROTATE_RED'],
#             )
#             return

#         if stage == 'ROTATE_BLUE':
#             self.send_arm_target(
#                 [SIM_BASE_BLUE_BIN, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['ROTATE_BLUE'],
#             )
#             return

#         if stage == 'PLACE_DOWN':
#             base_target = self.current_base_target()
#             self.send_arm_target(
#                 [base_target, SIM_PLACE_PITCH],
#                 TRAJECTORY_TIMES['PLACE_DOWN'],
#             )
#             return

#         if stage == 'GRIP_OPEN':
#             self.command_gripper(0.2)
#             time.sleep(0.2)
#             self.command_gripper(SIM_GRIPPER_OPEN)
#             time.sleep(GRIPPER_SETTLE_TIME)
#             return

#         if stage == 'PLACE_UP':
#             base_target = self.current_base_target()
#             self.send_arm_target(
#                 [base_target, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['PLACE_UP'],
#             )
#             return

#         if stage == 'RETURN_HOME':
#             self.send_arm_target(
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['RETURN_HOME'],
#             )
#             return

#         if stage == 'CYCLE_COMPLETE':
#             self.current_ball_name = None
#             return

#         raise ValueError(f'No simulation implementation for stage {stage}')

#     def current_base_target(self):
#         if self.active_key is None:
#             return SIM_BASE_HOME

#         # PLACE_DOWN / PLACE_UP follow the previously commanded bin yaw.
#         # The controller holds the base joint, so read the intended side from
#         # the most recently completed rotate stage stored below.
#         return getattr(self, '_last_bin_target', SIM_BASE_HOME)

#     def require_current_ball(self):
#         if not self.current_ball_name:
#             raise RuntimeError('No active simulated ball exists.')

#     # ==================================================
#     # Arm and gripper controller helpers
#     # ==================================================

#     def wait_for_future(self, future, timeout_sec):
#         event = threading.Event()
#         future.add_done_callback(lambda _future: event.set())

#         if not event.wait(timeout_sec):
#             raise TimeoutError('ROS action future timed out.')

#         return future.result()

#     def send_arm_target(self, positions, duration_sec):
#         if positions[0] in (SIM_BASE_RED_BIN, SIM_BASE_BLUE_BIN):
#             self._last_bin_target = positions[0]
#         elif positions[0] == SIM_BASE_HOME:
#             # Do not erase the remembered bin target during PICK_UP. It is
#             # overwritten on RETURN_HOME only after placement is complete.
#             if self.active_key and self.active_key[2] == 'RETURN_HOME':
#                 self._last_bin_target = SIM_BASE_HOME

#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint',
#         ]

#         point = JointTrajectoryPoint()
#         point.positions = [float(value) for value in positions]
#         point.time_from_start = Duration(
#             sec=int(duration_sec),
#             nanosec=int((duration_sec - int(duration_sec)) * 1e9),
#         )
#         goal.trajectory.points = [point]

#         goal_future = self.arm_client.send_goal_async(goal)
#         goal_handle = self.wait_for_future(goal_future, timeout_sec=3.0)

#         if goal_handle is None or not goal_handle.accepted:
#             raise RuntimeError('Trajectory goal was rejected.')

#         result_future = goal_handle.get_result_async()
#         wrapped_result = self.wait_for_future(
#             result_future,
#             timeout_sec=duration_sec + 4.0,
#         )

#         if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
#             raise RuntimeError(
#                 f'Trajectory action status was {wrapped_result.status}.'
#             )

#         result = wrapped_result.result

#         if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
#             raise RuntimeError(
#                 f'Trajectory controller error {result.error_code}: '
#                 f'{result.error_string}'
#             )

#     def command_gripper(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self.gripper_pub.publish(msg)

#     def start_conveyor_visual(self, speed):
#         msg = Float64()
#         msg.data = float(speed)
#         self.belt_vel_pub.publish(msg)

#     # ==================================================
#     # Ball helpers
#     # ==================================================

#     def spawn_ball(self, color, cycle_id, retries=3):
#         if color not in BALL_RGB:
#             raise ValueError(f'Unsupported ball colour: {color}')

#         name = f'{color}_ball_{cycle_id}'
#         r, g, b = BALL_RGB[color]
#         sdf = BALL_SDF.format(name=name, r=r, g=g, b=b)

#         cmd = [
#             'ros2',
#             'run',
#             'ros_gz_sim',
#             'create',
#             '-name',
#             name,
#             '-x',
#             str(SPAWN_X),
#             '-y',
#             str(SPAWN_Y),
#             '-z',
#             str(SPAWN_Z),
#             '-string',
#             sdf,
#         ]

#         for attempt in range(1, retries + 1):
#             try:
#                 result = subprocess.run(
#                     cmd,
#                     capture_output=True,
#                     text=True,
#                     timeout=10,
#                     check=False,
#                 )
#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     f'Spawn attempt {attempt} timed out.'
#                 )
#                 continue

#             if result.returncode == 0:
#                 self.get_logger().info(f'Spawned {name}.')
#                 return name

#             self.get_logger().warn(
#                 f'Spawn attempt {attempt} failed: {result.stderr.strip()}'
#             )

#         raise RuntimeError(f'Failed to spawn {name}.')

#     def move_ball_to_pickup(self, name):
#         x = SPAWN_X

#         while x < PICKUP_X:
#             x = min(x + STEP_SIZE, PICKUP_X)
#             self.set_ball_pose(name, x, SPAWN_Y, TRANSPORT_Z)
#             time.sleep(STEP_DELAY)

#     def fall_ball(self, name, direction):
#         x_end = (
#             BELT_END_L - 0.06
#             if direction < 0
#             else BELT_END_R + 0.06
#         )
#         x = SPAWN_X
#         z = TRANSPORT_Z
#         step = STEP_SIZE * direction

#         while (
#             (direction > 0 and x < x_end)
#             or (direction < 0 and x > x_end)
#         ):
#             x += step

#             if (
#                 (direction > 0 and x > BELT_END_R)
#                 or (direction < 0 and x < BELT_END_L)
#             ):
#                 z = max(z - 0.008, -0.05)

#             self.set_ball_pose(name, x, SPAWN_Y, z)
#             time.sleep(STEP_DELAY)

#     def set_ball_pose(self, name, x, y, z):
#         # Synchronous service execution prevents the short-lived async client
#         # processes that produced repeated "Host unreachable" responses.
#         cmd = [
#             'ign',
#             'service',
#             '-s',
#             f'/world/{WORLD_NAME}/set_pose',
#             '--reqtype',
#             'ignition.msgs.Pose',
#             '--reptype',
#             'ignition.msgs.Boolean',
#             '--timeout',
#             '1000',
#             '--req',
#             (
#                 f'name: "{name}" '
#                 f'position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#             ),
#         ]

#         try:
#             result = subprocess.run(
#                 cmd,
#                 stdout=subprocess.DEVNULL,
#                 stderr=subprocess.PIPE,
#                 text=True,
#                 timeout=2.0,
#                 check=False,
#             )
#         except subprocess.TimeoutExpired as exc:
#             raise RuntimeError(
#                 f'Set-pose request timed out for {name}.'
#             ) from exc

#         if result.returncode != 0:
#             raise RuntimeError(
#                 f'Set-pose failed for {name}: {result.stderr.strip()}'
#             )

#     # ==================================================
#     # Shutdown
#     # ==================================================

#     def destroy_node(self):
#         self.shutdown_event.set()
#         self.start_conveyor_visual(0.0)
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = SortingNode()

#     executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
#     executor.add_node(node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()














































# import math
# import subprocess
# import threading
# import time
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64, Float64MultiArray, String

# # ==================================================
# # GEOMETRY & SPATIAL ANCHORS
# # ==================================================

# SPAWN_X = -0.14824
# SPAWN_Y = 0.29075
# SPAWN_Z = 0.09232
# PICKUP_Z = SPAWN_Z + 0.022

# PICKUP_X = -0.015
# TRANSPORT_Z = SPAWN_Z + 0.005

# BELT_END_R = 0.17
# BELT_END_L = -0.17

# STEP_SIZE = 0.008
# STEP_DELAY = 0.090

# SIM_BASE_HOME = 0.0
# SIM_BASE_RED_BIN = -1.5708
# SIM_BASE_BLUE_BIN = 1.5708

# THETA1_MIN = -math.pi / 2
# THETA1_MAX = math.pi / 2
# THETA2_MIN = -0.55
# THETA2_MAX = math.pi / 3

# ARM_HOME = [0.0, 0.2]
# CLEARANCE_PITCH = 0.2

# SIM_GRIPPER_OPEN = 0.5
# SIM_GRIPPER_CLOSE = 0.0

# MAX_BALLS = 4

# BALL_RGB = {
#     'red': (1, 0, 0),
#     'blue': (0, 0, 1),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0, 0.8, 0),
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

#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

#     return theta1, theta2


# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting_node')

#         # ---------------- Controllers ----------------
#         self._arm_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory'
#         )

#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray,
#             '/gripper_controller/commands',
#             10
#         )

#         self._belt_vel_pub = self.create_publisher(
#             Float64,
#             '/conveyor_belt_vel',
#             10
#         )

#         # ---------------- Hardware interface subscribers ----------------
#         self._start_homing_sub = self.create_subscription(
#             String,
#             '/hw/start_homing',
#             self._start_homing_callback,
#             10
#         )

#         self._ev3_homed_sub = self.create_subscription(
#             String,
#             '/hw/ev3_homed',
#             self._ev3_homed_callback,
#             10
#         )

#         self._ev3_gripper_open_sub = self.create_subscription(
#             String,
#             '/hw/ev3_gripper_open',
#             self._ev3_gripper_open_callback,
#             10
#         )

#         self._color_sub = self.create_subscription(
#             String,
#             '/hw/ball_detected',
#             self._ball_detected_callback,
#             10
#         )

#         self._ready_pickup_sub = self.create_subscription(
#             String,
#             '/hw/ready_pickup',
#             self._ready_pickup_callback,
#             10
#         )

#         self._ready_place_sub = self.create_subscription(
#             String,
#             '/hw/ready_place',
#             self._ready_place_callback,
#             10
#         )

#         self._theta1_sub = self.create_subscription(
#             Float64,
#             '/hw/theta1',
#             self._theta1_callback,
#             10
#         )

#         self._theta2_sub = self.create_subscription(
#             Float64,
#             '/hw/theta2',
#             self._theta2_callback,
#             10
#         )

#         # ---------------- Sim -> hardware interface publishers ----------------
#         self._sim_homed_pub = self.create_publisher(
#             String,
#             '/sim/homed',
#             10
#         )

#         self._sim_gripper_open_pub = self.create_publisher(
#             String,
#             '/sim/gripper_open',
#             10
#         )

#         self._spawn_confirmed_pub = self.create_publisher(
#             String,
#             '/sim/spawn_confirmed',
#             10
#         )

#         self._sim_ready_pickup_pub = self.create_publisher(
#             String,
#             '/sim/ready_pickup',
#             10
#         )

#         self._sim_ready_place_pub = self.create_publisher(
#             String,
#             '/sim/ready_place',
#             10
#         )

#         self._sim_cycle_done_pub = self.create_publisher(
#             String,
#             '/sim/cycle_done',
#             10
#         )

#         # ---------------- Thread synchronization ----------------
#         self._start_homing_event = threading.Event()
#         self._ev3_homed_event = threading.Event()
#         self._ev3_gripper_open_event = threading.Event()

#         self._color_event = threading.Event()
#         self._pickup_event = threading.Event()
#         self._place_event = threading.Event()

#         self._detected_color = None
#         self.ball_count = 0

#         threading.Thread(
#             target=self._start,
#             daemon=True
#         ).start()

#     # ==================================================
#     # ROS utility
#     # ==================================================

#     def _publish_string(self, publisher, text):
#         msg = String()
#         msg.data = text
#         publisher.publish(msg)

#     # ==================================================
#     # Subscriber callbacks
#     # ==================================================

#     def _start_homing_callback(self, msg):
#         self.get_logger().info("Received /hw/start_homing.")
#         self._start_homing_event.set()

#     def _ev3_homed_callback(self, msg):
#         self.get_logger().info("Received /hw/ev3_homed.")
#         self._ev3_homed_event.set()

#     def _ev3_gripper_open_callback(self, msg):
#         self.get_logger().info("Received /hw/ev3_gripper_open.")
#         self._ev3_gripper_open_event.set()

#     def _ball_detected_callback(self, msg):
#         color = msg.data.strip().lower()

#         if color not in ['red', 'blue', 'black', 'green']:
#             self.get_logger().warn(f"Ignoring unknown color: {color}")
#             return

#         self._detected_color = color
#         self._color_event.set()

#         self.get_logger().info(f"Hardware detected ball: {color}")

#     def _ready_pickup_callback(self, msg):
#         self.get_logger().info("Hardware reached pickup-ready pose.")
#         self._pickup_event.set()

#     def _ready_place_callback(self, msg):
#         self.get_logger().info("Hardware reached place-ready pose.")
#         self._place_event.set()

#     def _theta1_callback(self, msg):
#         self.get_logger().info(f"[EV3 telemetry] theta1 = {msg.data:.2f} deg")

#     def _theta2_callback(self, msg):
#         self.get_logger().info(f"[EV3 telemetry] theta2 = {msg.data:.2f} deg")

#     # ==================================================
#     # Main startup thread
#     # ==================================================

#     def _start(self):
#         self.get_logger().info("Waiting for arm action server...")
#         self._arm_client.wait_for_server()
#         self.get_logger().info("Arm action server available.")

#         self.get_logger().info("Starting coordinated sorting loop.")
#         self._main_loop()

#     # ==================================================
#     # Homing
#     # ==================================================

#     def _execute_initial_homing(self):
#         self.get_logger().info("Executing simulated homing sequence...")

#         self._send_trajectory([[0.0, 0.0]], [1.5])
#         time.sleep(2.6)

#         self.get_logger().info("Homing step 1: arm_2 pitching up to 0.2 rad.")
#         self._send_trajectory([[0.0, 0.2]], [1.0])
#         time.sleep(1.1)

#         self.get_logger().info("Homing step 2: arm_1 rotating right by +60 deg.")
#         self._send_trajectory([[1.0472, 0.2]], [1.2])
#         time.sleep(1.3)

#         self.get_logger().info("Homing step 3: arm_1 returning to center/home.")
#         self._send_trajectory([[0.0, 0.2]], [1.2])
#         time.sleep(1.3)

#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(0.3)

#         self.get_logger().info("Sim homing motion complete.")

#     def _execute_cycle_homing(self):
#         """
#         Per-ball homing cycle.

#         EV3 sends START_HOMING.
#         Sim homes.
#         EV3 sends EV3_HOMED.
#         Sim publishes /sim/homed.
#         EV3 opens gripper and sends EV3_GRIPPER_OPEN.
#         Sim opens gripper and publishes /sim/gripper_open.
#         """

#         self.get_logger().info("Waiting for EV3 START_HOMING for this ball...")
#         self._start_homing_event.wait()
#         self._start_homing_event.clear()

#         self._ev3_homed_event.clear()
#         self._ev3_gripper_open_event.clear()

#         self._execute_initial_homing()

#         self.get_logger().info("Waiting for EV3_HOMED...")
#         self._ev3_homed_event.wait()
#         self._ev3_homed_event.clear()

#         self.get_logger().info("Publishing /sim/homed.")
#         self._publish_string(self._sim_homed_pub, "done")

#         self.get_logger().info("Waiting for EV3_GRIPPER_OPEN...")
#         self._ev3_gripper_open_event.wait()
#         self._ev3_gripper_open_event.clear()

#         self.get_logger().info("Opening simulated gripper.")
#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(1.0)

#         self.get_logger().info("Publishing /sim/gripper_open.")
#         self._publish_string(self._sim_gripper_open_pub, "done")

#     # ==================================================
#     # Main coordinated loop
#     # ==================================================

#     def _main_loop(self):
#         self.start_conveyor(0.05)

#         while self.ball_count < MAX_BALLS:
#             # Home the sim every cycle, triggered by EV3.
#             self._execute_cycle_homing()

#             self.get_logger().info("Waiting for hardware ball detection...")

#             self._color_event.wait()
#             color = self._detected_color

#             self._detected_color = None
#             self._color_event.clear()

#             if color is None:
#                 continue

#             self.get_logger().info(
#                 f"=== Ball {self.ball_count + 1}: {color.upper()} ==="
#             )

#             # Spawn immediately after EV3 sensor detects the ball.
#             name = self._spawn(color)
#             time.sleep(0.1)

#             self._publish_string(self._spawn_confirmed_pub, color)
#             self.get_logger().info("Published /sim/spawn_confirmed.")

#             if color in ['red', 'blue']:
#                 # Sim ball travels to pickup while EV3 conveyor moves real ball.
#                 self._move_ball(name, SPAWN_X, PICKUP_X)

#                 # Old smooth coarse sync.
#                 self._pick_place_sequence(left_side=(color == 'red'))

#                 self._publish_string(self._sim_cycle_done_pub, color)
#                 self.get_logger().info("Published /sim/cycle_done.")

#             elif color in ['black', 'green']:
#                 # Correct behavior:
#                 # black -> runs down conveyor
#                 # green -> falls near sensor side
#                 direction = 1 if color == 'black' else -1

#                 self._fall(name, direction=direction)
#                 time.sleep(1.5)

#                 self._publish_string(self._sim_cycle_done_pub, color)
#                 self.get_logger().info("Published /sim/cycle_done.")

#         self.stop_conveyor()
#         self._send_trajectory([ARM_HOME], [2.0])
#         self._grip(SIM_GRIPPER_OPEN)

#         self.get_logger().info("=== Execution cycle complete ===")

#     # ==================================================
#     # Red / Blue pick-place sequence - coarse sync
#     # ==================================================

#     def _pick_place_sequence(self, left_side=True):
#         bin_yaw = SIM_BASE_RED_BIN if left_side else SIM_BASE_BLUE_BIN

#         _, pick_pitch = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)

#         # ---------------- PICKUP READY ----------------
#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(0.1)

#         self.get_logger().info("Moving sim arm to pickup-ready pose.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
#             durations=[1.0]
#         )
#         time.sleep(1.1)

#         self.get_logger().info("Waiting for EV3 READY_PICKUP...")
#         self._pickup_event.wait()
#         self._pickup_event.clear()

#         self._publish_string(self._sim_ready_pickup_pub, "ready")
#         self.get_logger().info("Published /sim/ready_pickup.")

#         # ---------------- PICK ----------------
#         self.get_logger().info("Sim pitching down to pick ball.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, pick_pitch]],
#             durations=[0.8]
#         )
#         time.sleep(0.9)

#         self.get_logger().info("Closing simulated gripper.")
#         self._grip(SIM_GRIPPER_CLOSE)
#         time.sleep(0.5)

#         self.get_logger().info("Sim pitching up to clearance with ball.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
#             durations=[0.8]
#         )
#         time.sleep(0.9)

#         # ---------------- PLACE READY ----------------
#         self.get_logger().info("Waiting for EV3 READY_PLACE...")
#         self._place_event.wait()
#         self._place_event.clear()

#         self._publish_string(self._sim_ready_place_pub, "ready")
#         self.get_logger().info("Published /sim/ready_place.")

#         # ---------------- PLACE ----------------
#         self.get_logger().info(
#             f"Sim swiveling to bin yaw {math.degrees(bin_yaw):.1f} deg."
#         )
#         self._send_trajectory(
#             positions=[[bin_yaw, CLEARANCE_PITCH]],
#             durations=[1.2]
#         )
#         time.sleep(1.3)

#         self.get_logger().info("Sim pitching down into bin.")
#         self._send_trajectory(
#             positions=[[bin_yaw, -0.45]],
#             durations=[0.8]
#         )
#         time.sleep(0.9)

#         self.get_logger().info("Opening simulated gripper to release ball.")
#         self._grip(0.2)
#         time.sleep(0.4)

#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(0.6)

#         self.get_logger().info("Sim pitching up from bin.")
#         self._send_trajectory(
#             positions=[[bin_yaw, CLEARANCE_PITCH]],
#             durations=[0.8]
#         )
#         time.sleep(0.9)

#         self.get_logger().info("Sim returning to pickup/home state.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
#             durations=[1.2]
#         )
#         time.sleep(1.3)

#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(0.3)

#     # ==================================================
#     # Controller helpers
#     # ==================================================

#     def _send_trajectory(self, positions, durations):
#         goal = FollowJointTrajectory.Goal()

#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint'
#         ]

#         for pos, t in zip(positions, durations):
#             point = JointTrajectoryPoint()
#             point.positions = [float(v) for v in pos]
#             point.time_from_start = Duration(
#                 sec=int(t),
#                 nanosec=int((t - int(t)) * 1e9)
#             )
#             goal.trajectory.points.append(point)

#         future = self._arm_client.send_goal_async(goal)

#         while rclpy.ok() and not future.done():
#             time.sleep(0.01)

#         goal_handle = future.result()

#         if not goal_handle.accepted:
#             self.get_logger().error("Trajectory goal rejected.")
#             return

#         result_future = goal_handle.get_result_async()

#         while rclpy.ok() and not result_future.done():
#             time.sleep(0.01)

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)

#     def start_conveyor(self, speed=0.05):
#         msg = Float64()
#         msg.data = float(speed)
#         self._belt_vel_pub.publish(msg)

#     def stop_conveyor(self):
#         self.start_conveyor(0.0)

#     # ==================================================
#     # Ball motion helpers
#     # ==================================================

#     def _move_ball(self, name, x_start, x_end):
#         x = x_start

#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)

#             self._set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 TRANSPORT_Z
#             )

#             time.sleep(STEP_DELAY)

#     def _fall(self, name, direction):
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         x = SPAWN_X
#         z = TRANSPORT_Z

#         step = STEP_SIZE * direction

#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step

#             if (
#                 (direction > 0 and x > BELT_END_R)
#                 or
#                 (direction < 0 and x < BELT_END_L)
#             ):
#                 z = max(z - 0.006, -0.05)

#             self._set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 z
#             )

#             time.sleep(STEP_DELAY)

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign',
#             'service',
#             '-s',
#             '/world/empty/set_pose',
#             '--reqtype',
#             'ignition.msgs.Pose',
#             '--reptype',
#             'ignition.msgs.Boolean',
#             '--timeout',
#             '150',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]

#         subprocess.Popen(
#             cmd,
#             stdout=subprocess.DEVNULL,
#             stderr=subprocess.DEVNULL
#         )

#     def _spawn(self, color, retries=3):
#         self.ball_count += 1

#         name = f'{color}_ball_{self.ball_count}'

#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))

#         sdf = BALL_SDF.format(
#             name=name,
#             r=r,
#             g=g,
#             b=b
#         )

#         cmd = [
#             'ros2',
#             'run',
#             'ros_gz_sim',
#             'create',
#             '-name',
#             name,
#             '-x',
#             str(SPAWN_X),
#             '-y',
#             str(SPAWN_Y),
#             '-z',
#             str(SPAWN_Z),
#             '-string',
#             sdf
#         ]

#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(
#                     cmd,
#                     capture_output=True,
#                     text=True,
#                     timeout=10
#                 )

#                 if result.returncode == 0:
#                     self.get_logger().info(f"Spawned {name}.")
#                     return name

#                 self.get_logger().warn(
#                     f"Spawn attempt {attempt + 1} failed: {result.stderr}"
#                 )

#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     f"Spawn attempt {attempt + 1} timed out."
#                 )
#                 time.sleep(1.0)

#         self.get_logger().error(
#             f"Failed to spawn {name}; continuing anyway."
#         )

#         return name


# def main(args=None):
#     rclpy.init(args=args)

#     node = SortingNode()

#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)

#     try:
#         executor.spin()

#     except KeyboardInterrupt:
#         pass

#     finally:
#         node.stop_conveyor()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()




#!/usr/bin/env python3

# # import math
# # import subprocess
# # import threading
# # import time

# # import rclpy
# # from rclpy.node import Node
# # from rclpy.action import ActionClient

# # from control_msgs.action import FollowJointTrajectory
# # from trajectory_msgs.msg import JointTrajectoryPoint
# # from builtin_interfaces.msg import Duration
# # from std_msgs.msg import Float64, Float64MultiArray, String


# # # ==================================================
# # # GEOMETRY & SPATIAL ANCHORS
# # # ==================================================

# # SPAWN_X = -0.14824
# # SPAWN_Y = 0.29075
# # SPAWN_Z = 0.09232
# # PICKUP_Z = SPAWN_Z + 0.022

# # PICKUP_X = -0.015
# # TRANSPORT_Z = SPAWN_Z + 0.005

# # BELT_END_R = 0.17
# # BELT_END_L = -0.17

# # STEP_SIZE = 0.008
# # STEP_DELAY = 0.090

# # SIM_BASE_HOME = 0.0
# # SIM_BASE_RED_BIN = -1.5708
# # SIM_BASE_BLUE_BIN = 1.5708

# # THETA1_MIN = -math.pi / 2
# # THETA1_MAX = math.pi / 2
# # THETA2_MIN = -0.55
# # THETA2_MAX = math.pi / 3

# # ARM_HOME = [0.0, 0.2]
# # CLEARANCE_PITCH = 0.2

# # SIM_GRIPPER_OPEN = 0.5
# # SIM_GRIPPER_CLOSE = 0.0

# # MAX_BALLS = 4

# # # Increase/decrease this for visual sync.
# # # Good values: 1.0, 1.5, 2.0, 3.0
# # VISUAL_SYNC_DELAY = 1.5

# # BALL_RGB = {
# #     'red': (1, 0, 0),
# #     'blue': (0, 0, 1),
# #     'black': (0.05, 0.05, 0.05),
# #     'green': (0, 0.8, 0),
# # }

# # BALL_SDF = """<sdf version='1.7'>
# #   <model name='{name}'>
# #     <link name='link'>
# #       <inertial>
# #         <mass>0.05</mass>
# #         <inertia>
# #           <ixx>8e-06</ixx>
# #           <iyy>8e-06</iyy>
# #           <izz>8e-06</izz>
# #         </inertia>
# #       </inertial>
# #       <collision name='col'>
# #         <geometry>
# #           <sphere>
# #             <radius>0.0185</radius>
# #           </sphere>
# #         </geometry>
# #         <surface>
# #           <friction>
# #             <ode>
# #               <mu>0.7</mu>
# #               <mu2>0.7</mu2>
# #             </ode>
# #           </friction>
# #         </surface>
# #       </collision>
# #       <visual name='vis'>
# #         <geometry>
# #           <sphere>
# #             <radius>0.0185</radius>
# #           </sphere>
# #         </geometry>
# #         <material>
# #           <ambient>{r} {g} {b} 1</ambient>
# #           <diffuse>{r} {g} {b} 1</diffuse>
# #         </material>
# #       </visual>
# #     </link>
# #   </model>
# # </sdf>"""


# # def solve_ik_sim(x, y, z_target):
# #     theta1 = math.atan2(x, y)

# #     z0 = 0.220995
# #     r_arm = 0.226963

# #     sin_t2 = (z_target - z0) / r_arm
# #     sin_t2 = max(-1.0, min(1.0, sin_t2))

# #     theta2 = math.asin(sin_t2)

# #     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
# #     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

# #     return theta1, theta2


# # class SortingNode(Node):
# #     def __init__(self):
# #         super().__init__('sorting_node')

# #         # ---------------- Controllers ----------------
# #         self._arm_client = ActionClient(
# #             self,
# #             FollowJointTrajectory,
# #             '/arm_controller/follow_joint_trajectory'
# #         )

# #         self._gripper_pub = self.create_publisher(
# #             Float64MultiArray,
# #             '/gripper_controller/commands',
# #             10
# #         )

# #         self._belt_vel_pub = self.create_publisher(
# #             Float64,
# #             '/conveyor_belt_vel',
# #             10
# #         )

# #         # ---------------- Hardware interface subscribers ----------------
# #         self._start_homing_sub = self.create_subscription(
# #             String,
# #             '/hw/start_homing',
# #             self._start_homing_callback,
# #             10
# #         )

# #         self._ev3_homed_sub = self.create_subscription(
# #             String,
# #             '/hw/ev3_homed',
# #             self._ev3_homed_callback,
# #             10
# #         )

# #         self._ev3_gripper_open_sub = self.create_subscription(
# #             String,
# #             '/hw/ev3_gripper_open',
# #             self._ev3_gripper_open_callback,
# #             10
# #         )

# #         self._color_sub = self.create_subscription(
# #             String,
# #             '/hw/ball_detected',
# #             self._ball_detected_callback,
# #             10
# #         )

# #         self._ready_pickup_sub = self.create_subscription(
# #             String,
# #             '/hw/ready_pickup',
# #             self._ready_pickup_callback,
# #             10
# #         )

# #         self._ready_place_sub = self.create_subscription(
# #             String,
# #             '/hw/ready_place',
# #             self._ready_place_callback,
# #             10
# #         )

# #         self._theta1_sub = self.create_subscription(
# #             Float64,
# #             '/hw/theta1',
# #             self._theta1_callback,
# #             10
# #         )

# #         self._theta2_sub = self.create_subscription(
# #             Float64,
# #             '/hw/theta2',
# #             self._theta2_callback,
# #             10
# #         )

# #         # ---------------- Sim -> Hardware interface publishers ----------------
# #         self._sim_homed_pub = self.create_publisher(
# #             String,
# #             '/sim/homed',
# #             10
# #         )

# #         self._sim_gripper_open_pub = self.create_publisher(
# #             String,
# #             '/sim/gripper_open',
# #             10
# #         )

# #         self._spawn_confirmed_pub = self.create_publisher(
# #             String,
# #             '/sim/spawn_confirmed',
# #             10
# #         )

# #         self._sim_ready_pickup_pub = self.create_publisher(
# #             String,
# #             '/sim/ready_pickup',
# #             10
# #         )

# #         self._sim_ready_place_pub = self.create_publisher(
# #             String,
# #             '/sim/ready_place',
# #             10
# #         )

# #         self._sim_cycle_done_pub = self.create_publisher(
# #             String,
# #             '/sim/cycle_done',
# #             10
# #         )

# #         # ---------------- Thread synchronization ----------------
# #         self._start_homing_event = threading.Event()
# #         self._ev3_homed_event = threading.Event()
# #         self._ev3_gripper_open_event = threading.Event()

# #         self._color_event = threading.Event()
# #         self._pickup_event = threading.Event()
# #         self._place_event = threading.Event()

# #         self._detected_color = None
# #         self.ball_count = 0

# #         threading.Thread(
# #             target=self._start,
# #             daemon=True
# #         ).start()

# #     # ==================================================
# #     # ROS utility
# #     # ==================================================

# #     def _publish_string(self, publisher, text):
# #         msg = String()
# #         msg.data = text
# #         publisher.publish(msg)

# #     def _visual_delay(self, label=""):
# #         if VISUAL_SYNC_DELAY > 0:
# #             self.get_logger().info(
# #                 f"Visual sync delay {VISUAL_SYNC_DELAY:.1f}s {label}"
# #             )
# #             time.sleep(VISUAL_SYNC_DELAY)

# #     # ==================================================
# #     # Subscriber callbacks
# #     # ==================================================

# #     def _start_homing_callback(self, msg):
# #         self.get_logger().info("Received /hw/start_homing.")
# #         self._start_homing_event.set()

# #     def _ev3_homed_callback(self, msg):
# #         self.get_logger().info("Received /hw/ev3_homed.")
# #         self._ev3_homed_event.set()

# #     def _ev3_gripper_open_callback(self, msg):
# #         self.get_logger().info("Received /hw/ev3_gripper_open.")
# #         self._ev3_gripper_open_event.set()

# #     def _ball_detected_callback(self, msg):
# #         color = msg.data.strip().lower()

# #         if color not in ['red', 'blue', 'black', 'green']:
# #             self.get_logger().warn(f"Ignoring unknown color: {color}")
# #             return

# #         self._detected_color = color
# #         self._color_event.set()

# #         self.get_logger().info(f"Hardware detected ball: {color}")

# #     def _ready_pickup_callback(self, msg):
# #         self.get_logger().info("Hardware reached pickup-ready pose.")
# #         self._pickup_event.set()

# #     def _ready_place_callback(self, msg):
# #         self.get_logger().info("Hardware reached place-ready pose.")
# #         self._place_event.set()

# #     def _theta1_callback(self, msg):
# #         self.get_logger().info(f"[EV3 telemetry] theta1 = {msg.data:.2f} deg")

# #     def _theta2_callback(self, msg):
# #         self.get_logger().info(f"[EV3 telemetry] theta2 = {msg.data:.2f} deg")

# #     # ==================================================
# #     # Main startup thread
# #     # ==================================================

# #     def _start(self):
# #         self.get_logger().info("Waiting for arm action server...")
# #         self._arm_client.wait_for_server()
# #         self.get_logger().info("Arm action server available.")

# #         self.get_logger().info("Starting coordinated sorting loop.")
# #         self._main_loop()

# #     # ==================================================
# #     # Homing
# #     # ==================================================

# #     def _execute_initial_homing(self):
# #         self.get_logger().info("Executing simulated homing sequence...")

# #         self._send_trajectory([[0.0, 0.0]], [1.5])
# #         time.sleep(2.6)

# #         self.get_logger().info("Homing step 1: arm_2 pitching up to 0.2 rad.")
# #         self._send_trajectory([[0.0, 0.2]], [1.0])
# #         time.sleep(1.1)

# #         self.get_logger().info("Homing step 2: arm_1 rotating right by +60 deg.")
# #         self._send_trajectory([[1.0472, 0.2]], [1.2])
# #         time.sleep(1.3)

# #         self.get_logger().info("Homing step 3: arm_1 returning to center/home.")
# #         self._send_trajectory([[0.0, 0.2]], [1.2])
# #         time.sleep(1.3)

# #         self._grip(SIM_GRIPPER_OPEN)
# #         time.sleep(0.3)

# #         self.get_logger().info("Sim homing motion complete.")

# #     def _execute_cycle_homing(self):
# #         """
# #         Per-ball and final homing cycle.

# #         EV3 sends START_HOMING.
# #         Sim homes.
# #         EV3 sends EV3_HOMED.
# #         Sim publishes /sim/homed.
# #         EV3 opens gripper and sends EV3_GRIPPER_OPEN.
# #         Sim opens gripper and publishes /sim/gripper_open.
# #         """

# #         self.get_logger().info("Waiting for EV3 START_HOMING...")
# #         self._start_homing_event.wait()
# #         self._start_homing_event.clear()

# #         self._ev3_homed_event.clear()
# #         self._ev3_gripper_open_event.clear()

# #         self._execute_initial_homing()

# #         self.get_logger().info("Waiting for EV3_HOMED...")
# #         self._ev3_homed_event.wait()
# #         self._ev3_homed_event.clear()

# #         self._visual_delay("before ROS_HOMED")

# #         self.get_logger().info("Publishing /sim/homed.")
# #         self._publish_string(self._sim_homed_pub, "done")

# #         self.get_logger().info("Waiting for EV3_GRIPPER_OPEN...")
# #         self._ev3_gripper_open_event.wait()
# #         self._ev3_gripper_open_event.clear()

# #         self.get_logger().info("Opening simulated gripper.")
# #         self._grip(SIM_GRIPPER_OPEN)
# #         time.sleep(1.0)

# #         self._visual_delay("before ROS_GRIPPER_OPEN")

# #         self.get_logger().info("Publishing /sim/gripper_open.")
# #         self._publish_string(self._sim_gripper_open_pub, "done")

# #     # ==================================================
# #     # Main coordinated loop
# #     # ==================================================

# #     def _main_loop(self):
# #         self.start_conveyor(0.05)

# #         while self.ball_count < MAX_BALLS:
# #             # Home the sim every cycle, triggered by EV3.
# #             self._execute_cycle_homing()

# #             self.get_logger().info("Waiting for hardware ball detection...")

# #             self._color_event.wait()
# #             color = self._detected_color

# #             self._detected_color = None
# #             self._color_event.clear()

# #             if color is None:
# #                 continue

# #             self.get_logger().info(
# #                 f"=== Ball {self.ball_count + 1}: {color.upper()} ==="
# #             )

# #             # Spawn immediately after EV3 sensor detects the ball.
# #             name = self._spawn(color)
# #             time.sleep(0.1)

# #             self._visual_delay("after spawning ball, before ACK_SPAWN")

# #             self._publish_string(self._spawn_confirmed_pub, color)
# #             self.get_logger().info("Published /sim/spawn_confirmed.")

# #             if color in ['red', 'blue']:
# #                 # Sim ball travels to pickup while EV3 conveyor moves real ball.
# #                 self._move_ball(name, SPAWN_X, PICKUP_X)

# #                 # Old smooth coarse sync.
# #                 self._pick_place_sequence(left_side=(color == 'red'))

# #                 #self._visual_delay("before ROS_CYCLE_DONE")

# #                 self._publish_string(self._sim_cycle_done_pub, color)
# #                 self.get_logger().info("Published /sim/cycle_done.")

# #             elif color in ['black', 'green']:
# #                 # Correct behavior:
# #                 # black -> runs down conveyor
# #                 # green -> falls near sensor side
# #                 direction = 1 if color == 'black' else -1

# #                 self._fall(name, direction=direction)
# #                 time.sleep(1.5)

# #                 #self._visual_delay("before ROS_CYCLE_DONE")

# #                 self._publish_string(self._sim_cycle_done_pub, color)
# #                 self.get_logger().info("Published /sim/cycle_done.")

# #         self.get_logger().info(
# #             "All balls sorted. Waiting for final EV3 homing request..."
# #         )

# #         # EV3 should send one final START_HOMING after all balls are sorted.
# #         self._execute_cycle_homing()

# #         self.stop_conveyor()
# #         self._send_trajectory([ARM_HOME], [2.0])
# #         self._grip(SIM_GRIPPER_OPEN)

# #         self.get_logger().info("=== Execution cycle complete. Robot is home. ===")

# #     # ==================================================
# #     # Red / Blue pick-place sequence - coarse sync
# #     # ==================================================

# #     def _pick_place_sequence(self, left_side=True):
# #         bin_yaw = SIM_BASE_RED_BIN if left_side else SIM_BASE_BLUE_BIN

# #         _, pick_pitch = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)

# #         # ---------------- PICKUP READY ----------------
# #         self._grip(SIM_GRIPPER_OPEN)
# #         time.sleep(0.1)

# #         self.get_logger().info("Moving sim arm to pickup-ready pose.")
# #         self._send_trajectory(
# #             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
# #             durations=[1.0]
# #         )
# #         time.sleep(1.1)

# #         self.get_logger().info("Waiting for EV3 READY_PICKUP...")
# #         self._pickup_event.wait()
# #         self._pickup_event.clear()

# #         self._visual_delay("before ROS_READY_PICKUP")

# #         self._publish_string(self._sim_ready_pickup_pub, "ready")
# #         self.get_logger().info("Published /sim/ready_pickup.")

# #         # ---------------- PICK ----------------
# #         self.get_logger().info("Sim pitching down to pick ball.")
# #         self._send_trajectory(
# #             positions=[[SIM_BASE_HOME, pick_pitch]],
# #             durations=[0.8]
# #         )
# #         time.sleep(0.9)

# #         self.get_logger().info("Closing simulated gripper.")
# #         self._grip(SIM_GRIPPER_CLOSE)
# #         time.sleep(0.5)

# #         self.get_logger().info("Sim pitching up to clearance with ball.")
# #         self._send_trajectory(
# #             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
# #             durations=[0.8]
# #         )
# #         time.sleep(0.9)

# #         # ---------------- PLACE READY ----------------
# #         self.get_logger().info("Waiting for EV3 READY_PLACE...")
# #         self._place_event.wait()
# #         self._place_event.clear()

# #         self._visual_delay("before ROS_READY_PLACE")

# #         self._publish_string(self._sim_ready_place_pub, "ready")
# #         self.get_logger().info("Published /sim/ready_place.")

# #         # ---------------- PLACE ----------------
# #         self.get_logger().info(
# #             f"Sim swiveling to bin yaw {math.degrees(bin_yaw):.1f} deg."
# #         )
# #         self._send_trajectory(
# #             positions=[[bin_yaw, CLEARANCE_PITCH]],
# #             durations=[1.2]
# #         )
# #         time.sleep(1.3)

# #         self.get_logger().info("Sim pitching down into bin.")
# #         self._send_trajectory(
# #             positions=[[bin_yaw, -0.45]],
# #             durations=[0.8]
# #         )
# #         time.sleep(0.9)

# #         self.get_logger().info("Opening simulated gripper to release ball.")
# #         self._grip(0.2)
# #         time.sleep(0.4)

# #         self._grip(SIM_GRIPPER_OPEN)
# #         time.sleep(0.6)

# #         self.get_logger().info("Sim pitching up from bin.")
# #         self._send_trajectory(
# #             positions=[[bin_yaw, CLEARANCE_PITCH]],
# #             durations=[0.8]
# #         )
# #         time.sleep(0.9)

# #         self.get_logger().info("Sim returning to pickup/home state.")
# #         self._send_trajectory(
# #             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
# #             durations=[1.2]
# #         )
# #         time.sleep(1.3)

# #         self._grip(SIM_GRIPPER_OPEN)
# #         time.sleep(0.3)

# #     # ==================================================
# #     # Controller helpers
# #     # ==================================================

# #     def _send_trajectory(self, positions, durations):
# #         goal = FollowJointTrajectory.Goal()

# #         goal.trajectory.joint_names = [
# #             'arm_1_base_link_joint',
# #             'arm_2_left_arm_linkage_joint'
# #         ]

# #         for pos, t in zip(positions, durations):
# #             point = JointTrajectoryPoint()
# #             point.positions = [float(v) for v in pos]
# #             point.time_from_start = Duration(
# #                 sec=int(t),
# #                 nanosec=int((t - int(t)) * 1e9)
# #             )
# #             goal.trajectory.points.append(point)

# #         future = self._arm_client.send_goal_async(goal)

# #         while rclpy.ok() and not future.done():
# #             time.sleep(0.01)

# #         goal_handle = future.result()

# #         if not goal_handle.accepted:
# #             self.get_logger().error("Trajectory goal rejected.")
# #             return

# #         result_future = goal_handle.get_result_async()

# #         while rclpy.ok() and not result_future.done():
# #             time.sleep(0.01)

# #     def _grip(self, position):
# #         msg = Float64MultiArray()
# #         msg.data = [float(position)]
# #         self._gripper_pub.publish(msg)

# #     def start_conveyor(self, speed=0.05):
# #         msg = Float64()
# #         msg.data = float(speed)
# #         self._belt_vel_pub.publish(msg)

# #     def stop_conveyor(self):
# #         self.start_conveyor(0.0)

# #     # ==================================================
# #     # Ball motion helpers
# #     # ==================================================

# #     def _move_ball(self, name, x_start, x_end):
# #         x = x_start

# #         while x < x_end:
# #             x = min(x + STEP_SIZE, x_end)

# #             self._set_pose(
# #                 name,
# #                 x,
# #                 SPAWN_Y,
# #                 TRANSPORT_Z
# #             )

# #             time.sleep(STEP_DELAY)

# #     def _fall(self, name, direction):
# #         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
# #         x = SPAWN_X
# #         z = TRANSPORT_Z

# #         step = STEP_SIZE * direction

# #         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
# #             x += step

# #             if (
# #                 (direction > 0 and x > BELT_END_R)
# #                 or
# #                 (direction < 0 and x < BELT_END_L)
# #             ):
# #                 z = max(z - 0.006, -0.05)

# #             self._set_pose(
# #                 name,
# #                 x,
# #                 SPAWN_Y,
# #                 z
# #             )

# #             time.sleep(STEP_DELAY)

# #     def _set_pose(self, name, x, y, z):
# #         cmd = [
# #             'ign',
# #             'service',
# #             '-s',
# #             '/world/empty/set_pose',
# #             '--reqtype',
# #             'ignition.msgs.Pose',
# #             '--reptype',
# #             'ignition.msgs.Boolean',
# #             '--timeout',
# #             '150',
# #             '--req',
# #             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
# #         ]

# #         subprocess.Popen(
# #             cmd,
# #             stdout=subprocess.DEVNULL,
# #             stderr=subprocess.DEVNULL
# #         )

# #     def _spawn(self, color, retries=3):
# #         self.ball_count += 1

# #         name = f'{color}_ball_{self.ball_count}'

# #         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))

# #         sdf = BALL_SDF.format(
# #             name=name,
# #             r=r,
# #             g=g,
# #             b=b
# #         )

# #         cmd = [
# #             'ros2',
# #             'run',
# #             'ros_gz_sim',
# #             'create',
# #             '-name',
# #             name,
# #             '-x',
# #             str(SPAWN_X),
# #             '-y',
# #             str(SPAWN_Y),
# #             '-z',
# #             str(SPAWN_Z),
# #             '-string',
# #             sdf
# #         ]

# #         for attempt in range(retries):
# #             try:
# #                 result = subprocess.run(
# #                     cmd,
# #                     capture_output=True,
# #                     text=True,
# #                     timeout=10
# #                 )

# #                 if result.returncode == 0:
# #                     self.get_logger().info(f"Spawned {name}.")
# #                     return name

# #                 self.get_logger().warn(
# #                     f"Spawn attempt {attempt + 1} failed: {result.stderr}"
# #                 )

# #             except subprocess.TimeoutExpired:
# #                 self.get_logger().warn(
# #                     f"Spawn attempt {attempt + 1} timed out."
# #                 )
# #                 time.sleep(1.0)

# #         self.get_logger().error(
# #             f"Failed to spawn {name}; continuing anyway."
# #         )

# #         return name


# # def main(args=None):
# #     rclpy.init(args=args)

# #     node = SortingNode()

# #     executor = rclpy.executors.MultiThreadedExecutor()
# #     executor.add_node(node)

# #     try:
# #         executor.spin()

# #     except KeyboardInterrupt:
# #         pass

# #     finally:
# #         node.stop_conveyor()
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()



# #!/usr/bin/env python3

# # import math
# # import subprocess
# # import threading
# # import time

# # import rclpy
# # from rclpy.node import Node
# # from rclpy.action import ActionClient

# # from control_msgs.action import FollowJointTrajectory
# # from trajectory_msgs.msg import JointTrajectoryPoint
# # from builtin_interfaces.msg import Duration
# # from std_msgs.msg import Float64, Float64MultiArray, String


# # # ==================================================
# # # GEOMETRY & SPATIAL ANCHORS
# # # ==================================================

# # SPAWN_X = -0.14824
# # SPAWN_Y = 0.29075
# # SPAWN_Z = 0.09232
# # PICKUP_Z = SPAWN_Z + 0.022

# # PICKUP_X = -0.015
# # TRANSPORT_Z = SPAWN_Z + 0.005

# # BELT_END_R = 0.17
# # BELT_END_L = -0.17

# # STEP_SIZE = 0.008
# # STEP_DELAY = 0.090

# # SIM_BASE_HOME = 0.0
# # SIM_BASE_RED_BIN = -1.5708
# # SIM_BASE_BLUE_BIN = 1.5708

# # THETA1_MIN = -math.pi / 2
# # THETA1_MAX = math.pi / 2
# # THETA2_MIN = -0.55
# # THETA2_MAX = math.pi / 3

# # ARM_HOME = [0.0, 0.2]
# # CLEARANCE_PITCH = 0.2

# # SIM_GRIPPER_OPEN = 0.5
# # SIM_GRIPPER_CLOSE = 0.0

# # MAX_BALLS = 4


# # # ==================================================
# # # VISUAL TIMING
# # # ==================================================
# # # Increase these if the sim still looks too fast.
# # # These slow the actual sim motion, not the EV3 waiting time.

# # SIM_SETTLE_DELAY = 0.15

# # SIM_HOME_START_TIME = 3.5 # Time increased to match EV3 homing sequence
# # SIM_HOME_PITCH_UP_TIME = 2.5 # Time increased to match EV3 homing sequence
# # SIM_HOME_BASE_RIGHT_TIME = 3.5
# # SIM_HOME_BASE_CENTER_TIME = 3.0 

# # SIM_PICKUP_READY_TIME = 1.4
# # SIM_PICK_DOWN_TIME = 1.6
# # SIM_PICK_UP_TIME = 1.6

# # SIM_SWIVEL_TO_BIN_TIME = 2.2
# # SIM_DROP_DOWN_TIME = 1.7
# # SIM_RELEASE_WAIT = 0.8
# # SIM_DROP_UP_TIME = 1.7
# # SIM_RETURN_HOME_TIME = 2.2

# # SIM_FINAL_HOME_TIME = 2.0


# # BALL_RGB = {
# #     'red': (1, 0, 0),
# #     'blue': (0, 0, 1),
# #     'black': (0.05, 0.05, 0.05),
# #     'green': (0, 0.8, 0),
# # }

# # BALL_SDF = """<sdf version='1.7'>
# #   <model name='{name}'>
# #     <link name='link'>
# #       <inertial>
# #         <mass>0.05</mass>
# #         <inertia>
# #           <ixx>8e-06</ixx>
# #           <iyy>8e-06</iyy>
# #           <izz>8e-06</izz>
# #         </inertia>
# #       </inertial>
# #       <collision name='col'>
# #         <geometry>
# #           <sphere>
# #             <radius>0.0185</radius>
# #           </sphere>
# #         </geometry>
# #         <surface>
# #           <friction>
# #             <ode>
# #               <mu>0.7</mu>
# #               <mu2>0.7</mu2>
# #             </ode>
# #           </friction>
# #         </surface>
# #       </collision>
# #       <visual name='vis'>
# #         <geometry>
# #           <sphere>
# #             <radius>0.0185</radius>
# #           </sphere>
# #         </geometry>
# #         <material>
# #           <ambient>{r} {g} {b} 1</ambient>
# #           <diffuse>{r} {g} {b} 1</diffuse>
# #         </material>
# #       </visual>
# #     </link>
# #   </model>
# # </sdf>"""


# # def solve_ik_sim(x, y, z_target):
# #     theta1 = math.atan2(x, y)

# #     z0 = 0.220995
# #     r_arm = 0.226963

# #     sin_t2 = (z_target - z0) / r_arm
# #     sin_t2 = max(-1.0, min(1.0, sin_t2))

# #     theta2 = math.asin(sin_t2)

# #     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
# #     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

# #     return theta1, theta2


# # class SortingNode(Node):
# #     def __init__(self):
# #         super().__init__('sorting_node')

# #         # ---------------- Controllers ----------------
# #         self._arm_client = ActionClient(
# #             self,
# #             FollowJointTrajectory,
# #             '/arm_controller/follow_joint_trajectory'
# #         )

# #         self._gripper_pub = self.create_publisher(
# #             Float64MultiArray,
# #             '/gripper_controller/commands',
# #             10
# #         )

# #         self._belt_vel_pub = self.create_publisher(
# #             Float64,
# #             '/conveyor_belt_vel',
# #             10
# #         )

# #         # ---------------- Hardware interface subscribers ----------------
# #         self._start_homing_sub = self.create_subscription(
# #             String,
# #             '/hw/start_homing',
# #             self._start_homing_callback,
# #             10
# #         )

# #         self._ev3_homed_sub = self.create_subscription(
# #             String,
# #             '/hw/ev3_homed',
# #             self._ev3_homed_callback,
# #             10
# #         )

# #         self._ev3_gripper_open_sub = self.create_subscription(
# #             String,
# #             '/hw/ev3_gripper_open',
# #             self._ev3_gripper_open_callback,
# #             10
# #         )

# #         self._color_sub = self.create_subscription(
# #             String,
# #             '/hw/ball_detected',
# #             self._ball_detected_callback,
# #             10
# #         )

# #         self._ready_pickup_sub = self.create_subscription(
# #             String,
# #             '/hw/ready_pickup',
# #             self._ready_pickup_callback,
# #             10
# #         )

# #         self._ready_place_sub = self.create_subscription(
# #             String,
# #             '/hw/ready_place',
# #             self._ready_place_callback,
# #             10
# #         )

# #         self._theta1_sub = self.create_subscription(
# #             Float64,
# #             '/hw/theta1',
# #             self._theta1_callback,
# #             10
# #         )

# #         self._theta2_sub = self.create_subscription(
# #             Float64,
# #             '/hw/theta2',
# #             self._theta2_callback,
# #             10
# #         )

# #         # ---------------- Sim -> Hardware interface publishers ----------------
# #         self._sim_homed_pub = self.create_publisher(
# #             String,
# #             '/sim/homed',
# #             10
# #         )

# #         self._sim_gripper_open_pub = self.create_publisher(
# #             String,
# #             '/sim/gripper_open',
# #             10
# #         )

# #         self._spawn_confirmed_pub = self.create_publisher(
# #             String,
# #             '/sim/spawn_confirmed',
# #             10
# #         )

# #         self._sim_ready_pickup_pub = self.create_publisher(
# #             String,
# #             '/sim/ready_pickup',
# #             10
# #         )

# #         self._sim_ready_place_pub = self.create_publisher(
# #             String,
# #             '/sim/ready_place',
# #             10
# #         )

# #         self._sim_cycle_done_pub = self.create_publisher(
# #             String,
# #             '/sim/cycle_done',
# #             10
# #         )

# #         # ---------------- Thread synchronization ----------------
# #         self._start_homing_event = threading.Event()
# #         self._ev3_homed_event = threading.Event()
# #         self._ev3_gripper_open_event = threading.Event()

# #         self._color_event = threading.Event()
# #         self._pickup_event = threading.Event()
# #         self._place_event = threading.Event()

# #         self._detected_color = None
# #         self.ball_count = 0

# #         threading.Thread(
# #             target=self._start,
# #             daemon=True
# #         ).start()

# #     # ==================================================
# #     # ROS utility
# #     # ==================================================

# #     def _publish_string(self, publisher, text):
# #         msg = String()
# #         msg.data = text
# #         publisher.publish(msg)

# #     def _settle(self):
# #         time.sleep(SIM_SETTLE_DELAY)

# #     # ==================================================
# #     # Subscriber callbacks
# #     # ==================================================

# #     def _start_homing_callback(self, msg):
# #         self.get_logger().info("Received /hw/start_homing.")
# #         self._start_homing_event.set()

# #     def _ev3_homed_callback(self, msg):
# #         self.get_logger().info("Received /hw/ev3_homed.")
# #         self._ev3_homed_event.set()

# #     def _ev3_gripper_open_callback(self, msg):
# #         self.get_logger().info("Received /hw/ev3_gripper_open.")
# #         self._ev3_gripper_open_event.set()

# #     def _ball_detected_callback(self, msg):
# #         color = msg.data.strip().lower()

# #         if color not in ['red', 'blue', 'black', 'green']:
# #             self.get_logger().warn(f"Ignoring unknown color: {color}")
# #             return

# #         self._detected_color = color
# #         self._color_event.set()

# #         self.get_logger().info(f"Hardware detected ball: {color}")

# #     def _ready_pickup_callback(self, msg):
# #         self.get_logger().info("Hardware reached pickup-ready pose.")
# #         self._pickup_event.set()

# #     def _ready_place_callback(self, msg):
# #         self.get_logger().info("Hardware reached place-ready pose.")
# #         self._place_event.set()

# #     def _theta1_callback(self, msg):
# #         self.get_logger().info(f"[EV3 telemetry] theta1 = {msg.data:.2f} deg")

# #     def _theta2_callback(self, msg):
# #         self.get_logger().info(f"[EV3 telemetry] theta2 = {msg.data:.2f} deg")

# #     # ==================================================
# #     # Main startup thread
# #     # ==================================================

# #     def _start(self):
# #         self.get_logger().info("Waiting for arm action server...")
# #         self._arm_client.wait_for_server()
# #         self.get_logger().info("Arm action server available.")

# #         self.get_logger().info("Starting coordinated sorting loop.")
# #         self._main_loop()

# #     # ==================================================
# #     # Homing
# #     # ==================================================

# #     def _execute_initial_homing(self):
# #         self.get_logger().info("Executing simulated homing sequence...")

# #         self._send_trajectory([[0.0, 0.0]], [SIM_HOME_START_TIME])
# #         self._settle()

# #         self.get_logger().info("Homing step 1: arm_2 pitching up to 0.2 rad.")
# #         self._send_trajectory([[0.0, 0.2]], [SIM_HOME_PITCH_UP_TIME])
# #         self._settle()

# #         self.get_logger().info("Homing step 2: arm_1 rotating right by +60 deg.")
# #         self._send_trajectory([[1.0472, 0.2]], [SIM_HOME_BASE_RIGHT_TIME])
# #         self._settle()

# #         self.get_logger().info("Homing step 3: arm_1 returning to center/home.")
# #         self._send_trajectory([[0.0, 0.2]], [SIM_HOME_BASE_CENTER_TIME])
# #         self._settle()


# #         self.get_logger().info("Sim homing motion complete.")

# #     def _execute_cycle_homing(self):
# #         """
# #         Per-ball and final homing cycle.

# #         EV3 sends START_HOMING.
# #         Sim homes.
# #         EV3 sends EV3_HOMED.
# #         Sim publishes /sim/homed.
# #         EV3 opens gripper and sends EV3_GRIPPER_OPEN.
# #         Sim opens gripper and publishes /sim/gripper_open.
# #         """

# #         self.get_logger().info("Waiting for EV3 START_HOMING...")
# #         self._start_homing_event.wait()
# #         self._start_homing_event.clear()

# #         self._ev3_homed_event.clear()
# #         self._ev3_gripper_open_event.clear()
# #         self._pickup_event.clear()
# #         self._place_event.clear()

# #         self._execute_initial_homing()

# #         self.get_logger().info("Waiting for EV3_HOMED...")
# #         self._ev3_homed_event.wait()
# #         self._ev3_homed_event.clear()

# #         self.get_logger().info("Publishing /sim/homed.")
# #         self._publish_string(self._sim_homed_pub, "done")

# #         self.get_logger().info("Waiting for EV3_GRIPPER_OPEN...")
# #         self._ev3_gripper_open_event.wait()
# #         self._ev3_gripper_open_event.clear()

# #         self.get_logger().info("Opening simulated gripper.")
# #         self._grip(SIM_GRIPPER_OPEN)
# #         time.sleep(0.5)

# #         self.get_logger().info("Publishing /sim/gripper_open.")
# #         self._publish_string(self._sim_gripper_open_pub, "done")

# #     # ==================================================
# #     # Main coordinated loop
# #     # ==================================================

# #     def _main_loop(self):
# #         self.start_conveyor(0.05)

# #         while self.ball_count < MAX_BALLS:
# #             # Home the sim every cycle, triggered by EV3.
# #             self._execute_cycle_homing()

# #             self.get_logger().info("Waiting for hardware ball detection...")

# #             self._color_event.wait()
# #             color = self._detected_color

# #             self._detected_color = None
# #             self._color_event.clear()

# #             if color is None:
# #                 continue

# #             self.get_logger().info(
# #                 f"=== Ball {self.ball_count + 1}: {color.upper()} ==="
# #             )

# #             # Spawn immediately after EV3 sensor detects the ball.
# #             name = self._spawn(color)
# #             time.sleep(0.1)

# #             # Do not delay this. EV3 waits for ACK_SPAWN before moving conveyor.
# #             self._publish_string(self._spawn_confirmed_pub, color)
# #             self.get_logger().info("Published /sim/spawn_confirmed.")

# #             if color in ['red', 'blue']:
# #                 # Sim ball travels to pickup while EV3 conveyor moves real ball.
# #                 self._move_ball(name, SPAWN_X, PICKUP_X)

# #                 # Old smooth coarse sync.
# #                 self._pick_place_sequence(left_side=(color == 'red'))

# #                 # Do not delay cycle done. EV3 should start next sequence quickly.
# #                 self._publish_string(self._sim_cycle_done_pub, color)
# #                 self.get_logger().info("Published /sim/cycle_done.")

# #             elif color in ['black', 'green']:
# #                 # Correct behavior:
# #                 # black -> runs down conveyor
# #                 # green -> falls near sensor side
# #                 direction = 1 if color == 'black' else -1

# #                 self._fall(name, direction=direction)

# #                 # Do not delay cycle done. EV3 should start next sequence quickly.
# #                 self._publish_string(self._sim_cycle_done_pub, color)
# #                 self.get_logger().info("Published /sim/cycle_done.")

# #         self.get_logger().info(
# #             "All balls sorted. Waiting for final EV3 homing request..."
# #         )

# #         # EV3 should send one final START_HOMING after all balls are sorted.
# #         self._execute_cycle_homing()

# #         self.stop_conveyor()
# #         self._send_trajectory([ARM_HOME], [SIM_FINAL_HOME_TIME])
# #         self._grip(SIM_GRIPPER_OPEN)

# #         self.get_logger().info("=== Execution cycle complete. Robot is home. ===")

# #     # ==================================================
# #     # Red / Blue pick-place sequence - coarse sync
# #     # ==================================================

# #     def _pick_place_sequence(self, left_side=True):
# #         bin_yaw = SIM_BASE_RED_BIN if left_side else SIM_BASE_BLUE_BIN

# #         _, pick_pitch = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)

# #         # ---------------- PICKUP READY ----------------
# #         self._grip(SIM_GRIPPER_OPEN)
# #         time.sleep(0.1)

# #         self.get_logger().info("Moving sim arm to pickup-ready pose.")
# #         self._send_trajectory(
# #             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
# #             durations=[SIM_PICKUP_READY_TIME]
# #         )
# #         self._settle()

# #         self.get_logger().info("Waiting for EV3 READY_PICKUP...")
# #         self._pickup_event.wait()
# #         self._pickup_event.clear()

# #         # Do not add extra delay here. EV3 should pick immediately.
# #         self._publish_string(self._sim_ready_pickup_pub, "ready")
# #         self.get_logger().info("Published /sim/ready_pickup.")

# #         # ---------------- PICK ----------------
# #         self.get_logger().info("Sim pitching down to pick ball.")
# #         self._send_trajectory(
# #             positions=[[SIM_BASE_HOME, pick_pitch]],
# #             durations=[SIM_PICK_DOWN_TIME]
# #         )
# #         self._settle()

# #         self.get_logger().info("Closing simulated gripper.")
# #         self._grip(SIM_GRIPPER_CLOSE)
# #         time.sleep(0.5)

# #         self.get_logger().info("Sim pitching up to clearance with ball.")
# #         self._send_trajectory(
# #             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
# #             durations=[SIM_PICK_UP_TIME]
# #         )
# #         self._settle()

# #         # ---------------- PLACE READY ----------------
# #         self.get_logger().info("Waiting for EV3 READY_PLACE...")
# #         self._place_event.wait()
# #         self._place_event.clear()

# #         # Do not add extra delay here. EV3 should place immediately.
# #         self._publish_string(self._sim_ready_place_pub, "ready")
# #         self.get_logger().info("Published /sim/ready_place.")

# #         # ---------------- PLACE ----------------
# #         self.get_logger().info(
# #             f"Sim swiveling to bin yaw {math.degrees(bin_yaw):.1f} deg."
# #         )
# #         self._send_trajectory(
# #             positions=[[bin_yaw, CLEARANCE_PITCH]],
# #             durations=[SIM_SWIVEL_TO_BIN_TIME]
# #         )
# #         self._settle()

# #         # self.get_logger().info("Sim pitching down into bin.")
# #         # self._send_trajectory(
# #         #     positions=[[bin_yaw, -0.45]],
# #         #     durations=[SIM_DROP_DOWN_TIME]
# #         # )
# #         # self._settle()

# #         # self.get_logger().info("Opening simulated gripper to release ball.")
# #         # self._grip(0.2)
# #         # time.sleep(0.3)

# #         # self._grip(SIM_GRIPPER_OPEN)
# #         # time.sleep(SIM_RELEASE_WAIT)

# #         # self.get_logger().info("Sim pitching up from bin.")
# #         # self._send_trajectory(
# #         #     positions=[[bin_yaw, CLEARANCE_PITCH]],
# #         #     durations=[SIM_DROP_UP_TIME]
# #         # )
# #         # self._settle()

# #         # # self.get_logger().info("Sim returning to pickup/home state.")
# #         # # self._send_trajectory(
# #         # #     positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
# #         # #     durations=[SIM_RETURN_HOME_TIME]
# #         # # )
# #         # # self._settle()

# #         # # self._grip(SIM_GRIPPER_OPEN)
# #         # # time.sleep(0.2)

# #         self.get_logger().info("Sim pitching down into bin.")
# #         self._send_trajectory(
# #             positions=[[bin_yaw, -0.45]],
# #             durations=[SIM_DROP_DOWN_TIME]
# #         )
# #         self._settle()

# #         self.get_logger().info("Opening simulated gripper to release ball.")
# #         self._grip(0.2)
# #         time.sleep(0.3)

# #         self._grip(SIM_GRIPPER_OPEN)
# #         time.sleep(SIM_RELEASE_WAIT)

# #         self.get_logger().info("Sim pitching up from bin.")
# #         self._send_trajectory(
# #             positions=[[bin_yaw, CLEARANCE_PITCH]],
# #             durations=[SIM_DROP_UP_TIME]
# #         )
# #         self._settle()

# #         self._grip(SIM_GRIPPER_OPEN)
# #         time.sleep(0.2)

# #         self.get_logger().info("Sim place sequence complete.")




# #     # ==================================================
# #     # Controller helpers
# #     # ==================================================

# #     def _send_trajectory(self, positions, durations):
# #         goal = FollowJointTrajectory.Goal()

# #         goal.trajectory.joint_names = [
# #             'arm_1_base_link_joint',
# #             'arm_2_left_arm_linkage_joint'
# #         ]

# #         for pos, t in zip(positions, durations):
# #             point = JointTrajectoryPoint()
# #             point.positions = [float(v) for v in pos]
# #             point.time_from_start = Duration(
# #                 sec=int(t),
# #                 nanosec=int((t - int(t)) * 1e9)
# #             )
# #             goal.trajectory.points.append(point)

# #         future = self._arm_client.send_goal_async(goal)

# #         while rclpy.ok() and not future.done():
# #             time.sleep(0.01)

# #         goal_handle = future.result()

# #         if not goal_handle.accepted:
# #             self.get_logger().error("Trajectory goal rejected.")
# #             return

# #         result_future = goal_handle.get_result_async()

# #         while rclpy.ok() and not result_future.done():
# #             time.sleep(0.01)

# #     def _grip(self, position):
# #         msg = Float64MultiArray()
# #         msg.data = [float(position)]
# #         self._gripper_pub.publish(msg)

# #     def start_conveyor(self, speed=0.05):
# #         msg = Float64()
# #         msg.data = float(speed)
# #         self._belt_vel_pub.publish(msg)

# #     def stop_conveyor(self):
# #         self.start_conveyor(0.0)

# #     # ==================================================
# #     # Ball motion helpers
# #     # ==================================================

# #     def _move_ball(self, name, x_start, x_end):
# #         x = x_start

# #         while x < x_end:
# #             x = min(x + STEP_SIZE, x_end)

# #             self._set_pose(
# #                 name,
# #                 x,
# #                 SPAWN_Y,
# #                 TRANSPORT_Z
# #             )

# #             time.sleep(STEP_DELAY)

# #     def _fall(self, name, direction):
# #         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
# #         x = SPAWN_X
# #         z = TRANSPORT_Z

# #         step = STEP_SIZE * direction

# #         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
# #             x += step

# #             if (
# #                 (direction > 0 and x > BELT_END_R)
# #                 or
# #                 (direction < 0 and x < BELT_END_L)
# #             ):
# #                 z = max(z - 0.006, -0.05)

# #             self._set_pose(
# #                 name,
# #                 x,
# #                 SPAWN_Y,
# #                 z
# #             )

# #             time.sleep(STEP_DELAY)

# #     def _set_pose(self, name, x, y, z):
# #         cmd = [
# #             'ign',
# #             'service',
# #             '-s',
# #             '/world/empty/set_pose',
# #             '--reqtype',
# #             'ignition.msgs.Pose',
# #             '--reptype',
# #             'ignition.msgs.Boolean',
# #             '--timeout',
# #             '150',
# #             '--req',
# #             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
# #         ]

# #         subprocess.Popen(
# #             cmd,
# #             stdout=subprocess.DEVNULL,
# #             stderr=subprocess.DEVNULL
# #         )

# #     def _spawn(self, color, retries=3):
# #         self.ball_count += 1

# #         name = f'{color}_ball_{self.ball_count}'

# #         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))

# #         sdf = BALL_SDF.format(
# #             name=name,
# #             r=r,
# #             g=g,
# #             b=b
# #         )

# #         cmd = [
# #             'ros2',
# #             'run',
# #             'ros_gz_sim',
# #             'create',
# #             '-name',
# #             name,
# #             '-x',
# #             str(SPAWN_X),
# #             '-y',
# #             str(SPAWN_Y),
# #             '-z',
# #             str(SPAWN_Z),
# #             '-string',
# #             sdf
# #         ]

# #         for attempt in range(retries):
# #             try:
# #                 result = subprocess.run(
# #                     cmd,
# #                     capture_output=True,
# #                     text=True,
# #                     timeout=10
# #                 )

# #                 if result.returncode == 0:
# #                     self.get_logger().info(f"Spawned {name}.")
# #                     return name

# #                 self.get_logger().warn(
# #                     f"Spawn attempt {attempt + 1} failed: {result.stderr}"
# #                 )

# #             except subprocess.TimeoutExpired:
# #                 self.get_logger().warn(
# #                     f"Spawn attempt {attempt + 1} timed out."
# #                 )
# #                 time.sleep(1.0)

# #         self.get_logger().error(
# #             f"Failed to spawn {name}; continuing anyway."
# #         )

# #         return name


# # def main(args=None):
# #     rclpy.init(args=args)

# #     node = SortingNode()

# #     executor = rclpy.executors.MultiThreadedExecutor()
# #     executor.add_node(node)

# #     try:
# #         executor.spin()

# #     except KeyboardInterrupt:
# #         pass

# #     finally:
# #         node.stop_conveyor()
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()




# #!/usr/bin/env python3

# import math
# import subprocess
# import threading
# import time

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient

# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64, Float64MultiArray, String


# # ==================================================
# # GEOMETRY & SPATIAL ANCHORS
# # ==================================================

# SPAWN_X = -0.14824
# SPAWN_Y = 0.29075
# SPAWN_Z = 0.09232
# PICKUP_Z = SPAWN_Z + 0.022

# PICKUP_X = -0.015
# TRANSPORT_Z = SPAWN_Z + 0.005

# BELT_END_R = 0.17
# BELT_END_L = -0.17

# STEP_SIZE = 0.008
# STEP_DELAY = 0.090

# SIM_BASE_HOME = 0.0
# SIM_BASE_RED_BIN = -1.5708
# SIM_BASE_BLUE_BIN = 1.5708

# THETA1_MIN = -math.pi / 2
# THETA1_MAX = math.pi / 2
# THETA2_MIN = -0.55
# THETA2_MAX = math.pi / 3

# ARM_HOME = [0.0, 0.2]
# CLEARANCE_PITCH = 0.2

# SIM_GRIPPER_OPEN = 0.5
# SIM_GRIPPER_CLOSE = 0.0

# MAX_BALLS = 4


# # ==================================================
# # VISUAL TIMING
# # ==================================================

# SIM_SETTLE_DELAY = 0.15

# SIM_HOME_PITCH_UP_TIME = 2.2
# SIM_HOME_BASE_RIGHT_TIME = 3.0
# SIM_HOME_BASE_CENTER_TIME = 3.0

# SIM_PICKUP_READY_TIME = 1.4
# SIM_PICK_DOWN_TIME = 1.6
# SIM_PICK_UP_TIME = 1.6

# SIM_SWIVEL_TO_BIN_TIME = 2.2
# SIM_DROP_DOWN_TIME = 1.7
# SIM_RELEASE_WAIT = 0.8
# SIM_DROP_UP_TIME = 1.7

# SIM_FINAL_HOME_TIME = 2.0


# BALL_RGB = {
#     'red': (1, 0, 0),
#     'blue': (0, 0, 1),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0, 0.8, 0),
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

#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

#     return theta1, theta2


# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting_node')

#         # ---------------- Controllers ----------------
#         self._arm_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory'
#         )

#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray,
#             '/gripper_controller/commands',
#             10
#         )

#         self._belt_vel_pub = self.create_publisher(
#             Float64,
#             '/conveyor_belt_vel',
#             10
#         )

#         # ---------------- Hardware interface subscribers ----------------
#         self._start_homing_sub = self.create_subscription(
#             String,
#             '/hw/start_homing',
#             self._start_homing_callback,
#             10
#         )

#         self._ev3_homed_sub = self.create_subscription(
#             String,
#             '/hw/ev3_homed',
#             self._ev3_homed_callback,
#             10
#         )

#         self._ev3_gripper_open_sub = self.create_subscription(
#             String,
#             '/hw/ev3_gripper_open',
#             self._ev3_gripper_open_callback,
#             10
#         )

#         self._color_sub = self.create_subscription(
#             String,
#             '/hw/ball_detected',
#             self._ball_detected_callback,
#             10
#         )

#         self._ready_pickup_sub = self.create_subscription(
#             String,
#             '/hw/ready_pickup',
#             self._ready_pickup_callback,
#             10
#         )

#         self._ready_place_sub = self.create_subscription(
#             String,
#             '/hw/ready_place',
#             self._ready_place_callback,
#             10
#         )

#         self._start_pick_sub = self.create_subscription(
#             String,
#             '/hw/start_pick',
#             self._start_pick_callback,
#             10
#         )

#         self._start_place_sub = self.create_subscription(
#             String,
#             '/hw/start_place',
#             self._start_place_callback,
#             10
#         )

#         self._theta1_sub = self.create_subscription(
#             Float64,
#             '/hw/theta1',
#             self._theta1_callback,
#             10
#         )

#         self._theta2_sub = self.create_subscription(
#             Float64,
#             '/hw/theta2',
#             self._theta2_callback,
#             10
#         )

#         # ---------------- Sim -> Hardware interface publishers ----------------
#         self._sim_homed_pub = self.create_publisher(
#             String,
#             '/sim/homed',
#             10
#         )

#         self._sim_gripper_open_pub = self.create_publisher(
#             String,
#             '/sim/gripper_open',
#             10
#         )

#         self._spawn_confirmed_pub = self.create_publisher(
#             String,
#             '/sim/spawn_confirmed',
#             10
#         )

#         self._sim_ready_pickup_pub = self.create_publisher(
#             String,
#             '/sim/ready_pickup',
#             10
#         )

#         self._sim_ready_place_pub = self.create_publisher(
#             String,
#             '/sim/ready_place',
#             10
#         )

#         self._sim_cycle_done_pub = self.create_publisher(
#             String,
#             '/sim/cycle_done',
#             10
#         )

#         # ---------------- Thread synchronization ----------------
#         self._start_homing_event = threading.Event()
#         self._ev3_homed_event = threading.Event()
#         self._ev3_gripper_open_event = threading.Event()

#         self._color_event = threading.Event()
#         self._pickup_event = threading.Event()
#         self._place_event = threading.Event()

#         self._start_pick_event = threading.Event()
#         self._start_place_event = threading.Event()

#         self._detected_color = None
#         self.ball_count = 0

#         # Track current simulated pose.
#         # This lets homing start from wherever the sim actually is.
#         self._current_base = SIM_BASE_HOME
#         self._current_pitch = CLEARANCE_PITCH

#         threading.Thread(
#             target=self._start,
#             daemon=True
#         ).start()

#     # ==================================================
#     # ROS utility
#     # ==================================================

#     def _publish_string(self, publisher, text):
#         msg = String()
#         msg.data = text
#         publisher.publish(msg)

#     def _settle(self):
#         time.sleep(SIM_SETTLE_DELAY)

#     # ==================================================
#     # Subscriber callbacks
#     # ==================================================

#     def _start_homing_callback(self, msg):
#         self.get_logger().info("Received /hw/start_homing.")
#         self._start_homing_event.set()

#     def _ev3_homed_callback(self, msg):
#         self.get_logger().info("Received /hw/ev3_homed.")
#         self._ev3_homed_event.set()

#     def _ev3_gripper_open_callback(self, msg):
#         self.get_logger().info("Received /hw/ev3_gripper_open.")
#         self._ev3_gripper_open_event.set()

#     def _ball_detected_callback(self, msg):
#         color = msg.data.strip().lower()

#         if color not in ['red', 'blue', 'black', 'green']:
#             self.get_logger().warn(f"Ignoring unknown color: {color}")
#             return

#         self._detected_color = color
#         self._color_event.set()

#         self.get_logger().info(f"Hardware detected ball: {color}")

#     def _ready_pickup_callback(self, msg):
#         self.get_logger().info("Hardware reached pickup-ready pose.")
#         self._pickup_event.set()

#     def _ready_place_callback(self, msg):
#         self.get_logger().info("Hardware reached place-ready pose.")
#         self._place_event.set()

#     def _start_pick_callback(self, msg):
#         self.get_logger().info("Hardware started PICK stage.")
#         self._start_pick_event.set()

#     def _start_place_callback(self, msg):
#         self.get_logger().info("Hardware started PLACE stage.")
#         self._start_place_event.set()

#     def _theta1_callback(self, msg):
#         self.get_logger().info(f"[EV3 telemetry] theta1 = {msg.data:.2f} deg")

#     def _theta2_callback(self, msg):
#         self.get_logger().info(f"[EV3 telemetry] theta2 = {msg.data:.2f} deg")

#     # ==================================================
#     # Main startup thread
#     # ==================================================

#     def _start(self):
#         self.get_logger().info("Waiting for arm action server...")
#         self._arm_client.wait_for_server()
#         self.get_logger().info("Arm action server available.")

#         self.get_logger().info("Starting coordinated sorting loop.")
#         self._main_loop()

#     # ==================================================
#     # Homing
#     # ==================================================

#     def _execute_initial_homing(self):
#         """
#         Simulated homing from the current pose.

#         Important:
#         Do not first return to center.
#         After red/blue drop, the next homing starts from the bin side,
#         like the EV3 physically does.
#         """

#         self.get_logger().info("Executing simulated homing from current pose...")

#         self.get_logger().info("Homing step 1: pitch up at current base position.")
#         self._send_trajectory(
#             [[self._current_base, CLEARANCE_PITCH]],
#             [SIM_HOME_PITCH_UP_TIME]
#         )
#         self._settle()

#         self.get_logger().info("Homing step 2: base rotating toward home switch.")
#         self._send_trajectory(
#             [[1.0472, CLEARANCE_PITCH]],
#             [SIM_HOME_BASE_RIGHT_TIME]
#         )
#         self._settle()

#         self.get_logger().info("Homing step 3: base returning to pickup center.")
#         self._send_trajectory(
#             [[SIM_BASE_HOME, CLEARANCE_PITCH]],
#             [SIM_HOME_BASE_CENTER_TIME]
#         )
#         self._settle()

#         self.get_logger().info("Sim homing motion complete.")

#     def _execute_cycle_homing(self):
#         """
#         Per-ball and final homing cycle.

#         EV3 sends START_HOMING.
#         Sim homes from current pose.
#         EV3 sends EV3_HOMED.
#         Sim publishes /sim/homed.
#         EV3 opens gripper and sends EV3_GRIPPER_OPEN.
#         Sim opens gripper and publishes /sim/gripper_open.
#         """

#         self.get_logger().info("Waiting for EV3 START_HOMING...")
#         self._start_homing_event.wait()
#         self._start_homing_event.clear()

#         self._ev3_homed_event.clear()
#         self._ev3_gripper_open_event.clear()
#         self._pickup_event.clear()
#         self._place_event.clear()
#         self._start_pick_event.clear()
#         self._start_place_event.clear()

#         self._execute_initial_homing()

#         self.get_logger().info("Waiting for EV3_HOMED...")
#         self._ev3_homed_event.wait()
#         self._ev3_homed_event.clear()

#         self.get_logger().info("Publishing /sim/homed.")
#         self._publish_string(self._sim_homed_pub, "done")

#         self.get_logger().info("Waiting for EV3_GRIPPER_OPEN...")
#         self._ev3_gripper_open_event.wait()
#         self._ev3_gripper_open_event.clear()

#         self.get_logger().info("Opening simulated gripper.")
#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(0.4)

#         self.get_logger().info("Publishing /sim/gripper_open.")
#         self._publish_string(self._sim_gripper_open_pub, "done")

#     # ==================================================
#     # Main coordinated loop
#     # ==================================================

#     def _main_loop(self):
#         self.start_conveyor(0.05)

#         while self.ball_count < MAX_BALLS:
#             self._execute_cycle_homing()

#             self.get_logger().info("Waiting for hardware ball detection...")

#             self._color_event.wait()
#             color = self._detected_color

#             self._detected_color = None
#             self._color_event.clear()

#             if color is None:
#                 continue

#             self.get_logger().info(
#                 f"=== Ball {self.ball_count + 1}: {color.upper()} ==="
#             )

#             name = self._spawn(color)
#             time.sleep(0.1)

#             self._publish_string(self._spawn_confirmed_pub, color)
#             self.get_logger().info("Published /sim/spawn_confirmed.")

#             if color in ['red', 'blue']:
#                 self._move_ball(name, SPAWN_X, PICKUP_X)

#                 self._pick_place_sequence(left_side=(color == 'red'))

#                 self._publish_string(self._sim_cycle_done_pub, color)
#                 self.get_logger().info("Published /sim/cycle_done.")

#             elif color in ['black', 'green']:
#                 direction = 1 if color == 'black' else -1

#                 self._fall(name, direction=direction)

#                 self._publish_string(self._sim_cycle_done_pub, color)
#                 self.get_logger().info("Published /sim/cycle_done.")

#         self.get_logger().info(
#             "All balls sorted. Waiting for final EV3 homing request..."
#         )

#         self._execute_cycle_homing()

#         self.stop_conveyor()
#         self._send_trajectory([ARM_HOME], [SIM_FINAL_HOME_TIME])
#         self._grip(SIM_GRIPPER_OPEN)

#         self.get_logger().info("=== Execution cycle complete. Robot is home. ===")

#     # ==================================================
#     # Red / Blue pick-place sequence
#     # ==================================================

#     def _pick_place_sequence(self, left_side=True):
#         bin_yaw = SIM_BASE_RED_BIN if left_side else SIM_BASE_BLUE_BIN

#         _, pick_pitch = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)

#         # ---------------- PICKUP READY ----------------
#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(0.1)

#         self.get_logger().info("Moving sim arm to pickup-ready pose.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
#             durations=[SIM_PICKUP_READY_TIME]
#         )
#         self._settle()

#         self.get_logger().info("Waiting for EV3 READY_PICKUP...")
#         self._pickup_event.wait()
#         self._pickup_event.clear()

#         self._publish_string(self._sim_ready_pickup_pub, "ready")
#         self.get_logger().info("Published /sim/ready_pickup.")

#         # ---------------- WAIT FOR EV3 PICK ----------------
#         self.get_logger().info("Waiting for EV3 START_PICK...")
#         self._start_pick_event.wait()
#         self._start_pick_event.clear()

#         # ---------------- PICK ----------------
#         self.get_logger().info("Sim starting PICK stage.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, pick_pitch]],
#             durations=[SIM_PICK_DOWN_TIME]
#         )
#         self._settle()

#         self.get_logger().info("Closing simulated gripper.")
#         self._grip(SIM_GRIPPER_CLOSE)
#         time.sleep(0.5)

#         self.get_logger().info("Sim pitching up to clearance with ball.")
#         self._send_trajectory(
#             positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
#             durations=[SIM_PICK_UP_TIME]
#         )
#         self._settle()

#         # ---------------- PLACE READY ----------------
#         self.get_logger().info("Waiting for EV3 READY_PLACE...")
#         self._place_event.wait()
#         self._place_event.clear()

#         self._publish_string(self._sim_ready_place_pub, "ready")
#         self.get_logger().info("Published /sim/ready_place.")

#         # ---------------- WAIT FOR EV3 PLACE ----------------
#         self.get_logger().info("Waiting for EV3 START_PLACE...")
#         self._start_place_event.wait()
#         self._start_place_event.clear()

#         # ---------------- PLACE ----------------
#         self.get_logger().info(
#             f"Sim starting PLACE stage, bin yaw {math.degrees(bin_yaw):.1f} deg."
#         )
#         self._send_trajectory(
#             positions=[[bin_yaw, CLEARANCE_PITCH]],
#             durations=[SIM_SWIVEL_TO_BIN_TIME]
#         )
#         self._settle()

#         self.get_logger().info("Sim pitching down into bin.")
#         self._send_trajectory(
#             positions=[[bin_yaw, -0.45]],
#             durations=[SIM_DROP_DOWN_TIME]
#         )
#         self._settle()

#         self.get_logger().info("Opening simulated gripper to release ball.")
#         self._grip(0.2)
#         time.sleep(0.3)

#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(SIM_RELEASE_WAIT)

#         self.get_logger().info("Sim pitching up from bin.")
#         self._send_trajectory(
#             positions=[[bin_yaw, CLEARANCE_PITCH]],
#             durations=[SIM_DROP_UP_TIME]
#         )
#         self._settle()

#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(0.1)

#         self.get_logger().info("Sim place sequence complete.")

#     # ==================================================
#     # Controller helpers
#     # ==================================================

#     def _send_trajectory(self, positions, durations):
#         goal = FollowJointTrajectory.Goal()

#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint'
#         ]

#         for pos, t in zip(positions, durations):
#             point = JointTrajectoryPoint()
#             point.positions = [float(v) for v in pos]
#             point.time_from_start = Duration(
#                 sec=int(t),
#                 nanosec=int((t - int(t)) * 1e9)
#             )
#             goal.trajectory.points.append(point)

#         future = self._arm_client.send_goal_async(goal)

#         while rclpy.ok() and not future.done():
#             time.sleep(0.01)

#         goal_handle = future.result()

#         if not goal_handle.accepted:
#             self.get_logger().error("Trajectory goal rejected.")
#             return

#         result_future = goal_handle.get_result_async()

#         while rclpy.ok() and not result_future.done():
#             time.sleep(0.01)

#         if positions:
#             self._current_base = float(positions[-1][0])
#             self._current_pitch = float(positions[-1][1])

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)

#     def start_conveyor(self, speed=0.05):
#         msg = Float64()
#         msg.data = float(speed)
#         self._belt_vel_pub.publish(msg)

#     def stop_conveyor(self):
#         self.start_conveyor(0.0)

#     # ==================================================
#     # Ball motion helpers
#     # ==================================================

#     def _move_ball(self, name, x_start, x_end):
#         x = x_start

#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)

#             self._set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 TRANSPORT_Z
#             )

#             time.sleep(STEP_DELAY)

#     def _fall(self, name, direction):
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         x = SPAWN_X
#         z = TRANSPORT_Z

#         step = STEP_SIZE * direction

#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step

#             if (
#                 (direction > 0 and x > BELT_END_R)
#                 or
#                 (direction < 0 and x < BELT_END_L)
#             ):
#                 z = max(z - 0.006, -0.05)

#             self._set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 z
#             )

#             time.sleep(STEP_DELAY)

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign',
#             'service',
#             '-s',
#             '/world/empty/set_pose',
#             '--reqtype',
#             'ignition.msgs.Pose',
#             '--reptype',
#             'ignition.msgs.Boolean',
#             '--timeout',
#             '150',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]

#         subprocess.Popen(
#             cmd,
#             stdout=subprocess.DEVNULL,
#             stderr=subprocess.DEVNULL
#         )

#     def _spawn(self, color, retries=3):
#         self.ball_count += 1

#         name = f'{color}_ball_{self.ball_count}'

#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))

#         sdf = BALL_SDF.format(
#             name=name,
#             r=r,
#             g=g,
#             b=b
#         )

#         cmd = [
#             'ros2',
#             'run',
#             'ros_gz_sim',
#             'create',
#             '-name',
#             name,
#             '-x',
#             str(SPAWN_X),
#             '-y',
#             str(SPAWN_Y),
#             '-z',
#             str(SPAWN_Z),
#             '-string',
#             sdf
#         ]

#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(
#                     cmd,
#                     capture_output=True,
#                     text=True,
#                     timeout=10
#                 )

#                 if result.returncode == 0:
#                     self.get_logger().info(f"Spawned {name}.")
#                     return name

#                 self.get_logger().warn(
#                     f"Spawn attempt {attempt + 1} failed: {result.stderr}"
#                 )

#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     f"Spawn attempt {attempt + 1} timed out."
#                 )
#                 time.sleep(1.0)

#         self.get_logger().error(
#             f"Failed to spawn {name}; continuing anyway."
#         )

#         return name


# def main(args=None):
#     rclpy.init(args=args)

#     node = SortingNode()

#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)

#     try:
#         executor.spin()

#     except KeyboardInterrupt:
#         pass

#     finally:
#         node.stop_conveyor()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
# """Gazebo follower for the EV3 stage-synchronised digital twin.

# The simulation never decides the task order. It follows the EV3 through:

#   1. an initial synchronized HOME stage,
#   2. a synchronized ball task,
#   3. another synchronized HOME stage only after RED or BLUE.

# A stage is acknowledged only after the simulation action has completed and
# STAGE_HW_DONE for the same cycle/sequence/stage has arrived.
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
# # SIMULATION-SPECIFIC TARGETS
# # ==================================================

# # These targets intentionally belong only to the larger simulated model.
# # They do not have to match the EV3 model's Cartesian dimensions.
# SIM_BASE_HOME = 0.0
# SIM_BASE_RED_BIN = -math.pi / 2
# SIM_BASE_BLUE_BIN = math.pi / 2
# CLEARANCE_PITCH = 0.2
# SIM_PICK_PITCH = -0.48
# SIM_PLACE_PITCH = -0.45

# SIM_GRIPPER_OPEN = 0.5
# SIM_GRIPPER_CLOSE = 0.0

# # Visible homing animation. This is intentionally a behavioural equivalent
# # of the EV3 switch-homing sequence, not a geometric copy of it.
# SIM_HOME_SWEEP_YAW = math.radians(60.0)
# SIM_HOME_RAISE_TIME = 2.0
# SIM_HOME_SWEEP_TIME = 5.0
# SIM_HOME_RETURN_TIME = 5.0

# # Deterministic grasp backend. The finger controller still opens/closes the
# # model, while the active ball is logically held at the gripper pose. This
# # avoids relying on contact friction for the project demo. Set to False later
# # if a physical grasp / link-attacher plugin is introduced.
# USE_LOGICAL_GRASP = True
# BALL_FOLLOW_PERIOD = 0.10

# # Effective gripper-tip geometry for the enlarged simulation model. These
# # values are simulation-only and were selected so PICK_DOWN aligns with the
# # existing pickup position.
# SIM_GRIPPER_BASE_X = -0.015
# SIM_GRIPPER_PIVOT_Z = 0.265
# SIM_GRIPPER_EFFECTIVE_REACH = 0.328

# TRAJECTORY_TIMES = {
#     'HOME': 1.5,
#     'PICKUP_READY': 1.0,
#     'PICK_DOWN': 1.2,
#     'PICK_UP': 1.2,
#     'ROTATE_RED': 1.5,
#     'ROTATE_BLUE': 1.5,
#     'PLACE_DOWN': 1.2,
#     'PLACE_UP': 1.2,
# }

# GRIPPER_SETTLE_TIME = 0.45


# # ==================================================
# # BALL / CONVEYOR MODEL
# # ==================================================

# WORLD_NAME = 'empty'
# SPAWN_X = -0.14824
# SPAWN_Y = 0.29075
# SPAWN_Z = 0.09232
# TRANSPORT_Z = SPAWN_Z + 0.005
# PICKUP_X = -0.015
# BELT_END_R = 0.17
# BELT_END_L = -0.17

# # This pose-based backend is retained for the first synchronisation test.
# # It is isolated in _move_ball_to_pickup() and _fall_ball(), so it can later
# # be replaced by the IFRA conveyor plugin without changing the state machine.
# STEP_SIZE = 0.008
# STEP_DELAY = 0.090

# BALL_RGB = {
#     'red': (1.0, 0.0, 0.0),
#     'blue': (0.0, 0.0, 1.0),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0.0, 0.8, 0.0),
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
#           <sphere><radius>0.0185</radius></sphere>
#         </geometry>
#         <surface>
#           <friction>
#             <ode><mu>0.7</mu><mu2>0.7</mu2></ode>
#           </friction>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry>
#           <sphere><radius>0.0185</radius></sphere>
#         </geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting_node')

#         self.arm_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory',
#         )

#         self.gripper_pub = self.create_publisher(
#             Float64MultiArray,
#             '/gripper_controller/commands',
#             10,
#         )

#         self.belt_vel_pub = self.create_publisher(
#             Float64,
#             '/conveyor_belt_vel',
#             10,
#         )

#         self.stage_event_sub = self.create_subscription(
#             String,
#             '/hw/stage_event',
#             self.stage_event_callback,
#             20,
#         )

#         self.stage_sync_pub = self.create_publisher(
#             String,
#             '/sim/stage_sync',
#             20,
#         )

#         self.event_queue = queue.Queue()
#         self.shutdown_event = threading.Event()

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False
#         self.current_ball_name = None
#         self.ball_attached = False
#         self.current_arm_target = [SIM_BASE_HOME, 0.0]

#         self.worker_thread = threading.Thread(
#             target=self.worker_loop,
#             daemon=True,
#         )
#         self.worker_thread.start()

#     # ==================================================
#     # Protocol helpers
#     # ==================================================

#     @staticmethod
#     def parse_stage_line(line, expected_kind):
#         parts = line.split('|')

#         if len(parts) != 4 or parts[0] != expected_kind:
#             raise ValueError(f'Invalid {expected_kind} packet: {line}')

#         return int(parts[1]), int(parts[2]), parts[3]

#     @staticmethod
#     def make_stage_line(kind, cycle_id, sequence_id, stage):
#         return f'{kind}|{cycle_id}|{sequence_id}|{stage}'

#     def publish_stage_sync(self, text):
#         msg = String()
#         msg.data = text
#         self.stage_sync_pub.publish(msg)
#         self.get_logger().info(f'-> EV3 [{text}]')

#     def stage_event_callback(self, msg):
#         self.event_queue.put(msg.data.strip())

#     # ==================================================
#     # Event worker and barrier
#     # ==================================================

#     def worker_loop(self):
#         self.get_logger().info('Waiting for arm action server...')

#         if not self.arm_client.wait_for_server(timeout_sec=30.0):
#             self.get_logger().error('Arm action server was not available.')
#             return

#         self.get_logger().info('Arm action server available.')
#         self.start_conveyor_visual(0.05)

#         while rclpy.ok() and not self.shutdown_event.is_set():
#             try:
#                 line = self.event_queue.get(timeout=0.2)
#             except queue.Empty:
#                 continue

#             try:
#                 if line.startswith('STAGE_START|'):
#                     self.handle_stage_start(line)
#                 elif line.startswith('STAGE_HW_DONE|'):
#                     self.handle_hardware_done(line)
#                 elif line == 'EV3_DONE':
#                     self.get_logger().info('EV3 completed the full task.')
#                     self.start_conveyor_visual(0.0)
#                 else:
#                     self.get_logger().warn(
#                         f'Ignoring unknown stage event: {line}'
#                     )
#             except Exception as exc:
#                 self.get_logger().error(
#                     f'Error while processing [{line}]: {exc}'
#                 )
#                 self.publish_failure_for_active_stage(str(exc))

#     def handle_stage_start(self, line):
#         cycle_id, sequence_id, stage = self.parse_stage_line(
#             line,
#             'STAGE_START',
#         )
#         key = (cycle_id, sequence_id, stage)

#         if self.active_key is not None:
#             raise RuntimeError(
#                 f'Received {key}, but stage {self.active_key} is still active.'
#             )

#         self.active_key = key
#         self.sim_done = False
#         self.hardware_done = False

#         self.get_logger().info(
#             f'=== START cycle={cycle_id} seq={sequence_id} stage={stage} ==='
#         )

#         self.execute_sim_stage(cycle_id, stage)
#         self.sim_done = True
#         self.try_complete_active_stage()

#     def handle_hardware_done(self, line):
#         cycle_id, sequence_id, stage = self.parse_stage_line(
#             line,
#             'STAGE_HW_DONE',
#         )
#         key = (cycle_id, sequence_id, stage)

#         if self.active_key != key:
#             raise RuntimeError(
#                 f'Hardware completed {key}, active stage is {self.active_key}.'
#             )

#         self.hardware_done = True
#         self.get_logger().info(
#             f'Hardware completed cycle={cycle_id} seq={sequence_id} '
#             f'stage={stage}.'
#         )
#         self.try_complete_active_stage()

#     def try_complete_active_stage(self):
#         if not (self.sim_done and self.hardware_done):
#             return

#         cycle_id, sequence_id, stage = self.active_key
#         response = self.make_stage_line(
#             'STAGE_SYNC_DONE',
#             cycle_id,
#             sequence_id,
#             stage,
#         )
#         self.publish_stage_sync(response)

#         self.get_logger().info(
#             f'=== DONE cycle={cycle_id} seq={sequence_id} stage={stage} ==='
#         )

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#     def publish_failure_for_active_stage(self, reason):
#         if self.active_key is None:
#             return

#         cycle_id, sequence_id, stage = self.active_key
#         safe_reason = reason.replace('|', '/').replace('\n', ' ')
#         response = (
#             f'STAGE_SYNC_FAILED|{cycle_id}|{sequence_id}|{stage}|{safe_reason}'
#         )
#         self.publish_stage_sync(response)

#         self.active_key = None
#         self.sim_done = False
#         self.hardware_done = False

#     # ==================================================
#     # Stage-to-simulation mapping
#     # ==================================================

#     def execute_sim_stage(self, cycle_id, stage):
#         # HOME is used twice:
#         #   * once before the first ball,
#         #   * after every red or blue pick-and-place.
#         # It is never sent after black or green.
#         if stage == 'HOME':
#             self.execute_sim_home()
#             return

#         if stage.startswith('SPAWN_'):
#             color = stage.split('_', 1)[1].lower()
#             self.current_ball_name = self.spawn_ball(color, cycle_id)
#             return

#         if stage == 'CONVEYOR_TO_PICKUP':
#             self.require_current_ball()
#             self.move_ball_to_pickup(self.current_ball_name)
#             return

#         if stage == 'CONVEYOR_BLACK':
#             self.require_current_ball()
#             self.fall_ball(self.current_ball_name, direction=1)
#             return

#         if stage == 'CONVEYOR_GREEN':
#             self.require_current_ball()
#             self.fall_ball(self.current_ball_name, direction=-1)
#             return

#         if stage == 'PICKUP_READY':
#             self.send_arm_target(
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['PICKUP_READY'],
#             )
#             return

#         # Simulation gripper values are independent of EV3 encoder signs.
#         # This stage is used both before pickup and during release.
#         if stage == 'GRIPPER_WIDE_OPEN':
#             self.command_gripper(SIM_GRIPPER_OPEN)
#             time.sleep(GRIPPER_SETTLE_TIME)
#             self.release_current_ball()
#             return

#         if stage == 'PICK_DOWN':
#             self.send_arm_target(
#                 [SIM_BASE_HOME, SIM_PICK_PITCH],
#                 TRAJECTORY_TIMES['PICK_DOWN'],
#             )
#             return

#         if stage == 'GRIP_CLOSE':
#             self.command_gripper(SIM_GRIPPER_CLOSE)
#             time.sleep(GRIPPER_SETTLE_TIME)
#             self.attach_current_ball()
#             return

#         if stage == 'PICK_UP':
#             self.send_arm_target(
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['PICK_UP'],
#             )
#             return

#         if stage == 'ROTATE_RED':
#             self.send_arm_target(
#                 [SIM_BASE_RED_BIN, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['ROTATE_RED'],
#             )
#             return

#         if stage == 'ROTATE_BLUE':
#             self.send_arm_target(
#                 [SIM_BASE_BLUE_BIN, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['ROTATE_BLUE'],
#             )
#             return

#         if stage == 'PLACE_DOWN':
#             base_target = self.current_base_target()
#             self.send_arm_target(
#                 [base_target, SIM_PLACE_PITCH],
#                 TRAJECTORY_TIMES['PLACE_DOWN'],
#             )
#             return

#         if stage == 'PLACE_UP':
#             base_target = self.current_base_target()
#             self.send_arm_target(
#                 [base_target, CLEARANCE_PITCH],
#                 TRAJECTORY_TIMES['PLACE_UP'],
#             )
#             return

#         if stage == 'CYCLE_COMPLETE':
#             self.ball_attached = False
#             self.current_ball_name = None
#             self.get_logger().info('Simulation cycle state cleared.')
#             return

#         raise ValueError(f'No simulation implementation for stage {stage}')

#     def execute_sim_home(self):
#         """Run the visible Gazebo equivalent of the EV3 switch homing routine.

#         STAGE_START|...|HOME starts EV3 homing and this sequence together.
#         The stage barrier is released only after both have finished.
#         """

#         current_base = self.current_arm_target[0]

#         self.get_logger().info(
#             'HOME 1/3: EV3 homes arm; sim raises arm to clearance.'
#         )
#         self.send_arm_target(
#             [current_base, CLEARANCE_PITCH],
#             SIM_HOME_RAISE_TIME,
#         )

#         self.get_logger().info(
#             'HOME 2/3: EV3 searches base switch; sim sweeps base to +60 deg.'
#         )
#         self.send_arm_target(
#             [SIM_HOME_SWEEP_YAW, CLEARANCE_PITCH],
#             SIM_HOME_SWEEP_TIME,
#         )

#         self.get_logger().info(
#             'HOME 3/3: EV3 applies pickup offset; sim returns to center.'
#         )
#         self.send_arm_target(
#             [SIM_BASE_HOME, CLEARANCE_PITCH],
#             SIM_HOME_RETURN_TIME,
#         )

#         self._last_bin_target = SIM_BASE_HOME

#     def simulated_gripper_pose(self, base_yaw, arm_pitch):
#         """Approximate world pose of the gripper center for the large model."""

#         radial = SIM_GRIPPER_EFFECTIVE_REACH * math.cos(arm_pitch)
#         x = SIM_GRIPPER_BASE_X + radial * math.sin(base_yaw)
#         y = radial * math.cos(base_yaw)
#         z = (
#             SIM_GRIPPER_PIVOT_Z
#             + SIM_GRIPPER_EFFECTIVE_REACH * math.sin(arm_pitch)
#         )
#         return x, y, z

#     def attach_current_ball(self):
#         if not USE_LOGICAL_GRASP:
#             return

#         if not self.current_ball_name:
#             self.get_logger().info(
#                 'GRIP_CLOSE completed with no active ball (no-ball test).'
#             )
#             return

#         self.ball_attached = True
#         x, y, z = self.simulated_gripper_pose(*self.current_arm_target)
#         self.set_ball_pose(self.current_ball_name, x, y, z)
#         self.get_logger().info(
#             f'Logically attached {self.current_ball_name} to the gripper.'
#         )

#     def release_current_ball(self):
#         if not self.ball_attached:
#             return

#         if self.current_ball_name:
#             x, y, z = self.simulated_gripper_pose(*self.current_arm_target)
#             # Place it just below the finger center, then let Gazebo gravity act.
#             self.set_ball_pose(self.current_ball_name, x, y, z - 0.025)
#             self.get_logger().info(
#                 f'Released {self.current_ball_name} from the gripper.'
#             )

#         self.ball_attached = False

#     def follow_attached_ball(self, start_target, end_target, duration_sec):
#         if not (USE_LOGICAL_GRASP and self.ball_attached and self.current_ball_name):
#             return

#         start_time = time.monotonic()

#         while True:
#             elapsed = time.monotonic() - start_time
#             alpha = min(1.0, elapsed / max(duration_sec, 0.001))

#             base_yaw = start_target[0] + alpha * (end_target[0] - start_target[0])
#             arm_pitch = start_target[1] + alpha * (end_target[1] - start_target[1])
#             x, y, z = self.simulated_gripper_pose(base_yaw, arm_pitch)

#             try:
#                 self.set_ball_pose(self.current_ball_name, x, y, z)
#             except Exception as exc:
#                 self.get_logger().warn(
#                     f'Ball-follow pose update failed: {exc}'
#                 )

#             if alpha >= 1.0:
#                 return

#             time.sleep(BALL_FOLLOW_PERIOD)

#     def current_base_target(self):
#         if self.active_key is None:
#             return SIM_BASE_HOME

#         # PLACE_DOWN / PLACE_UP follow the previously commanded bin yaw.
#         # The controller holds the base joint, so read the intended side from
#         # the most recently completed rotate stage stored below.
#         return getattr(self, '_last_bin_target', SIM_BASE_HOME)

#     def require_current_ball(self):
#         if not self.current_ball_name:
#             raise RuntimeError('No active simulated ball exists.')

#     # ==================================================
#     # Arm and gripper controller helpers
#     # ==================================================

#     def wait_for_future(self, future, timeout_sec):
#         event = threading.Event()
#         future.add_done_callback(lambda _future: event.set())

#         if not event.wait(timeout_sec):
#             raise TimeoutError('ROS action future timed out.')

#         return future.result()

#     def send_arm_target(self, positions, duration_sec):
#         start_target = list(self.current_arm_target)
#         end_target = [float(positions[0]), float(positions[1])]
#         follow_thread = None

#         if USE_LOGICAL_GRASP and self.ball_attached and self.current_ball_name:
#             follow_thread = threading.Thread(
#                 target=self.follow_attached_ball,
#                 args=(start_target, end_target, duration_sec),
#                 daemon=True,
#             )
#             follow_thread.start()

#         if positions[0] in (SIM_BASE_RED_BIN, SIM_BASE_BLUE_BIN):
#             self._last_bin_target = positions[0]
#         elif positions[0] == SIM_BASE_HOME:
#             # Do not erase the remembered bin target during PICK_UP. It is
#             # overwritten on RETURN_HOME only after placement is complete.
#             if self.active_key and self.active_key[2] == 'RETURN_HOME':
#                 self._last_bin_target = SIM_BASE_HOME

#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint',
#         ]

#         point = JointTrajectoryPoint()
#         point.positions = [float(value) for value in positions]
#         point.time_from_start = Duration(
#             sec=int(duration_sec),
#             nanosec=int((duration_sec - int(duration_sec)) * 1e9),
#         )
#         goal.trajectory.points = [point]

#         goal_future = self.arm_client.send_goal_async(goal)
#         goal_handle = self.wait_for_future(goal_future, timeout_sec=3.0)

#         if goal_handle is None or not goal_handle.accepted:
#             raise RuntimeError('Trajectory goal was rejected.')

#         result_future = goal_handle.get_result_async()
#         wrapped_result = self.wait_for_future(
#             result_future,
#             timeout_sec=duration_sec + 4.0,
#         )

#         if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
#             raise RuntimeError(
#                 f'Trajectory action status was {wrapped_result.status}.'
#             )

#         result = wrapped_result.result

#         if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
#             raise RuntimeError(
#                 f'Trajectory controller error {result.error_code}: '
#                 f'{result.error_string}'
#             )

#         self.current_arm_target = end_target

#         if follow_thread is not None:
#             follow_thread.join(timeout=duration_sec + 2.0)

#         if USE_LOGICAL_GRASP and self.ball_attached and self.current_ball_name:
#             x, y, z = self.simulated_gripper_pose(*self.current_arm_target)
#             self.set_ball_pose(self.current_ball_name, x, y, z)

#     def command_gripper(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self.gripper_pub.publish(msg)

#     def start_conveyor_visual(self, speed):
#         msg = Float64()
#         msg.data = float(speed)
#         self.belt_vel_pub.publish(msg)

#     # ==================================================
#     # Ball helpers
#     # ==================================================

#     def spawn_ball(self, color, cycle_id, retries=3):
#         if color not in BALL_RGB:
#             raise ValueError(f'Unsupported ball colour: {color}')

#         name = f'{color}_ball_{cycle_id}'
#         r, g, b = BALL_RGB[color]
#         sdf = BALL_SDF.format(name=name, r=r, g=g, b=b)

#         cmd = [
#             'ros2',
#             'run',
#             'ros_gz_sim',
#             'create',
#             '-name',
#             name,
#             '-x',
#             str(SPAWN_X),
#             '-y',
#             str(SPAWN_Y),
#             '-z',
#             str(SPAWN_Z),
#             '-string',
#             sdf,
#         ]

#         for attempt in range(1, retries + 1):
#             try:
#                 result = subprocess.run(
#                     cmd,
#                     capture_output=True,
#                     text=True,
#                     timeout=10,
#                     check=False,
#                 )
#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     f'Spawn attempt {attempt} timed out.'
#                 )
#                 continue

#             if result.returncode == 0:
#                 self.get_logger().info(f'Spawned {name}.')
#                 return name

#             self.get_logger().warn(
#                 f'Spawn attempt {attempt} failed: {result.stderr.strip()}'
#             )

#         raise RuntimeError(f'Failed to spawn {name}.')

#     def move_ball_to_pickup(self, name):
#         x = SPAWN_X

#         while x < PICKUP_X:
#             x = min(x + STEP_SIZE, PICKUP_X)
#             self.set_ball_pose(name, x, SPAWN_Y, TRANSPORT_Z)
#             time.sleep(STEP_DELAY)

#     def fall_ball(self, name, direction):
#         x_end = (
#             BELT_END_L - 0.06
#             if direction < 0
#             else BELT_END_R + 0.06
#         )
#         x = SPAWN_X
#         z = TRANSPORT_Z
#         step = STEP_SIZE * direction

#         while (
#             (direction > 0 and x < x_end)
#             or (direction < 0 and x > x_end)
#         ):
#             x += step

#             if (
#                 (direction > 0 and x > BELT_END_R)
#                 or (direction < 0 and x < BELT_END_L)
#             ):
#                 z = max(z - 0.008, -0.05)

#             self.set_ball_pose(name, x, SPAWN_Y, z)
#             time.sleep(STEP_DELAY)

#     def set_ball_pose(self, name, x, y, z):
#         # Synchronous service execution prevents the short-lived async client
#         # processes that produced repeated "Host unreachable" responses.
#         cmd = [
#             'ign',
#             'service',
#             '-s',
#             f'/world/{WORLD_NAME}/set_pose',
#             '--reqtype',
#             'ignition.msgs.Pose',
#             '--reptype',
#             'ignition.msgs.Boolean',
#             '--timeout',
#             '1000',
#             '--req',
#             (
#                 f'name: "{name}" '
#                 f'position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#             ),
#         ]

#         try:
#             result = subprocess.run(
#                 cmd,
#                 stdout=subprocess.DEVNULL,
#                 stderr=subprocess.PIPE,
#                 text=True,
#                 timeout=2.0,
#                 check=False,
#             )
#         except subprocess.TimeoutExpired as exc:
#             raise RuntimeError(
#                 f'Set-pose request timed out for {name}.'
#             ) from exc

#         if result.returncode != 0:
#             raise RuntimeError(
#                 f'Set-pose failed for {name}: {result.stderr.strip()}'
#             )

#     # ==================================================
#     # Shutdown
#     # ==================================================

#     def destroy_node(self):
#         self.shutdown_event.set()
#         self.start_conveyor_visual(0.0)
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = SortingNode()

#     executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
#     executor.add_node(node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



# """Gazebo follower for the EV3 stage-synchronised digital twin.

# The EV3 owns the state order. This node only executes the stage named by the
# EV3 and acknowledges it after:

#     simulation stage completed
#     AND
#     STAGE_HW_DONE for the same cycle / sequence / stage arrived

# The ball spawning and set-pose movement intentionally reproduce the earlier
# working script:

#     STEP_SIZE = 0.008
#     STEP_DELAY = 0.090
#     asynchronous `ign service /set_pose` calls

# No logical ball attachment or ball-follow teleportation is used here.
# The simulated gripper and Gazebo contact behaviour are allowed to pick the
# ball as in the earlier working implementation.
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

# # Restored exactly from the earlier smooth ball-motion script.
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

# SIM_HOME_START_TIME = 1.5
# SIM_HOME_PITCH_UP_TIME = 1.0
# SIM_HOME_BASE_RIGHT_TIME = 1.2
# SIM_HOME_BASE_CENTER_TIME = 1.2

# SIM_PICKUP_READY_TIME = 1.0
# SIM_PICK_DOWN_TIME = 0.8
# SIM_PICK_UP_TIME = 0.8
# SIM_SWIVEL_TO_BIN_TIME = 1.2
# SIM_DROP_DOWN_TIME = 0.8
# SIM_DROP_UP_TIME = 0.8

# GRIPPER_CLOSE_SETTLE_TIME = 0.5
# GRIPPER_RELEASE_WAIT_1 = 0.4
# GRIPPER_RELEASE_WAIT_2 = 0.6


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
#         if stage == "HOME":
#             self.execute_sim_homing()
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

#             time.sleep(0.5)

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
#                 SIM_SWIVEL_TO_BIN_TIME,
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
#                 SIM_SWIVEL_TO_BIN_TIME,
#             )

#             return

#         if stage == "PLACE_DOWN":
#             self.send_arm_target(
#                 [
#                     self.last_bin_target,
#                     -0.45,
#                 ],
#                 SIM_DROP_DOWN_TIME,
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
#             self.last_bin_target = SIM_BASE_HOME

#             self.send_arm_target(
#                 [
#                     SIM_BASE_HOME,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_PICKUP_READY_TIME,
#             )

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

#     def execute_sim_homing(self):
#         self.get_logger().info(
#             "Executing original simulated homing sequence."
#         )

#         self.send_arm_target(
#             [0.0, 0.0],
#             SIM_HOME_START_TIME,
#         )

#         self.send_arm_target(
#             [0.0, CLEARANCE_PITCH],
#             SIM_HOME_PITCH_UP_TIME,
#         )

#         self.send_arm_target(
#             [1.0472, CLEARANCE_PITCH],
#             SIM_HOME_BASE_RIGHT_TIME,
#         )

#         self.send_arm_target(
#             [SIM_BASE_HOME, CLEARANCE_PITCH],
#             SIM_HOME_BASE_CENTER_TIME,
#         )

#         self.last_bin_target = SIM_BASE_HOME

#         self.get_logger().info(
#             "Simulated homing complete."
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

#     def send_arm_target(
#         self,
#         positions,
#         duration_sec,
#     ):
#         goal = FollowJointTrajectory.Goal()

#         goal.trajectory.joint_names = [
#             "arm_1_base_link_joint",
#             "arm_2_left_arm_linkage_joint",
#         ]

#         point = JointTrajectoryPoint()

#         point.positions = [
#             float(value)
#             for value in positions
#         ]

#         point.time_from_start = Duration(
#             sec=int(duration_sec),
#             nanosec=int(
#                 (
#                     duration_sec
#                     - int(duration_sec)
#                 )
#                 * 1e9
#             ),
#         )

#         goal.trajectory.points = [point]

#         goal_future = (
#             self.arm_client.send_goal_async(goal)
#         )

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

#         result_future = (
#             goal_handle.get_result_async()
#         )

#         wrapped_result = self.wait_for_future(
#             result_future,
#             timeout_sec=duration_sec + 4.0,
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
#         self.start_conveyor_visual(0.0)

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
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
# """Gazebo follower for the EV3 direct-centre stage-synchronised test.

# Execution order
# ---------------
# 1. HOME is used only for the initial synchronized homing sequence.

# 2. RED / BLUE:
#        pick-and-place
#        -> PLACE_UP
#        -> CENTER_HOLD

#    CENTER_HOLD returns the simulated base directly to zero while keeping
#    the arm at clearance. It does not replay the simulated homing animation.

# 3. GREEN / BLACK:
#        conveyor route only
#        -> CENTER_HOLD

# The EV3 owns the state order. This node executes the stage named by the EV3
# and acknowledges it only after:

#     simulation stage completed
#     AND
#     STAGE_HW_DONE for the same cycle / sequence / stage arrived

# The ball spawning and set-pose movement intentionally reproduce the earlier
# working script:

#     STEP_SIZE = 0.008
#     STEP_DELAY = 0.090
#     asynchronous `ign service /set_pose` calls

# No logical ball attachment or ball-follow teleportation is used here.
# The simulated gripper and Gazebo contact behaviour are allowed to pick the
# ball as in the earlier working implementation.
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

# # Restored exactly from the earlier smooth ball-motion script.
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

# SIM_HOME_START_TIME = 1.5
# SIM_HOME_PITCH_UP_TIME = 1.0
# SIM_HOME_BASE_RIGHT_TIME = 1.2
# SIM_HOME_BASE_CENTER_TIME = 1.2

# SIM_PICKUP_READY_TIME = 1.0
# SIM_PICK_DOWN_TIME = 0.8
# SIM_PICK_UP_TIME = 0.8
# SIM_SWIVEL_TO_BIN_TIME = 1.2
# SIM_DROP_DOWN_TIME = 0.8
# SIM_DROP_UP_TIME = 0.8

# # Direct base return after red/blue placement and centre hold for rejects.
# SIM_CENTER_RETURN_TIME = 1.1

# GRIPPER_CLOSE_SETTLE_TIME = 0.5
# GRIPPER_RELEASE_WAIT_1 = 0.4
# GRIPPER_RELEASE_WAIT_2 = 0.6


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
#         if stage == "HOME":
#             self.execute_sim_homing()
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

#             time.sleep(0.5)

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
#                 SIM_SWIVEL_TO_BIN_TIME,
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
#                 SIM_SWIVEL_TO_BIN_TIME,
#             )

#             return

#         if stage == "PLACE_DOWN":
#             self.send_arm_target(
#                 [
#                     self.last_bin_target,
#                     -0.45,
#                 ],
#                 SIM_DROP_DOWN_TIME,
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
#                 "Direct centre return: base -> 0, arm held at clearance."
#             )

#             self.last_bin_target = SIM_BASE_HOME

#             self.send_arm_target(
#                 [
#                     SIM_BASE_HOME,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_CENTER_RETURN_TIME,
#             )

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

#     def execute_sim_homing(self):
#         self.get_logger().info(
#             "Executing original simulated homing sequence."
#         )

#         self.send_arm_target(
#             [0.0, 0.0],
#             SIM_HOME_START_TIME,
#         )

#         self.send_arm_target(
#             [0.0, CLEARANCE_PITCH],
#             SIM_HOME_PITCH_UP_TIME,
#         )

#         self.send_arm_target(
#             [1.0472, CLEARANCE_PITCH],
#             SIM_HOME_BASE_RIGHT_TIME,
#         )

#         self.send_arm_target(
#             [SIM_BASE_HOME, CLEARANCE_PITCH],
#             SIM_HOME_BASE_CENTER_TIME,
#         )

#         self.last_bin_target = SIM_BASE_HOME

#         self.get_logger().info(
#             "Simulated homing complete."
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

#     def send_arm_target(
#         self,
#         positions,
#         duration_sec,
#     ):
#         goal = FollowJointTrajectory.Goal()

#         goal.trajectory.joint_names = [
#             "arm_1_base_link_joint",
#             "arm_2_left_arm_linkage_joint",
#         ]

#         point = JointTrajectoryPoint()

#         point.positions = [
#             float(value)
#             for value in positions
#         ]

#         point.time_from_start = Duration(
#             sec=int(duration_sec),
#             nanosec=int(
#                 (
#                     duration_sec
#                     - int(duration_sec)
#                 )
#                 * 1e9
#             ),
#         )

#         goal.trajectory.points = [point]

#         goal_future = (
#             self.arm_client.send_goal_async(goal)
#         )

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

#         result_future = (
#             goal_handle.get_result_async()
#         )

#         wrapped_result = self.wait_for_future(
#             result_future,
#             timeout_sec=duration_sec + 4.0,
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
#         self.start_conveyor_visual(0.0)

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
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()



# """Gazebo follower for the synchronized RED -> BLUE -> reject test.

# Stage order
# -----------
# HOME_INITIAL
# RED pick-and-place
# HOME_AFTER_RED
# BLUE pick-and-place
# HOME_AFTER_BLUE
# GREEN / BLACK conveyor-only cycles with CENTER_HOLD

# The three homing stages have separate smooth multi-point trajectories because
# the EV3's physical base-switch search takes different amounts of time from
# centre, the red bin, and the blue bin.

# The original ball spawn and asynchronous set-pose service call are preserved.
# Only the movement step/timing constants are adjusted for synchronization.
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

# # Same asynchronous set-pose implementation as the working script.
# # Pickup is split into smaller spatial steps while keeping about a 9.5 Hz
# # update rate. This gives ~3.55 s travel, matching the EV3 conveyor.
# PICKUP_STEP_SIZE = 0.004
# PICKUP_STEP_DELAY = 0.105

# # Separate reject values match the very different GREEN and BLACK travel.
# REJECT_STEP_SIZE = 0.006
# REJECT_STEP_DELAY = 0.090

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
# SIM_HOME_INITIAL_TIMES = [1.0, 2.2, 8.0, 12.8]
# SIM_HOME_AFTER_RED_TIMES = [1.2, 2.2, 11.2, 15.8]
# SIM_HOME_AFTER_BLUE_TIMES = [0.7, 1.5, 3.5, 10.7]

# # First-pass stage timing calibration from the latest timestamp log.
# # Some stages are reduced; stages where Gazebo was faster are increased.
# SIM_PICKUP_READY_TIME = 0.25
# SIM_PICK_DOWN_TIME = 1.40
# SIM_PICK_UP_TIME = 0.65

# SIM_ROTATE_RED_TIME = 0.50
# SIM_ROTATE_BLUE_TIME = 0.60

# SIM_DROP_DOWN_TIME = 0.80
# SIM_DROP_UP_TIME = 1.60

# SIM_REJECT_CENTER_HOLD_TIME = 0.25

# SIM_GRIPPER_READY_SETTLE = 0.15
# GRIPPER_CLOSE_SETTLE_TIME = 1.25
# GRIPPER_RELEASE_WAIT_1 = 0.50
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
#             self.send_arm_target(
#                 [
#                     self.last_bin_target,
#                     -0.45,
#                 ],
#                 SIM_DROP_DOWN_TIME,
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
#                 "Direct centre return: base -> 0, arm held at clearance."
#             )

#             self.last_bin_target = SIM_BASE_HOME

#             self.send_arm_target(
#                 [
#                     SIM_BASE_HOME,
#                     CLEARANCE_PITCH,
#                 ],
#                 SIM_REJECT_CENTER_HOLD_TIME,
#             )

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
#             positions = [
#                 [SIM_BASE_HOME, 0.0],
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#                 [1.0472, CLEARANCE_PITCH],
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#             ]
#             cumulative_times = SIM_HOME_INITIAL_TIMES

#         elif profile == "after_red":
#             positions = [
#                 [SIM_BASE_RED_BIN, 0.0],
#                 [SIM_BASE_RED_BIN, CLEARANCE_PITCH],
#                 [1.0472, CLEARANCE_PITCH],
#                 [SIM_BASE_HOME, CLEARANCE_PITCH],
#             ]
#             cumulative_times = SIM_HOME_AFTER_RED_TIMES

#         elif profile == "after_blue":
#             positions = [
#                 [SIM_BASE_BLUE_BIN, 0.0],
#                 [SIM_BASE_BLUE_BIN, CLEARANCE_PITCH],
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
#                 x + PICKUP_STEP_SIZE,
#                 x_end,
#             )

#             self.set_pose(
#                 name,
#                 x,
#                 SPAWN_Y,
#                 TRANSPORT_Z,
#             )

#             time.sleep(PICKUP_STEP_DELAY)

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

#         step = REJECT_STEP_SIZE * direction

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

#             time.sleep(REJECT_STEP_DELAY)

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


# ==================================================
# ORIGINAL SIMULATION TIMINGS
# ==================================================

# Smooth homing trajectories use cumulative wall-clock targets.
#
# Measured physical EV3 values from previous runs:
#   initial home       ~13.1 s
#   home after RED     ~16.2 s
#   home after BLUE    ~11.0 s
# Tuned from the most recent synchronized timestamp log.
#
# Latest measured gaps:
#   HOME_INITIAL: sim 0.555 s late
#   HOME_AFTER_RED: sim 1.771 s late
#
# HOME_AFTER_BLUE is based on the earlier valid blue-home measurement.
SIM_HOME_INITIAL_TIMES = [0.9, 1.9, 7.2, 12.2]
SIM_HOME_AFTER_RED_TIMES = [0.9, 1.8, 9.8, 14.0]
SIM_HOME_AFTER_BLUE_TIMES = [0.6, 1.2, 3.0, 9.5]

SIM_PICKUP_READY_TIME = 0.18
SIM_PICK_DOWN_TIME = 1.40
SIM_PICK_UP_TIME = 0.65

SIM_ROTATE_RED_TIME = 0.50
SIM_ROTATE_BLUE_TIME = 0.60

# PLACE_DOWN simulation finished 0.550 s early in the latest run.
SIM_DROP_DOWN_TIME = 1.35
SIM_DROP_UP_TIME = 1.60

# Reject cycles occur after BLUE homing, so the manipulator is already centred.
SIM_REJECT_CENTER_HOLD_TIME = 0.25

SIM_GRIPPER_READY_SETTLE = 0.15

# Keep close timing based on successful grasp runs. The latest zero-travel
# hardware stall is invalid timing data and must not be used for calibration.
GRIPPER_CLOSE_SETTLE_TIME = 1.25

# Physical release took about 2.09 s in the latest run.
GRIPPER_RELEASE_WAIT_1 = 0.55
GRIPPER_RELEASE_WAIT_2 = 1.50


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
            self.send_arm_target(
                [
                    self.last_bin_target,
                    -0.45,
                ],
                SIM_DROP_DOWN_TIME,
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
                "Centre hold: manipulator is already centred after BLUE home."
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
            positions = [
                [SIM_BASE_HOME, 0.0],
                [SIM_BASE_HOME, CLEARANCE_PITCH],
                [1.0472, CLEARANCE_PITCH],
                [SIM_BASE_HOME, CLEARANCE_PITCH],
            ]
            cumulative_times = SIM_HOME_INITIAL_TIMES

        elif profile == "after_red":
            positions = [
                [SIM_BASE_RED_BIN, 0.0],
                [SIM_BASE_RED_BIN, CLEARANCE_PITCH],
                [1.0472, CLEARANCE_PITCH],
                [SIM_BASE_HOME, CLEARANCE_PITCH],
            ]
            cumulative_times = SIM_HOME_AFTER_RED_TIMES

        elif profile == "after_blue":
            positions = [
                [SIM_BASE_BLUE_BIN, 0.0],
                [SIM_BASE_BLUE_BIN, CLEARANCE_PITCH],
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