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

# # Increase/decrease this for visual sync.
# # Good values: 1.0, 1.5, 2.0, 3.0
# VISUAL_SYNC_DELAY = 1.5

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

#     def _visual_delay(self, label=""):
#         if VISUAL_SYNC_DELAY > 0:
#             self.get_logger().info(
#                 f"Visual sync delay {VISUAL_SYNC_DELAY:.1f}s {label}"
#             )
#             time.sleep(VISUAL_SYNC_DELAY)

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
#         Per-ball and final homing cycle.

#         EV3 sends START_HOMING.
#         Sim homes.
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

#         self._execute_initial_homing()

#         self.get_logger().info("Waiting for EV3_HOMED...")
#         self._ev3_homed_event.wait()
#         self._ev3_homed_event.clear()

#         self._visual_delay("before ROS_HOMED")

#         self.get_logger().info("Publishing /sim/homed.")
#         self._publish_string(self._sim_homed_pub, "done")

#         self.get_logger().info("Waiting for EV3_GRIPPER_OPEN...")
#         self._ev3_gripper_open_event.wait()
#         self._ev3_gripper_open_event.clear()

#         self.get_logger().info("Opening simulated gripper.")
#         self._grip(SIM_GRIPPER_OPEN)
#         time.sleep(1.0)

#         self._visual_delay("before ROS_GRIPPER_OPEN")

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

#             self._visual_delay("after spawning ball, before ACK_SPAWN")

#             self._publish_string(self._spawn_confirmed_pub, color)
#             self.get_logger().info("Published /sim/spawn_confirmed.")

#             if color in ['red', 'blue']:
#                 # Sim ball travels to pickup while EV3 conveyor moves real ball.
#                 self._move_ball(name, SPAWN_X, PICKUP_X)

#                 # Old smooth coarse sync.
#                 self._pick_place_sequence(left_side=(color == 'red'))

#                 #self._visual_delay("before ROS_CYCLE_DONE")

#                 self._publish_string(self._sim_cycle_done_pub, color)
#                 self.get_logger().info("Published /sim/cycle_done.")

#             elif color in ['black', 'green']:
#                 # Correct behavior:
#                 # black -> runs down conveyor
#                 # green -> falls near sensor side
#                 direction = 1 if color == 'black' else -1

#                 self._fall(name, direction=direction)
#                 time.sleep(1.5)

#                 #self._visual_delay("before ROS_CYCLE_DONE")

#                 self._publish_string(self._sim_cycle_done_pub, color)
#                 self.get_logger().info("Published /sim/cycle_done.")

#         self.get_logger().info(
#             "All balls sorted. Waiting for final EV3 homing request..."
#         )

#         # EV3 should send one final START_HOMING after all balls are sorted.
#         self._execute_cycle_homing()

#         self.stop_conveyor()
#         self._send_trajectory([ARM_HOME], [2.0])
#         self._grip(SIM_GRIPPER_OPEN)

#         self.get_logger().info("=== Execution cycle complete. Robot is home. ===")

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

#         self._visual_delay("before ROS_READY_PICKUP")

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

#         self._visual_delay("before ROS_READY_PLACE")

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

import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64, Float64MultiArray, String


# ==================================================
# GEOMETRY & SPATIAL ANCHORS
# ==================================================

SPAWN_X = -0.14824
SPAWN_Y = 0.29075
SPAWN_Z = 0.09232
PICKUP_Z = SPAWN_Z + 0.022

PICKUP_X = -0.015
TRANSPORT_Z = SPAWN_Z + 0.005

BELT_END_R = 0.17
BELT_END_L = -0.17

STEP_SIZE = 0.008
STEP_DELAY = 0.090

SIM_BASE_HOME = 0.0
SIM_BASE_RED_BIN = -1.5708
SIM_BASE_BLUE_BIN = 1.5708

THETA1_MIN = -math.pi / 2
THETA1_MAX = math.pi / 2
THETA2_MIN = -0.55
THETA2_MAX = math.pi / 3

ARM_HOME = [0.0, 0.2]
CLEARANCE_PITCH = 0.2

SIM_GRIPPER_OPEN = 0.5
SIM_GRIPPER_CLOSE = 0.0

MAX_BALLS = 4


# ==================================================
# VISUAL TIMING
# ==================================================
# Increase these if the sim still looks too fast.
# These slow the actual sim motion, not the EV3 waiting time.

SIM_SETTLE_DELAY = 0.15

SIM_HOME_START_TIME = 3.5 # Time increased to match EV3 homing sequence
SIM_HOME_PITCH_UP_TIME = 2.5 # Time increased to match EV3 homing sequence
SIM_HOME_BASE_RIGHT_TIME = 3.5
SIM_HOME_BASE_CENTER_TIME = 3.0 

SIM_PICKUP_READY_TIME = 1.4
SIM_PICK_DOWN_TIME = 1.6
SIM_PICK_UP_TIME = 1.6

SIM_SWIVEL_TO_BIN_TIME = 2.2
SIM_DROP_DOWN_TIME = 1.7
SIM_RELEASE_WAIT = 0.8
SIM_DROP_UP_TIME = 1.7
SIM_RETURN_HOME_TIME = 2.2

SIM_FINAL_HOME_TIME = 2.0


BALL_RGB = {
    'red': (1, 0, 0),
    'blue': (0, 0, 1),
    'black': (0.05, 0.05, 0.05),
    'green': (0, 0.8, 0),
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

    theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
    theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

    return theta1, theta2


class SortingNode(Node):
    def __init__(self):
        super().__init__('sorting_node')

        # ---------------- Controllers ----------------
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        self._belt_vel_pub = self.create_publisher(
            Float64,
            '/conveyor_belt_vel',
            10
        )

        # ---------------- Hardware interface subscribers ----------------
        self._start_homing_sub = self.create_subscription(
            String,
            '/hw/start_homing',
            self._start_homing_callback,
            10
        )

        self._ev3_homed_sub = self.create_subscription(
            String,
            '/hw/ev3_homed',
            self._ev3_homed_callback,
            10
        )

        self._ev3_gripper_open_sub = self.create_subscription(
            String,
            '/hw/ev3_gripper_open',
            self._ev3_gripper_open_callback,
            10
        )

        self._color_sub = self.create_subscription(
            String,
            '/hw/ball_detected',
            self._ball_detected_callback,
            10
        )

        self._ready_pickup_sub = self.create_subscription(
            String,
            '/hw/ready_pickup',
            self._ready_pickup_callback,
            10
        )

        self._ready_place_sub = self.create_subscription(
            String,
            '/hw/ready_place',
            self._ready_place_callback,
            10
        )

        self._theta1_sub = self.create_subscription(
            Float64,
            '/hw/theta1',
            self._theta1_callback,
            10
        )

        self._theta2_sub = self.create_subscription(
            Float64,
            '/hw/theta2',
            self._theta2_callback,
            10
        )

        # ---------------- Sim -> Hardware interface publishers ----------------
        self._sim_homed_pub = self.create_publisher(
            String,
            '/sim/homed',
            10
        )

        self._sim_gripper_open_pub = self.create_publisher(
            String,
            '/sim/gripper_open',
            10
        )

        self._spawn_confirmed_pub = self.create_publisher(
            String,
            '/sim/spawn_confirmed',
            10
        )

        self._sim_ready_pickup_pub = self.create_publisher(
            String,
            '/sim/ready_pickup',
            10
        )

        self._sim_ready_place_pub = self.create_publisher(
            String,
            '/sim/ready_place',
            10
        )

        self._sim_cycle_done_pub = self.create_publisher(
            String,
            '/sim/cycle_done',
            10
        )

        # ---------------- Thread synchronization ----------------
        self._start_homing_event = threading.Event()
        self._ev3_homed_event = threading.Event()
        self._ev3_gripper_open_event = threading.Event()

        self._color_event = threading.Event()
        self._pickup_event = threading.Event()
        self._place_event = threading.Event()

        self._detected_color = None
        self.ball_count = 0

        threading.Thread(
            target=self._start,
            daemon=True
        ).start()

    # ==================================================
    # ROS utility
    # ==================================================

    def _publish_string(self, publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def _settle(self):
        time.sleep(SIM_SETTLE_DELAY)

    # ==================================================
    # Subscriber callbacks
    # ==================================================

    def _start_homing_callback(self, msg):
        self.get_logger().info("Received /hw/start_homing.")
        self._start_homing_event.set()

    def _ev3_homed_callback(self, msg):
        self.get_logger().info("Received /hw/ev3_homed.")
        self._ev3_homed_event.set()

    def _ev3_gripper_open_callback(self, msg):
        self.get_logger().info("Received /hw/ev3_gripper_open.")
        self._ev3_gripper_open_event.set()

    def _ball_detected_callback(self, msg):
        color = msg.data.strip().lower()

        if color not in ['red', 'blue', 'black', 'green']:
            self.get_logger().warn(f"Ignoring unknown color: {color}")
            return

        self._detected_color = color
        self._color_event.set()

        self.get_logger().info(f"Hardware detected ball: {color}")

    def _ready_pickup_callback(self, msg):
        self.get_logger().info("Hardware reached pickup-ready pose.")
        self._pickup_event.set()

    def _ready_place_callback(self, msg):
        self.get_logger().info("Hardware reached place-ready pose.")
        self._place_event.set()

    def _theta1_callback(self, msg):
        self.get_logger().info(f"[EV3 telemetry] theta1 = {msg.data:.2f} deg")

    def _theta2_callback(self, msg):
        self.get_logger().info(f"[EV3 telemetry] theta2 = {msg.data:.2f} deg")

    # ==================================================
    # Main startup thread
    # ==================================================

    def _start(self):
        self.get_logger().info("Waiting for arm action server...")
        self._arm_client.wait_for_server()
        self.get_logger().info("Arm action server available.")

        self.get_logger().info("Starting coordinated sorting loop.")
        self._main_loop()

    # ==================================================
    # Homing
    # ==================================================

    def _execute_initial_homing(self):
        self.get_logger().info("Executing simulated homing sequence...")

        self._send_trajectory([[0.0, 0.0]], [SIM_HOME_START_TIME])
        self._settle()

        self.get_logger().info("Homing step 1: arm_2 pitching up to 0.2 rad.")
        self._send_trajectory([[0.0, 0.2]], [SIM_HOME_PITCH_UP_TIME])
        self._settle()

        self.get_logger().info("Homing step 2: arm_1 rotating right by +60 deg.")
        self._send_trajectory([[1.0472, 0.2]], [SIM_HOME_BASE_RIGHT_TIME])
        self._settle()

        self.get_logger().info("Homing step 3: arm_1 returning to center/home.")
        self._send_trajectory([[0.0, 0.2]], [SIM_HOME_BASE_CENTER_TIME])
        self._settle()

        self._grip(SIM_GRIPPER_OPEN)
        time.sleep(0.3)

        self.get_logger().info("Sim homing motion complete.")

    def _execute_cycle_homing(self):
        """
        Per-ball and final homing cycle.

        EV3 sends START_HOMING.
        Sim homes.
        EV3 sends EV3_HOMED.
        Sim publishes /sim/homed.
        EV3 opens gripper and sends EV3_GRIPPER_OPEN.
        Sim opens gripper and publishes /sim/gripper_open.
        """

        self.get_logger().info("Waiting for EV3 START_HOMING...")
        self._start_homing_event.wait()
        self._start_homing_event.clear()

        self._ev3_homed_event.clear()
        self._ev3_gripper_open_event.clear()
        self._pickup_event.clear()
        self._place_event.clear()

        self._execute_initial_homing()

        self.get_logger().info("Waiting for EV3_HOMED...")
        self._ev3_homed_event.wait()
        self._ev3_homed_event.clear()

        self.get_logger().info("Publishing /sim/homed.")
        self._publish_string(self._sim_homed_pub, "done")

        self.get_logger().info("Waiting for EV3_GRIPPER_OPEN...")
        self._ev3_gripper_open_event.wait()
        self._ev3_gripper_open_event.clear()

        self.get_logger().info("Opening simulated gripper.")
        self._grip(SIM_GRIPPER_OPEN)
        time.sleep(0.5)

        self.get_logger().info("Publishing /sim/gripper_open.")
        self._publish_string(self._sim_gripper_open_pub, "done")

    # ==================================================
    # Main coordinated loop
    # ==================================================

    def _main_loop(self):
        self.start_conveyor(0.05)

        while self.ball_count < MAX_BALLS:
            # Home the sim every cycle, triggered by EV3.
            self._execute_cycle_homing()

            self.get_logger().info("Waiting for hardware ball detection...")

            self._color_event.wait()
            color = self._detected_color

            self._detected_color = None
            self._color_event.clear()

            if color is None:
                continue

            self.get_logger().info(
                f"=== Ball {self.ball_count + 1}: {color.upper()} ==="
            )

            # Spawn immediately after EV3 sensor detects the ball.
            name = self._spawn(color)
            time.sleep(0.1)

            # Do not delay this. EV3 waits for ACK_SPAWN before moving conveyor.
            self._publish_string(self._spawn_confirmed_pub, color)
            self.get_logger().info("Published /sim/spawn_confirmed.")

            if color in ['red', 'blue']:
                # Sim ball travels to pickup while EV3 conveyor moves real ball.
                self._move_ball(name, SPAWN_X, PICKUP_X)

                # Old smooth coarse sync.
                self._pick_place_sequence(left_side=(color == 'red'))

                # Do not delay cycle done. EV3 should start next sequence quickly.
                self._publish_string(self._sim_cycle_done_pub, color)
                self.get_logger().info("Published /sim/cycle_done.")

            elif color in ['black', 'green']:
                # Correct behavior:
                # black -> runs down conveyor
                # green -> falls near sensor side
                direction = 1 if color == 'black' else -1

                self._fall(name, direction=direction)

                # Do not delay cycle done. EV3 should start next sequence quickly.
                self._publish_string(self._sim_cycle_done_pub, color)
                self.get_logger().info("Published /sim/cycle_done.")

        self.get_logger().info(
            "All balls sorted. Waiting for final EV3 homing request..."
        )

        # EV3 should send one final START_HOMING after all balls are sorted.
        self._execute_cycle_homing()

        self.stop_conveyor()
        self._send_trajectory([ARM_HOME], [SIM_FINAL_HOME_TIME])
        self._grip(SIM_GRIPPER_OPEN)

        self.get_logger().info("=== Execution cycle complete. Robot is home. ===")

    # ==================================================
    # Red / Blue pick-place sequence - coarse sync
    # ==================================================

    def _pick_place_sequence(self, left_side=True):
        bin_yaw = SIM_BASE_RED_BIN if left_side else SIM_BASE_BLUE_BIN

        _, pick_pitch = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)

        # ---------------- PICKUP READY ----------------
        self._grip(SIM_GRIPPER_OPEN)
        time.sleep(0.1)

        self.get_logger().info("Moving sim arm to pickup-ready pose.")
        self._send_trajectory(
            positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
            durations=[SIM_PICKUP_READY_TIME]
        )
        self._settle()

        self.get_logger().info("Waiting for EV3 READY_PICKUP...")
        self._pickup_event.wait()
        self._pickup_event.clear()

        # Do not add extra delay here. EV3 should pick immediately.
        self._publish_string(self._sim_ready_pickup_pub, "ready")
        self.get_logger().info("Published /sim/ready_pickup.")

        # ---------------- PICK ----------------
        self.get_logger().info("Sim pitching down to pick ball.")
        self._send_trajectory(
            positions=[[SIM_BASE_HOME, pick_pitch]],
            durations=[SIM_PICK_DOWN_TIME]
        )
        self._settle()

        self.get_logger().info("Closing simulated gripper.")
        self._grip(SIM_GRIPPER_CLOSE)
        time.sleep(0.5)

        self.get_logger().info("Sim pitching up to clearance with ball.")
        self._send_trajectory(
            positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
            durations=[SIM_PICK_UP_TIME]
        )
        self._settle()

        # ---------------- PLACE READY ----------------
        self.get_logger().info("Waiting for EV3 READY_PLACE...")
        self._place_event.wait()
        self._place_event.clear()

        # Do not add extra delay here. EV3 should place immediately.
        self._publish_string(self._sim_ready_place_pub, "ready")
        self.get_logger().info("Published /sim/ready_place.")

        # ---------------- PLACE ----------------
        self.get_logger().info(
            f"Sim swiveling to bin yaw {math.degrees(bin_yaw):.1f} deg."
        )
        self._send_trajectory(
            positions=[[bin_yaw, CLEARANCE_PITCH]],
            durations=[SIM_SWIVEL_TO_BIN_TIME]
        )
        self._settle()

        # self.get_logger().info("Sim pitching down into bin.")
        # self._send_trajectory(
        #     positions=[[bin_yaw, -0.45]],
        #     durations=[SIM_DROP_DOWN_TIME]
        # )
        # self._settle()

        # self.get_logger().info("Opening simulated gripper to release ball.")
        # self._grip(0.2)
        # time.sleep(0.3)

        # self._grip(SIM_GRIPPER_OPEN)
        # time.sleep(SIM_RELEASE_WAIT)

        # self.get_logger().info("Sim pitching up from bin.")
        # self._send_trajectory(
        #     positions=[[bin_yaw, CLEARANCE_PITCH]],
        #     durations=[SIM_DROP_UP_TIME]
        # )
        # self._settle()

        # # self.get_logger().info("Sim returning to pickup/home state.")
        # # self._send_trajectory(
        # #     positions=[[SIM_BASE_HOME, CLEARANCE_PITCH]],
        # #     durations=[SIM_RETURN_HOME_TIME]
        # # )
        # # self._settle()

        # # self._grip(SIM_GRIPPER_OPEN)
        # # time.sleep(0.2)

        self.get_logger().info("Sim pitching down into bin.")
        self._send_trajectory(
            positions=[[bin_yaw, -0.45]],
            durations=[SIM_DROP_DOWN_TIME]
        )
        self._settle()

        self.get_logger().info("Opening simulated gripper to release ball.")
        self._grip(0.2)
        time.sleep(0.3)

        self._grip(SIM_GRIPPER_OPEN)
        time.sleep(SIM_RELEASE_WAIT)

        self.get_logger().info("Sim pitching up from bin.")
        self._send_trajectory(
            positions=[[bin_yaw, CLEARANCE_PITCH]],
            durations=[SIM_DROP_UP_TIME]
        )
        self._settle()

        self._grip(SIM_GRIPPER_OPEN)
        time.sleep(0.2)

        self.get_logger().info("Sim place sequence complete.")




    # ==================================================
    # Controller helpers
    # ==================================================

    def _send_trajectory(self, positions, durations):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'arm_1_base_link_joint',
            'arm_2_left_arm_linkage_joint'
        ]

        for pos, t in zip(positions, durations):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in pos]
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t - int(t)) * 1e9)
            )
            goal.trajectory.points.append(point)

        future = self._arm_client.send_goal_async(goal)

        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return

        result_future = goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            time.sleep(0.01)

    def _grip(self, position):
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self._gripper_pub.publish(msg)

    def start_conveyor(self, speed=0.05):
        msg = Float64()
        msg.data = float(speed)
        self._belt_vel_pub.publish(msg)

    def stop_conveyor(self):
        self.start_conveyor(0.0)

    # ==================================================
    # Ball motion helpers
    # ==================================================

    def _move_ball(self, name, x_start, x_end):
        x = x_start

        while x < x_end:
            x = min(x + STEP_SIZE, x_end)

            self._set_pose(
                name,
                x,
                SPAWN_Y,
                TRANSPORT_Z
            )

            time.sleep(STEP_DELAY)

    def _fall(self, name, direction):
        x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
        x = SPAWN_X
        z = TRANSPORT_Z

        step = STEP_SIZE * direction

        while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
            x += step

            if (
                (direction > 0 and x > BELT_END_R)
                or
                (direction < 0 and x < BELT_END_L)
            ):
                z = max(z - 0.006, -0.05)

            self._set_pose(
                name,
                x,
                SPAWN_Y,
                z
            )

            time.sleep(STEP_DELAY)

    def _set_pose(self, name, x, y, z):
        cmd = [
            'ign',
            'service',
            '-s',
            '/world/empty/set_pose',
            '--reqtype',
            'ignition.msgs.Pose',
            '--reptype',
            'ignition.msgs.Boolean',
            '--timeout',
            '150',
            '--req',
            f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
        ]

        subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def _spawn(self, color, retries=3):
        self.ball_count += 1

        name = f'{color}_ball_{self.ball_count}'

        r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))

        sdf = BALL_SDF.format(
            name=name,
            r=r,
            g=g,
            b=b
        )

        cmd = [
            'ros2',
            'run',
            'ros_gz_sim',
            'create',
            '-name',
            name,
            '-x',
            str(SPAWN_X),
            '-y',
            str(SPAWN_Y),
            '-z',
            str(SPAWN_Z),
            '-string',
            sdf
        ]

        for attempt in range(retries):
            try:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=10
                )

                if result.returncode == 0:
                    self.get_logger().info(f"Spawned {name}.")
                    return name

                self.get_logger().warn(
                    f"Spawn attempt {attempt + 1} failed: {result.stderr}"
                )

            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    f"Spawn attempt {attempt + 1} timed out."
                )
                time.sleep(1.0)

        self.get_logger().error(
            f"Failed to spawn {name}; continuing anyway."
        )

        return name


def main(args=None):
    rclpy.init(args=args)

    node = SortingNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_conveyor()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()