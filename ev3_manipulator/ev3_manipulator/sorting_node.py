#!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64MultiArray
# import subprocess
# import threading
# import time

# # ── Geometry ──────────────────────────────────────────────────────────────────
# SPAWN_X    = -0.14824
# SPAWN_Y    =  0.29075
# SPAWN_Z    =  0.09232

# # ── Link lengths (m) ──────────────────────────────────────────────────────────
# L_0   = 0.094
# L_1   = 0.041
# L_2   = 0.09005
# L_3   = 0.226211
# L_4   = 0.196208
# L_ARM = L_2 + L_3
# SHOULDER_Z = 0.2666

# PICKUP_X    = -0.020
# # Ball rests on belt at SPAWN_Z; transport 5 mm higher so ball bottom clears
# # the mesh surface (belt top ≈ Z 0.074) and eliminates contact impulses.
# TRANSPORT_Z = SPAWN_Z + 0.005
# # Gripper 20 mm above TRANSPORT_Z so finger tips clear the belt during pick.
# PICKUP_Z    = TRANSPORT_Z + 0.020
# BELT_END_R  =  0.17
# BELT_END_L  = -0.17
# STEP_SIZE  =  0.004
# STEP_DELAY =  0.04

# # ── Joint limits ──────────────────────────────────────────────────────────────
# THETA1_MIN = -math.pi / 2
# THETA1_MAX =  math.pi / 2
# THETA2_MIN = -0.6
# THETA2_MAX =  math.pi / 3

# BALL_CYCLE = ['red', 'blue', 'black', 'green']
# BALL_RGB   = {
#     'red':   (1,    0,    0   ),
#     'blue':  (0,    0,    1   ),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0,    0.8,  0   ),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
#       </inertial>
#       <collision name='col'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <surface>
#           <friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction>
#           <contact>
#             <ode>
#               <kp>5e3</kp><kd>50.0</kd>
#               <min_depth>0.005</min_depth>
#               <max_vel>0.1</max_vel>
#             </ode>
#           </contact>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# # ═════════════════════════════════════════════════════════════════════════════
# # INVERSE KINEMATICS
# # ═════════════════════════════════════════════════════════════════════════════

# # Forward-kinematics constants derived from the URDF joint chain:
# #   effective arm length (shoulder → gripper projected): 0.226963 m
# #   gripper Z when theta2 = 0 (arm horizontal): SHOULDER_Z − 0.11937 ≈ 0.1472 m
# _L_ARM2  = 0.226963
# _Z_GRIP0 = SHOULDER_Z - 0.11937

# def solve_ik_sim(x, y, z_target):
#     """
#     theta1 = atan2(x, y)                      — base rotation toward target
#     theta2 = asin((z_target − Z_GRIP0) / L)   — pitch to bring gripper to z_target
#     """
#     theta1 = math.atan2(x, y)
#     sin_t2 = (z_target - _Z_GRIP0) / _L_ARM2
#     sin_t2 = max(-1.0, min(1.0, sin_t2))
#     theta2 = math.asin(sin_t2)
#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))
#     return theta1, theta2


# # ═════════════════════════════════════════════════════════════════════════════
# # SORTING NODE
# # ═════════════════════════════════════════════════════════════════════════════

# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting1_node')
#         self._arm_client = ActionClient(
#             self, FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory')
#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray, '/gripper_controller/commands', 10)
#         self.ball_count  = 0
#         self.cycle_index = 0
#         threading.Thread(target=self._start, daemon=True).start()

#     def _start(self):
#         time.sleep(2.0)
#         self.get_logger().info('Waiting for arm controller...')
#         self._arm_client.wait_for_server()
#         self.get_logger().info('Ready — starting sort loop')
#         self._main_loop()

#     def _main_loop(self):
#         self._arm([0.0, 0.0], secs=2)
#         self._grip(0.0)

#         while self.ball_count < 4:
#             color = BALL_CYCLE[self.cycle_index % 4]
#             self.cycle_index += 1
#             self.get_logger().info(
#                 f'=== Ball {self.cycle_index}: {color.upper()} ===')

#             name = self._spawn(color)
#             time.sleep(0.1)  # brief pause for spawn to register; set_pose corrects Z

#             if color == 'red':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_left()
#             elif color == 'blue':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_right()
#             elif color == 'black':
#                 self._fall(name, direction=-1)
#             elif color == 'green':
#                 self._fall(name, direction=1)

#             time.sleep(0.3)

#         self._arm([0.0, 0.0], secs=2)
#         self._grip(0.5)
#         self.get_logger().info('=== All 4 balls processed — sorting complete ✓ ===')

#     # ── Pick & Place LEFT (red) ───────────────────────────────────────────────
#     def _pick_place_left(self):
#         self.get_logger().info('Pick → Place LEFT')
#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)
#         self.get_logger().info(f'IK pickup → theta1=0.0, theta2={pick_t2:.4f}')

#         self._arm([0.0, 0.0],     secs=1)
#         self._grip(0.3)
#         self._arm([0.0, pick_t2], secs=1)
#         time.sleep(0.5)
#         self._grip(-0.25)
#         time.sleep(0.8)
#         self._arm([0.0, 0.0],       secs=1)
#         self._arm([-1.5708, 0.0],   secs=3)
#         time.sleep(0.5)
#         self._arm([-1.5708, -0.46], secs=1)
#         time.sleep(0.3)
#         self._grip(0.7)
#         time.sleep(1.0)
#         self._arm([-1.5708, 0.0],   secs=1)
#         time.sleep(0.6)
#         self._grip(0.0)
#         self._arm([0.0, 0.0],       secs=2)
#         self.get_logger().info('Place LEFT complete ✓')

#     # ── Pick & Place RIGHT (blue) ─────────────────────────────────────────────
#     def _pick_place_right(self):
#         self.get_logger().info('Pick → Place RIGHT')
#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)
#         self.get_logger().info(f'IK pickup → theta1=0.0, theta2={pick_t2:.4f}')

#         self._arm([0.0, 0.0],     secs=1)
#         self._grip(0.3)
#         self._arm([0.0, pick_t2], secs=1)
#         time.sleep(0.5)
#         self._grip(-0.25)
#         time.sleep(0.8)
#         self._arm([0.0, 0.0],      secs=1)
#         self._arm([1.5708, 0.0],   secs=3)
#         time.sleep(0.5)
#         self._arm([1.5708, -0.46], secs=1)
#         time.sleep(0.3)
#         self._grip(0.7)
#         time.sleep(1.0)
#         self._arm([1.5708, 0.0],   secs=1)
#         time.sleep(0.5)
#         self._grip(0.0)
#         self._arm([0.0, 0.0],      secs=2)
#         self.get_logger().info('Place RIGHT complete ✓')

#     # ── Ball movement via set_pose ────────────────────────────────────────────
#     def _move_ball(self, name, x_start, x_end):
#         x = x_start
#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)
#             self._set_pose(name, x, SPAWN_Y, TRANSPORT_Z)
#             time.sleep(STEP_DELAY)
#         self.get_logger().info(f'{name} at pickup zone x={x_end:.4f}')

#     def _fall(self, name, direction):
#         side  = 'LEFT' if direction < 0 else 'RIGHT'
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         self.get_logger().info(f'{name} → falling off {side} end')
#         x    = SPAWN_X
#         z    = TRANSPORT_Z
#         step = STEP_SIZE * direction
#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step
#             if (direction > 0 and x > BELT_END_R) or \
#                (direction < 0 and x < BELT_END_L):
#                 z = max(z - 0.006, -0.05)
#             self._set_pose(name, x, SPAWN_Y, z)
#             time.sleep(STEP_DELAY)
#         self.get_logger().info(f'{name} fell off {side} ✓')

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign', 'service', '-s', '/world/empty/set_pose',
#             '--reqtype', 'ignition.msgs.Pose',
#             '--reptype', 'ignition.msgs.Boolean',
#             '--timeout', '200',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]
#         try:
#             subprocess.run(cmd, capture_output=True, timeout=0.3)
#         except subprocess.TimeoutExpired:
#             pass

#     # ── Arm ───────────────────────────────────────────────────────────────────
#     def _arm(self, positions, secs=2):
#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint',
#         ]
#         p = JointTrajectoryPoint()
#         p.positions       = [float(v) for v in positions]
#         p.velocities      = [0.0, 0.0]
#         p.time_from_start = Duration(sec=int(secs))
#         goal.trajectory.points = [p]
#         future = self._arm_client.send_goal_async(goal)
#         while not future.done():
#             time.sleep(0.02)
#         result_future = future.result().get_result_async()
#         while not result_future.done():
#             time.sleep(0.02)

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)
#         time.sleep(0.8)

#     # ── Spawn with retry ──────────────────────────────────────────────────────
#     def _spawn(self, color, retries=3):
#         self.ball_count += 1
#         name    = f'{color}_ball_{self.ball_count}'
#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))
#         sdf     = BALL_SDF.format(name=name, r=r, g=g, b=b)
#         cmd = [
#             'ros2', 'run', 'ros_gz_sim', 'create',
#             '-name', name,
#             '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
#             '-string', sdf
#         ]
#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(
#                     cmd, capture_output=True, text=True, timeout=15)
#                 if result.returncode == 0:
#                     self.get_logger().info(f'Spawned {name}')
#                     return name
#                 else:
#                     self.get_logger().warn(
#                         f'Spawn attempt {attempt+1} failed: {result.stderr[:80]}')
#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     f'Spawn timeout attempt {attempt+1}/{retries} — retrying...')
#                 time.sleep(2.0)
#         self.get_logger().error(f'All {retries} spawn attempts failed for {name}')
#         return name


# def main(args=None):
#     rclpy.init(args=args)
#     node = SortingNode()
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)
#     executor.spin()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

 #!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64, Float64MultiArray
# import subprocess
# import threading
# import time

# # ── Geometry ──────────────────────────────────────────────────────────────────
# SPAWN_X    = -0.14824
# SPAWN_Y    =  0.29075
# SPAWN_Z    =  0.09232

# # ── Link lengths (m) ──────────────────────────────────────────────────────────
# L_0   = 0.094
# L_1   = 0.041
# L_2   = 0.09005
# L_3   = 0.226211
# L_4   = 0.196208
# L_ARM = L_2 + L_3
# SHOULDER_Z = 0.2666

# PICKUP_X   = -0.020
# BELT_END_R =  0.17
# BELT_END_L = -0.17
# STEP_SIZE  =  0.004
# STEP_DELAY =  0.04

# # ── Joint limits ──────────────────────────────────────────────────────────────
# THETA1_MIN = -math.pi / 2
# THETA1_MAX =  math.pi / 2
# THETA2_MIN = -0.6
# THETA2_MAX =  math.pi / 3

# # ── Gripper constants ─────────────────────────────────────────────────────────
# # GRIPPER_OPEN  =  0.0    # neutral / release
# # GRIPPER_CLOSE = -0.7    # grip a ball

# BALL_CYCLE = ['red', 'blue', 'black', 'green']
# BALL_RGB   = {
#     'red':   (1,    0,    0   ),
#     'blue':  (0,    0,    1   ),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0,    0.8,  0   ),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
#       </inertial>
#       <collision name='col'>
#         <geometry><sphere><radius>0.0183</radius></sphere></geometry>
#         <surface>
#           <friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction>
#           <contact>
#             <ode>
#               <kp>5e3</kp><kd>50.0</kd>
#               <min_depth>0.005</min_depth>
#               <max_vel>0.1</max_vel>
#             </ode>
#           </contact>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry><sphere><radius>0.0183</radius></sphere></geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# # ═════════════════════════════════════════════════════════════════════════════
# # INVERSE KINEMATICS
# # ═════════════════════════════════════════════════════════════════════════════

# def solve_ik_sim(x, y, z_target):
#     """
#     Returns (theta1, theta2) in radians.
#     theta1 = atan2(x, y)    — which direction to point
#     theta2 = atan2(dz, r)   — how far to pitch down
#     """
#     theta1 = math.atan2(x, y)
#     r      = math.sqrt(x**2 + y**2)
#     dz     = z_target - SHOULDER_Z
#     theta2 = math.atan2(dz, r)
#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))
#     return theta1, theta2


# # ═════════════════════════════════════════════════════════════════════════════
# # SORTING NODE
# # ═════════════════════════════════════════════════════════════════════════════

# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting1_node')
#         self._arm_client = ActionClient(
#             self, FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory')
#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray, '/gripper_controller/commands', 10)
#         self._belt_vel_pub = self.create_publisher(
#             Float64, '/conveyor_belt_vel', 10)
#         self.ball_count  = 0
#         self.cycle_index = 0
#         threading.Thread(target=self._start, daemon=True).start()

#     def _start(self):
#         time.sleep(2.0)
#         self.get_logger().info('Waiting for arm controller...')
#         self._arm_client.wait_for_server()
#         self.get_logger().info('Ready — starting sort loop')
#         self._main_loop()

#     def _main_loop(self):
#         self._arm([0.0, 0.0], secs=2)
#         self._grip(0.0)
#         self.start_conveyor(50.0)
#         time.sleep(1.0)

#         while self.ball_count < 4:
#             color = BALL_CYCLE[self.cycle_index % 4]
#             self.cycle_index += 1
#             self.get_logger().info(
#                 f'=== Ball {self.cycle_index}: {color.upper()} ===')

#             name = self._spawn(color)
#             time.sleep(0.5)

#             if color == 'red':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_left()
#             elif color == 'blue':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_right()
#             elif color == 'black':
#                 self._fall(name, direction=-1)
#                 time.sleep(1.5)
#             elif color == 'green':
#                 self._fall(name, direction=1)
#                 time.sleep(1.5)

#             time.sleep(0.3)

#         self.stop_conveyor()
#         self._arm([0.0, 0.0], secs=2)
#         self._grip(0.5)
#         self.get_logger().info('=== All 4 balls processed — sorting complete ✓ ===')

#     # ── Conveyor ──────────────────────────────────────────────────────────────
#     def start_conveyor(self, speed=0.05):
#         msg = Float64()
#         msg.data = float(speed)
#         self._belt_vel_pub.publish(msg)
#         self.get_logger().info(f'Conveyor set to {speed}')

#     def stop_conveyor(self):
#         self.start_conveyor(0.0)

#     # ── Pick & Place LEFT (red) ───────────────────────────────────────────────
#     def _pick_place_left(self):
#         self.get_logger().info('Pick → Place LEFT')

#         # theta1=0.0: ball is directly ahead — no base rotation needed for pickup
#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, SPAWN_Z)
#         self.get_logger().info(f'IK pickup → theta1=0.0, theta2={pick_t2:.4f}')

#         # 1. Home
#         self._arm([0.0, 0.0],     secs=1)
#         self._grip(0.4)
#         # 2. Straight down — no swing
#         self._arm([0.0, pick_t2], secs=1)
#         time.sleep(0.2)
#         # 3. Close gripper
#         self._grip(-0.25)
#         time.sleep(0.8)
#         # 4. Lift straight up
#         self._arm([0.0, 0.0],       secs=1)
#         # 5. Rotate left — slowly so inertia doesn't drop ball
#         self._arm([-1.5708, 0.0],   secs=3)
#         time.sleep(0.5)
#         # 6. Descend to drop height
#         self._arm([-1.5708, -0.46], secs=1)
#         time.sleep(0.3)
#         # 7. Open gripper — neutral (0.0), NOT positive value
#         self._grip(0.7)
#         time.sleep(1.0)             # wait for ball to fall completely clear
#         # 8. Lift after ball has fallen — won't re-catch
#         self._arm([-1.5708, 0.0],   secs=1)
#         time.sleep(0.6)
#         self._grip(0.0)
#         # 9. Return home
#         self._arm([0.0, 0.0],       secs=2)
#         self.get_logger().info('Place LEFT complete ✓')

#     # ── Pick & Place RIGHT (blue) ─────────────────────────────────────────────
#     def _pick_place_right(self):
#         self.get_logger().info('Pick → Place RIGHT')

#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, SPAWN_Z)
#         self.get_logger().info(f'IK pickup → theta1=0.0, theta2={pick_t2:.4f}')

#         # 1. Home
#         self._arm([0.0, 0.0],     secs=1)
#         self._grip(0.4)
#         # 2. Straight down — no swing
#         self._arm([0.0, pick_t2], secs=1)
#         time.sleep(0.2)
#         # 3. Close gripper
#         self._grip(-0.25)
#         time.sleep(0.8)
#         # 4. Lift straight up
#         self._arm([0.0, 0.0],      secs=1)
#         # 5. Rotate right — slowly
#         self._arm([1.5708, 0.0],   secs=3)
#         time.sleep(0.5)
#         # 6. Descend to drop height
#         self._arm([1.5708, -0.46], secs=1)
#         time.sleep(0.3)
#         # 7. Open gripper — neutral
#         self._grip(0.7)
#         time.sleep(1.0)             # wait for ball to fall completely clear
#         # 8. Lift after ball has fallen
#         self._arm([1.5708, 0.0],   secs=1)
#         time.sleep(0.5)
#         self._grip(0.0)
#         # 9. Return home
#         self._arm([0.0, 0.0],      secs=2)
#         self.get_logger().info('Place RIGHT complete ✓')

#     # ── Ball movement ─────────────────────────────────────────────────────────
#     def _move_ball(self, name, x_start, x_end):
#         x = x_start
#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)
#             self._set_pose(name, x, SPAWN_Y, SPAWN_Z)
#             time.sleep(STEP_DELAY)
#         self.get_logger().info(f'{name} at pickup zone x={x_end:.4f}')

#     def _fall(self, name, direction):
#         side  = 'LEFT' if direction < 0 else 'RIGHT'
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         self.get_logger().info(f'{name} → falling off {side} end')
#         x    = SPAWN_X
#         z    = SPAWN_Z
#         step = STEP_SIZE * direction
#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step
#             if (direction > 0 and x > BELT_END_R) or \
#                (direction < 0 and x < BELT_END_L):
#                 z = max(z - 0.006, -0.05)
#             self._set_pose(name, x, SPAWN_Y, z)
#             time.sleep(STEP_DELAY)
#         self.get_logger().info(f'{name} fell off {side} ✓')

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign', 'service', '-s', '/world/empty/set_pose',
#             '--reqtype', 'ignition.msgs.Pose',
#             '--reptype', 'ignition.msgs.Boolean',
#             '--timeout', '200',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]
#         try:
#             subprocess.run(cmd, capture_output=True, timeout=0.3)
#         except subprocess.TimeoutExpired:
#             pass

#     # ── Arm ───────────────────────────────────────────────────────────────────
#     def _arm(self, positions, secs=2):
#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint',
#         ]
#         p = JointTrajectoryPoint()
#         p.positions       = [float(v) for v in positions]
#         p.velocities      = [0.0, 0.0]
#         p.time_from_start = Duration(sec=int(secs))
#         goal.trajectory.points = [p]
#         future = self._arm_client.send_goal_async(goal)
#         while not future.done():
#             time.sleep(0.02)
#         result_future = future.result().get_result_async()
#         while not result_future.done():
#             time.sleep(0.02)

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)
#         time.sleep(0.8)

#     # ── Spawn with retry ──────────────────────────────────────────────────────
#     def _spawn(self, color, retries=3):
#         self.ball_count += 1
#         name    = f'{color}_ball_{self.ball_count}'
#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))
#         sdf     = BALL_SDF.format(name=name, r=r, g=g, b=b)
#         cmd = [
#             'ros2', 'run', 'ros_gz_sim', 'create',
#             '-name', name,
#             '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
#             '-string', sdf
#         ]
#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(
#                     cmd, capture_output=True, text=True, timeout=15)
#                 if result.returncode == 0:
#                     self.get_logger().info(f'Spawned {name}')
#                     return name
#                 else:
#                     self.get_logger().warn(
#                         f'Spawn attempt {attempt+1} failed: {result.stderr[:80]}')
#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     f'Spawn timeout attempt {attempt+1}/{retries} — retrying...')
#                 time.sleep(2.0)
#         self.get_logger().error(f'All {retries} spawn attempts failed for {name}')
#         return name


# def main(args=None):
#     rclpy.init(args=args)
#     node = SortingNode()
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)
#     executor.spin()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




#!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64MultiArray
# import subprocess
# import threading
# import time

# # ── Geometry ──────────────────────────────────────────────────────────────────
# SPAWN_X    = -0.14824
# SPAWN_Y    =  0.29075
# SPAWN_Z    =  0.09232

# # ── Link lengths (m) ──────────────────────────────────────────────────────────
# L_0   = 0.094
# L_1   = 0.041
# L_2   = 0.09005
# L_3   = 0.226211
# L_4   = 0.196208
# L_ARM = L_2 + L_3
# SHOULDER_Z = 0.2666

# PICKUP_X    = -0.020
# # Ball rests at SPAWN_Z; move 5mm above to reduce belt contact physics cost
# TRANSPORT_Z = SPAWN_Z + 0.005
# BELT_END_R  =  0.17
# BELT_END_L  = -0.17
# STEP_SIZE   =  0.009
# STEP_DELAY  =  0.08

# # ── Joint limits ──────────────────────────────────────────────────────────────
# THETA1_MIN = -math.pi / 2
# THETA1_MAX =  math.pi / 2
# THETA2_MIN = -0.6        # max pitch down — avoids conveyor collision
# THETA2_MAX =  math.pi / 3

# # ── Gripper constants ─────────────────────────────────────────────────────────
# GRIPPER_PREOPEN =  0.3    # open wide before descending onto ball
# GRIPPER_CLOSE   = -0.25   # grip the ball
# GRIPPER_RELEASE =  0.6   # gentle push — breaks friction without jamming fingers
# GRIPPER_NEUTRAL =  0.2   # neutral after arm clears ball

# BALL_CYCLE = ['red', 'blue', 'black', 'green']
# BALL_RGB   = {
#     'red':   (1,    0,    0   ),
#     'blue':  (0,    0,    1   ),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0,    0.8,  0   ),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
#       </inertial>
#       <collision name='col'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <surface>
#           <friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction>
#           <contact>
#             <ode>
#               <kp>5e3</kp><kd>50.0</kd>
#               <min_depth>0.005</min_depth>
#               <max_vel>0.1</max_vel>
#             </ode>
#           </contact>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# # ═════════════════════════════════════════════════════════════════════════════
# # INVERSE KINEMATICS
# # ═════════════════════════════════════════════════════════════════════════════

# def solve_ik_sim(x, y, z_target):

#     theta1 = math.atan2(x, y)
#     # r      = math.sqrt(x**2 + y**2)
#     # dz     = z_target - SHOULDER_Z

#     # theta2 = math.asin(dz, r)
#     Z0 = 0.220995          # circle centre height (verified, was wrong as SHOULDER_Z-0.11937)
#     R_ARM = 0.226963  

#     sin_t2 = (z_target - Z0) / R_ARM
#     sin_t2 = max(-1.0, min(1.0, sin_t2))
#     theta2 = math.asin(sin_t2)

#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

#     return theta1, theta2


# # ═════════════════════════════════════════════════════════════════════════════
# # SORTING NODE
# # ═════════════════════════════════════════════════════════════════════════════

# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting1_node')
#         self._arm_client = ActionClient(
#             self, FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory')
#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray, '/gripper_controller/commands', 10)
#         self.ball_count  = 0
#         self.cycle_index = 0
#         threading.Thread(target=self._start, daemon=True).start()

#     def _start(self):
#         time.sleep(2.0)
#         self.get_logger().info('Waiting for arm controller...')
#         self._arm_client.wait_for_server()

#         # Command home immediately — don't let gravity sag the arm
#         self.get_logger().info('Commanding home position...')
#         self._arm([0.0, 0.0], secs=2)
#         time.sleep(2.5)

#         self.get_logger().info('Ready — starting sort loop')
#         self._main_loop()

#     def _main_loop(self):
#         self._arm([0.0, 0.0], secs=2)
#         self._grip(GRIPPER_NEUTRAL)

#         while self.ball_count < 4:
#             color = BALL_CYCLE[self.cycle_index % 4]
#             self.cycle_index += 1
#             self.get_logger().info(
#                 f'=== Ball {self.cycle_index}: {color.upper()} ===')

#             name = self._spawn(color)
#             time.sleep(0.1)

#             if color == 'red':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_left()
#             elif color == 'blue':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_right()
#             elif color == 'black':
#                 self._fall(name, direction=-1)
#                 time.sleep(1.5)
#             elif color == 'green':
#                 self._fall(name, direction=1)
#                 time.sleep(1.5)

#             time.sleep(0.3)

#         self._arm([0.0, 0.0], secs=2)
#         self._grip(GRIPPER_NEUTRAL)
#         self.get_logger().info('=== All 4 balls processed — sorting complete ✓ ===')

#     # ── Pick & Place LEFT (red) ───────────────────────────────────────────────
#     def _pick_place_left(self):
#         self.get_logger().info('Pick → Place LEFT')

#         # theta1=0.0: ball is directly ahead — no base rotation at pickup
#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)
#         self.get_logger().info(f'IK pickup → theta1=0.0, theta2={pick_t2:.4f}')

#         # 1. Home
#         self._arm([0.0, 0.0],     secs=1)
#         # 2. Open fingers wide before descending — no collision on entry
#         self._grip(GRIPPER_PREOPEN)
#         # 3. Straight down — theta1 stays 0.0, no lateral sway
#         self._arm([0.0, pick_t2], secs=1)
#         time.sleep(0.5)
#         # 4. Close gripper
#         self._grip(GRIPPER_CLOSE)
#         time.sleep(0.8)
#         # 5. Lift straight up — stay at theta1=0, only theta2 changes
#         self._arm([0.0, 0.0],       secs=1)
#         # 6. Rotate left slowly — inertia won't drop ball at secs=3
#         self._arm([-1.5708, 0.0],   secs=3)
#         time.sleep(0.5)
#         # 7. Descend to drop — deeper than before so ball has clearance
#         self._arm([-1.5708, -0.55], secs=1)
#         time.sleep(0.3)
#         # 8. Gentle push open — breaks friction without jamming fingers into ball
#         self._grip(GRIPPER_RELEASE)
#         time.sleep(2.0)            # long wait — gravity pulls ball completely clear
#         # 9. Lift AFTER ball has fallen — won't re-catch
#         self._arm([-1.5708, 0.0],   secs=1)
#         # 10. Return gripper to neutral AFTER arm is clear
#         self._grip(GRIPPER_NEUTRAL)
#         # 11. Return home
#         self._arm([0.0, 0.0],       secs=2)
#         self.get_logger().info('Place LEFT complete ✓')

#     # ── Pick & Place RIGHT (blue) ─────────────────────────────────────────────
#     def _pick_place_right(self):
#         self.get_logger().info('Pick → Place RIGHT')

#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)
#         self.get_logger().info(f'IK pickup → theta1=0.0, theta2={pick_t2:.4f}')

#         # 1. Home
#         self._arm([0.0, 0.0],     secs=1)
#         # 2. Pre-open
#         self._grip(GRIPPER_PREOPEN)
#         # 3. Straight down
#         self._arm([0.0, pick_t2], secs=1)
#         time.sleep(0.5)
#         # 4. Close
#         self._grip(GRIPPER_CLOSE)
#         time.sleep(0.8)
#         # 5. Lift
#         self._arm([0.0, 0.0],      secs=1)
#         # 6. Rotate right slowly
#         self._arm([1.5708, 0.0],   secs=3)
#         time.sleep(0.5)
#         # 7. Descend to drop
#         self._arm([1.5708, -0.55], secs=1)
#         time.sleep(0.3)
#         # 8. Gentle release
#         self._grip(GRIPPER_RELEASE)
#         time.sleep(2.0)
#         # 9. Lift after ball has fallen
#         self._arm([1.5708, 0.0],   secs=1)
#         # 10. Neutral after arm is clear
#         self._grip(GRIPPER_NEUTRAL)
#         # 11. Home
#         self._arm([0.0, 0.0],      secs=2)
#         self.get_logger().info('Place RIGHT complete ✓')

#     # ── Ball movement ─────────────────────────────────────────────────────────
#     def _move_ball(self, name, x_start, x_end):
#         x = x_start
#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)
#             self._set_pose(name, x, SPAWN_Y, TRANSPORT_Z)
#             time.sleep(STEP_DELAY)
#         self.get_logger().info(f'{name} at pickup zone x={x_end:.4f}')

#     def _fall(self, name, direction):
#         side  = 'LEFT' if direction < 0 else 'RIGHT'
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         self.get_logger().info(f'{name} → falling off {side} end')
#         x    = SPAWN_X
#         z    = TRANSPORT_Z
#         step = STEP_SIZE * direction
#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step
#             if (direction > 0 and x > BELT_END_R) or \
#                (direction < 0 and x < BELT_END_L):
#                 z = max(z - 0.006, -0.05)
#             self._set_pose(name, x, SPAWN_Y, z)
#             time.sleep(STEP_DELAY)
#         self.get_logger().info(f'{name} fell off {side} ✓')

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign', 'service', '-s', '/world/empty/set_pose',
#             '--reqtype', 'ignition.msgs.Pose',
#             '--reptype', 'ignition.msgs.Boolean',
#             '--timeout', '200',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]
#         try:
#             subprocess.run(cmd, capture_output=True, timeout=0.3)
#         except subprocess.TimeoutExpired:
#             pass

#     # ── Arm ───────────────────────────────────────────────────────────────────
#     def _arm(self, positions, secs=2):
#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint',
#         ]
#         p = JointTrajectoryPoint()
#         p.positions       = [float(v) for v in positions]
#         p.velocities      = [0.0, 0.0]
#         p.time_from_start = Duration(sec=int(secs))
#         goal.trajectory.points = [p]
#         future = self._arm_client.send_goal_async(goal)
#         while not future.done():
#             time.sleep(0.02)
#         result_future = future.result().get_result_async()
#         while not result_future.done():
#             time.sleep(0.02)

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)
#         time.sleep(0.8)

#     # ── Spawn with retry ──────────────────────────────────────────────────────
#     def _spawn(self, color, retries=3):
#         self.ball_count += 1
#         name    = f'{color}_ball_{self.ball_count}'
#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))
#         sdf     = BALL_SDF.format(name=name, r=r, g=g, b=b)
#         cmd = [
#             'ros2', 'run', 'ros_gz_sim', 'create',
#             '-name', name,
#             '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
#             '-string', sdf
#         ]
#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(
#                     cmd, capture_output=True, text=True, timeout=15)
#                 if result.returncode == 0:
#                     self.get_logger().info(f'Spawned {name}')
#                     return name
#                 else:
#                     self.get_logger().warn(
#                         f'Spawn attempt {attempt+1} failed: {result.stderr[:80]}')
#             except subprocess.TimeoutExpired:
#                 self.get_logger().warn(
#                     f'Spawn timeout attempt {attempt+1}/{retries} — retrying...')
#                 time.sleep(2.0)
#         self.get_logger().error(f'All {retries} spawn attempts failed for {name}')
#         return name


# def main(args=None):
#     rclpy.init(args=args)
#     node = SortingNode()
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)
#     executor.spin()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64MultiArray
# import subprocess
# import threading
# import time

# # ── Geometry ──────────────────────────────────────────────────────────────────
# SPAWN_X    = -0.14824
# SPAWN_Y    =  0.29075
# SPAWN_Z    =  0.09232
# PICKUP_Z   =  SPAWN_Z  # Added missing constant

# # ── Link lengths (m) ──────────────────────────────────────────────────────────
# L_0   = 0.094
# L_1   = 0.041
# L_2   = 0.09005
# L_3   = 0.226211
# L_4   = 0.196208
# L_ARM = L_2 + L_3
# SHOULDER_Z = 0.2666

# PICKUP_X    = -0.020
# TRANSPORT_Z = SPAWN_Z + 0.005
# BELT_END_R  =  0.17
# BELT_END_L  = -0.17
# STEP_SIZE   =  0.005  # Decreased step size for finer resolution
# STEP_DELAY  =  0.03  # Decreased delay for higher frame-rate updates

# # ── Joint limits ──────────────────────────────────────────────────────────────
# THETA1_MIN = -math.pi / 2
# THETA1_MAX =  math.pi / 2
# THETA2_MIN = -0.6        
# THETA2_MAX =  math.pi / 3

# # ── Gripper constants ─────────────────────────────────────────────────────────
# GRIPPER_PREOPEN =  0.3    
# GRIPPER_CLOSE   = -0.25   
# GRIPPER_RELEASE =  0.6   
# GRIPPER_NEUTRAL =  0.2   

# BALL_CYCLE = ['red', 'blue', 'black', 'green']
# BALL_RGB   = {
#     'red':   (1,    0,    0   ),
#     'blue':  (0,    0,    1   ),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0,    0.8,  0   ),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
#       </inertial>
#       <collision name='col'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <surface>
#           <friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction>
#           <contact>
#             <ode>
#               <kp>5e3</kp><kd>50.0</kd>
#               <min_depth>0.005</min_depth>
#               <max_vel>0.1</max_vel>
#             </ode>
#           </contact>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# # ═════════════════════════════════════════════════════════════════════════════
# # INVERSE KINEMATICS
# # ═════════════════════════════════════════════════════════════════════════════
# def solve_ik_sim(x, y, z_target):
#     theta1 = math.atan2(x, y)
#     Z0 = 0.220995          
#     R_ARM = 0.226963  

#     sin_t2 = (z_target - Z0) / R_ARM
#     sin_t2 = max(-1.0, min(1.0, sin_t2))
#     theta2 = math.asin(sin_t2)

#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

#     return theta1, theta2


# # ═════════════════════════════════════════════════════════════════════════════
# # SORTING NODE
# # ═════════════════════════════════════════════════════════════════════════════
# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting1_node')
#         self._arm_client = ActionClient(
#             self, FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory')
#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray, '/gripper_controller/commands', 10)
        
#         # NOTE: For perfectly smooth ball movement, you should replace the subprocess
#         # method entirely with a native ROS 2 publisher or service client bridged to Gazebo.
        
#         self.ball_count  = 0
#         self.cycle_index = 0
#         threading.Thread(target=self._start, daemon=True).start()

#     def _start(self):
#         time.sleep(2.0)
#         self.get_logger().info('Waiting for arm controller...')
#         self._arm_client.wait_for_server()

#         self.get_logger().info('Commanding home position...')
#         self._send_arm_trajectory([[0.0, 0.0]], durations=[2.0])
#         time.sleep(2.5)

#         self.get_logger().info('Ready — starting sort loop')
#         self._main_loop()

#     def _main_loop(self):
#         self._send_arm_trajectory([[0.0, 0.0]], durations=[2.0])
#         self._grip(GRIPPER_NEUTRAL)

#         while self.ball_count < 4:
#             color = BALL_CYCLE[self.cycle_index % 4]
#             self.cycle_index += 1
#             self.get_logger().info(f'=== Ball {self.cycle_index}: {color.upper()} ===')

#             name = self._spawn(color)
#             time.sleep(0.1)

#             if color == 'red':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_sequence(left_side=True)
#             elif color == 'blue':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_sequence(left_side=False)
#             elif color == 'black':
#                 self._fall(name, direction=-1)
#                 time.sleep(1.5)
#             elif color == 'green':
#                 self._fall(name, direction=1)
#                 time.sleep(1.5)

#             time.sleep(0.3)

#         self._send_arm_trajectory([[0.0, 0.0]], durations=[2.0])
#         self._grip(GRIPPER_NEUTRAL)
#         self.get_logger().info('=== All 4 balls processed — sorting complete ✓ ===')

#     # ── Consolidated & Smoothed Pick & Place Sequence ─────────────────────────
#     def _pick_place_sequence(self, left_side=True):
#         side_str = 'LEFT' if left_side else 'RIGHT'
#         target_theta1 = -1.5708 if left_side else 1.5708
#         self.get_logger().info(f'Smooth Pick → Place {side_str}')

#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)

#         # Phase 1: Move smoothly from current position down to the ball
#         # Waypoints: [Home, Open Gripper mid-flight, Reach target]
#         self._grip(GRIPPER_PREOPEN)
#         self._send_arm_trajectory(
#             positions=[[0.0, 0.0], [0.0, pick_t2]], 
#             durations=[1.0, 2.0]
#         )
#         time.sleep(2.1) # Wait for trajectory completion + safety buffer

#         # Actuate gripper firmly
#         self._grip(GRIPPER_CLOSE)
#         time.sleep(0.8)

#         # Phase 2: Combined smooth travel path to the delivery zone
#         # Waypoints: [Lift vertically up, Smoothly swing to drop zone, Lower down slightly]
#         self._send_arm_trajectory(
#             positions=[
#                 [0.0, 0.0],                   # Point 1: Lift Straight Up
#                 [target_theta1, 0.0],         # Point 2: Swing to Side
#                 [target_theta1, -0.55]        # Point 3: Descend over bin
#             ],
#             durations=[1.0, 3.0, 4.0]         # Cumulative times from starting execution
#         )
#         time.sleep(4.2)

#         # Release ball
#         self._grip(GRIPPER_RELEASE)
#         time.sleep(1.5) 

#         # Phase 3: Clean escape and return home
#         self._send_arm_trajectory(
#             positions=[[target_theta1, 0.0], [0.0, 0.0]], 
#             durations=[1.0, 2.5]
#         )
#         time.sleep(1.0)
#         self._grip(GRIPPER_NEUTRAL) # Reset gripper structure safely on return
#         time.sleep(1.6)

#         self.get_logger().info(f'Place {side_str} complete ✓')

#     # ── Smooth Multi-Waypoint Trajectory Sender ──────────────────────────────
#     def _send_arm_trajectory(self, positions, durations):
#         """
#         Sends a smooth path containing multiple sequential configurations.
#         'positions' should be a list of lists: e.g., [[j1, j2], [j1, j2]]
#         'durations' is a list of timestamps relative to start: e.g., [1.0, 2.5]
#         """
#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint',
#         ]
        
#         goal.trajectory.points = []
#         for pos, t in zip(positions, durations):
#             p = JointTrajectoryPoint()
#             p.positions = [float(v) for v in pos]
#             # Letting the joint controller automatically interpolate velocities 
#             # by omitting hardcoded 0.0 array vectors at dynamic intermediate steps
#             p.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
#             goal.trajectory.points.append(p)

#         future = self._arm_client.send_goal_async(goal)
#         while not future.done():
#             time.sleep(0.01)
#         result_future = future.result().get_result_async()
#         while not result_future.done():
#             time.sleep(0.01)

#     # ── Ball movement ─────────────────────────────────────────────────────────
#     def _move_ball(self, name, x_start, x_end):
#         x = x_start
#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)
#             self._set_pose(name, x, SPAWN_Y, TRANSPORT_Z)
#             time.sleep(STEP_DELAY)
#         self.get_logger().info(f'{name} at pickup zone x={x_end:.4f}')

#     def _fall(self, name, direction):
#         side  = 'LEFT' if direction < 0 else 'RIGHT'
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         self.get_logger().info(f'{name} → falling off {side} end')
#         x    = SPAWN_X
#         z    = TRANSPORT_Z
#         step = STEP_SIZE * direction
#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step
#             if (direction > 0 and x > BELT_END_R) or \
#                (direction < 0 and x < BELT_END_L):
#                 z = max(z - 0.006, -0.05)
#             self._set_pose(name, x, SPAWN_Y, z)
#             time.sleep(STEP_DELAY)
#         self.get_logger().info(f'{name} fell off {side} ✓')

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign', 'service', '-s', '/world/empty/set_pose',
#             '--reqtype', 'ignition.msgs.Pose',
#             '--reptype', 'ignition.msgs.Boolean',
#             '--timeout', '100',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]
#         try:
#             # High-speed background processing to minimize loop choking
#             subprocess.run(cmd, capture_output=True, timeout=0.05)
#         except subprocess.TimeoutExpired:
#             pass

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)

#     # ── Spawn ─────────────────────────────────────────────────────────────────
#     def _spawn(self, color, retries=3):
#         self.ball_count += 1
#         name    = f'{color}_ball_{self.ball_count}'
#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))
#         sdf     = BALL_SDF.format(name=name, r=r, g=g, b=b)
#         cmd = [
#             'ros2', 'run', 'ros_gz_sim', 'create',
#             '-name', name,
#             '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
#             '-string', sdf
#         ]
#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
#                 if result.returncode == 0:
#                     self.get_logger().info(f'Spawned {name}')
#                     return name
#             except subprocess.TimeoutExpired:
#                 time.sleep(1.0)
#         return name


# def main(args=None):
#     rclpy.init(args=args)
#     node = SortingNode()
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)
#     executor.spin()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()






#!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64MultiArray
# import subprocess
# import threading
# import time

# # ── Geometry ──────────────────────────────────────────────────────────────────
# SPAWN_X    = -0.14824
# SPAWN_Y    =  0.29075
# SPAWN_Z    =  0.09232

# # Offset target upward so the gripper doesn't plunge into the conveyor belt
# PICKUP_Z   =  SPAWN_Z + 0.022 

# # ── Link lengths (m) ──────────────────────────────────────────────────────────
# L_0   = 0.094
# L_1   = 0.041
# L_2   = 0.09005
# L_3   = 0.226211
# L_4   = 0.196208
# L_ARM = L_2 + L_3
# SHOULDER_Z = 0.2666

# PICKUP_X    = -0.020
# TRANSPORT_Z = SPAWN_Z + 0.005
# BELT_END_R  =  0.17
# BELT_END_L  = -0.17
# STEP_SIZE   =  0.006  
# STEP_DELAY  =  0.04  # Well balanced for Popen background executions

# # ── Joint limits ──────────────────────────────────────────────────────────────
# THETA1_MIN = -math.pi / 2
# THETA1_MAX =  math.pi / 2
# THETA2_MIN = -0.55       # Clamped to avoid conveyor collision
# THETA2_MAX =  math.pi / 3

# # ── Gripper constants ─────────────────────────────────────────────────────────
# GRIPPER_PREOPEN =  0.3    
# GRIPPER_CLOSE   = -0.25   
# GRIPPER_RELEASE =  0.6   
# GRIPPER_NEUTRAL =  0.2   

# BALL_CYCLE = ['red', 'blue', 'black', 'green']
# BALL_RGB   = {
#     'red':   (1,    0,    0   ),
#     'blue':  (0,    0,    1   ),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0,    0.8,  0   ),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
#       </inertial>
#       <collision name='col'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <surface>
#           <friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction>
#           <contact>
#             <ode>
#               <kp>5e3</kp><kd>50.0</kd>
#               <min_depth>0.005</min_depth>
#               <max_vel>0.1</max_vel>
#             </ode>
#           </contact>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# # ═════════════════════════════════════════════════════════════════════════════
# # INVERSE KINEMATICS
# # ═════════════════════════════════════════════════════════════════════════════
# def solve_ik_sim(x, y, z_target):
#     theta1 = math.atan2(x, y)
#     Z0 = 0.220995          
#     R_ARM = 0.226963  

#     sin_t2 = (z_target - Z0) / R_ARM
#     sin_t2 = max(-1.0, min(1.0, sin_t2))
#     theta2 = math.asin(sin_t2)

#     # Apply protective safety boundaries
#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

#     return theta1, theta2


# # ═════════════════════════════════════════════════════════════════════════════
# # SORTING NODE
# # ═════════════════════════════════════════════════════════════════════════════
# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting1_node')
#         self._arm_client = ActionClient(
#             self, FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory')
#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray, '/gripper_controller/commands', 10)
#         self.ball_count  = 0
#         self.cycle_index = 0
#         threading.Thread(target=self._start, daemon=True).start()

#     def _start(self):
#         time.sleep(2.0)
#         self.get_logger().info('Waiting for arm controller...')
#         self._arm_client.wait_for_server()

#         self.get_logger().info('Commanding home position...')
#         self._send_trajectory([[0.0, 0.0]], [2.0])
#         time.sleep(2.5)

#         self.get_logger().info('Ready — starting sort loop')
#         self._main_loop()

#     def _main_loop(self):
#         self._send_trajectory([[0.0, 0.0]], [2.0])
#         self._grip(GRIPPER_NEUTRAL)

#         while self.ball_count < 4:
#             color = BALL_CYCLE[self.cycle_index % 4]
#             self.cycle_index += 1
#             self.get_logger().info(f'=== Ball {self.cycle_index}: {color.upper()} ===')

#             name = self._spawn(color)
#             time.sleep(0.1)

#             if color == 'red':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_sequence(left_side=True)
#             elif color == 'blue':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_sequence(left_side=False)
#             elif color == 'black':
#                 self._fall(name, direction=-1)
#                 time.sleep(1.5)
#             elif color == 'green':
#                 self._fall(name, direction=1)
#                 time.sleep(1.5)

#             time.sleep(0.3)

#         self._send_trajectory([[0.0, 0.0]], [2.0])
#         self._grip(GRIPPER_NEUTRAL)
#         self.get_logger().info('=== All 4 balls processed — sorting complete ✓ ===')

#     # ── Continuous Multi-Waypoint Motion Profile ──────────────────────────────
#     def _pick_place_sequence(self, left_side=True):
#         side_str = 'LEFT' if left_side else 'RIGHT'
#         target_theta1 = -1.5708 if left_side else 1.5708
#         self.get_logger().info(f'Executing seamless Pick → Place {side_str}')

#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)
#         self.get_logger().info(f'Calculated Safe Pickup Angle: theta2={pick_t2:.4f} rad')

#         # 1. Approach & Descend smoothly
#         self._grip(GRIPPER_PREOPEN)
#         self._send_trajectory(
#             positions=[[0.0, 0.0], [0.0, pick_t2]], 
#             durations=[1.0, 2.2]
#         )
#         time.sleep(2.3)

#         # 2. Secure Object
#         self._grip(GRIPPER_CLOSE)
#         time.sleep(0.8)

#         # 3. Continuous flight path to delivery profile (No stuttering mid-air)
#         self._send_trajectory(
#             positions=[
#                 [0.0, 0.0],               # Lift up vertically
#                 [target_theta1, 0.0],     # Swing around cleanly
#                 [target_theta1, -0.52]    # Position over destination bin
#             ],
#             durations=[1.0, 2.8, 3.8]     # Cumulative timeline sequence
#         )
#         time.sleep(4.0)

#         # 4. Release & Reset cleanly
#         self._grip(GRIPPER_RELEASE)
#         time.sleep(1.2)

#         self._send_trajectory(
#             positions=[[target_theta1, 0.0], [0.0, 0.0]], 
#             durations=[1.0, 2.5]
#         )
#         time.sleep(1.0)
#         self._grip(GRIPPER_NEUTRAL)
#         time.sleep(1.6)

#         self.get_logger().info(f'Place {side_str} complete ✓')

#     # ── Multi-Waypoint Controller Interface ───────────────────────────────────
#     def _send_trajectory(self, positions, durations):
#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint',
#         ]
        
#         for pos, t in zip(positions, durations):
#             p = JointTrajectoryPoint()
#             p.positions = [float(v) for v in pos]
#             p.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
#             goal.trajectory.points.append(p)

#         future = self._arm_client.send_goal_async(goal)
#         while not future.done():
#             time.sleep(0.01)
#         result_future = future.result().get_result_async()
#         while not result_future.done():
#             time.sleep(0.01)

#     # ── Smooth Ball Translation Engine ────────────────────────────────────────
#     def _move_ball(self, name, x_start, x_end):
#         x = x_start
#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)
#             self._set_pose(name, x, SPAWN_Y, TRANSPORT_Z)
#             time.sleep(STEP_DELAY)

#     def _fall(self, name, direction):
#         side  = 'LEFT' if direction < 0 else 'RIGHT'
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         x    = SPAWN_X
#         z    = TRANSPORT_Z
#         step = STEP_SIZE * direction
#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step
#             if (direction > 0 and x > BELT_END_R) or \
#                (direction < 0 and x < BELT_END_L):
#                 z = max(z - 0.006, -0.05)
#             self._set_pose(name, x, SPAWN_Y, z)
#             time.sleep(STEP_DELAY)

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign', 'service', '-s', '/world/empty/set_pose',
#             '--reqtype', 'ignition.msgs.Pose',
#             '--reptype', 'ignition.msgs.Boolean',
#             '--timeout', '1000',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]
#         # Popen runs instantly in the background without blocking execution loops
#         subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)

#     def _spawn(self, color, retries=3):
#         self.ball_count += 1
#         name    = f'{color}_ball_{self.ball_count}'
#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))
#         sdf     = BALL_SDF.format(name=name, r=r, g=g, b=b)
#         cmd = [
#             'ros2', 'run', 'ros_gz_sim', 'create',
#             '-name', name,
#             '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
#             '-string', sdf
#         ]
#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
#                 if result.returncode == 0:
#                     return name
#             except subprocess.TimeoutExpired:
#                 time.sleep(1.0)
#         return name


# def main(args=None):
#     rclpy.init(args=args)
#     node = SortingNode()
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)
#     executor.spin()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




#!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64MultiArray
# import subprocess
# import threading
# import time

# # ── Geometry ──────────────────────────────────────────────────────────────────
# SPAWN_X    = -0.14824
# SPAWN_Y    =  0.29075
# SPAWN_Z    =  0.09232

# # Safe height target to catch the upper hemisphere of the ball without belt rubbing
# PICKUP_Z   =  SPAWN_Z + 0.022 

# # ── Link lengths (m) ──────────────────────────────────────────────────────────
# L_0   = 0.094
# L_1   = 0.041
# L_2   = 0.09005
# L_3   = 0.226211
# L_4   = 0.196208
# L_ARM = L_2 + L_3
# SHOULDER_Z = 0.2666

# PICKUP_X    = -0.020
# TRANSPORT_Z = SPAWN_Z + 0.005
# BELT_END_R  =  0.17
# BELT_END_L  = -0.17
# STEP_SIZE   =  0.005  
# STEP_DELAY  =  0.035  # Smooth pacing for continuous background simulation frames

# # ── Joint limits ──────────────────────────────────────────────────────────────
# THETA1_MIN = -math.pi / 2
# THETA1_MAX =  math.pi / 2
# THETA2_MIN = -0.55       
# THETA2_MAX =  math.pi / 3  # ~1.047 rad

# # Safe high-clearance home stance definition
# ARM_HOME = [0.0, 0.8]      # arm_2 is pitched high up, completely clearing the belt

# # ── Gripper constants ─────────────────────────────────────────────────────────
# GRIPPER_PREOPEN =  0.3    
# GRIPPER_CLOSE   = -0.25   
# GRIPPER_RELEASE =  0.6   
# GRIPPER_NEUTRAL =  0.2   

# BALL_CYCLE = ['red', 'blue', 'black', 'green']
# BALL_RGB   = {
#     'red':   (1,    0,    0   ),
#     'blue':  (0,    0,    1   ),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0,    0.8,  0   ),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
#       </inertial>
#       <collision name='col'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <surface>
#           <friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction>
#           <contact>
#             <ode>
#               <kp>5e3</kp><kd>50.0</kd>
#               <min_depth>0.005</min_depth>
#               <max_vel>0.1</max_vel>
#             </ode>
#           </contact>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# # ═════════════════════════════════════════════════════════════════════════════
# # INVERSE KINEMATICS
# # ═════════════════════════════════════════════════════════════════════════════
# def solve_ik_sim(x, y, z_target):
#     theta1 = math.atan2(x, y)
#     Z0 = 0.220995          
#     R_ARM = 0.226963  

#     sin_t2 = (z_target - Z0) / R_ARM
#     sin_t2 = max(-1.0, min(1.0, sin_t2))
#     theta2 = math.asin(sin_t2)

#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

#     return theta1, theta2


# # ═════════════════════════════════════════════════════════════════════════════
# # SORTING NODE
# # ═════════════════════════════════════════════════════════════════════════════
# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting1_node')
#         self._arm_client = ActionClient(
#             self, FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory')
#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray, '/gripper_controller/commands', 10)
#         self.ball_count  = 0
#         self.cycle_index = 0
#         threading.Thread(target=self._start, daemon=True).start()

#     def _start(self):
#         time.sleep(2.0)
#         self.get_logger().info('Waiting for arm controller...')
#         self._arm_client.wait_for_server()

#         self.get_logger().info('Parking arm into elevated stance...')
#         self._send_trajectory([ARM_HOME], [2.0])
#         time.sleep(2.5)

#         self.get_logger().info('Ready — starting sort loop')
#         self._main_loop()

#     def _main_loop(self):
#         self._send_trajectory([ARM_HOME], [2.0])
#         self._grip(GRIPPER_NEUTRAL)

#         while self.ball_count < 4:
#             color = BALL_CYCLE[self.cycle_index % 4]
#             self.cycle_index += 1
#             self.get_logger().info(f'=== Ball {self.cycle_index}: {color.upper()} ===')

#             name = self._spawn(color)
#             time.sleep(0.1)

#             if color == 'red':
#                 # Ball rolls cleanly; arm is up and safe
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_sequence(left_side=True)
#             elif color == 'blue':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_sequence(left_side=False)
#             elif color == 'black':
#                 self._fall(name, direction=-1)
#                 time.sleep(1.5)
#             elif color == 'green':
#                 self._fall(name, direction=1)
#                 time.sleep(1.5)

#             time.sleep(0.3)

#         self._send_trajectory([ARM_HOME], [2.0])
#         self._grip(GRIPPER_NEUTRAL)
#         self.get_logger().info('=== All 4 balls processed — sorting complete ✓ ===')

#     # ── High-Clearance Pick & Place Stance Profile ────────────────────────────
#     def _pick_place_sequence(self, left_side=True):
#         side_str = 'LEFT' if left_side else 'RIGHT'
#         target_theta1 = -1.5708 if left_side else 1.5708
#         self.get_logger().info(f'Executing seamless Pick → Place {side_str}')

#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)

#         # 1. Open gripper and plunge straight down from high-park position
#         self._grip(GRIPPER_PREOPEN)
#         self._send_trajectory(
#             positions=[ARM_HOME, [0.0, pick_t2]], 
#             durations=[0.5, 2.0]
#         )
#         time.sleep(2.1)

#         # 2. Grab Ball
#         self._grip(GRIPPER_CLOSE)
#         time.sleep(0.8)

#         # 3. Lift vertically up back into high stance, then rotate out cleanly
#         self._send_trajectory(
#             positions=[
#                 ARM_HOME,                     # Lift straight back up out of belt space
#                 [target_theta1, 0.8],         # Rotate while keeping safe height clearance
#                 [target_theta1, -0.52]        # Drop down gently over bin
#             ],
#             durations=[1.2, 3.0, 4.0]
#         )
#         time.sleep(4.1)

#         # 4. Release Object
#         self._grip(GRIPPER_RELEASE)
#         time.sleep(1.2)

#         # 5. Safe Return home using high corridors
#         self._send_trajectory(
#             positions=[[target_theta1, 0.8], ARM_HOME], 
#             durations=[1.0, 2.5]
#         )
#         time.sleep(1.0)
#         self._grip(GRIPPER_NEUTRAL)
#         time.sleep(1.6)

#         self.get_logger().info(f'Place {side_str} complete ✓')

#     # ── Trajectory Infrastructure ─────────────────────────────────────────────
#     def _send_trajectory(self, positions, durations):
#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint',
#         ]
        
#         for pos, t in zip(positions, durations):
#             p = JointTrajectoryPoint()
#             p.positions = [float(v) for v in pos]
#             p.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
#             goal.trajectory.points.append(p)

#         future = self._arm_client.send_goal_async(goal)
#         while not future.done():
#             time.sleep(0.01)
#         result_future = future.result().get_result_async()
#         while not result_future.done():
#             time.sleep(0.01)

#     # ── Ball Translation Engine ────────────────────────────────────────
#     def _move_ball(self, name, x_start, x_end):
#         x = x_start
#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)
#             self._set_pose(name, x, SPAWN_Y, TRANSPORT_Z)
#             time.sleep(STEP_DELAY)

#     def _fall(self, name, direction):
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         x    = SPAWN_X
#         z    = TRANSPORT_Z
#         step = STEP_SIZE * direction
#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step
#             if (direction > 0 and x > BELT_END_R) or \
#                (direction < 0 and x < BELT_END_L):
#                 z = max(z - 0.006, -0.05)
#             self._set_pose(name, x, SPAWN_Y, z)
#             time.sleep(STEP_DELAY)

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign', 'service', '-s', '/world/empty/set_pose',
#             '--reqtype', 'ignition.msgs.Pose',
#             '--reptype', 'ignition.msgs.Boolean',
#             '--timeout', '1000',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]
#         subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)

#     def _spawn(self, color, retries=3):
#         self.ball_count += 1
#         name    = f'{color}_ball_{self.ball_count}'
#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))
#         sdf     = BALL_SDF.format(name=name, r=r, g=g, b=b)
#         cmd = [
#             'ros2', 'run', 'ros_gz_sim', 'create',
#             '-name', name,
#             '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
#             '-string', sdf
#         ]
#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
#                 if result.returncode == 0:
#                     return name
#             except subprocess.TimeoutExpired:
#                 time.sleep(1.0)
#         return name


# def main(args=None):
#     rclpy.init(args=args)
#     node = SortingNode()
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)
#     executor.spin()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from std_msgs.msg import Float64MultiArray
# import subprocess
# import threading
# import time

# # ── Geometry & Target Positions ───────────────────────────────────────────────
# SPAWN_X     = -0.14824
# SPAWN_Y     =  0.29075
# SPAWN_Z     =  0.09232

# # Adjusted forward slightly to center perfectly within the gripper fingers
# PICKUP_X    = -0.025
# PICKUP_Z    =  SPAWN_Z + 0.022 
# DROP_Z      =  0.108  # Target clearance height for the collection bins

# # ── Link lengths (m) ──────────────────────────────────────────────────────────
# L_0   = 0.094
# L_1   = 0.041
# L_2   = 0.09005
# L_3   = 0.226211
# L_4   = 0.196208
# L_ARM = L_2 + L_3
# SHOULDER_Z = 0.2666

# TRANSPORT_Z = SPAWN_Z + 0.005
# BELT_END_R  =  0.17
# BELT_END_L  = -0.17
# STEP_SIZE   =  0.006  
# STEP_DELAY  =  0.030  # Optimized pacing for stable synchronous execution

# # ── Joint limits ──────────────────────────────────────────────────────────────
# THETA1_MIN = -math.pi / 2
# THETA1_MAX =  math.pi / 2
# THETA2_MIN = -0.55       
# THETA2_MAX =  math.pi / 3  

# # Set to 0.3 rad to provide clean structural clearance without over-extending
# ARM_HOME = [0.0, 0.3]      

# # ── Gripper constants ─────────────────────────────────────────────────────────
# GRIPPER_PREOPEN =  0.3    
# GRIPPER_CLOSE   = -0.25   
# GRIPPER_RELEASE =  0.6   
# GRIPPER_NEUTRAL =  0.2   

# BALL_CYCLE = ['red', 'blue', 'black', 'green']
# BALL_RGB   = {
#     'red':   (1,    0,    0   ),
#     'blue':  (0,    0,    1   ),
#     'black': (0.05, 0.05, 0.05),
#     'green': (0,    0.8,  0   ),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
#       </inertial>
#       <collision name='col'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <surface>
#           <friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction>
#           <contact>
#             <ode>
#               <kp>5e3</kp><kd>50.0</kd>
#               <min_depth>0.005</min_depth>
#               <max_vel>0.1</max_vel>
#             </ode>
#           </contact>
#         </surface>
#       </collision>
#       <visual name='vis'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# # ═════════════════════════════════════════════════════════════════════════════
# # INVERSE KINEMATICS
# # ═════════════════════════════════════════════════════════════════════════════
# def solve_ik_sim(x, y, z_target):
#     theta1 = math.atan2(x, y)
#     Z0 = 0.220995          
#     R_ARM = 0.226963  

#     sin_t2 = (z_target - Z0) / R_ARM
#     sin_t2 = max(-1.0, min(1.0, sin_t2))
#     theta2 = math.asin(sin_t2)

#     theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
#     theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

#     return theta1, theta2


# # ═════════════════════════════════════════════════════════════════════════════
# # SORTING NODE
# # ═════════════════════════════════════════════════════════════════════════════
# class SortingNode(Node):
#     def __init__(self):
#         super().__init__('sorting1_node')
#         self._arm_client = ActionClient(
#             self, FollowJointTrajectory,
#             '/arm_controller/follow_joint_trajectory')
#         self._gripper_pub = self.create_publisher(
#             Float64MultiArray, '/gripper_controller/commands', 10)
#         self.ball_count  = 0
#         self.cycle_index = 0
#         threading.Thread(target=self._start, daemon=True).start()

#     def _start(self):
#         time.sleep(2.0)
#         self.get_logger().info('Waiting for arm controller...')
#         self._arm_client.wait_for_server()

#         self.get_logger().info('Parking arm into safety stance...')
#         self._send_trajectory([ARM_HOME], [2.0])
#         time.sleep(2.5)

#         self.get_logger().info('Ready — starting sort loop')
#         self._main_loop()

#     def _main_loop(self):
#         self._send_trajectory([ARM_HOME], [2.0])
#         self._grip(GRIPPER_NEUTRAL)

#         while self.ball_count < 4:
#             color = BALL_CYCLE[self.cycle_index % 4]
#             self.cycle_index += 1
#             self.get_logger().info(f'=== Ball {self.cycle_index}: {color.upper()} ===')

#             name = self._spawn(color)
#             time.sleep(0.1)

#             if color == 'red':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_sequence(left_side=True)
#             elif color == 'blue':
#                 self._move_ball(name, SPAWN_X, PICKUP_X)
#                 self._pick_place_sequence(left_side=False)
#             elif color == 'black':
#                 self._fall(name, direction=-1)
#                 time.sleep(1.5)
#             elif color == 'green':
#                 self._fall(name, direction=1)
#                 time.sleep(1.5)

#             time.sleep(0.3)

#         self._send_trajectory([ARM_HOME], [2.0])
#         self._grip(GRIPPER_NEUTRAL)
#         self.get_logger().info('=== All 4 balls processed — sorting complete ✓ ===')

#     # ── Pure IK Driven Pick & Place Sequence ──────────────────────────────────
#     def _pick_place_sequence(self, left_side=True):
#         side_str = 'LEFT' if left_side else 'RIGHT'
#         target_theta1 = -1.5708 if left_side else 1.5708
#         self.get_logger().info(f'Executing Pure IK Pick → Place {side_str}')

#         # Compute positions strictly using the Inverse Kinematics solver
#         _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)
#         _, drop_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, DROP_Z)

#         # 1. Clear high stance, preopen claws, and plunge downward onto target
#         self._grip(GRIPPER_PREOPEN)
#         self._send_trajectory(
#             positions=[ARM_HOME, [0.0, pick_t2]], 
#             durations=[0.6, 2.0]
#         )
#         time.sleep(2.1)

#         # 2. Close around object center
#         self._grip(GRIPPER_CLOSE)
#         time.sleep(0.8)

#         # 3. Clean continuous delivery profile using computed IK solutions
#         self._send_trajectory(
#             positions=[
#                 ARM_HOME,                         # Elevate vertically clear of belt
#                 [target_theta1, ARM_HOME[1]],     # Smooth yaw rotation at safe height
#                 [target_theta1, drop_t2]          # Precise IK drop depth over bin
#             ],
#             durations=[1.2, 2.8, 3.8]
#         )
#         time.sleep(4.0)

#         # 4. Drop object
#         self._grip(GRIPPER_RELEASE)
#         time.sleep(1.2)

#         # 5. Continuous return home path via safe altitude corridors
#         self._send_trajectory(
#             positions=[[target_theta1, ARM_HOME[1]], ARM_HOME], 
#             durations=[1.0, 2.5]
#         )
#         time.sleep(1.0)
#         self._grip(GRIPPER_NEUTRAL)
#         time.sleep(1.6)

#         self.get_logger().info(f'Place {side_str} complete ✓')

#     # ── Trajectory Message Builder ────────────────────────────────────────────
#     def _send_trajectory(self, positions, durations):
#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [
#             'arm_1_base_link_joint',
#             'arm_2_left_arm_linkage_joint',
#         ]
        
#         for pos, t in zip(positions, durations):
#             p = JointTrajectoryPoint()
#             p.positions = [float(v) for v in pos]
#             p.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
#             goal.trajectory.points.append(p)

#         future = self._arm_client.send_goal_async(goal)
#         while not future.done():
#             time.sleep(0.01)
#         result_future = future.result().get_result_async()
#         while not result_future.done():
#             time.sleep(0.01)

#     # ── Sequential Constant Linear Velocity Engine ────────────────────────────
#     def _move_ball(self, name, x_start, x_end):
#         x = x_start
#         while x < x_end:
#             x = min(x + STEP_SIZE, x_end)
#             self._set_pose(name, x, SPAWN_Y, TRANSPORT_Z)
#             time.sleep(STEP_DELAY)

#     def _fall(self, name, direction):
#         x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
#         x    = SPAWN_X
#         z    = TRANSPORT_Z
#         step = STEP_SIZE * direction
#         while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
#             x += step
#             if (direction > 0 and x > BELT_END_R) or \
#                (direction < 0 and x < BELT_END_L):
#                 z = max(z - 0.006, -0.05)
#             self._set_pose(name, x, SPAWN_Y, z)
#             time.sleep(STEP_DELAY)

#     def _set_pose(self, name, x, y, z):
#         cmd = [
#             'ign', 'service', '-s', '/world/empty/set_pose',
#             '--reqtype', 'ignition.msgs.Pose',
#             '--reptype', 'ignition.msgs.Boolean',
#             '--req',
#             f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
#         ]
#         # Synchronous execution with suppressed output guarantees clean sequential pacing
#         subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

#     def _grip(self, position):
#         msg = Float64MultiArray()
#         msg.data = [float(position)]
#         self._gripper_pub.publish(msg)

#     def _spawn(self, color, retries=3):
#         self.ball_count += 1
#         name    = f'{color}_ball_{self.ball_count}'
#         r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))
#         sdf     = BALL_SDF.format(name=name, r=r, g=g, b=b)
#         cmd = [
#             'ros2', 'run', 'ros_gz_sim', 'create',
#             '-name', name,
#             '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
#             '-string', sdf
#         ]
#         for attempt in range(retries):
#             try:
#                 result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
#                 if result.returncode == 0:
#                     return name
#             except subprocess.TimeoutExpired:
#                 time.sleep(1.0)
#         return name


# def main(args=None):
#     rclpy.init(args=args)
#     node = SortingNode()
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)
#     executor.spin()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64, Float64MultiArray
from conveyorbelt_msgs.srv import ConveyorBeltControl
import subprocess
import threading
import time

# ── Geometry & Spatial Anchors ────────────────────────────────────────────────
SPAWN_X    = -0.14824
SPAWN_Y    =  0.29075
SPAWN_Z    =  0.09232

# Offset target upward so the gripper doesn't plunge into the conveyor belt
PICKUP_Z   =  SPAWN_Z + 0.022 

# ── Link lengths (m) ──────────────────────────────────────────────────────────
L_0   = 0.094
L_1   = 0.041
L_2   = 0.09005
L_3   = 0.226211
L_4   = 0.196208
L_ARM = L_2 + L_3
SHOULDER_Z = 0.2666

PICKUP_X    = -0.020
TRANSPORT_Z = SPAWN_Z + 0.005  # Slight vertical padding to avoid friction drag
BELT_END_R  =  0.17
BELT_END_L  = -0.17
STEP_SIZE   =  0.006
STEP_DELAY  =  0.04

# ── Joint limits ──────────────────────────────────────────────────────────────
THETA1_MIN = -math.pi / 2
THETA1_MAX =  math.pi / 2
THETA2_MIN = -0.55             # Clamped to guarantee conveyor safety
THETA2_MAX =  math.pi / 3

# ── Gripper constants ─────────────────────────────────────────────────────────
GRIPPER_PREOPEN =  0.3
GRIPPER_CLOSE   = -0.25
GRIPPER_RELEASE =  0.6
GRIPPER_NEUTRAL =  0.2

BALL_CYCLE = ['red', 'blue', 'black', 'green']
BALL_RGB   = {
    'red':   (1,    0,    0   ),
    'blue':  (0,    0,    1   ),
    'black': (0.05, 0.05, 0.05),
    'green': (0,    0.8,  0   ),
}

BALL_SDF = """<sdf version='1.7'>
  <model name='{name}'>
    <link name='link'>
      <inertial>
        <mass>0.05</mass>
        <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
      </inertial>
      <collision name='col'>
        <geometry><sphere><radius>0.0185</radius></sphere></geometry>
        <surface>
          <friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction>
          <contact>
            <ode>
              <kp>5e3</kp><kd>50.0</kd>
              <min_depth>0.005</min_depth>
              <max_vel>0.1</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name='vis'>
        <geometry><sphere><radius>0.0185</radius></sphere></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


# ═════════════════════════════════════════════════════════════════════════════
# INVERSE KINEMATICS
# ═════════════════════════════════════════════════════════════════════════════
def solve_ik_sim(x, y, z_target):
    theta1 = math.atan2(x, y)
    
    # Using your specific arm mounting parameters
    Z0 = 0.220995          
    R_ARM = 0.226963  

    sin_t2 = (z_target - Z0) / R_ARM
    sin_t2 = max(-1.0, min(1.0, sin_t2))  # Numerical bounds safety check
    theta2 = math.asin(sin_t2)

    # Apply physical link protective boundaries
    theta1 = max(THETA1_MIN, min(THETA1_MAX, theta1))
    theta2 = max(THETA2_MIN, min(THETA2_MAX, theta2))

    return theta1, theta2


# ═════════════════════════════════════════════════════════════════════════════
# SORTING NODE
# ═════════════════════════════════════════════════════════════════════════════
class SortingNode(Node):
    def __init__(self):
        super().__init__('sorting1_node')
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)
        self._conveyor_cli = self.create_client(
            ConveyorBeltControl, '/CONVEYORPOWER')
        self.ball_count  = 0
        self.cycle_index = 0
        threading.Thread(target=self._start, daemon=True).start()

    def _start(self):
        time.sleep(2.0)
        self.get_logger().info('Waiting for arm controller...')
        self._arm_client.wait_for_server()
        
        self.get_logger().info('Initializing home position...')
        self._send_trajectory([[0.0, 0.0]], [2.0])
        time.sleep(2.5)
        
        self.get_logger().info('Ready — starting sort loop')
        self._main_loop()

    def _main_loop(self):
        self._send_trajectory([[0.0, 0.0]], [2.0])
        self._grip(GRIPPER_NEUTRAL)
        self.start_conveyor(60.0)   # 0-100% via CONVEYORPOWER service
        time.sleep(1.0)

        while self.ball_count < 4:
            color = BALL_CYCLE[self.cycle_index % 4]
            self.cycle_index += 1
            self.get_logger().info(f'=== Ball {self.cycle_index}: {color.upper()} ===')

            name = self._spawn(color)
            time.sleep(0.1)

            if color == 'red':
                self._move_ball(name, SPAWN_X, PICKUP_X)
                self._pick_place_sequence(left_side=True)
            elif color == 'blue':
                self._move_ball(name, SPAWN_X, PICKUP_X)
                self._pick_place_sequence(left_side=False)
            elif color == 'black':
                self._fall(name, direction=-1)
                time.sleep(1.5)
            elif color == 'green':
                self._fall(name, direction=1)
                time.sleep(1.5)

            time.sleep(0.3)

        self.stop_conveyor()
        self._send_trajectory([[0.0, 0.0]], [2.0])
        self._grip(GRIPPER_NEUTRAL)
        self.get_logger().info('=== All 4 balls processed — sorting complete ✓ ===')

    def start_conveyor(self, power=60.0):
        """Turn the belt on via the IFRA conveyor plugin's CONVEYORPOWER service (0-100%)."""
        if not self._conveyor_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('CONVEYORPOWER service unavailable — belt not started')
            return
        req = ConveyorBeltControl.Request()
        req.power = float(power)
        future = self._conveyor_cli.call_async(req)
        t0 = time.time()
        while not future.done() and time.time() - t0 < 5.0:
            time.sleep(0.05)
        self.get_logger().info(f'Conveyor power set to {power:.0f}%')

    def stop_conveyor(self):
        self.start_conveyor(0.0)

    # ── Universal Multi-Waypoint Motion Pipeline ──────────────────────────────
    def _pick_place_sequence(self, left_side=True):
        side_str = 'LEFT' if left_side else 'RIGHT'
        target_theta1 = -1.5708 if left_side else 1.5708
        self.get_logger().info(f'Executing fluid Pick → Place {side_str}')

        # Compute safe pickup kinematics targets
        _, pick_t2 = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)
        self.get_logger().info(f'IK calculated safe angle: theta2={pick_t2:.4f} rad')

        # 1. Approach & Descend smoothly
        self._grip(GRIPPER_PREOPEN)
        self._send_trajectory(
            positions=[[0.0, 0.0], [0.0, pick_t2]], 
            durations=[1.0, 2.2]
        )
        time.sleep(2.3)

        # 2. Secure Object
        self._grip(GRIPPER_CLOSE)
        time.sleep(0.8)

        # 3. Combined Flightpath (Prevents stuttering mid-air)
        self._send_trajectory(
            positions=[
                [0.0, 0.0],                # Lift up vertically
                [target_theta1, 0.0],      # Swing towards drop zone
                [target_theta1, -0.52]     # Final drop positioning
            ],
            durations=[1.0, 2.8, 3.8]      # Progressive timeline increments
        )
        time.sleep(4.0)

        # 4. Release & Reset Cleanly
        self._grip(GRIPPER_RELEASE)
        time.sleep(1.2)

        self._send_trajectory(
            positions=[[target_theta1, 0.0], [0.0, 0.0]], 
            durations=[1.0, 2.5]
        )
        time.sleep(1.0)
        self._grip(GRIPPER_NEUTRAL)
        time.sleep(1.6)

        self.get_logger().info(f'Place {side_str} complete ✓')

    # ── Advanced Action Trajectory Execution Client ───────────────────────────
    def _send_trajectory(self, positions, durations):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'arm_1_base_link_joint',
            'arm_2_left_arm_linkage_joint',
        ]
        
        for pos, t in zip(positions, durations):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in pos]
            p.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            goal.trajectory.points.append(p)

        future = self._arm_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.01)
        result_future = future.result().get_result_async()
        while not result_future.done():
            time.sleep(0.01)

    # ── Non-Blocking Ball Transport Logic ─────────────────────────────────────
    def _move_ball(self, name, x_start, x_end):
        x = x_start
        while x < x_end:
            x = min(x + STEP_SIZE, x_end)
            self._set_pose(name, x, SPAWN_Y, TRANSPORT_Z)
            time.sleep(STEP_DELAY)

    def _fall(self, name, direction):
        x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
        x    = SPAWN_X
        z    = TRANSPORT_Z
        step = STEP_SIZE * direction
        while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
            x += step
            if (direction > 0 and x > BELT_END_R) or \
               (direction < 0 and x < BELT_END_L):
                z = max(z - 0.006, -0.05)
            self._set_pose(name, x, SPAWN_Y, z)
            time.sleep(STEP_DELAY)

    def _set_pose(self, name, x, y, z):
        cmd = [
            'ign', 'service', '-s', '/world/empty/set_pose',
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000',
            '--req',
            f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
        ]
        # Popen fires the command asynchronously to eliminate process wait locks
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _grip(self, position):
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self._gripper_pub.publish(msg)

    def _spawn(self, color, retries=3):
        self.ball_count += 1
        name    = f'{color}_ball_{self.ball_count}'
        r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))
        sdf     = BALL_SDF.format(name=name, r=r, g=g, b=b)
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', name,
            '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
            '-string', sdf
        ]
        for attempt in range(retries):
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                if result.returncode == 0:
                    return name
            except subprocess.TimeoutExpired:
                time.sleep(1.0)
        return name


def main(args=None):
    rclpy.init(args=args)
    node = SortingNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()