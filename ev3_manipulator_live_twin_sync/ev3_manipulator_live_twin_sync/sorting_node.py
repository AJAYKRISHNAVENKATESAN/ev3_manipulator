#!/usr/bin/env python3

# """Mirror EV3 state into Gazebo and synchronize the IFRA conveyor plugin.

# ========================================================================
# CURRENT MODEL: MIRROR (Time-Driven, One-Way Communication)
# ========================================================================
# This is a "replay" model where:
# - EV3 sends events with fixed durations (ACTION_START/DONE)
# - Simulation echoes back joint positions
# - No sensor feedback from sim to hardware
# - Motion is time-based, not position-based
# - Conveyor power applied for event duration, not until ball reaches sensor

# KNOWN LIMITATION:
# Simulation ball stops at different X positions on each run because motion 
# is not feedback-driven. Real hardware uses color sensor at pickup_x to stop 
# conveyor reliably. This model does not replicate that behavior.
# ========================================================================

# PLANNED: DIGITAL TWIN MODEL (Sensor-Feedback, Duplex Communication)
# ========================================================================
# A true duplex model where:
# - EV3 sends COMMANDS (power level, not duration)
# - Simulation simulates physics and sensors
# - Simulation sends back SENSOR MEASUREMENTS (ball position, sensor triggers)
# - Motion is position-based (stops when sensor fires, not by timer)
# - Both hardware and sim stay synchronized on shared reality

# Implementation steps:
# 1. Add virtual conveyor color sensor in Gazebo at pickup_x position
# 2. Modify GazeboStateMirror to read ball X position from Gazebo physics
# 3. Publish sensor trigger events back when ball crosses pickup_x threshold
# 4. Update EV3 firmware to consume sensor events and stop conveyor
# ========================================================================
# """

# import subprocess
# import time
# from typing import Dict, Optional

# import rclpy
# from conveyorbelt_msgs.srv import ConveyorBeltControl
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray, String


# BALL_RGB = {
#     "red": (1.0, 0.0, 0.0),
#     "blue": (0.0, 0.0, 1.0),
#     "black": (0.05, 0.05, 0.05),
#     "green": (0.0, 0.8, 0.0),
# }

# # Distinct physics profiles tuned for immediate belt grip and reduced solver sticking.
# # Previous values kept for reference:
# #   red/blue: friction 0.18, kp 10000.0, kd 50.0, min_depth 0.0005
# #   black/green: friction 0.15/0.12, kp 50000.0, kd 50.0, min_depth 0.0010
# BALL_PHYSICS_CONFIG = {
#     "red": {"friction": 0.25, "kp": 100000.0, "kd": 100.0, "min_depth": 0.001},
#     "blue": {"friction": 0.25, "kp": 100000.0, "kd": 100.0, "min_depth": 0.001},
#     "black": {"friction": 0.2, "kp": 50000.0, "kd": 10.0, "min_depth": 0.001},
#     "green": {"friction": 0.2, "kp": 50000.0, "kd": 10.0, "min_depth": 0.001},
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia>
#           <ixx>3.92e-06</ixx>
#           <iyy>3.92e-06</iyy>
#           <izz>3.92e-06</izz>
#           <ixy>0.0</ixy>
#           <ixz>0.0</ixz>
#           <iyz>0.0</iyz>
#         </inertia>
#       </inertial>

#       <collision name='collision'>
#         <geometry>
#           <sphere><radius>{collision_radius}</radius></sphere>
#         </geometry>
#         <surface>
#           <friction>
#             <ode>
#               <mu>{friction}</mu>
#               <mu2>{friction}</mu2>
#             </ode>
#           </friction>
#           <bounce>
#             <restitution_coefficient>0.0</restitution_coefficient>
#             <threshold>100000.0</threshold>
#           </bounce>
#           <contact>
#             <ode>
#               <!-- Gripper sticking handled here instead of by reducing friction -->
#               <kp>{kp}</kp>
#               <kd>{kd}</kd>
#               <max_vel>0.01</max_vel>
#               <min_depth>{min_depth}</min_depth>
#             </ode>
#           </contact>
#         </surface>
#       </collision>

#       <visual name='visual'>
#         <geometry>
#           <sphere><radius>{visual_radius}</radius></sphere>
#         </geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# class GazeboStateMirror(Node):
#     """Mirror EV3 joints and synchronize the simulated conveyor."""

#     def __init__(self) -> None:
#         super().__init__("gazebo_state_mirror")

#         # Joint mirroring parameters
#         self.declare_parameter(
#             "position_joints",
#             ["arm1_base_link_joint", "arm1_arm2_joint", "left_gear_arm4_joint"],
#         )
#         self.declare_parameter("position_command_topic", "/twin_position_controller/commands")
#         self.declare_parameter("command_rate_hz", 50.0)
#         self.declare_parameter("state_timeout_sec", 0.5)
#         self.declare_parameter("low_pass_alpha", 1.0)

#         # Pose & Conveyor parameters
#         self.declare_parameter("ball_radius", 0.014)
#         self.declare_parameter("spawn_x", -0.154099)
#         self.declare_parameter("spawn_y", 0.233)
#         self.declare_parameter("spawn_z", 0.0610)

#         # Calibrated ball centre under the gripper TCP.
#         self.declare_parameter("pickup_x", -0.020859)

#         self.declare_parameter("conveyor_power_service", "/CONVEYORPOWER")
#         self.declare_parameter("conveyor_pickup_power", 19.7)
#         # Previous ejection values kept for reference:
#         # self.declare_parameter("conveyor_black_power", 25.0)
#         # self.declare_parameter("conveyor_green_power", -35.0)
#         self.declare_parameter("conveyor_black_power", 95.0)
#         self.declare_parameter("conveyor_green_power", -100.0)

#         # Extra simulated belt time after EV3 reject motion finishes.
#         self.declare_parameter("conveyor_green_tail_sec", 2.5)
#         self.declare_parameter("conveyor_black_tail_sec", 0.5)

#         self.position_joints = [str(n) for n in self.get_parameter("position_joints").value]
#         self.timeout_sec = float(self.get_parameter("state_timeout_sec").value)
#         self.alpha = max(0.0, min(1.0, float(self.get_parameter("low_pass_alpha").value)))

#         self.ball_radius = float(self.get_parameter("ball_radius").value)
#         self.spawn_x = float(self.get_parameter("spawn_x").value)
#         self.spawn_y = float(self.get_parameter("spawn_y").value)
#         self.spawn_z = float(self.get_parameter("spawn_z").value)
#         self.pickup_x = float(self.get_parameter("pickup_x").value)

#         self.conveyor_power_service = str(self.get_parameter("conveyor_power_service").value)
#         self.conveyor_pickup_power = float(self.get_parameter("conveyor_pickup_power").value)
#         self.conveyor_black_power = float(self.get_parameter("conveyor_black_power").value)
#         self.conveyor_green_power = float(self.get_parameter("conveyor_green_power").value)
#         self.conveyor_green_tail_sec = float(self.get_parameter("conveyor_green_tail_sec").value)
#         self.conveyor_black_tail_sec = float(self.get_parameter("conveyor_black_tail_sec").value)

#         position_topic = str(self.get_parameter("position_command_topic").value)
#         self.position_pub = self.create_publisher(Float64MultiArray, position_topic, 20)

#         self.create_subscription(JointState, "/digital_twin/joint_states", self.state_callback, 20)
#         self.create_subscription(String, "/digital_twin/events", self.event_callback, 20)

#         self.conveyor_client = self.create_client(ConveyorBeltControl, self.conveyor_power_service)

#         self.latest_position: Optional[list[float]] = None
#         self.filtered_position: Optional[list[float]] = None
#         self.last_state_wall_time: Optional[float] = None

#         self.spawned_cycles: set[int] = set()
#         self.current_ball_name: Optional[str] = None
#         self.current_ball_color: Optional[str] = None
#         self.current_cycle_id: Optional[int] = None

#         # Used only for GREEN / BLACK reject actions.
#         # ACTION_DONE schedules a delayed stop instead of stopping immediately.
#         self.conveyor_stop_deadline: Optional[float] = None

#         rate_hz = max(float(self.get_parameter("command_rate_hz").value), 1.0)
#         self.create_timer(1.0 / rate_hz, self.publish_commands)

#     def state_callback(self, msg: JointState) -> None:
#         position_by_name: Dict[str, float] = dict(zip(msg.name, msg.position))
#         missing = [n for n in self.position_joints if n not in position_by_name]
#         if missing:
#             return
#         self.latest_position = [float(position_by_name[n]) for n in self.position_joints]
#         self.last_state_wall_time = time.monotonic()

#     def publish_commands(self) -> None:
#         # Non-blocking delayed stop for GREEN / BLACK reject actions.
#         if (
#             self.conveyor_stop_deadline is not None
#             and time.monotonic() >= self.conveyor_stop_deadline
#         ):
#             self.set_conveyor_power(0.0)
#             self.conveyor_stop_deadline = None
#             self.get_logger().info(
#                 "Reject conveyor stopped after configured tail time."
#             )

#         if self.latest_position is None or self.last_state_wall_time is None:
#             return
#         if time.monotonic() - self.last_state_wall_time > self.timeout_sec:
#             return

#         if self.filtered_position is None:
#             self.filtered_position = list(self.latest_position)
#         else:
#             self.filtered_position = [
#                 self.alpha * n + (1.0 - self.alpha) * o
#                 for n, o in zip(self.latest_position, self.filtered_position)
#             ]

#         command = Float64MultiArray()
#         command.data = list(self.filtered_position)
#         self.position_pub.publish(command)

#     def event_callback(self, msg: String) -> None:
#         parts = msg.data.split("|")
#         if len(parts) != 4 or parts[0] != "EVENT":
#             return

#         try:
#             cycle_id = int(parts[1])
#         except ValueError:
#             return

#         event_name = parts[2].strip().upper()
#         value = parts[3].strip()

#         if event_name == "BALL_DETECTED":
#             color = value.lower()
#             if cycle_id in self.spawned_cycles:
#                 return

#             name = self.spawn_ball(color, cycle_id)
#             if name is not None:
#                 self.spawned_cycles.add(cycle_id)
#                 self.current_ball_name = name
#                 self.current_ball_color = color
#                 self.current_cycle_id = cycle_id
#             return

#         if event_name == "ACTION_START":
#             action = value.split(":", 1)[0].strip().upper()

#             # Any new conveyor action supersedes an older pending stop.
#             if action in (
#                 "CONVEYOR_TO_PICKUP",
#                 "CONVEYOR_BLACK",
#                 "CONVEYOR_GREEN",
#             ):
#                 self.conveyor_stop_deadline = None

#             if action == "CONVEYOR_TO_PICKUP":
#                 self.set_conveyor_power(self.conveyor_pickup_power)
#                 return
#             if action == "CONVEYOR_BLACK":
#                 self.set_conveyor_power(self.conveyor_black_power)
#                 return
#             if action == "CONVEYOR_GREEN":
#                 self.set_conveyor_power(self.conveyor_green_power)
#                 return

#         if event_name == "ACTION_DONE":
#             fields = value.split(":", 1)
#             action = fields[0].strip().upper()

#             # Log the physical EV3 action duration when present.
#             if len(fields) == 2:
#                 try:
#                     duration_ms = int(fields[1])
#                     self.get_logger().info(
#                         "{} EV3 duration: {} ms".format(
#                             action,
#                             duration_ms,
#                         )
#                     )
#                 except ValueError:
#                     pass

#             if action == "CONVEYOR_TO_PICKUP":
#                 # Pickup alignment should stop when the physical conveyor
#                 # action finishes.
#                 self.conveyor_stop_deadline = None
#                 self.set_conveyor_power(0.0)

#             elif action == "CONVEYOR_GREEN":
#                 # Keep the reverse conveyor moving briefly so the simulated
#                 # green ball fully clears the conveyor edge.
#                 self.conveyor_stop_deadline = (
#                     time.monotonic() + self.conveyor_green_tail_sec
#                 )
#                 self.get_logger().info(
#                     "GREEN reject tail scheduled for {:.3f} s.".format(
#                         self.conveyor_green_tail_sec
#                     )
#                 )

#             elif action == "CONVEYOR_BLACK":
#                 # Keep the forward conveyor moving briefly so the simulated
#                 # black ball fully clears the conveyor edge.
#                 self.conveyor_stop_deadline = (
#                     time.monotonic() + self.conveyor_black_tail_sec
#                 )
#                 self.get_logger().info(
#                     "BLACK reject tail scheduled for {:.3f} s.".format(
#                         self.conveyor_black_tail_sec
#                     )
#                 )

#             return

#         if event_name in ("FAULT", "ACTION_FAILED"):
#             # Faults always stop immediately.
#             self.conveyor_stop_deadline = None
#             self.set_conveyor_power(0.0)
#             return

#         if event_name == "CYCLE_COMPLETE":
#             # Defensive final stop and cycle cleanup.
#             self.conveyor_stop_deadline = None
#             self.set_conveyor_power(0.0)
#             self.current_ball_name = None
#             self.current_ball_color = None
#             self.current_cycle_id = None
#             return

#     def set_conveyor_power(self, power: float) -> None:
#         requested_power = max(-100.0, min(100.0, float(power)))
#         if not self.conveyor_client.service_is_ready():
#             return
#         request = ConveyorBeltControl.Request()
#         request.power = requested_power
#         self.conveyor_client.call_async(request)

#     def spawn_ball(self, color: str, cycle_id: int) -> Optional[str]:
#         if color not in BALL_RGB:
#             return None

#         r, g, b = BALL_RGB[color]
#         cfg = BALL_PHYSICS_CONFIG.get(color, {"friction": 0.20, "kp": 20000.0, "kd": 50.0, "min_depth": 0.0005})
#         name = "{}_ball_{}".format(color, cycle_id)

#         sdf = BALL_SDF.format(
#             name=name,
#             visual_radius=self.ball_radius,
#             collision_radius=self.ball_radius,
#             friction=cfg["friction"],
#             kp=cfg["kp"],
#             kd=cfg["kd"],
#             min_depth=cfg["min_depth"],
#             r=r,
#             g=g,
#             b=b,
#         )

#         command = [
#             "ros2", "run", "ros_gz_sim", "create",
#             "-name", name,
#             "-x", str(self.spawn_x),
#             "-y", str(self.spawn_y),
#             "-z", str(self.spawn_z),
#             "-string", sdf,
#         ]

#         try:
#             result = subprocess.run(command, capture_output=True, text=True, timeout=10, check=False)
#             if result.returncode != 0:
#                 return None
#         except subprocess.TimeoutExpired:
#             return None

#         return name


# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = GazeboStateMirror()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if node.conveyor_client.service_is_ready():
#             req = ConveyorBeltControl.Request()
#             req.power = 0.0
#             node.conveyor_client.call_async(req)
#             rclpy.spin_once(node, timeout_sec=0.2)
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3

"""Gazebo interface for the EV3 digital twin.

Consumes canonical measured physical state, drives the simulated robot, and
publishes Gazebo's own measured joint state separately for twin comparison.
"""

import subprocess
import time
from typing import Dict, Optional

import rclpy
from conveyorbelt_msgs.srv import ConveyorBeltControl
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

# ---------------------------------------------------------------------
# Legacy implementation kept for reference.
# ---------------------------------------------------------------------
# #!/usr/bin/env python3
#
# """Mirror EV3 state into Gazebo and synchronize the IFRA conveyor plugin.
#
# Behaviour
# ---------
# - Mirrors base, arm, and gripper positions continuously.
# - Spawns a physics-enabled ball when the EV3 reports BALL_DETECTED.
# - Starts /CONVEYORPOWER when the EV3 reports
#   ACTION_START|CONVEYOR_TO_PICKUP.
# - Stops /CONVEYORPOWER when the EV3 reports
#   ACTION_DONE|CONVEYOR_TO_PICKUP:<duration_ms>.
# - Stops the conveyor defensively on faults and cycle completion.
#
# This is event-boundary synchronization. Exact spatial alignment at the pickup
# point depends on calibrating conveyor_pickup_power against the EV3 action
# duration and the simulated belt geometry.
# """
#
# import subprocess
# import time
# from typing import Dict, Optional
#
# import rclpy
# from conveyorbelt_msgs.srv import ConveyorBeltControl
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray, String
#
#
# BALL_RGB = {
#     "red": (1.0, 0.0, 0.0),
#     "blue": (0.0, 0.0, 1.0),
#     "black": (0.05, 0.05, 0.05),
#     "green": (0.0, 0.8, 0.0),
# }
#
# # Physics-enabled ball: collision is present and gravity is enabled by default.
# # ----------------------------------------------------------------------
# # Old SDF kept for reference:
# # BALL_SDF = """<sdf version='1.7'>
# #   <model name='{name}'>
# #     <link name='link'>
# #       <inertial>
# #         <mass>0.05</mass>
# #         <inertia>
# #           <ixx>3.92e-06</ixx>
# #           <iyy>3.92e-06</iyy>
# #           <izz>3.92e-06</izz>
# #           <ixy>0.0</ixy>
# #           <ixz>0.0</ixz>
# #           <iyz>0.0</iyz>
# #         </inertia>
# #       </inertial>
# #
# #       <collision name='collision'>
# #         <geometry>
# #           <sphere><radius>{radius}</radius></sphere>
# #         </geometry>
# #         <surface>
# #           <friction>
# #             <ode>
# #               <mu>{friction}</mu>
# #               <mu2>{friction}</mu2>
# #             </ode>
# #           </friction>
# #           <bounce>
# #             <restitution_coefficient>0.0</restitution_coefficient>
# #             <threshold>100000.0</threshold>
# #           </bounce>
# #           <contact>
# #             <ode>
# #               <kp>1000000.0</kp>
# #               <kd>10.0</kd>
# #               <max_vel>0.1</max_vel>
# #               <min_depth>0.0001</min_depth>
# #             </ode>
# #           </contact>
# #         </surface>
# #       </collision>
# #
# #       <visual name='visual'>
# #         <geometry>
# #           <sphere><radius>{radius}</radius></sphere>
# #         </geometry>
# #         <material>
# #           <ambient>{r} {g} {b} 1</ambient>
# #           <diffuse>{r} {g} {b} 1</diffuse>
# #         </material>
# #       </visual>
# #     </link>
# #   </model>
# # </sdf>"""
# # ----------------------------------------------------------------------
# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia>
#           <ixx>3.92e-06</ixx>
#           <iyy>3.92e-06</iyy>
#           <izz>3.92e-06</izz>
#           <ixy>0.0</ixy>
#           <ixz>0.0</ixz>
#           <iyz>0.0</iyz>
#         </inertia>
#       </inertial>
#
#       <collision name='collision'>
#         <geometry>
#           <!-- 1mm larger collision radius prevents dipping into frame seams -->
#           <sphere><radius>{collision_radius}</radius></sphere>
#         </geometry>
#         <surface>
#           <friction>
#             <ode>
#               <mu>{friction}</mu>
#               <mu2>{friction}</mu2>
#             </ode>
#           </friction>
#           <bounce>
#             <restitution_coefficient>0.0</restitution_coefficient>
#             <threshold>100000.0</threshold>
#           </bounce>
#           <contact>
#             <ode>
#               <kp>20000.0</kp>
#               <kd>50.0</kd>
#               <max_vel>0.01</max_vel>
#               <min_depth>0.0002</min_depth>
#             </ode>
#           </contact>
#         </surface>
#       </collision>
#
#       <visual name='visual'>
#         <geometry>
#           <sphere><radius>{visual_radius}</radius></sphere>
#         </geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""
# ---------------------------------------------------------------------

BALL_RGB = {
    "red": (1.0, 0.0, 0.0),
    "blue": (0.0, 0.0, 1.0),
    "black": (0.05, 0.05, 0.05),
    "green": (0.0, 0.8, 0.0),
}

# Distinct physics profiles tuned for immediate belt grip and reduced solver sticking.
# Previous values kept for reference:
#   red/blue: friction 0.18, kp 10000.0, kd 50.0, min_depth 0.0005
#   black/green: friction 0.15/0.12, kp 50000.0, kd 50.0, min_depth 0.0010
BALL_PHYSICS_CONFIG = {
    "red": {"friction": 0.25, "kp": 100000.0, "kd": 100.0, "min_depth": 0.001},
    "blue": {"friction": 0.25, "kp": 100000.0, "kd": 100.0, "min_depth": 0.001},
    "black": {"friction": 0.2, "kp": 50000.0, "kd": 10.0, "min_depth": 0.001},
    "green": {"friction": 0.2, "kp": 50000.0, "kd": 10.0, "min_depth": 0.001},
}

BALL_SDF = """<sdf version='1.7'>
  <model name='{name}'>
    <link name='link'>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>3.92e-06</ixx>
          <iyy>3.92e-06</iyy>
          <izz>3.92e-06</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>

      <collision name='collision'>
        <geometry>
          <sphere><radius>{collision_radius}</radius></sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{friction}</mu>
              <mu2>{friction}</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.0</restitution_coefficient>
            <threshold>100000.0</threshold>
          </bounce>
          <contact>
            <ode>
              <!-- Gripper sticking handled here instead of by reducing friction -->
              <kp>{kp}</kp>
              <kd>{kd}</kd>
              <max_vel>0.01</max_vel>
              <min_depth>{min_depth}</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name='visual'>
        <geometry>
          <sphere><radius>{visual_radius}</radius></sphere>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class GazeboTwinInterface(Node):
    """Mirror EV3 joints and synchronize the simulated conveyor."""

    def __init__(self) -> None:
        super().__init__("gazebo_twin_interface")

        # Canonical physical joints, in the same order expected by the
        # Gazebo position controller command array: [base, arm, gripper].
        self.declare_parameter(
            "source_joints",
            [
                 "arm1_base_link_joint",
                 "arm1_arm2_joint",
                "left_gear_arm4_joint",
            ],
        )
        self.declare_parameter("physical_joint_state_topic", "/twin/physical/joint_states")
        self.declare_parameter("physical_event_topic", "/twin/physical/events")
        self.declare_parameter("position_command_topic", "/twin_position_controller/commands")
        self.declare_parameter("gazebo_joint_state_topic", "/joint_states")
        self.declare_parameter("sim_joint_state_topic", "/twin/sim/joint_states")
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("low_pass_alpha", 1.0)

        # Pose & Conveyor parameters
        self.declare_parameter("ball_radius", 0.014)
        self.declare_parameter("spawn_x", -0.154099)
        self.declare_parameter("spawn_y", 0.233)
        self.declare_parameter("spawn_z", 0.0610)

        # Calibrated ball centre under the gripper TCP.
        self.declare_parameter("pickup_x", -0.020859)

        self.declare_parameter("conveyor_power_service", "/CONVEYORPOWER")
        self.declare_parameter("conveyor_pickup_power", 25.0)
        # Previous ejection values kept for reference:
        # self.declare_parameter("conveyor_black_power", 25.0)
        # self.declare_parameter("conveyor_green_power", -35.0)
        self.declare_parameter("conveyor_black_power", 100.0)
        self.declare_parameter("conveyor_green_power", -100.0)

        # Extra simulated belt time after EV3 reject motion finishes.
        self.declare_parameter("conveyor_green_tail_sec", 0.5)
        self.declare_parameter("conveyor_black_tail_sec", 0.5)

        self.source_joints = [str(n) for n in self.get_parameter("source_joints").value]
        self.timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self.alpha = max(0.0, min(1.0, float(self.get_parameter("low_pass_alpha").value)))

        self.ball_radius = float(self.get_parameter("ball_radius").value)
        self.spawn_x = float(self.get_parameter("spawn_x").value)
        self.spawn_y = float(self.get_parameter("spawn_y").value)
        self.spawn_z = float(self.get_parameter("spawn_z").value)
        self.pickup_x = float(self.get_parameter("pickup_x").value)

        self.conveyor_power_service = str(self.get_parameter("conveyor_power_service").value)
        self.conveyor_pickup_power = float(self.get_parameter("conveyor_pickup_power").value)
        self.conveyor_black_power = float(self.get_parameter("conveyor_black_power").value)
        self.conveyor_green_power = float(self.get_parameter("conveyor_green_power").value)
        self.conveyor_green_tail_sec = float(self.get_parameter("conveyor_green_tail_sec").value)
        self.conveyor_black_tail_sec = float(self.get_parameter("conveyor_black_tail_sec").value)

        position_topic = str(self.get_parameter("position_command_topic").value)
        self.position_pub = self.create_publisher(Float64MultiArray, position_topic, 20)

        self.sim_joint_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("sim_joint_state_topic").value),
            20,
        )

        self.create_subscription(
            JointState,
            str(self.get_parameter("physical_joint_state_topic").value),
            self.state_callback,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("physical_event_topic").value),
            self.event_callback,
            20,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("gazebo_joint_state_topic").value),
            self.sim_state_callback,
            20,
        )

        self.conveyor_client = self.create_client(ConveyorBeltControl, self.conveyor_power_service)

        self.latest_position: Optional[list[float]] = None
        self.filtered_position: Optional[list[float]] = None
        self.last_state_wall_time: Optional[float] = None

        self.spawned_cycles: set[int] = set()
        self.current_ball_name: Optional[str] = None
        self.current_ball_color: Optional[str] = None
        self.current_cycle_id: Optional[int] = None

        # Used only for GREEN / BLACK reject actions.
        # ACTION_DONE schedules a delayed stop instead of stopping immediately.
        self.conveyor_stop_deadline: Optional[float] = None

        rate_hz = max(float(self.get_parameter("command_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate_hz, self.publish_commands)

    def state_callback(self, msg: JointState) -> None:
        position_by_name: Dict[str, float] = dict(zip(msg.name, msg.position))
        missing = [n for n in self.source_joints if n not in position_by_name]
        if missing:
            return
        self.latest_position = [float(position_by_name[n]) for n in self.source_joints]
        self.last_state_wall_time = time.monotonic()

    def sim_state_callback(self, msg: JointState) -> None:
        """Publish Gazebo measured state independently from physical state."""
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)
        self.sim_joint_state_pub.publish(out)

    def publish_commands(self) -> None:
        # Non-blocking delayed stop for GREEN / BLACK reject actions.
        if (
            self.conveyor_stop_deadline is not None
            and time.monotonic() >= self.conveyor_stop_deadline
        ):
            self.set_conveyor_power(0.0)
            self.conveyor_stop_deadline = None
            self.get_logger().info(
                "Reject conveyor stopped after configured tail time."
            )

        if self.latest_position is None or self.last_state_wall_time is None:
            return
        if time.monotonic() - self.last_state_wall_time > self.timeout_sec:
            return

        if self.filtered_position is None:
            self.filtered_position = list(self.latest_position)
        else:
            self.filtered_position = [
                self.alpha * n + (1.0 - self.alpha) * o
                for n, o in zip(self.latest_position, self.filtered_position)
            ]

        command = Float64MultiArray()
        command.data = list(self.filtered_position)
        self.position_pub.publish(command)

    def event_callback(self, msg: String) -> None:
        parts = msg.data.split("|")
        if len(parts) != 4 or parts[0] != "EVENT":
            return

        try:
            cycle_id = int(parts[1])
        except ValueError:
            return

        event_name = parts[2].strip().upper()
        value = parts[3].strip()

        if event_name == "BALL_DETECTED":
            color = value.lower()
            if cycle_id in self.spawned_cycles:
                return

            name = self.spawn_ball(color, cycle_id)
            if name is not None:
                self.spawned_cycles.add(cycle_id)
                self.current_ball_name = name
                self.current_ball_color = color
                self.current_cycle_id = cycle_id
            return

        if event_name == "ACTION_START":
            action = value.split(":", 1)[0].strip().upper()

            # Any new conveyor action supersedes an older pending stop.
            if action in (
                "CONVEYOR_TO_PICKUP",
                "CONVEYOR_BLACK",
                "CONVEYOR_GREEN",
            ):
                self.conveyor_stop_deadline = None

            if action == "CONVEYOR_TO_PICKUP":
                self.set_conveyor_power(self.conveyor_pickup_power)
                return
            if action == "CONVEYOR_BLACK":
                self.set_conveyor_power(self.conveyor_black_power)
                return
            if action == "CONVEYOR_GREEN":
                self.set_conveyor_power(self.conveyor_green_power)
                return

        if event_name == "ACTION_DONE":
            fields = value.split(":", 1)
            action = fields[0].strip().upper()

            # Log the physical EV3 action duration when present.
            if len(fields) == 2:
                try:
                    duration_ms = int(fields[1])
                    self.get_logger().info(
                        "{} EV3 duration: {} ms".format(
                            action,
                            duration_ms,
                        )
                    )
                except ValueError:
                    pass

            if action == "CONVEYOR_TO_PICKUP":
                # Pickup alignment should stop when the physical conveyor
                # action finishes.
                self.conveyor_stop_deadline = None
                self.set_conveyor_power(0.0)

            elif action == "CONVEYOR_GREEN":
                # Keep the reverse conveyor moving briefly so the simulated
                # green ball fully clears the conveyor edge.
                self.conveyor_stop_deadline = (
                    time.monotonic() + self.conveyor_green_tail_sec
                )
                self.get_logger().info(
                    "GREEN reject tail scheduled for {:.3f} s.".format(
                        self.conveyor_green_tail_sec
                    )
                )

            elif action == "CONVEYOR_BLACK":
                # Keep the forward conveyor moving briefly so the simulated
                # black ball fully clears the conveyor edge.
                self.conveyor_stop_deadline = (
                    time.monotonic() + self.conveyor_black_tail_sec
                )
                self.get_logger().info(
                    "BLACK reject tail scheduled for {:.3f} s.".format(
                        self.conveyor_black_tail_sec
                    )
                )

            return

        if event_name in ("FAULT", "ACTION_FAILED"):
            # Faults always stop immediately.
            self.conveyor_stop_deadline = None
            self.set_conveyor_power(0.0)
            return

        if event_name == "CYCLE_COMPLETE":
            # Clean up the cycle, but do not interrupt an active GREEN / BLACK
            # reject tail. The timer will stop the conveyor at the deadline.
            if self.conveyor_stop_deadline is None:
                self.set_conveyor_power(0.0)
            else:
                self.get_logger().info(
                    "Cycle complete received; keeping reject conveyor running "
                    "until the configured tail deadline."
                )

            self.current_ball_name = None
            self.current_ball_color = None
            self.current_cycle_id = None
            return

    def set_conveyor_power(self, power: float) -> None:
        requested_power = max(-100.0, min(100.0, float(power)))
        if not self.conveyor_client.service_is_ready():
            return
        request = ConveyorBeltControl.Request()
        request.power = requested_power
        self.conveyor_client.call_async(request)

    def spawn_ball(self, color: str, cycle_id: int) -> Optional[str]:
        if color not in BALL_RGB:
            return None

        r, g, b = BALL_RGB[color]
        cfg = BALL_PHYSICS_CONFIG.get(color, {"friction": 0.20, "kp": 20000.0, "kd": 50.0, "min_depth": 0.0005})
        name = "{}_ball_{}".format(color, cycle_id)

        sdf = BALL_SDF.format(
            name=name,
            visual_radius=self.ball_radius,
            collision_radius=self.ball_radius,
            friction=cfg["friction"],
            kp=cfg["kp"],
            kd=cfg["kd"],
            min_depth=cfg["min_depth"],
            r=r,
            g=g,
            b=b,
        )

        command = [
            "ros2", "run", "ros_gz_sim", "create",
            "-name", name,
            "-x", str(self.spawn_x),
            "-y", str(self.spawn_y),
            "-z", str(self.spawn_z),
            "-string", sdf,
        ]

        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=10, check=False)
            if result.returncode != 0:
                return None
        except subprocess.TimeoutExpired:
            return None

        return name


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboTwinInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.conveyor_client.service_is_ready():
            req = ConveyorBeltControl.Request()
            req.power = 0.0
            node.conveyor_client.call_async(req)
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()