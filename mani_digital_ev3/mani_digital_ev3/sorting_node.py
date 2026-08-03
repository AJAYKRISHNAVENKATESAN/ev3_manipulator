#!/usr/bin/env python3
# """Mirror measured EV3 joint states into the Gazebo digital twin.

# This node does not replay a duplicated stage machine. It continuously sends
# measured hardware positions to a Gazebo position controller and measured
# conveyor velocity to a Gazebo velocity controller.
# """

# import subprocess
# import threading
# from typing import Dict, Optional

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray, String


# BALL_RGB = {
#     "red": (1.0, 0.0, 0.0),
#     "blue": (0.0, 0.0, 1.0),
#     "black": (0.05, 0.05, 0.05),
#     "green": (0.0, 0.8, 0.0),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <pose>{x} {y} {z} 0 0 0</pose>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia>
#           <ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz>
#         </inertia>
#       </inertial>
#       <collision name='collision'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <surface>
#           <friction>
#             <ode><mu>0.7</mu><mu2>0.7</mu2></ode>
#           </friction>
#         </surface>
#       </collision>
#       <visual name='visual'>
#         <geometry><sphere><radius>0.0185</radius></sphere></geometry>
#         <material>
#           <ambient>{r} {g} {b} 1</ambient>
#           <diffuse>{r} {g} {b} 1</diffuse>
#         </material>
#       </visual>
#     </link>
#   </model>
# </sdf>"""


# class GazeboStateMirror(Node):
#     def __init__(self):
#         super().__init__("gazebo_state_mirror")

#         self.declare_parameter(
#             "position_joints",
#             [
#                 "arm_1_base_link_joint",
#                 "arm_2_left_arm_linkage_joint",
#                 "gripper_joint",
#             ],
#         )
#         self.declare_parameter("conveyor_joint", "conveyor_joint")
#         self.declare_parameter(
#             "position_command_topic",
#             "/twin_position_controller/commands",
#         )
#         self.declare_parameter(
#             "conveyor_command_topic",
#             "/twin_conveyor_controller/commands",
#         )
#         self.declare_parameter("command_rate_hz", 50.0)
#         self.declare_parameter("state_timeout_sec", 0.5)
#         self.declare_parameter("low_pass_alpha", 1.0)

#         # Ball spawn is event-based because the EV3 color sensor provides an
#         # event, not a measured 6D object pose.
#         self.declare_parameter("spawn_x", -0.14824)
#         self.declare_parameter("spawn_y", 0.29075)
#         self.declare_parameter("spawn_z", 0.09232)

#         self.position_joints = [
#             str(name)
#             for name in self.get_parameter("position_joints").value
#         ]
#         self.conveyor_joint = str(
#             self.get_parameter("conveyor_joint").value
#         )
#         self.timeout_sec = float(
#             self.get_parameter("state_timeout_sec").value
#         )
#         self.alpha = float(self.get_parameter("low_pass_alpha").value)
#         self.alpha = max(0.0, min(1.0, self.alpha))

#         position_topic = str(
#             self.get_parameter("position_command_topic").value
#         )
#         conveyor_topic = str(
#             self.get_parameter("conveyor_command_topic").value
#         )

#         self.position_pub = self.create_publisher(
#             Float64MultiArray,
#             position_topic,
#             20,
#         )
#         self.conveyor_pub = self.create_publisher(
#             Float64MultiArray,
#             conveyor_topic,
#             20,
#         )

#         self.create_subscription(
#             JointState,
#             "/digital_twin/joint_states",
#             self.state_callback,
#             20,
#         )
#         self.create_subscription(
#             String,
#             "/digital_twin/events",
#             self.event_callback,
#             20,
#         )

#         self.latest_position: Optional[list[float]] = None
#         self.latest_conveyor_velocity = 0.0
#         self.filtered_position: Optional[list[float]] = None
#         self.last_state_time = None
#         self.spawned_cycles: set[int] = set()
#         self.lock = threading.Lock()

#         rate_hz = float(self.get_parameter("command_rate_hz").value)
#         self.create_timer(1.0 / max(rate_hz, 1.0), self.publish_commands)

#     def state_callback(self, msg: JointState) -> None:
#         position_by_name: Dict[str, float] = dict(zip(msg.name, msg.position))
#         velocity_by_name: Dict[str, float] = dict(zip(msg.name, msg.velocity))

#         missing = [
#             name
#             for name in self.position_joints
#             if name not in position_by_name
#         ]

#         if missing:
#             self.get_logger().warning(
#                 "JointState missing position joints: {}".format(missing)
#             )
#             return

#         if self.conveyor_joint not in velocity_by_name:
#             self.get_logger().warning(
#                 "JointState missing conveyor velocity for {}".format(
#                     self.conveyor_joint
#                 )
#             )
#             return

#         position = [position_by_name[name] for name in self.position_joints]
#         conveyor_velocity = velocity_by_name[self.conveyor_joint]

#         with self.lock:
#             self.latest_position = position
#             self.latest_conveyor_velocity = conveyor_velocity
#             self.last_state_time = self.get_clock().now()

#     def publish_commands(self) -> None:
#         with self.lock:
#             if self.latest_position is None or self.last_state_time is None:
#                 return

#             age = (self.get_clock().now() - self.last_state_time).nanoseconds / 1e9

#             if age > self.timeout_sec:
#                 conveyor_velocity = 0.0
#             else:
#                 conveyor_velocity = self.latest_conveyor_velocity

#             if self.filtered_position is None:
#                 self.filtered_position = list(self.latest_position)
#             else:
#                 self.filtered_position = [
#                     self.alpha * new + (1.0 - self.alpha) * old
#                     for new, old in zip(
#                         self.latest_position,
#                         self.filtered_position,
#                     )
#                 ]

#             positions = list(self.filtered_position)

#         position_msg = Float64MultiArray()
#         position_msg.data = positions
#         self.position_pub.publish(position_msg)

#         conveyor_msg = Float64MultiArray()
#         conveyor_msg.data = [float(conveyor_velocity)]
#         self.conveyor_pub.publish(conveyor_msg)

#     def event_callback(self, msg: String) -> None:
#         parts = msg.data.split("|")

#         if len(parts) != 4 or parts[0] != "EVENT":
#             self.get_logger().warning(
#                 "Malformed digital-twin event: {}".format(msg.data)
#             )
#             return

#         try:
#             cycle_id = int(parts[1])
#         except ValueError:
#             self.get_logger().warning("Invalid cycle id: {}".format(msg.data))
#             return

#         event_name = parts[2]
#         value = parts[3]

#         if event_name == "BALL_DETECTED":
#             if cycle_id not in self.spawned_cycles:
#                 self.spawned_cycles.add(cycle_id)
#                 self.spawn_ball(value, cycle_id)
#             return

#         if event_name == "CYCLE_COMPLETE":
#             self.get_logger().info(
#                 "Physical cycle {} completed for {} ball".format(
#                     cycle_id,
#                     value,
#                 )
#             )
#             return

#         if event_name == "FAULT":
#             self.get_logger().error("EV3 fault: {}".format(value))

#     def spawn_ball(self, color: str, cycle_id: int) -> None:
#         if color not in BALL_RGB:
#             self.get_logger().warning(
#                 "Cannot spawn unsupported color: {}".format(color)
#             )
#             return

#         x = float(self.get_parameter("spawn_x").value)
#         y = float(self.get_parameter("spawn_y").value)
#         z = float(self.get_parameter("spawn_z").value)
#         r, g, b = BALL_RGB[color]
#         name = "{}_ball_{}".format(color, cycle_id)

#         sdf = BALL_SDF.format(
#             name=name,
#             x=x,
#             y=y,
#             z=z,
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
#             "-string",
#             sdf,
#         ]

#         try:
#             result = subprocess.run(
#                 command,
#                 capture_output=True,
#                 text=True,
#                 timeout=10,
#                 check=False,
#             )
#         except subprocess.TimeoutExpired:
#             self.get_logger().error("Ball spawn timed out: {}".format(name))
#             return

#         if result.returncode != 0:
#             self.get_logger().error(
#                 "Ball spawn failed for {}: {}".format(
#                     name,
#                     result.stderr.strip(),
#                 )
#             )
#             return

#         self.get_logger().info("Spawned {}".format(name))


# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = GazeboStateMirror()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
# """Mirror measured EV3 state into the Gazebo digital twin.

# The physical EV3 is the source of truth:
# - base, arm, and gripper encoder positions -> direct Gazebo position commands
# - conveyor encoder velocity -> visible pulley velocity command
# - optional IFRA conveyor service -> moving belt contact surface
# - EV3 colour events -> simulated ball spawning
# """

# import subprocess
# import threading
# import time
# from typing import Dict, Optional

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray, String

# try:
#     from conveyorbelt_msgs.srv import ConveyorBeltControl
# except ImportError:
#     ConveyorBeltControl = None


# BALL_RGB = {
#     "red": (1.0, 0.0, 0.0),
#     "blue": (0.0, 0.0, 1.0),
#     "black": (0.05, 0.05, 0.05),
#     "green": (0.0, 0.8, 0.0),
# }

# BALL_SDF = """<sdf version='1.7'>
#   <model name='{name}'>
#     <pose>{x} {y} {z} 0 0 0</pose>
#     <link name='link'>
#       <inertial>
#         <mass>0.05</mass>
#         <inertia>
#           <ixx>8e-06</ixx>
#           <iyy>8e-06</iyy>
#           <izz>8e-06</izz>
#         </inertia>
#       </inertial>
#       <collision name='collision'>
#         <geometry>
#           <sphere><radius>0.0185</radius></sphere>
#         </geometry>
#         <surface>
#           <friction>
#             <ode><mu>0.7</mu><mu2>0.7</mu2></ode>
#           </friction>
#         </surface>
#       </collision>
#       <visual name='visual'>
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


# class GazeboStateMirror(Node):
#     def __init__(self) -> None:
#         super().__init__("gazebo_state_mirror")

#         self.declare_parameter(
#             "position_joints",
#             [
#                 "arm1_base_link_joint",
#                 "arm1_arm2_joint",
#                 "left_gear_arm4_joint",
#             ],
#         )
#         self.declare_parameter(
#             "conveyor_joint",
#             "conveyor_left_pulley_joint",
#         )

#         self.declare_parameter(
#             "position_command_topic",
#             "/twin_position_controller/commands",
#         )
#         self.declare_parameter(
#             "conveyor_command_topic",
#             "/conveyor_controller/commands",
#         )

#         self.declare_parameter("command_rate_hz", 50.0)
#         self.declare_parameter("state_timeout_sec", 0.5)
#         self.declare_parameter("low_pass_alpha", 1.0)

#         self.declare_parameter("use_conveyor_plugin", True)
#         self.declare_parameter("conveyor_service", "/CONVEYORPOWER")
#         self.declare_parameter("conveyor_service_rate_hz", 10.0)
#         self.declare_parameter("conveyor_power_deadband", 1.0)
#         self.declare_parameter("ev3_conveyor_full_speed_rad_s", 1.0)
#         self.declare_parameter("conveyor_plugin_bidirectional", False)

#         self.declare_parameter("spawn_x", -0.14824)
#         self.declare_parameter("spawn_y", 0.29075)
#         self.declare_parameter("spawn_z", 0.09232)

#         self.position_joints = [
#             str(name)
#             for name in self.get_parameter("position_joints").value
#         ]
#         self.conveyor_joint = str(
#             self.get_parameter("conveyor_joint").value
#         )

#         self.timeout_sec = float(
#             self.get_parameter("state_timeout_sec").value
#         )
#         self.alpha = float(
#             self.get_parameter("low_pass_alpha").value
#         )
#         self.alpha = max(0.0, min(1.0, self.alpha))

#         position_topic = str(
#             self.get_parameter("position_command_topic").value
#         )
#         conveyor_topic = str(
#             self.get_parameter("conveyor_command_topic").value
#         )

#         self.position_pub = self.create_publisher(
#             Float64MultiArray,
#             position_topic,
#             20,
#         )
#         self.conveyor_pub = self.create_publisher(
#             Float64MultiArray,
#             conveyor_topic,
#             20,
#         )

#         self.create_subscription(
#             JointState,
#             "/digital_twin/joint_states",
#             self.state_callback,
#             20,
#         )
#         self.create_subscription(
#             String,
#             "/digital_twin/events",
#             self.event_callback,
#             20,
#         )

#         self.latest_position: Optional[list[float]] = None
#         self.latest_conveyor_velocity = 0.0
#         self.filtered_position: Optional[list[float]] = None
#         self.last_state_wall_time: Optional[float] = None

#         self.spawned_cycles: set[int] = set()
#         self.lock = threading.Lock()

#         self.warned_missing_conveyor_velocity = False
#         self.warned_reverse_plugin = False

#         self.use_conveyor_plugin = bool(
#             self.get_parameter("use_conveyor_plugin").value
#         )
#         self.plugin_bidirectional = bool(
#             self.get_parameter("conveyor_plugin_bidirectional").value
#         )
#         self.full_speed_rad_s = abs(
#             float(
#                 self.get_parameter(
#                     "ev3_conveyor_full_speed_rad_s"
#                 ).value
#             )
#         )
#         if self.full_speed_rad_s < 1e-6:
#             self.full_speed_rad_s = 1.0
#             self.get_logger().warning(
#                 "ev3_conveyor_full_speed_rad_s was zero; using 1.0."
#             )

#         service_rate = max(
#             float(
#                 self.get_parameter(
#                     "conveyor_service_rate_hz"
#                 ).value
#             ),
#             0.1,
#         )
#         self.service_period_sec = 1.0 / service_rate
#         self.power_deadband = max(
#             float(
#                 self.get_parameter(
#                     "conveyor_power_deadband"
#                 ).value
#             ),
#             0.0,
#         )

#         self.conveyor_client = None
#         self.last_service_call_time = 0.0
#         self.last_requested_power: Optional[float] = None

#         if self.use_conveyor_plugin:
#             if ConveyorBeltControl is None:
#                 self.get_logger().error(
#                     "use_conveyor_plugin is true, but conveyorbelt_msgs "
#                     "is not installed. The pulley can still rotate, but the "
#                     "belt contact surface will not move."
#                 )
#                 self.use_conveyor_plugin = False
#             else:
#                 service_name = str(
#                     self.get_parameter("conveyor_service").value
#                 )
#                 self.conveyor_client = self.create_client(
#                     ConveyorBeltControl,
#                     service_name,
#                 )
#                 self.get_logger().info(
#                     "Waiting for conveyor plugin service {}".format(
#                         service_name
#                     )
#                 )

#         rate_hz = max(
#             float(self.get_parameter("command_rate_hz").value),
#             1.0,
#         )
#         self.create_timer(1.0 / rate_hz, self.publish_commands)

#         self.get_logger().info(
#             "Mirroring positions {} to {} and conveyor {} to {}.".format(
#                 self.position_joints,
#                 position_topic,
#                 self.conveyor_joint,
#                 conveyor_topic,
#             )
#         )

#     def state_callback(self, msg: JointState) -> None:
#         position_by_name: Dict[str, float] = dict(
#             zip(msg.name, msg.position)
#         )
#         velocity_by_name: Dict[str, float] = dict(
#             zip(msg.name, msg.velocity)
#         )

#         missing = [
#             name
#             for name in self.position_joints
#             if name not in position_by_name
#         ]

#         if missing:
#             self.get_logger().warning(
#                 "JointState missing position joints: {}".format(missing)
#             )
#             return

#         if self.conveyor_joint in velocity_by_name:
#             conveyor_velocity = velocity_by_name[self.conveyor_joint]
#             self.warned_missing_conveyor_velocity = False
#         else:
#             conveyor_velocity = 0.0
#             if not self.warned_missing_conveyor_velocity:
#                 self.get_logger().warning(
#                     "JointState has no conveyor velocity for {}; "
#                     "continuing arm mirroring with conveyor stopped.".format(
#                         self.conveyor_joint
#                     )
#                 )
#                 self.warned_missing_conveyor_velocity = True

#         position = [
#             position_by_name[name]
#             for name in self.position_joints
#         ]

#         with self.lock:
#             self.latest_position = position
#             self.latest_conveyor_velocity = float(conveyor_velocity)
#             self.last_state_wall_time = time.monotonic()

#     def publish_commands(self) -> None:
#         with self.lock:
#             if (
#                 self.latest_position is None
#                 or self.last_state_wall_time is None
#             ):
#                 return

#             age = time.monotonic() - self.last_state_wall_time

#             if age > self.timeout_sec:
#                 conveyor_velocity = 0.0
#             else:
#                 conveyor_velocity = self.latest_conveyor_velocity

#             if self.filtered_position is None:
#                 self.filtered_position = list(self.latest_position)
#             else:
#                 self.filtered_position = [
#                     self.alpha * new + (1.0 - self.alpha) * old
#                     for new, old in zip(
#                         self.latest_position,
#                         self.filtered_position,
#                     )
#                 ]

#             positions = list(self.filtered_position)

#         position_msg = Float64MultiArray()
#         position_msg.data = positions
#         self.position_pub.publish(position_msg)

#         conveyor_msg = Float64MultiArray()
#         conveyor_msg.data = [float(conveyor_velocity)]
#         self.conveyor_pub.publish(conveyor_msg)

#         self.update_conveyor_plugin(conveyor_velocity)

#     def update_conveyor_plugin(self, velocity_rad_s: float) -> None:
#         if (
#             not self.use_conveyor_plugin
#             or self.conveyor_client is None
#             or ConveyorBeltControl is None
#         ):
#             return

#         now = time.monotonic()
#         if now - self.last_service_call_time < self.service_period_sec:
#             return

#         power = 100.0 * velocity_rad_s / self.full_speed_rad_s

#         if self.plugin_bidirectional:
#             power = max(-100.0, min(100.0, power))
#         else:
#             if power < 0.0:
#                 if not self.warned_reverse_plugin:
#                     self.get_logger().warning(
#                         "Negative conveyor velocity was received, but the "
#                         "stock conveyor plugin is configured as one-directional. "
#                         "The plugin surface is being stopped; the visible pulley "
#                         "will still mirror the signed velocity."
#                     )
#                     self.warned_reverse_plugin = True
#                 power = 0.0
#             else:
#                 self.warned_reverse_plugin = False
#                 power = min(100.0, power)

#         if (
#             self.last_requested_power is not None
#             and abs(power - self.last_requested_power) < self.power_deadband
#         ):
#             return

#         if not self.conveyor_client.service_is_ready():
#             return

#         request = ConveyorBeltControl.Request()
#         request.power = float(power)
#         future = self.conveyor_client.call_async(request)
#         future.add_done_callback(self.conveyor_service_done)

#         self.last_requested_power = float(power)
#         self.last_service_call_time = now

#     def conveyor_service_done(self, future) -> None:
#         try:
#             response = future.result()
#         except Exception as exc:
#             self.get_logger().error(
#                 "Conveyor service call failed: {}".format(exc)
#             )
#             return

#         if response is None:
#             self.get_logger().warning(
#                 "Conveyor service returned no response."
#             )

#     def event_callback(self, msg: String) -> None:
#         parts = msg.data.split("|")

#         if len(parts) != 4 or parts[0] != "EVENT":
#             self.get_logger().warning(
#                 "Malformed digital-twin event: {}".format(msg.data)
#             )
#             return

#         try:
#             cycle_id = int(parts[1])
#         except ValueError:
#             self.get_logger().warning(
#                 "Invalid cycle id: {}".format(msg.data)
#             )
#             return

#         event_name = parts[2]
#         value = parts[3]

#         if event_name == "BALL_DETECTED":
#             if cycle_id not in self.spawned_cycles:
#                 if self.spawn_ball(value, cycle_id):
#                     self.spawned_cycles.add(cycle_id)
#             return

#         if event_name == "CYCLE_COMPLETE":
#             self.get_logger().info(
#                 "Physical cycle {} completed for {} ball".format(
#                     cycle_id,
#                     value,
#                 )
#             )
#             return

#         if event_name == "FAULT":
#             self.get_logger().error(
#                 "EV3 fault: {}".format(value)
#             )

#     def spawn_ball(self, color: str, cycle_id: int) -> bool:
#         if color not in BALL_RGB:
#             self.get_logger().warning(
#                 "Cannot spawn unsupported color: {}".format(color)
#             )
#             return False

#         x = float(self.get_parameter("spawn_x").value)
#         y = float(self.get_parameter("spawn_y").value)
#         z = float(self.get_parameter("spawn_z").value)
#         r, g, b = BALL_RGB[color]
#         name = "{}_ball_{}".format(color, cycle_id)

#         sdf = BALL_SDF.format(
#             name=name,
#             x=x,
#             y=y,
#             z=z,
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
#             "-string",
#             sdf,
#         ]

#         try:
#             result = subprocess.run(
#                 command,
#                 capture_output=True,
#                 text=True,
#                 timeout=10,
#                 check=False,
#             )
#         except subprocess.TimeoutExpired:
#             self.get_logger().error(
#                 "Ball spawn timed out: {}".format(name)
#             )
#             return False

#         if result.returncode != 0:
#             self.get_logger().error(
#                 "Ball spawn failed for {}: {}".format(
#                     name,
#                     result.stderr.strip(),
#                 )
#             )
#             return False

#         self.get_logger().info(
#             "Spawned {}".format(name)
#         )
#         return True

#     def destroy_node(self) -> None:
#         try:
#             conveyor_msg = Float64MultiArray()
#             conveyor_msg.data = [0.0]
#             self.conveyor_pub.publish(conveyor_msg)
#         except Exception:
#             pass

#         super().destroy_node()


# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = GazeboStateMirror()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
"""Mirror measured EV3 arm and gripper states into the Gazebo digital twin.

For now:
- base, arm, and gripper encoder positions are mirrored continuously
- balls are spawned from EV3 BALL_DETECTED events
- conveyor control is intentionally disabled
"""

import subprocess
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


BALL_RGB = {
    "red": (1.0, 0.0, 0.0),
    "blue": (0.0, 0.0, 1.0),
    "black": (0.05, 0.05, 0.05),
    "green": (0.0, 0.8, 0.0),
}

BALL_SDF = """<sdf version='1.7'>
  <model name='{name}'>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name='link'>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>8e-06</ixx>
          <iyy>8e-06</iyy>
          <izz>8e-06</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere><radius>0.0185</radius></sphere>
        </geometry>
        <surface>
          <friction>
            <ode><mu>0.7</mu><mu2>0.7</mu2></ode>
          </friction>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <sphere><radius>0.0185</radius></sphere>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class GazeboStateMirror(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_state_mirror")

        self.declare_parameter(
            "position_joints",
            [
                "arm1_base_link_joint",
                "arm1_arm2_joint",
                "left_gear_arm4_joint",
            ],
        )
        self.declare_parameter(
            "position_command_topic",
            "/twin_position_controller/commands",
        )
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("low_pass_alpha", 1.0)

        self.declare_parameter("spawn_x", -0.154099)
        self.declare_parameter("spawn_y", 0.233)
        self.declare_parameter("spawn_z", 0.042998)

        self.position_joints = [
            str(name)
            for name in self.get_parameter("position_joints").value
        ]
        self.timeout_sec = float(
            self.get_parameter("state_timeout_sec").value
        )
        self.alpha = float(
            self.get_parameter("low_pass_alpha").value
        )
        self.alpha = max(0.0, min(1.0, self.alpha))

        position_topic = str(
            self.get_parameter("position_command_topic").value
        )

        self.position_pub = self.create_publisher(
            Float64MultiArray,
            position_topic,
            20,
        )

        self.create_subscription(
            JointState,
            "/digital_twin/joint_states",
            self.state_callback,
            20,
        )
        self.create_subscription(
            String,
            "/digital_twin/events",
            self.event_callback,
            20,
        )

        self.latest_position: Optional[list[float]] = None
        self.filtered_position: Optional[list[float]] = None
        self.last_state_wall_time: Optional[float] = None
        self.spawned_cycles: set[int] = set()
        self.lock = threading.Lock()

        rate_hz = max(
            float(self.get_parameter("command_rate_hz").value),
            1.0,
        )
        self.create_timer(
            1.0 / rate_hz,
            self.publish_commands,
        )

        self.get_logger().info(
            "Mirroring joints {} to {}. Conveyor control is disabled."
            .format(self.position_joints, position_topic)
        )

    def state_callback(self, msg: JointState) -> None:
        position_by_name: Dict[str, float] = dict(
            zip(msg.name, msg.position)
        )

        missing = [
            name
            for name in self.position_joints
            if name not in position_by_name
        ]

        if missing:
            self.get_logger().warning(
                "JointState missing position joints: {}".format(
                    missing
                )
            )
            return

        position = [
            float(position_by_name[name])
            for name in self.position_joints
        ]

        with self.lock:
            self.latest_position = position
            self.last_state_wall_time = time.monotonic()

    def publish_commands(self) -> None:
        with self.lock:
            if (
                self.latest_position is None
                or self.last_state_wall_time is None
            ):
                return

            age = time.monotonic() - self.last_state_wall_time

            if age > self.timeout_sec:
                return

            if self.filtered_position is None:
                self.filtered_position = list(
                    self.latest_position
                )
            else:
                self.filtered_position = [
                    self.alpha * new
                    + (1.0 - self.alpha) * old
                    for new, old in zip(
                        self.latest_position,
                        self.filtered_position,
                    )
                ]

            positions = list(self.filtered_position)

        position_msg = Float64MultiArray()
        position_msg.data = positions
        self.position_pub.publish(position_msg)

    def event_callback(self, msg: String) -> None:
        parts = msg.data.split("|")

        if len(parts) != 4 or parts[0] != "EVENT":
            self.get_logger().warning(
                "Malformed digital-twin event: {}".format(
                    msg.data
                )
            )
            return

        try:
            cycle_id = int(parts[1])
        except ValueError:
            self.get_logger().warning(
                "Invalid cycle id: {}".format(msg.data)
            )
            return

        event_name = parts[2]
        value = parts[3].strip().lower()

        if event_name == "BALL_DETECTED":
            if cycle_id in self.spawned_cycles:
                return

            if self.spawn_ball(value, cycle_id):
                self.spawned_cycles.add(cycle_id)

            return

        if event_name == "CYCLE_COMPLETE":
            self.get_logger().info(
                "Physical cycle {} completed for {} ball."
                .format(cycle_id, value)
            )
            return

        if event_name == "FAULT":
            self.get_logger().error(
                "EV3 fault: {}".format(value)
            )

    def spawn_ball(
        self,
        color: str,
        cycle_id: int,
    ) -> bool:
        if color not in BALL_RGB:
            self.get_logger().warning(
                "Cannot spawn unsupported color: {}".format(
                    color
                )
            )
            return False

        x = float(self.get_parameter("spawn_x").value)
        y = float(self.get_parameter("spawn_y").value)
        z = float(self.get_parameter("spawn_z").value)

        r, g, b = BALL_RGB[color]
        name = "{}_ball_{}".format(color, cycle_id)

        sdf = BALL_SDF.format(
            name=name,
            x=x,
            y=y,
            z=z,
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
            "-string",
            sdf,
        ]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                "Ball spawn timed out: {}".format(name)
            )
            return False

        if result.returncode != 0:
            self.get_logger().error(
                "Ball spawn failed for {}: {}".format(
                    name,
                    result.stderr.strip(),
                )
            )
            return False

        self.get_logger().info(
            "Spawned {} at ({:.5f}, {:.5f}, {:.5f})."
            .format(name, x, y, z)
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboStateMirror()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()