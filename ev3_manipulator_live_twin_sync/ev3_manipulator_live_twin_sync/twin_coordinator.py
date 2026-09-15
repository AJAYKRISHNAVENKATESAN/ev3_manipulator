#!/usr/bin/env python3
# """Compare measured EV3 state with measured Gazebo state.

# Phase 1 deliberately observes and validates synchronization before command
# arbitration / MoveIt execution is added.
# """

# import json
# import time
# from typing import Dict, Optional

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Bool, String


# class TwinCoordinator(Node):
#     def __init__(self) -> None:
#         super().__init__("twin_coordinator")

#         self.declare_parameter("physical_joint_state_topic", "/twin/physical/joint_states")
#         self.declare_parameter("sim_joint_state_topic", "/twin/sim/joint_states")

#         # Lists are paired by index. Change sim_joints to match /joint_states exactly.
#         self.declare_parameter(
#             "physical_joints",
#             [
#                 "arm_1_base_link_joint",
#                 "arm_2_left_arm_linkage_joint",
#                 "gripper_joint",
#             ],
#         )
#         self.declare_parameter(
#             "sim_joints",
#             [
#                 "arm1_base_link_joint",
#                 "arm1_arm2_joint",
#                 "left_gear_arm4_joint",
#             ],
#         )
#         self.declare_parameter("joint_tolerances_rad", [0.03, 0.03, 0.02])
#         self.declare_parameter("state_timeout_sec", 0.5)
#         self.declare_parameter("comparison_rate_hz", 20.0)
#         self.declare_parameter("status_topic", "/twin/status")
#         self.declare_parameter("synchronized_topic", "/twin/synchronized")

#         self.physical_joints = [str(v) for v in self.get_parameter("physical_joints").value]
#         self.sim_joints = [str(v) for v in self.get_parameter("sim_joints").value]
#         self.tolerances = [float(v) for v in self.get_parameter("joint_tolerances_rad").value]
#         if not (len(self.physical_joints) == len(self.sim_joints) == len(self.tolerances)):
#             raise ValueError("physical_joints, sim_joints and joint_tolerances_rad must have equal lengths")

#         self.timeout = float(self.get_parameter("state_timeout_sec").value)
#         self.physical_state: Optional[Dict[str, float]] = None
#         self.sim_state: Optional[Dict[str, float]] = None
#         self.physical_time: Optional[float] = None
#         self.sim_time: Optional[float] = None

#         self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
#         self.sync_pub = self.create_publisher(Bool, str(self.get_parameter("synchronized_topic").value), 10)

#         self.create_subscription(
#             JointState,
#             str(self.get_parameter("physical_joint_state_topic").value),
#             self.physical_callback,
#             20,
#         )
#         self.create_subscription(
#             JointState,
#             str(self.get_parameter("sim_joint_state_topic").value),
#             self.sim_callback,
#             20,
#         )

#         rate = max(float(self.get_parameter("comparison_rate_hz").value), 1.0)
#         self.create_timer(1.0 / rate, self.compare)

#     @staticmethod
#     def to_map(msg: JointState) -> Dict[str, float]:
#         return {str(n): float(p) for n, p in zip(msg.name, msg.position)}

#     def physical_callback(self, msg: JointState) -> None:
#         self.physical_state = self.to_map(msg)
#         self.physical_time = time.monotonic()

#     def sim_callback(self, msg: JointState) -> None:
#         self.sim_state = self.to_map(msg)
#         self.sim_time = time.monotonic()

#     def publish(self, synchronized: bool, payload: dict) -> None:
#         sync = Bool()
#         sync.data = synchronized
#         self.sync_pub.publish(sync)

#         status = String()
#         status.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
#         self.status_pub.publish(status)

#     def compare(self) -> None:
#         now = time.monotonic()
#         if self.physical_state is None or self.sim_state is None or self.physical_time is None or self.sim_time is None:
#             self.publish(False, {"synchronized": False, "reason": "waiting_for_state"})
#             return

#         physical_age = now - self.physical_time
#         sim_age = now - self.sim_time
#         if physical_age > self.timeout or sim_age > self.timeout:
#             self.publish(False, {
#                 "synchronized": False,
#                 "reason": "stale_state",
#                 "physical_age_sec": physical_age,
#                 "sim_age_sec": sim_age,
#             })
#             return

#         missing_physical = [j for j in self.physical_joints if j not in self.physical_state]
#         missing_sim = [j for j in self.sim_joints if j not in self.sim_state]
#         if missing_physical or missing_sim:
#             self.publish(False, {
#                 "synchronized": False,
#                 "reason": "missing_joints",
#                 "missing_physical": missing_physical,
#                 "missing_sim": missing_sim,
#             })
#             return

#         synchronized = True
#         errors = {}
#         for physical_joint, sim_joint, tolerance in zip(self.physical_joints, self.sim_joints, self.tolerances):
#             physical = self.physical_state[physical_joint]
#             sim = self.sim_state[sim_joint]
#             error = physical - sim
#             ok = abs(error) <= tolerance
#             synchronized = synchronized and ok
#             errors[physical_joint] = {
#                 "sim_joint": sim_joint,
#                 "physical": physical,
#                 "sim": sim,
#                 "error": error,
#                 "abs_error": abs(error),
#                 "tolerance": tolerance,
#                 "within_tolerance": ok,
#             }

#         self.publish(synchronized, {
#             "synchronized": synchronized,
#             "physical_age_sec": physical_age,
#             "sim_age_sec": sim_age,
#             "joints": errors,
#         })


# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = TwinCoordinator()
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



# """Digital-twin synchronization coordinator.

# Compares measured physical EV3 joint state with measured Gazebo joint state.

# Publishes:
#     /twin/synchronized   std_msgs/Bool
#     /twin/state          std_msgs/String
#     /twin/status         std_msgs/String (JSON)

# Twin states:
#     WAITING_FOR_DATA
#     SYNCED
#     OUT_OF_SYNC
#     PHYSICAL_STALE
#     SIM_STALE
#     BOTH_STALE
#     MISSING_JOINTS
# """

# import json
# import time
# from typing import Dict, Optional

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Bool, String



# class TwinCoordinator(Node):
#     def __init__(self) -> None:
#         super().__init__("twin_coordinator")

#         # ============================================================
#         # Parameters
#         # ============================================================

#         self.declare_parameter(
#             "physical_joint_state_topic",
#             "/twin/physical/joint_states",
#         )

#         self.declare_parameter(
#             "sim_joint_state_topic",
#             "/twin/sim/joint_states",
#         )

#         self.declare_parameter(
#             "physical_joints",
#             [
#                 "arm1_base_link_joint",
#                 "arm1_arm2_joint",
#                 "left_gear_arm4_joint",
#             ],
#         )

#         self.declare_parameter(
#             "sim_joints",
#             [
#                 "arm1_base_link_joint",
#                 "arm1_arm2_joint",
#                 "left_gear_arm4_joint",
#             ],
#         )

#         self.declare_parameter("base_tolerance_rad", 0.03)
#         self.declare_parameter("arm_tolerance_rad", 0.03)
#         self.declare_parameter("gripper_tolerance_rad", 0.02)

#         self.declare_parameter("state_timeout_sec", 0.5)
#         self.declare_parameter("status_rate_hz", 20.0)

#         self.physical_connected = False

#         # ============================================================
#         # Read parameters
#         # ============================================================

#         self.physical_topic = str(
#             self.get_parameter(
#                 "physical_joint_state_topic"
#             ).value
#         )

#         self.sim_topic = str(
#             self.get_parameter(
#                 "sim_joint_state_topic"
#             ).value
#         )

#         self.physical_joints = [
#             str(name)
#             for name in self.get_parameter(
#                 "physical_joints"
#             ).value
#         ]

#         self.sim_joints = [
#             str(name)
#             for name in self.get_parameter(
#                 "sim_joints"
#             ).value
#         ]

#         if len(self.physical_joints) != len(self.sim_joints):
#             raise ValueError(
#                 "physical_joints and sim_joints must have "
#                 "the same number of entries."
#             )

#         if len(self.physical_joints) != 3:
#             raise ValueError(
#                 "Expected exactly three controlled joints: "
#                 "base, arm, gripper."
#             )

#         self.tolerances = [
#             float(
#                 self.get_parameter(
#                     "base_tolerance_rad"
#                 ).value
#             ),
#             float(
#                 self.get_parameter(
#                     "arm_tolerance_rad"
#                 ).value
#             ),
#             float(
#                 self.get_parameter(
#                     "gripper_tolerance_rad"
#                 ).value
#             ),
#         ]

#         self.state_timeout_sec = float(
#             self.get_parameter(
#                 "state_timeout_sec"
#             ).value
#         )

#         status_rate_hz = max(
#             1.0,
#             float(
#                 self.get_parameter(
#                     "status_rate_hz"
#                 ).value
#             ),
#         )

#         # ============================================================
#         # State
#         # ============================================================

#         self.latest_physical: Optional[JointState] = None
#         self.latest_sim: Optional[JointState] = None

#         self.last_physical_wall_time: Optional[float] = None
#         self.last_sim_wall_time: Optional[float] = None

#         self.last_twin_state: Optional[str] = None

#         # ============================================================
#         # Subscriptions
#         # ============================================================

#         self.create_subscription(
#             JointState,
#             self.physical_topic,
#             self.physical_callback,
#             20,
#         )

#         self.create_subscription(
#             JointState,
#             self.sim_topic,
#             self.sim_callback,
#             20,
#         )

#         self.create_subscription(
#             Bool,
#             "/twin/physical/connected",
#             self.physical_connected_callback,
#             10,
#         )

#         # ============================================================
#         # Publishers
#         # ============================================================

#         self.synchronized_pub = self.create_publisher(
#             Bool,
#             "/twin/synchronized",
#             10,
#         )

#         self.state_pub = self.create_publisher(
#             String,
#             "/twin/state",
#             10,
#         )

#         self.status_pub = self.create_publisher(
#             String,
#             "/twin/status",
#             10,
#         )

#         # ============================================================
#         # Comparison timer
#         # ============================================================

#         self.create_timer(
#             1.0 / status_rate_hz,
#             self.evaluate,
#         )

#         self.get_logger().info(
#             "Twin coordinator started."
#         )

#         self.get_logger().info(
#             "Physical state: {}".format(
#                 self.physical_topic
#             )
#         )

#         self.get_logger().info(
#             "Simulation state: {}".format(
#                 self.sim_topic
#             )
#         )

#     # ================================================================
#     # Callbacks
#     # ================================================================

#     def physical_connected_callback(self, msg: Bool) -> None:
#         self.physical_connected = bool(msg.data)

#     def physical_callback(
#         self,
#         msg: JointState,
#     ) -> None:
#         self.latest_physical = msg
#         self.last_physical_wall_time = time.monotonic()

#     def sim_callback(
#         self,
#         msg: JointState,
#     ) -> None:
#         self.latest_sim = msg
#         self.last_sim_wall_time = time.monotonic()

#     # ================================================================
#     # Helpers
#     # ================================================================

#     @staticmethod
#     def joint_positions(
#         msg: JointState,
#     ) -> Dict[str, float]:
#         return {
#             name: float(position)
#             for name, position in zip(
#                 msg.name,
#                 msg.position,
#             )
#         }

#     def publish_result(
#         self,
#         twin_state: str,
#         synchronized: bool,
#         status: dict,
#     ) -> None:
#         sync_msg = Bool()
#         sync_msg.data = synchronized
#         self.synchronized_pub.publish(sync_msg)

#         state_msg = String()
#         state_msg.data = twin_state
#         self.state_pub.publish(state_msg)

#         status_msg = String()
#         status_msg.data = json.dumps(
#             status,
#             separators=(",", ":"),
#             sort_keys=True,
#         )
#         self.status_pub.publish(status_msg)

#         # Only log state transitions.
#         if twin_state != self.last_twin_state:
#             self.get_logger().info(
#                 "Twin state: {} -> {}".format(
#                     self.last_twin_state
#                     if self.last_twin_state is not None
#                     else "NONE",
#                     twin_state,
#                 )
#             )

#             self.last_twin_state = twin_state

#     # ================================================================
#     # Twin evaluation
#     # ================================================================

#     def evaluate(self) -> None:
#         if not self.physical_connected:
#             self.publish_result(
#                 twin_state="PHYSICAL_DISCONNECTED",
#                 synchronized=False,
#                 status={
#                     "state": "PHYSICAL_DISCONNECTED",
#                     "synchronized": False,
#                     "reason": "physical_device_disconnected",
#                 },
#             )
#             return

#         now = time.monotonic()

#         # ------------------------------------------------------------
#         # No data yet
#         # ------------------------------------------------------------

#         if (
#             self.latest_physical is None
#             or self.latest_sim is None
#             or self.last_physical_wall_time is None
#             or self.last_sim_wall_time is None
#         ):
#             self.publish_result(
#                 twin_state="WAITING_FOR_DATA",
#                 synchronized=False,
#                 status={
#                     "state": "WAITING_FOR_DATA",
#                     "synchronized": False,
#                     "reason": "waiting_for_data",
#                     "physical_received": (
#                         self.latest_physical is not None
#                     ),
#                     "sim_received": (
#                         self.latest_sim is not None
#                     ),
#                 },
#             )
#             return

#         # ------------------------------------------------------------
#         # State freshness
#         # ------------------------------------------------------------

#         physical_age = (
#             now - self.last_physical_wall_time
#         )

#         sim_age = (
#             now - self.last_sim_wall_time
#         )

#         physical_stale = (
#             physical_age > self.state_timeout_sec
#         )

#         sim_stale = (
#             sim_age > self.state_timeout_sec
#         )

#         if physical_stale and sim_stale:
#             self.publish_result(
#                 twin_state="BOTH_STALE",
#                 synchronized=False,
#                 status={
#                     "state": "BOTH_STALE",
#                     "synchronized": False,
#                     "reason": "both_states_stale",
#                     "physical_age_sec": physical_age,
#                     "sim_age_sec": sim_age,
#                     "timeout_sec": self.state_timeout_sec,
#                 },
#             )
#             return

#         if physical_stale:
#             self.publish_result(
#                 twin_state="PHYSICAL_STALE",
#                 synchronized=False,
#                 status={
#                     "state": "PHYSICAL_STALE",
#                     "synchronized": False,
#                     "reason": "physical_state_stale",
#                     "physical_age_sec": physical_age,
#                     "sim_age_sec": sim_age,
#                     "timeout_sec": self.state_timeout_sec,
#                 },
#             )
#             return

#         if sim_stale:
#             self.publish_result(
#                 twin_state="SIM_STALE",
#                 synchronized=False,
#                 status={
#                     "state": "SIM_STALE",
#                     "synchronized": False,
#                     "reason": "sim_state_stale",
#                     "physical_age_sec": physical_age,
#                     "sim_age_sec": sim_age,
#                     "timeout_sec": self.state_timeout_sec,
#                 },
#             )
#             return

#         # ------------------------------------------------------------
#         # Extract joints
#         # ------------------------------------------------------------

#         physical_by_name = self.joint_positions(
#             self.latest_physical
#         )

#         sim_by_name = self.joint_positions(
#             self.latest_sim
#         )

#         missing_physical = [
#             name
#             for name in self.physical_joints
#             if name not in physical_by_name
#         ]

#         missing_sim = [
#             name
#             for name in self.sim_joints
#             if name not in sim_by_name
#         ]

#         if missing_physical or missing_sim:
#             self.publish_result(
#                 twin_state="MISSING_JOINTS",
#                 synchronized=False,
#                 status={
#                     "state": "MISSING_JOINTS",
#                     "synchronized": False,
#                     "reason": "required_joint_missing",
#                     "missing_physical": missing_physical,
#                     "missing_sim": missing_sim,
#                     "physical_age_sec": physical_age,
#                     "sim_age_sec": sim_age,
#                 },
#             )
#             return

#         # ------------------------------------------------------------
#         # Compare joints
#         # ------------------------------------------------------------

#         joint_status = {}
#         all_within_tolerance = True

#         for (
#             physical_name,
#             sim_name,
#             tolerance,
#         ) in zip(
#             self.physical_joints,
#             self.sim_joints,
#             self.tolerances,
#         ):
#             physical_value = physical_by_name[
#                 physical_name
#             ]

#             sim_value = sim_by_name[
#                 sim_name
#             ]

#             error = (
#                 physical_value - sim_value
#             )

#             abs_error = abs(error)

#             within_tolerance = (
#                 abs_error <= tolerance
#             )

#             if not within_tolerance:
#                 all_within_tolerance = False

#             joint_status[physical_name] = {
#                 "physical": physical_value,
#                 "sim": sim_value,
#                 "error": error,
#                 "abs_error": abs_error,
#                 "tolerance": tolerance,
#                 "within_tolerance": within_tolerance,
#             }

#         # ------------------------------------------------------------
#         # Final twin state
#         # ------------------------------------------------------------

#         if all_within_tolerance:
#             twin_state = "SYNCED"
#         else:
#             twin_state = "OUT_OF_SYNC"

#         self.publish_result(
#             twin_state=twin_state,
#             synchronized=all_within_tolerance,
#             status={
#                 "state": twin_state,
#                 "synchronized": all_within_tolerance,
#                 "physical_age_sec": physical_age,
#                 "sim_age_sec": sim_age,
#                 "joints": joint_status,
#             },
#         )


# def main(args=None) -> None:
#     rclpy.init(args=args)

#     node = TwinCoordinator()

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

"""Whole-system digital-twin synchronization coordinator.

Compares:
* Physical EV3 base / arm / gripper measured state vs Gazebo measured state.
* Physical EV3 conveyor task progress vs the last ball pose Gazebo actually
  accepted through SetEntityPose.

This intentionally compares task-space conveyor effect rather than pulley
velocity because the physical and simulated conveyors have different geometry.
"""

from __future__ import annotations

import json
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String


class TwinCoordinator(Node):
    CONVEYOR_ACTIONS = {
        "CONVEYOR_TO_PICKUP",
        "CONVEYOR_BLACK",
        "CONVEYOR_GREEN",
    }

    def __init__(self) -> None:
        super().__init__("twin_coordinator")

        # ============================================================
        # Topics
        # ============================================================
        self.declare_parameter(
            "physical_joint_state_topic",
            "/twin/physical/joint_states",
        )
        self.declare_parameter(
            "sim_joint_state_topic",
            "/twin/sim/joint_states",
        )
        self.declare_parameter(
            "physical_connected_topic",
            "/twin/physical/connected",
        )
        self.declare_parameter(
            "physical_event_topic",
            "/twin/physical/events",
        )
        self.declare_parameter(
            "physical_conveyor_progress_topic",
            "/twin/physical/conveyor_progress",
        )
        self.declare_parameter(
            "sim_ball_state_topic",
            "/twin/sim/ball_state",
        )

        # ============================================================
        # Manipulator comparison
        # ============================================================
        self.declare_parameter(
            "physical_joints",
            [
                "arm1_base_link_joint",
                "arm1_arm2_joint",
                "left_gear_arm4_joint",
            ],
        )
        self.declare_parameter(
            "sim_joints",
            [
                "arm1_base_link_joint",
                "arm1_arm2_joint",
                "left_gear_arm4_joint",
            ],
        )
        self.declare_parameter("base_tolerance_rad", 0.03)
        self.declare_parameter("arm_tolerance_rad", 0.03)
        self.declare_parameter("gripper_tolerance_rad", 0.02)

        # ============================================================
        # Transport comparison
        # ============================================================
        self.declare_parameter("transport_progress_tolerance", 0.10)
        self.declare_parameter("transport_terminal_tolerance", 0.03)

        # ============================================================
        # Freshness
        # ============================================================
        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("sim_state_timeout_sec", 1.0)
        self.declare_parameter("status_rate_hz", 20.0)

        # ============================================================
        # Read params
        # ============================================================
        self.physical_topic = str(
            self.get_parameter("physical_joint_state_topic").value
        )
        self.sim_topic = str(
            self.get_parameter("sim_joint_state_topic").value
        )
        self.physical_connected_topic = str(
            self.get_parameter("physical_connected_topic").value
        )
        self.physical_event_topic = str(
            self.get_parameter("physical_event_topic").value
        )
        self.physical_progress_topic = str(
            self.get_parameter("physical_conveyor_progress_topic").value
        )
        self.sim_ball_topic = str(
            self.get_parameter("sim_ball_state_topic").value
        )

        self.physical_joints = [
            str(name)
            for name in self.get_parameter("physical_joints").value
        ]
        self.sim_joints = [
            str(name)
            for name in self.get_parameter("sim_joints").value
        ]

        if len(self.physical_joints) != len(self.sim_joints):
            raise ValueError(
                "physical_joints and sim_joints must have the same length"
            )
        if len(self.physical_joints) != 3:
            raise ValueError(
                "Expected base, arm and gripper as the three manipulator joints"
            )

        self.tolerances = [
            float(self.get_parameter("base_tolerance_rad").value),
            float(self.get_parameter("arm_tolerance_rad").value),
            float(self.get_parameter("gripper_tolerance_rad").value),
        ]
        self.transport_tolerance = abs(
            float(
                self.get_parameter("transport_progress_tolerance").value
            )
        )
        self.transport_terminal_tolerance = abs(
            float(
                self.get_parameter("transport_terminal_tolerance").value
            )
        )
        self.state_timeout_sec = float(
            self.get_parameter("state_timeout_sec").value
        )
        self.sim_state_timeout_sec = float(
            self.get_parameter("sim_state_timeout_sec").value
        )
        status_rate_hz = max(
            1.0,
            float(self.get_parameter("status_rate_hz").value),
        )

        # ============================================================
        # State
        # ============================================================
        self.latest_physical: Optional[JointState] = None
        self.latest_sim: Optional[JointState] = None
        self.last_physical_wall_time: Optional[float] = None
        self.last_sim_wall_time: Optional[float] = None

        self.physical_connected: Optional[bool] = None
        self.fault_active = False
        self.fault_detail = ""

        self.active_cycle_id: Optional[int] = None
        self.active_transport_action: Optional[str] = None
        self.transport_done = False
        self.physical_transport_progress: Optional[float] = None
        self.latest_sim_ball_state: Optional[dict] = None
        self.last_sim_ball_wall_time: Optional[float] = None

        self.last_twin_state: Optional[str] = None

        # ============================================================
        # Subscriptions
        # ============================================================
        self.create_subscription(
            JointState,
            self.physical_topic,
            self.physical_callback,
            20,
        )
        self.create_subscription(
            JointState,
            self.sim_topic,
            self.sim_callback,
            20,
        )
        self.create_subscription(
            Bool,
            self.physical_connected_topic,
            self.physical_connected_callback,
            10,
        )
        self.create_subscription(
            String,
            self.physical_event_topic,
            self.physical_event_callback,
            20,
        )
        self.create_subscription(
            Float64,
            self.physical_progress_topic,
            self.physical_progress_callback,
            20,
        )
        self.create_subscription(
            String,
            self.sim_ball_topic,
            self.sim_ball_state_callback,
            20,
        )

        # ============================================================
        # Publishers
        # ============================================================
        self.synchronized_pub = self.create_publisher(
            Bool,
            "/twin/synchronized",
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            "/twin/state",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/twin/status",
            10,
        )

        self.create_timer(
            1.0 / status_rate_hz,
            self.evaluate,
        )

        self.get_logger().info(
            "Twin coordinator started: manipulator state + deterministic "
            "conveyor task-progress comparison."
        )

    # ================================================================
    # Callbacks
    # ================================================================

    def physical_connected_callback(self, msg: Bool) -> None:
        connected = bool(msg.data)
        if connected and self.physical_connected is not True:
            self.fault_active = False
            self.fault_detail = ""
            self.active_cycle_id = None
            self.active_transport_action = None
            self.transport_done = False
            self.physical_transport_progress = None
            self.latest_sim_ball_state = None
            self.last_sim_ball_wall_time = None
        self.physical_connected = connected

    def physical_callback(self, msg: JointState) -> None:
        self.latest_physical = msg
        self.last_physical_wall_time = time.monotonic()
        self.physical_connected = True

    def sim_callback(self, msg: JointState) -> None:
        self.latest_sim = msg
        self.last_sim_wall_time = time.monotonic()

    def physical_progress_callback(self, msg: Float64) -> None:
        progress = float(msg.data)
        if progress < 0.0:
            return
        self.physical_transport_progress = max(
            0.0,
            min(1.0, progress),
        )

    def sim_ball_state_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        self.latest_sim_ball_state = payload
        self.last_sim_ball_wall_time = time.monotonic()

    def physical_event_callback(self, msg: String) -> None:
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
            self.active_cycle_id = cycle_id
            return

        if event_name == "ACTION_START":
            action = value.split(":", 1)[0].strip().upper()
            if action == "HOME_INITIAL":
                self.fault_active = False
                self.fault_detail = ""
            if action in self.CONVEYOR_ACTIONS:
                self.active_cycle_id = cycle_id
                self.active_transport_action = action
                self.transport_done = False
                self.physical_transport_progress = 0.0
            return

        if event_name == "ACTION_DONE":
            action = value.split(":", 1)[0].strip().upper()
            if action in self.CONVEYOR_ACTIONS:
                self.active_cycle_id = cycle_id
                self.active_transport_action = action
                self.transport_done = True
                self.physical_transport_progress = 1.0
            return

        if event_name in ("ACTION_FAILED", "FAULT"):
            self.fault_active = True
            self.fault_detail = "{}:{}".format(event_name, value)
            return

        if event_name == "CYCLE_COMPLETE":
            self.active_transport_action = None
            self.transport_done = False
            self.physical_transport_progress = None
            return

    # ================================================================
    # Helpers
    # ================================================================

    @staticmethod
    def joint_positions(msg: JointState) -> Dict[str, float]:
        return {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
        }

    def publish_result(
        self,
        twin_state: str,
        synchronized: bool,
        status: dict,
    ) -> None:
        sync_msg = Bool()
        sync_msg.data = synchronized
        self.synchronized_pub.publish(sync_msg)

        state_msg = String()
        state_msg.data = twin_state
        self.state_pub.publish(state_msg)

        status_msg = String()
        status_msg.data = json.dumps(
            status,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.status_pub.publish(status_msg)

        if twin_state != self.last_twin_state:
            self.get_logger().info(
                "Twin state: {} -> {}".format(
                    self.last_twin_state
                    if self.last_twin_state is not None
                    else "NONE",
                    twin_state,
                )
            )
            self.last_twin_state = twin_state

    def evaluate_transport(self) -> tuple[bool, dict]:
        if self.active_transport_action is None:
            return True, {
                "active": False,
                "within_tolerance": True,
            }

        if self.physical_transport_progress is None:
            return False, {
                "active": True,
                "action": self.active_transport_action,
                "cycle_id": self.active_cycle_id,
                "reason": "waiting_for_physical_progress",
                "within_tolerance": False,
            }

        sim_state = self.latest_sim_ball_state
        if sim_state is None:
            return False, {
                "active": True,
                "action": self.active_transport_action,
                "cycle_id": self.active_cycle_id,
                "reason": "waiting_for_sim_ball_state",
                "within_tolerance": False,
            }

        sim_cycle = sim_state.get("cycle_id")
        sim_action = sim_state.get("action")
        sim_progress = sim_state.get("applied_progress")
        sim_mode = str(sim_state.get("mode", ""))

        if sim_cycle != self.active_cycle_id:
            return False, {
                "active": True,
                "action": self.active_transport_action,
                "cycle_id": self.active_cycle_id,
                "reason": "cycle_mismatch",
                "sim_cycle_id": sim_cycle,
                "within_tolerance": False,
            }

        if sim_action != self.active_transport_action:
            return False, {
                "active": True,
                "action": self.active_transport_action,
                "cycle_id": self.active_cycle_id,
                "reason": "action_mismatch",
                "sim_action": sim_action,
                "within_tolerance": False,
            }

        try:
            sim_progress_float = float(sim_progress)
        except (TypeError, ValueError):
            return False, {
                "active": True,
                "action": self.active_transport_action,
                "cycle_id": self.active_cycle_id,
                "reason": "invalid_sim_progress",
                "within_tolerance": False,
            }

        error = (
            self.physical_transport_progress - sim_progress_float
        )
        abs_error = abs(error)
        tolerance = (
            self.transport_terminal_tolerance
            if self.transport_done
            else self.transport_tolerance
        )
        within_tolerance = abs_error <= tolerance

        terminal_ok = True
        if self.transport_done:
            terminal_ok = sim_progress_float >= (
                1.0 - self.transport_terminal_tolerance
            )
            if self.active_transport_action == "CONVEYOR_TO_PICKUP":
                terminal_ok = terminal_ok and sim_mode in {
                    "AT_PICKUP",
                    "GRASPED",
                    "RELEASED",
                    "COMPLETE",
                }
            else:
                terminal_ok = terminal_ok and sim_mode in {
                    "REJECTED",
                    "COMPLETE",
                }

        synced = within_tolerance and terminal_ok

        return synced, {
            "active": True,
            "cycle_id": self.active_cycle_id,
            "action": self.active_transport_action,
            "physical_progress": self.physical_transport_progress,
            "sim_applied_progress": sim_progress_float,
            "progress_error": error,
            "abs_progress_error": abs_error,
            "progress_tolerance": tolerance,
            "transport_done": self.transport_done,
            "sim_mode": sim_mode,
            "terminal_ok": terminal_ok,
            "within_tolerance": synced,
        }

    # ================================================================
    # Evaluation
    # ================================================================

    def evaluate(self) -> None:
        now = time.monotonic()

        if self.physical_connected is False:
            self.publish_result(
                "PHYSICAL_DISCONNECTED",
                False,
                {
                    "state": "PHYSICAL_DISCONNECTED",
                    "synchronized": False,
                    "reason": "physical_device_disconnected",
                },
            )
            return

        if self.fault_active:
            self.publish_result(
                "SYSTEM_FAULT",
                False,
                {
                    "state": "SYSTEM_FAULT",
                    "synchronized": False,
                    "reason": "physical_fault",
                    "fault": self.fault_detail,
                },
            )
            return

        if (
            self.latest_physical is None
            or self.latest_sim is None
            or self.last_physical_wall_time is None
            or self.last_sim_wall_time is None
        ):
            self.publish_result(
                "WAITING_FOR_DATA",
                False,
                {
                    "state": "WAITING_FOR_DATA",
                    "synchronized": False,
                    "reason": "waiting_for_data",
                    "physical_received": self.latest_physical is not None,
                    "sim_received": self.latest_sim is not None,
                },
            )
            return

        physical_age = now - self.last_physical_wall_time
        sim_age = now - self.last_sim_wall_time

        physical_stale = physical_age > self.state_timeout_sec
        sim_stale = sim_age > self.sim_state_timeout_sec

        if physical_stale and sim_stale:
            self.publish_result(
                "BOTH_STALE",
                False,
                {
                    "state": "BOTH_STALE",
                    "synchronized": False,
                    "physical_age_sec": physical_age,
                    "sim_age_sec": sim_age,
                },
            )
            return

        if physical_stale:
            self.publish_result(
                "PHYSICAL_STALE",
                False,
                {
                    "state": "PHYSICAL_STALE",
                    "synchronized": False,
                    "physical_age_sec": physical_age,
                    "sim_age_sec": sim_age,
                },
            )
            return

        if sim_stale:
            self.publish_result(
                "SIM_STALE",
                False,
                {
                    "state": "SIM_STALE",
                    "synchronized": False,
                    "physical_age_sec": physical_age,
                    "sim_age_sec": sim_age,
                },
            )
            return

        physical_positions = self.joint_positions(
            self.latest_physical
        )
        sim_positions = self.joint_positions(self.latest_sim)

        missing_physical = [
            name
            for name in self.physical_joints
            if name not in physical_positions
        ]
        missing_sim = [
            name
            for name in self.sim_joints
            if name not in sim_positions
        ]

        if missing_physical or missing_sim:
            self.publish_result(
                "MISSING_JOINTS",
                False,
                {
                    "state": "MISSING_JOINTS",
                    "synchronized": False,
                    "missing_physical": missing_physical,
                    "missing_sim": missing_sim,
                },
            )
            return

        joint_status = {}
        manipulator_synced = True

        for physical_name, sim_name, tolerance in zip(
            self.physical_joints,
            self.sim_joints,
            self.tolerances,
        ):
            physical_value = physical_positions[physical_name]
            sim_value = sim_positions[sim_name]
            error = physical_value - sim_value
            abs_error = abs(error)
            within_tolerance = abs_error <= tolerance

            if not within_tolerance:
                manipulator_synced = False

            joint_status[physical_name] = {
                "physical": physical_value,
                "sim": sim_value,
                "error": error,
                "abs_error": abs_error,
                "tolerance": tolerance,
                "within_tolerance": within_tolerance,
            }

        transport_synced, transport_status = self.evaluate_transport()
        all_synced = manipulator_synced and transport_synced
        twin_state = "SYNCED" if all_synced else "OUT_OF_SYNC"

        self.publish_result(
            twin_state,
            all_synced,
            {
                "state": twin_state,
                "synchronized": all_synced,
                "physical_age_sec": physical_age,
                "sim_age_sec": sim_age,
                "manipulator_synchronized": manipulator_synced,
                "transport_synchronized": transport_synced,
                "joints": joint_status,
                "transport": transport_status,
            },
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinCoordinator()

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