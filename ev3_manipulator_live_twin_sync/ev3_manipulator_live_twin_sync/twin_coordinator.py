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



"""Digital-twin synchronization coordinator.

Compares measured physical EV3 joint state with measured Gazebo joint state.

Publishes:
    /twin/synchronized   std_msgs/Bool
    /twin/state          std_msgs/String
    /twin/status         std_msgs/String (JSON)

Twin states:
    WAITING_FOR_DATA
    SYNCED
    OUT_OF_SYNC
    PHYSICAL_STALE
    SIM_STALE
    BOTH_STALE
    MISSING_JOINTS
"""

import json
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String



class TwinCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("twin_coordinator")

        # ============================================================
        # Parameters
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

        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("status_rate_hz", 20.0)

        self.physical_connected = False

        # ============================================================
        # Read parameters
        # ============================================================

        self.physical_topic = str(
            self.get_parameter(
                "physical_joint_state_topic"
            ).value
        )

        self.sim_topic = str(
            self.get_parameter(
                "sim_joint_state_topic"
            ).value
        )

        self.physical_joints = [
            str(name)
            for name in self.get_parameter(
                "physical_joints"
            ).value
        ]

        self.sim_joints = [
            str(name)
            for name in self.get_parameter(
                "sim_joints"
            ).value
        ]

        if len(self.physical_joints) != len(self.sim_joints):
            raise ValueError(
                "physical_joints and sim_joints must have "
                "the same number of entries."
            )

        if len(self.physical_joints) != 3:
            raise ValueError(
                "Expected exactly three controlled joints: "
                "base, arm, gripper."
            )

        self.tolerances = [
            float(
                self.get_parameter(
                    "base_tolerance_rad"
                ).value
            ),
            float(
                self.get_parameter(
                    "arm_tolerance_rad"
                ).value
            ),
            float(
                self.get_parameter(
                    "gripper_tolerance_rad"
                ).value
            ),
        ]

        self.state_timeout_sec = float(
            self.get_parameter(
                "state_timeout_sec"
            ).value
        )

        status_rate_hz = max(
            1.0,
            float(
                self.get_parameter(
                    "status_rate_hz"
                ).value
            ),
        )

        # ============================================================
        # State
        # ============================================================

        self.latest_physical: Optional[JointState] = None
        self.latest_sim: Optional[JointState] = None

        self.last_physical_wall_time: Optional[float] = None
        self.last_sim_wall_time: Optional[float] = None

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
            "/twin/physical/connected",
            self.physical_connected_callback,
            10,
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

        # ============================================================
        # Comparison timer
        # ============================================================

        self.create_timer(
            1.0 / status_rate_hz,
            self.evaluate,
        )

        self.get_logger().info(
            "Twin coordinator started."
        )

        self.get_logger().info(
            "Physical state: {}".format(
                self.physical_topic
            )
        )

        self.get_logger().info(
            "Simulation state: {}".format(
                self.sim_topic
            )
        )

    # ================================================================
    # Callbacks
    # ================================================================

    def physical_connected_callback(self, msg: Bool) -> None:
        self.physical_connected = bool(msg.data)

    def physical_callback(
        self,
        msg: JointState,
    ) -> None:
        self.latest_physical = msg
        self.last_physical_wall_time = time.monotonic()

    def sim_callback(
        self,
        msg: JointState,
    ) -> None:
        self.latest_sim = msg
        self.last_sim_wall_time = time.monotonic()

    # ================================================================
    # Helpers
    # ================================================================

    @staticmethod
    def joint_positions(
        msg: JointState,
    ) -> Dict[str, float]:
        return {
            name: float(position)
            for name, position in zip(
                msg.name,
                msg.position,
            )
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

        # Only log state transitions.
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

    # ================================================================
    # Twin evaluation
    # ================================================================

    def evaluate(self) -> None:
        if not self.physical_connected:
            self.publish_result(
                twin_state="PHYSICAL_DISCONNECTED",
                synchronized=False,
                status={
                    "state": "PHYSICAL_DISCONNECTED",
                    "synchronized": False,
                    "reason": "physical_device_disconnected",
                },
            )
            return

        now = time.monotonic()

        # ------------------------------------------------------------
        # No data yet
        # ------------------------------------------------------------

        if (
            self.latest_physical is None
            or self.latest_sim is None
            or self.last_physical_wall_time is None
            or self.last_sim_wall_time is None
        ):
            self.publish_result(
                twin_state="WAITING_FOR_DATA",
                synchronized=False,
                status={
                    "state": "WAITING_FOR_DATA",
                    "synchronized": False,
                    "reason": "waiting_for_data",
                    "physical_received": (
                        self.latest_physical is not None
                    ),
                    "sim_received": (
                        self.latest_sim is not None
                    ),
                },
            )
            return

        # ------------------------------------------------------------
        # State freshness
        # ------------------------------------------------------------

        physical_age = (
            now - self.last_physical_wall_time
        )

        sim_age = (
            now - self.last_sim_wall_time
        )

        physical_stale = (
            physical_age > self.state_timeout_sec
        )

        sim_stale = (
            sim_age > self.state_timeout_sec
        )

        if physical_stale and sim_stale:
            self.publish_result(
                twin_state="BOTH_STALE",
                synchronized=False,
                status={
                    "state": "BOTH_STALE",
                    "synchronized": False,
                    "reason": "both_states_stale",
                    "physical_age_sec": physical_age,
                    "sim_age_sec": sim_age,
                    "timeout_sec": self.state_timeout_sec,
                },
            )
            return

        if physical_stale:
            self.publish_result(
                twin_state="PHYSICAL_STALE",
                synchronized=False,
                status={
                    "state": "PHYSICAL_STALE",
                    "synchronized": False,
                    "reason": "physical_state_stale",
                    "physical_age_sec": physical_age,
                    "sim_age_sec": sim_age,
                    "timeout_sec": self.state_timeout_sec,
                },
            )
            return

        if sim_stale:
            self.publish_result(
                twin_state="SIM_STALE",
                synchronized=False,
                status={
                    "state": "SIM_STALE",
                    "synchronized": False,
                    "reason": "sim_state_stale",
                    "physical_age_sec": physical_age,
                    "sim_age_sec": sim_age,
                    "timeout_sec": self.state_timeout_sec,
                },
            )
            return

        # ------------------------------------------------------------
        # Extract joints
        # ------------------------------------------------------------

        physical_by_name = self.joint_positions(
            self.latest_physical
        )

        sim_by_name = self.joint_positions(
            self.latest_sim
        )

        missing_physical = [
            name
            for name in self.physical_joints
            if name not in physical_by_name
        ]

        missing_sim = [
            name
            for name in self.sim_joints
            if name not in sim_by_name
        ]

        if missing_physical or missing_sim:
            self.publish_result(
                twin_state="MISSING_JOINTS",
                synchronized=False,
                status={
                    "state": "MISSING_JOINTS",
                    "synchronized": False,
                    "reason": "required_joint_missing",
                    "missing_physical": missing_physical,
                    "missing_sim": missing_sim,
                    "physical_age_sec": physical_age,
                    "sim_age_sec": sim_age,
                },
            )
            return

        # ------------------------------------------------------------
        # Compare joints
        # ------------------------------------------------------------

        joint_status = {}
        all_within_tolerance = True

        for (
            physical_name,
            sim_name,
            tolerance,
        ) in zip(
            self.physical_joints,
            self.sim_joints,
            self.tolerances,
        ):
            physical_value = physical_by_name[
                physical_name
            ]

            sim_value = sim_by_name[
                sim_name
            ]

            error = (
                physical_value - sim_value
            )

            abs_error = abs(error)

            within_tolerance = (
                abs_error <= tolerance
            )

            if not within_tolerance:
                all_within_tolerance = False

            joint_status[physical_name] = {
                "physical": physical_value,
                "sim": sim_value,
                "error": error,
                "abs_error": abs_error,
                "tolerance": tolerance,
                "within_tolerance": within_tolerance,
            }

        # ------------------------------------------------------------
        # Final twin state
        # ------------------------------------------------------------

        if all_within_tolerance:
            twin_state = "SYNCED"
        else:
            twin_state = "OUT_OF_SYNC"

        self.publish_result(
            twin_state=twin_state,
            synchronized=all_within_tolerance,
            status={
                "state": twin_state,
                "synchronized": all_within_tolerance,
                "physical_age_sec": physical_age,
                "sim_age_sec": sim_age,
                "joints": joint_status,
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