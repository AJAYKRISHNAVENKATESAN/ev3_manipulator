#! usr/bin/env python3


"""Gazebo-side interface for the EV3 digital twin.

Design
------
* Manipulator joints follow a smooth 50 Hz calibrated "live telemetry" stream.
* The simulated conveyor belt is NOT used to determine ball transport.
* Ball transport is deterministic: measured EV3 conveyor progress [0, 1]
  maps directly to a task-space path in Gazebo.
* Red / blue balls are held exactly at pickup until GRIPPER_CLOSED. The
  physical close event is authoritative; the ball is attached at the current
  simulated contact pose and carried until GRIPPER_OPENED.
* Green / black balls are moved to deterministic reject poses outside the
  belt and then released to Gazebo physics.
* A simulated ball-state topic exposes the last pose that Gazebo accepted,
  allowing the coordinator to compare physical progress with applied sim
  progress.
"""

from __future__ import annotations

import json
import subprocess
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener


BALL_RGB = {
    "red": (1.0, 0.0, 0.0),
    "blue": (0.0, 0.0, 1.0),
    "black": (0.05, 0.05, 0.05),
    "green": (0.0, 0.8, 0.0),
}

BALL_PHYSICS_CONFIG = {
    "red": {
        "friction": 0.25,
        "kp": 100000.0,
        "kd": 100.0,
        "min_depth": 0.001,
    },
    "blue": {
        "friction": 0.25,
        "kp": 100000.0,
        "kd": 100.0,
        "min_depth": 0.001,
    },
    "black": {
        "friction": 0.20,
        "kp": 50000.0,
        "kd": 10.0,
        "min_depth": 0.001,
    },
    "green": {
        "friction": 0.20,
        "kp": 50000.0,
        "kd": 10.0,
        "min_depth": 0.001,
    },
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
    CONVEYOR_ACTIONS = {
        "CONVEYOR_TO_PICKUP",
        "CONVEYOR_BLACK",
        "CONVEYOR_GREEN",
    }

    PICK_COLORS = {"red", "blue"}
    REJECT_COLORS = {"green", "black"}

    def __init__(self) -> None:
        super().__init__("gazebo_twin_interface")

        # ============================================================
        # Manipulator mirroring
        # ============================================================
        self.declare_parameter(
            "source_joints",
            [
                "arm1_base_link_joint",
                "arm1_arm2_joint",
                "left_gear_arm4_joint",
            ],
        )
        self.declare_parameter(
            "physical_live_joint_state_topic",
            "/twin/physical/live_joint_states",
        )
        self.declare_parameter(
            "physical_event_topic",
            "/twin/physical/events",
        )
        self.declare_parameter(
            "physical_connected_topic",
            "/twin/physical/connected",
        )
        self.declare_parameter(
            "physical_conveyor_progress_topic",
            "/twin/physical/conveyor_progress",
        )
        self.declare_parameter(
            "position_command_topic",
            "/twin_position_controller/commands",
        )
        self.declare_parameter(
            "gazebo_joint_state_topic",
            "/joint_states",
        )
        self.declare_parameter(
            "sim_joint_state_topic",
            "/twin/sim/joint_states",
        )
        self.declare_parameter(
            "sim_ball_state_topic",
            "/twin/sim/ball_state",
        )
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("state_timeout_sec", 0.5)

        # ============================================================
        # Ball / task-space calibration
        # ============================================================
        self.declare_parameter("ball_radius", 0.014)
        self.declare_parameter("spawn_x", -0.154099)
        self.declare_parameter("spawn_y", 0.233)
        self.declare_parameter("spawn_z", 0.0610)
        self.declare_parameter("pickup_x", -0.020859)

        # The current URDF places the conveyor pulleys at approximately
        # x=-0.174 m and x=+0.151 m in base coordinates. These defaults put
        # reject targets roughly 3 cm beyond each edge. Keep them as launch
        # parameters so final visual calibration is one-line configuration.
        self.declare_parameter("green_reject_x", -0.205)
        self.declare_parameter("black_reject_x", 0.181)

        self.declare_parameter(
            "set_pose_service",
            "/world/empty/set_pose",
        )
        self.declare_parameter("ball_pose_rate_hz", 20.0)
        self.declare_parameter("ball_pose_service_timeout_sec", 2.0)
        self.declare_parameter("attach_world_frame", "world")
        self.declare_parameter("attach_frame", "arm_4_1")

        # ============================================================
        # Read parameters
        # ============================================================
        self.source_joints = [
            str(name)
            for name in self.get_parameter("source_joints").value
        ]
        if len(self.source_joints) != 3:
            raise ValueError(
                "source_joints must contain base, arm and gripper"
            )

        self.timeout_sec = float(
            self.get_parameter("state_timeout_sec").value
        )

        self.ball_radius = float(self.get_parameter("ball_radius").value)
        self.spawn_x = float(self.get_parameter("spawn_x").value)
        self.spawn_y = float(self.get_parameter("spawn_y").value)
        self.spawn_z = float(self.get_parameter("spawn_z").value)
        self.pickup_x = float(self.get_parameter("pickup_x").value)
        self.green_reject_x = float(
            self.get_parameter("green_reject_x").value
        )
        self.black_reject_x = float(
            self.get_parameter("black_reject_x").value
        )
        self.set_pose_service = str(
            self.get_parameter("set_pose_service").value
        )
        self.ball_pose_period = 1.0 / max(
            1.0,
            float(self.get_parameter("ball_pose_rate_hz").value),
        )
        self.ball_pose_service_timeout = max(
            0.1,
            float(
                self.get_parameter(
                    "ball_pose_service_timeout_sec"
                ).value
            ),
        )
        self.attach_world_frame = str(
            self.get_parameter("attach_world_frame").value
        )
        self.attach_frame = str(
            self.get_parameter("attach_frame").value
        )
        self.transport_targets = {
            "CONVEYOR_TO_PICKUP": (
                self.pickup_x,
                self.spawn_y,
                self.spawn_z,
            ),
            "CONVEYOR_GREEN": (
                self.green_reject_x,
                self.spawn_y,
                self.spawn_z,
            ),
            "CONVEYOR_BLACK": (
                self.black_reject_x,
                self.spawn_y,
                self.spawn_z,
            ),
        }

        # ============================================================
        # ROS I/O
        # ============================================================
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("position_command_topic").value),
            20,
        )
        self.sim_joint_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("sim_joint_state_topic").value),
            20,
        )
        self.sim_ball_state_pub = self.create_publisher(
            String,
            str(self.get_parameter("sim_ball_state_topic").value),
            20,
        )

        self.create_subscription(
            JointState,
            str(
                self.get_parameter(
                    "physical_live_joint_state_topic"
                ).value
            ),
            self.live_state_callback,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("physical_event_topic").value),
            self.event_callback,
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("physical_connected_topic").value),
            self.connection_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(
                self.get_parameter(
                    "physical_conveyor_progress_topic"
                ).value
            ),
            self.conveyor_progress_callback,
            20,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("gazebo_joint_state_topic").value),
            self.sim_state_callback,
            20,
        )

        self.set_pose_client = self.create_client(
            SetEntityPose,
            self.set_pose_service,
        )

        # TF is used only for logical grasp attachment.  The ball offset is
        # captured at GRIPPER_CLOSED and then transformed with arm_4_1.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ============================================================
        # Runtime state: manipulator
        # ============================================================
        self.latest_position: Optional[list[float]] = None
        self.last_live_state_wall_time: Optional[float] = None
        self.latest_sim_gripper_position: Optional[float] = None

        # ============================================================
        # Runtime state: session / ball
        # ============================================================
        self.physical_connected = False
        self.session_id = 0
        self.spawned_cycles: set[int] = set()

        self.current_ball_name: Optional[str] = None
        self.current_ball_color: Optional[str] = None
        self.current_cycle_id: Optional[int] = None

        self.ball_mode = "IDLE"
        self.active_conveyor_action: Optional[str] = None

        self.physical_progress = 0.0
        self.desired_ball_progress = 0.0
        self.applied_ball_progress = 0.0

        self.ball_pose_control = False
        self.release_after_pose = False
        self.ball_pose_future = None
        self.ball_pose_future_started: Optional[float] = None
        self.ball_pose_request_progress: Optional[float] = None
        self.ball_pose_request_xyz: Optional[tuple[float, float, float]] = None
        self.last_ball_pose_request_wall_time = 0.0
        self.last_applied_xyz = (
            self.spawn_x,
            self.spawn_y,
            self.spawn_z,
        )

        self.ball_attached = False
        self.ball_attach_pending = False
        self.ball_attach_offset_local: Optional[
            tuple[float, float, float]
        ] = None
        self.last_attach_warning_wall_time = 0.0

        command_rate_hz = max(
            1.0,
            float(self.get_parameter("command_rate_hz").value),
        )
        self.create_timer(
            1.0 / command_rate_hz,
            self.publish_commands,
        )

        self.get_logger().info(
            "Gazebo twin ready: live manipulator telemetry + deterministic "
            "task-space ball transport. Conveyor power no longer determines "
            "ball position."
        )
        self.get_logger().info(
            "Ball targets: pickup_x={:.6f}, green_reject_x={:.6f}, "
            "black_reject_x={:.6f}".format(
                self.pickup_x,
                self.green_reject_x,
                self.black_reject_x,
            )
        )

    # ================================================================
    # Manipulator state
    # ================================================================

    def live_state_callback(self, msg: JointState) -> None:
        position_by_name: Dict[str, float] = dict(
            zip(msg.name, msg.position)
        )
        missing = [
            name
            for name in self.source_joints
            if name not in position_by_name
        ]
        if missing:
            return

        self.latest_position = [
            position_by_name[name]
            for name in self.source_joints
        ]
        self.last_live_state_wall_time = time.monotonic()

    def sim_state_callback(self, msg: JointState) -> None:
        # This is independent measured Gazebo feedback, not the command.
        self.sim_joint_state_pub.publish(msg)

        positions = dict(zip(msg.name, msg.position))
        gripper_name = self.source_joints[2]
        if gripper_name in positions:
            self.latest_sim_gripper_position = float(
                positions[gripper_name]
            )

    def publish_commands(self) -> None:
        now = time.monotonic()

        if (
            self.latest_position is not None
            and self.last_live_state_wall_time is not None
            and now - self.last_live_state_wall_time <= self.timeout_sec
        ):
            command = Float64MultiArray()
            command.data = list(self.latest_position)
            self.position_pub.publish(command)

        self.update_ball_pose_control(now)
        self.update_attached_ball_pose(now)

    # ================================================================
    # Session
    # ================================================================

    def connection_callback(self, msg: Bool) -> None:
        connected = bool(msg.data)

        if connected and not self.physical_connected:
            self.session_id += 1
            self.spawned_cycles.clear()
            self.reset_active_ball_state()
            self.get_logger().info(
                "Started EV3 twin session {}.".format(self.session_id)
            )

        if not connected and self.physical_connected:
            # Freeze deterministic transport at the last physical progress.
            self.active_conveyor_action = None
            if self.current_ball_name is not None:
                self.ball_mode = "PHYSICAL_DISCONNECTED"
                self.publish_ball_state()

        self.physical_connected = connected

    def reset_active_ball_state(self) -> None:
        self.current_ball_name = None
        self.current_ball_color = None
        self.current_cycle_id = None
        self.ball_mode = "IDLE"
        self.active_conveyor_action = None
        self.physical_progress = 0.0
        self.desired_ball_progress = 0.0
        self.applied_ball_progress = 0.0
        self.ball_pose_control = False
        self.release_after_pose = False
        self.ball_pose_future = None
        self.ball_pose_future_started = None
        self.ball_pose_request_progress = None
        self.ball_pose_request_xyz = None
        self.last_applied_xyz = (
            self.spawn_x,
            self.spawn_y,
            self.spawn_z,
        )
        self.ball_attached = False
        self.ball_attach_pending = False
        self.ball_attach_offset_local = None

    # ================================================================
    # Logical grasp attachment
    # ================================================================

    @staticmethod
    def rotate_vector_by_quaternion(
        vector: tuple[float, float, float],
        quaternion: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        """Rotate a 3-D vector by an xyzw quaternion."""
        vx, vy, vz = vector
        qx, qy, qz, qw = quaternion

        # t = 2 * cross(q.xyz, v)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        # v' = v + qw * t + cross(q.xyz, t)
        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    def lookup_attach_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.attach_world_frame,
                self.attach_frame,
                Time(),
            )
        except TransformException:
            return None

    def start_ball_attachment(self) -> bool:
        if self.current_ball_name is None:
            return False

        transform = self.lookup_attach_transform()
        if transform is None:
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        arm_xyz = (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )
        q = (
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )

        delta_world = (
            self.last_applied_xyz[0] - arm_xyz[0],
            self.last_applied_xyz[1] - arm_xyz[1],
            self.last_applied_xyz[2] - arm_xyz[2],
        )

        # world -> arm rotation is the inverse (conjugate) quaternion.
        q_inverse = (-q[0], -q[1], -q[2], q[3])
        self.ball_attach_offset_local = (
            self.rotate_vector_by_quaternion(
                delta_world,
                q_inverse,
            )
        )

        self.ball_attached = True
        self.ball_attach_pending = False

        self.get_logger().info(
            "Logically attached {} to {} with local offset "
            "({:.4f}, {:.4f}, {:.4f}).".format(
                self.current_ball_name,
                self.attach_frame,
                self.ball_attach_offset_local[0],
                self.ball_attach_offset_local[1],
                self.ball_attach_offset_local[2],
            )
        )
        return True

    def update_attached_ball_pose(self, now: float) -> None:
        if self.ball_attach_pending and not self.ball_attached:
            if not self.start_ball_attachment():
                if now - self.last_attach_warning_wall_time >= 1.0:
                    self.get_logger().warning(
                        "Physical grasp received; waiting for TF {} -> {} "
                        "before attaching ball.".format(
                            self.attach_world_frame,
                            self.attach_frame,
                        )
                    )
                    self.last_attach_warning_wall_time = now
                return

            self.ball_pose_control = False
            self.ball_mode = "GRASPED"
            self.publish_ball_state()

        if not self.ball_attached:
            return
        if self.current_ball_name is None:
            return
        if self.ball_attach_offset_local is None:
            return

        if self.ball_pose_future is not None:
            if self.ball_pose_future.done():
                self.ball_pose_future = None
                self.ball_pose_future_started = None
            else:
                return

        if now - self.last_ball_pose_request_wall_time < self.ball_pose_period:
            return

        transform = self.lookup_attach_transform()
        if transform is None:
            if now - self.last_attach_warning_wall_time >= 1.0:
                self.get_logger().warning(
                    "Lost TF {} -> {} while ball is attached.".format(
                        self.attach_world_frame,
                        self.attach_frame,
                    )
                )
                self.last_attach_warning_wall_time = now
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        q = (
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )

        offset_world = self.rotate_vector_by_quaternion(
            self.ball_attach_offset_local,
            q,
        )
        xyz = (
            float(translation.x) + offset_world[0],
            float(translation.y) + offset_world[1],
            float(translation.z) + offset_world[2],
        )

        # Keep applied progress at 1.0 while carrying the successfully
        # transported ball.
        self.request_ball_pose(1.0, xyz)

    # ================================================================
    # Physical conveyor progress -> deterministic ball path
    # ================================================================

    def conveyor_progress_callback(self, msg: Float64) -> None:
        if self.active_conveyor_action not in self.CONVEYOR_ACTIONS:
            return
        if self.current_ball_name is None:
            return

        progress = float(msg.data)
        if progress < 0.0:
            return

        progress = max(0.0, min(1.0, progress))
        self.physical_progress = progress
        self.desired_ball_progress = progress

    def target_for_action(
        self,
        action: str,
    ) -> tuple[float, float, float]:
        return self.transport_targets[action]

    def interpolated_ball_pose(
        self,
        action: str,
        progress: float,
    ) -> tuple[float, float, float]:
        target_x, target_y, target_z = self.target_for_action(action)

        x = self.spawn_x + progress * (target_x - self.spawn_x)
        y = self.spawn_y + progress * (target_y - self.spawn_y)
        z = self.spawn_z + progress * (target_z - self.spawn_z)
        return x, y, z

    def update_ball_pose_control(self, now: float) -> None:
        if not self.ball_pose_control:
            return
        if self.current_ball_name is None:
            return
        if self.active_conveyor_action not in self.CONVEYOR_ACTIONS:
            # FAULT mode can intentionally hold the last applied pose.
            if self.ball_mode != "FAULT":
                return
            action = self.last_transport_action_for_current_ball()
            if action is None:
                return
        else:
            action = self.active_conveyor_action

        # Only one service request in flight. If it gets stuck, allow a retry
        # after the configured timeout rather than blocking the twin forever.
        if self.ball_pose_future is not None:
            if self.ball_pose_future.done():
                self.ball_pose_future = None
                self.ball_pose_future_started = None
            elif (
                self.ball_pose_future_started is not None
                and now - self.ball_pose_future_started
                > self.ball_pose_service_timeout
            ):
                self.get_logger().warning(
                    "SetEntityPose request timed out; retrying latest ball pose."
                )
                self.ball_pose_future = None
                self.ball_pose_future_started = None
            else:
                return

        # While the ball is at pickup we intentionally reassert the pose at
        # ball_pose_rate_hz so contact / belt physics cannot drift it before
        # GRIPPER_CLOSED. During transport the same rate is sufficient because
        # physical progress itself is ~20 Hz.
        if now - self.last_ball_pose_request_wall_time < self.ball_pose_period:
            return

        progress = max(0.0, min(1.0, self.desired_ball_progress))
        xyz = self.interpolated_ball_pose(action, progress)
        self.request_ball_pose(progress, xyz)

    def request_ball_pose(
        self,
        progress: float,
        xyz: tuple[float, float, float],
    ) -> None:
        if not rclpy.ok():
            return
        if not self.set_pose_client.service_is_ready():
            self.get_logger().warning(
                "SetEntityPose service '{}' is not ready.".format(
                    self.set_pose_service
                )
            )
            return
        if self.current_ball_name is None:
            return

        request = SetEntityPose.Request()
        request.entity.name = self.current_ball_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(xyz[0])
        request.pose.position.y = float(xyz[1])
        request.pose.position.z = float(xyz[2])
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0
        request.pose.orientation.w = 1.0

        self.ball_pose_request_progress = progress
        self.ball_pose_request_xyz = xyz
        self.last_ball_pose_request_wall_time = time.monotonic()
        self.ball_pose_future_started = self.last_ball_pose_request_wall_time
        self.ball_pose_future = self.set_pose_client.call_async(request)
        self.ball_pose_future.add_done_callback(
            self.ball_pose_done_callback
        )

    def ball_pose_done_callback(self, future) -> None:
        progress = self.ball_pose_request_progress
        xyz = self.ball_pose_request_xyz

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                "SetEntityPose failed: {}".format(exc)
            )
            return

        if response is None or not response.success:
            self.get_logger().error(
                "Gazebo rejected SetEntityPose for {}".format(
                    self.current_ball_name
                )
            )
            return

        if progress is not None:
            self.applied_ball_progress = float(progress)
        if xyz is not None:
            self.last_applied_xyz = xyz

        self.publish_ball_state()

        # Reject balls are released only after Gazebo confirmed the exact
        # endpoint pose. They are then outside the belt and gravity takes over.
        if (
            self.release_after_pose
            and self.applied_ball_progress >= 0.999
        ):
            self.ball_pose_control = False
            self.release_after_pose = False
            self.publish_ball_state()

    def last_transport_action_for_current_ball(self) -> Optional[str]:
        if self.current_ball_color in self.PICK_COLORS:
            return "CONVEYOR_TO_PICKUP"
        if self.current_ball_color == "green":
            return "CONVEYOR_GREEN"
        if self.current_ball_color == "black":
            return "CONVEYOR_BLACK"
        return None

    # ================================================================
    # Semantic events
    # ================================================================

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
                self.ball_mode = "SPAWNED"
                self.active_conveyor_action = None
                self.physical_progress = 0.0
                self.desired_ball_progress = 0.0
                self.applied_ball_progress = 0.0
                self.ball_pose_control = False
                self.release_after_pose = False
                self.ball_attached = False
                self.ball_attach_pending = False
                self.ball_attach_offset_local = None
                self.last_applied_xyz = (
                    self.spawn_x,
                    self.spawn_y,
                    self.spawn_z,
                )
                self.publish_ball_state()
            return

        if event_name == "ACTION_START":
            action = value.split(":", 1)[0].strip().upper()

            if action in self.CONVEYOR_ACTIONS:
                if self.current_ball_name is None:
                    self.get_logger().warning(
                        "{} started but no simulated ball exists.".format(
                            action
                        )
                    )
                    return

                self.active_conveyor_action = action
                self.physical_progress = 0.0
                self.desired_ball_progress = 0.0
                self.applied_ball_progress = 0.0
                self.ball_pose_control = True
                self.release_after_pose = False
                self.ball_mode = "TRANSPORTING"
                self.publish_ball_state()
                self.get_logger().info(
                    "Deterministic transport armed: {} -> {}".format(
                        action,
                        self.target_for_action(action),
                    )
                )
            return

        if event_name == "ACTION_DONE":
            fields = value.split(":", 1)
            action = fields[0].strip().upper()

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

            if action in self.CONVEYOR_ACTIONS:
                # Successful physical completion means the exact task-space
                # endpoint is authoritative, irrespective of tiny encoder
                # quantization at the final STATE packet.
                self.physical_progress = 1.0
                self.desired_ball_progress = 1.0

                if action == "CONVEYOR_TO_PICKUP":
                    self.ball_mode = "AT_PICKUP"
                    # Keep reasserting pickup pose until the physical gripper
                    # has actually closed. This removes pickup stochasticity.
                    self.ball_pose_control = True
                    self.release_after_pose = False
                else:
                    self.ball_mode = "REJECTED"
                    # Release only once Gazebo confirms the exact reject pose.
                    self.ball_pose_control = True
                    self.release_after_pose = True

                self.publish_ball_state()
            return

        if event_name == "GRIPPER_CLOSED":
            if (
                self.current_ball_color in self.PICK_COLORS
                and self.current_ball_name is not None
            ):
                # The physical EV3 stall/contact is the grasp source of truth.
                # +0.35 rad is the EMPTY-gripper closed pose; a ball between
                # the fingers can stop Gazebo at a much earlier contact angle.
                self.ball_mode = "GRASP_PENDING"
                self.ball_pose_control = True
                self.release_after_pose = False
                self.ball_attach_pending = True
                self.ball_attached = False
                self.ball_attach_offset_local = None

                measured = self.latest_sim_gripper_position
                self.get_logger().info(
                    "Physical GRIPPER_CLOSED received; Gazebo contact "
                    "position={}. Attaching at contact pose.".format(
                        "unknown"
                        if measured is None
                        else "{:.3f} rad".format(measured)
                    )
                )

                if self.start_ball_attachment():
                    self.ball_pose_control = False
                    self.ball_mode = "GRASPED"

                self.publish_ball_state()
            return

        if event_name == "GRIPPER_OPENED":
            if self.current_ball_name is not None:
                # Stop kinematic attachment first; from this instant Gazebo
                # gravity / contact physics owns the released ball.
                self.ball_attach_pending = False
                self.ball_attached = False
                self.ball_attach_offset_local = None
                self.ball_mode = "RELEASED"
                self.publish_ball_state()
            return

        if event_name in ("ACTION_FAILED", "FAULT"):
            # Never snap a failed physical conveyor action to success. Freeze
            # at the last physical progress instead.
            if self.current_ball_name is not None:
                self.ball_mode = "FAULT"
                self.ball_pose_control = True
                self.release_after_pose = False
                self.publish_ball_state()

            self.active_conveyor_action = None
            self.ball_attach_pending = False
            self.ball_attached = False
            self.ball_attach_offset_local = None
            self.get_logger().error(
                "Physical EV3 fault received: {}".format(value)
            )
            return

        if event_name == "CYCLE_COMPLETE":
            if self.current_ball_name is not None:
                self.ball_mode = "COMPLETE"
                self.publish_ball_state()
            self.active_conveyor_action = None
            self.ball_pose_control = False
            self.release_after_pose = False
            self.ball_attach_pending = False
            self.ball_attached = False
            self.ball_attach_offset_local = None
            return

        if event_name == "TASK_COMPLETE":
            self.active_conveyor_action = None
            self.ball_pose_control = False
            self.release_after_pose = False
            self.ball_attach_pending = False
            self.ball_attached = False
            self.ball_attach_offset_local = None
            return

    # ================================================================
    # Ball state observability
    # ================================================================

    def publish_ball_state(self) -> None:
        if self.current_ball_name is None:
            return

        action = self.active_conveyor_action
        if action is None:
            action = self.last_transport_action_for_current_ball()

        payload = {
            "session_id": self.session_id,
            "cycle_id": self.current_cycle_id,
            "name": self.current_ball_name,
            "color": self.current_ball_color,
            "mode": self.ball_mode,
            "action": action,
            "physical_progress": self.physical_progress,
            "desired_progress": self.desired_ball_progress,
            "applied_progress": self.applied_ball_progress,
            "pose_control_active": self.ball_pose_control,
            "attached": self.ball_attached,
            "attach_pending": self.ball_attach_pending,
            "attach_frame": self.attach_frame,
            "sim_gripper_position": self.latest_sim_gripper_position,
            "x": self.last_applied_xyz[0],
            "y": self.last_applied_xyz[1],
            "z": self.last_applied_xyz[2],
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.sim_ball_state_pub.publish(msg)

    # ================================================================
    # Ball spawning
    # ================================================================

    def spawn_ball(self, color: str, cycle_id: int) -> Optional[str]:
        if color not in BALL_RGB:
            self.get_logger().warning(
                "Unknown ball color: {}".format(color)
            )
            return None

        r, g, b = BALL_RGB[color]
        cfg = BALL_PHYSICS_CONFIG[color]

        name = "s{}_{}_ball_{}".format(
            self.session_id,
            color,
            cycle_id,
        )

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
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-name",
            name,
            "-x",
            str(self.spawn_x),
            "-y",
            str(self.spawn_y),
            "-z",
            str(self.spawn_z),
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
            return None

        if result.returncode != 0:
            self.get_logger().error(
                "Ball spawn failed for {}: {}".format(
                    name,
                    result.stderr.strip(),
                )
            )
            return None

        self.get_logger().info(
            "Spawned {} at ({:.4f}, {:.4f}, {:.4f})".format(
                name,
                self.spawn_x,
                self.spawn_y,
                self.spawn_z,
            )
        )
        return name


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboTwinInterface()

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