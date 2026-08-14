#!/usr/bin/env python3

"""Mirror EV3 state into Gazebo and synchronize the IFRA conveyor plugin.

Behaviour
---------
- Mirrors base, arm, and gripper positions continuously.
- Spawns a physics-enabled ball when the EV3 reports BALL_DETECTED.
- Starts /CONVEYORPOWER when the EV3 reports
  ACTION_START|CONVEYOR_TO_PICKUP.
- Stops /CONVEYORPOWER when the EV3 reports
  ACTION_DONE|CONVEYOR_TO_PICKUP:<duration_ms>.
- Stops the conveyor defensively on faults and cycle completion.

This is event-boundary synchronization. Exact spatial alignment at the pickup
point depends on calibrating conveyor_pickup_power against the EV3 action
duration and the simulated belt geometry.
"""

import subprocess
import time
from typing import Dict, Optional

import rclpy
from conveyorbelt_msgs.srv import ConveyorBeltControl
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


BALL_RGB = {
    "red": (1.0, 0.0, 0.0),
    "blue": (0.0, 0.0, 1.0),
    "black": (0.05, 0.05, 0.05),
    "green": (0.0, 0.8, 0.0),
}

# Physics-enabled ball: collision is present and gravity is enabled by default.
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
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.5</mu>
              <mu2>1.5</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.0</restitution_coefficient>
            <threshold>100000.0</threshold>
          </bounce>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>10.0</kd>
              <max_vel>0.1</max_vel>
              <min_depth>0.0001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name='visual'>
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
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
    """Mirror EV3 joints and synchronize the simulated conveyor."""

    def __init__(self) -> None:
        super().__init__("gazebo_state_mirror")

        # Joint mirroring.
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

        # Ball geometry and pose.
        self.declare_parameter("ball_radius", 0.014)
        self.declare_parameter("spawn_x", -0.154099)
        self.declare_parameter("spawn_y", 0.233)
        self.declare_parameter("spawn_z", 0.0581)

        # IFRA conveyor service.
        self.declare_parameter(
            "conveyor_power_service",
            "/CONVEYORPOWER",
        )
        self.declare_parameter("conveyor_pickup_power", 5.0)
        self.declare_parameter("conveyor_black_power", 18.0)

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

        self.ball_radius = float(self.get_parameter("ball_radius").value)
        self.spawn_x = float(self.get_parameter("spawn_x").value)
        self.spawn_y = float(self.get_parameter("spawn_y").value)
        self.spawn_z = float(self.get_parameter("spawn_z").value)

        self.conveyor_power_service = str(
            self.get_parameter("conveyor_power_service").value
        )
        self.conveyor_pickup_power = float(
            self.get_parameter("conveyor_pickup_power").value
        )
        self.conveyor_pickup_power = max(
            0.0,
            min(100.0, self.conveyor_pickup_power),
        )

        self.conveyor_black_power = float(
            self.get_parameter("conveyor_black_power").value
        )
        self.conveyor_black_power = max(
            0.0,
            min(100.0, self.conveyor_black_power),
        )

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

        self.conveyor_client = self.create_client(
            ConveyorBeltControl,
            self.conveyor_power_service,
        )

        self.latest_position: Optional[list[float]] = None
        self.filtered_position: Optional[list[float]] = None
        self.last_state_wall_time: Optional[float] = None

        self.spawned_cycles: set[int] = set()
        self.current_ball_name: Optional[str] = None
        self.current_ball_color: Optional[str] = None
        self.current_cycle_id: Optional[int] = None

        rate_hz = max(
            float(self.get_parameter("command_rate_hz").value),
            1.0,
        )
        self.create_timer(1.0 / rate_hz, self.publish_commands)

        self.get_logger().info(
            "Mirroring joints {} to {}. Ball transport uses {}."
            .format(
                self.position_joints,
                position_topic,
                self.conveyor_power_service,
            )
        )
        self.get_logger().info(
            "Ball: radius={:.4f} spawn=({:.6f}, {:.5f}, {:.6f}); "
            "pickup conveyor power={:.2f}."
            .format(
                self.ball_radius,
                self.spawn_x,
                self.spawn_y,
                self.spawn_z,
                self.conveyor_pickup_power,
            )
        )

    # ==============================================================
    # Joint mirroring
    # ==============================================================

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
                "JointState missing position joints: {}".format(missing)
            )
            return

        self.latest_position = [
            float(position_by_name[name])
            for name in self.position_joints
        ]
        self.last_state_wall_time = time.monotonic()

    def publish_commands(self) -> None:
        if (
            self.latest_position is None
            or self.last_state_wall_time is None
        ):
            return

        if time.monotonic() - self.last_state_wall_time > self.timeout_sec:
            return

        if self.filtered_position is None:
            self.filtered_position = list(self.latest_position)
        else:
            self.filtered_position = [
                self.alpha * new + (1.0 - self.alpha) * old
                for new, old in zip(
                    self.latest_position,
                    self.filtered_position,
                )
            ]

        command = Float64MultiArray()
        command.data = list(self.filtered_position)
        self.position_pub.publish(command)

    # ==============================================================
    # EV3 events
    # ==============================================================

    def event_callback(self, msg: String) -> None:
        parts = msg.data.split("|")

        if len(parts) != 4 or parts[0] != "EVENT":
            self.get_logger().warning(
                "Malformed digital-twin event: {}".format(msg.data)
            )
            return

        try:
            cycle_id = int(parts[1])
        except ValueError:
            self.get_logger().warning(
                "Invalid cycle id: {}".format(msg.data)
            )
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

            if action == "CONVEYOR_TO_PICKUP":
                pickup_power = (
                    18.0
                    if self.current_ball_color in ("red", "blue", "black")
                    else self.conveyor_pickup_power
                )
                self.get_logger().info(
                    "EV3 conveyor started for cycle {}; starting simulated "
                    "conveyor at power {:.2f}."
                    .format(cycle_id, pickup_power)
                )
                self.set_conveyor_power(pickup_power)
                return

            if action == "CONVEYOR_BLACK":
                self.get_logger().info(
                    "EV3 black-ball conveyor started for cycle {}; "
                    "starting simulated conveyor forward at power {:.2f}."
                    .format(cycle_id, self.conveyor_black_power)
                )
                self.set_conveyor_power(self.conveyor_black_power)
                return

            if action == "CONVEYOR_GREEN":
                self.get_logger().warning(
                    "CONVEYOR_GREEN requires reverse motion, but the stock "
                    "IFRA service accepts only non-negative power."
                )
                return

        if event_name == "ACTION_DONE":
            action = value.split(":", 1)[0].strip().upper()

            if action in (
                "CONVEYOR_TO_PICKUP",
                "CONVEYOR_BLACK",
            ):
                self.get_logger().info(
                    "EV3 conveyor action {} completed for cycle {}; "
                    "stopping simulated conveyor."
                    .format(action, cycle_id)
                )
                self.set_conveyor_power(0.0)
                return

        if event_name == "CYCLE_COMPLETE":
            self.set_conveyor_power(0.0)
            self.get_logger().info(
                "Physical cycle {} completed for {} ball."
                .format(cycle_id, value.lower())
            )
            self.current_ball_name = None
            self.current_ball_color = None
            self.current_cycle_id = None
            return

        if event_name in ("FAULT", "ACTION_FAILED"):
            self.set_conveyor_power(0.0)
            self.get_logger().error("EV3 error: {}".format(value))

    # ==============================================================
    # IFRA conveyor service
    # ==============================================================

    def set_conveyor_power(self, power: float) -> None:
        """Set conveyor power asynchronously without blocking callbacks."""
        requested_power = max(0.0, min(100.0, float(power)))

        if not self.conveyor_client.service_is_ready():
            self.get_logger().error(
                "{} is not ready; cannot set conveyor power to {:.2f}."
                .format(
                    self.conveyor_power_service,
                    requested_power,
                )
            )
            return

        request = ConveyorBeltControl.Request()
        request.power = requested_power
        future = self.conveyor_client.call_async(request)

        def response_callback(completed_future) -> None:
            try:
                response = completed_future.result()
            except Exception as exc:
                self.get_logger().error(
                    "Conveyor service call failed: {}".format(exc)
                )
                return

            if response is not None and response.success:
                self.get_logger().info(
                    "Simulated conveyor power set to {:.2f}."
                    .format(requested_power)
                )
            else:
                self.get_logger().error(
                    "Conveyor plugin rejected power {:.2f}."
                    .format(requested_power)
                )

        future.add_done_callback(response_callback)

    # ==============================================================
    # Ball spawn
    # ==============================================================

    def spawn_ball(
        self,
        color: str,
        cycle_id: int,
    ) -> Optional[str]:
        if color not in BALL_RGB:
            self.get_logger().warning(
                "Cannot spawn unsupported color: {}".format(color)
            )
            return None

        r, g, b = BALL_RGB[color]
        name = "{}_ball_{}".format(color, cycle_id)

        sdf = BALL_SDF.format(
            name=name,
            radius=self.ball_radius,
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
            "Spawned {} at ({:.6f}, {:.5f}, {:.6f})."
            .format(
                name,
                self.spawn_x,
                self.spawn_y,
                self.spawn_z,
            )
        )
        return name


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboStateMirror()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.conveyor_client.service_is_ready():
            request = ConveyorBeltControl.Request()
            request.power = 0.0
            node.conveyor_client.call_async(request)
            rclpy.spin_once(node, timeout_sec=0.2)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()