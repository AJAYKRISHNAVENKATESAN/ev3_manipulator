#!/usr/bin/env python3
"""TCP -> ROS 2 bridge for an EV3 state-mirroring digital twin.

Receives continuous EV3 encoder telemetry and publishes ROS JointState.
Semantic events are forwarded separately. There is no stage barrier and no
simulation acknowledgement path.
"""

import math
import queue
import socket
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String


class Ev3StateBridge(Node):
    def __init__(self):
        super().__init__("ev3_state_bridge")

        self.declare_parameter("port", 5005)
        self.declare_parameter("base_joint", "arm_1_base_link_joint")
        self.declare_parameter("arm_joint", "arm_2_left_arm_linkage_joint")
        self.declare_parameter("gripper_joint", "gripper_joint")
        self.declare_parameter("conveyor_joint", "conveyor_joint")

        self.declare_parameter("base_gear_ratio", 3.0)
        self.declare_parameter("arm_gear_ratio", 5.0)
        self.declare_parameter("conveyor_gear_ratio", 1.0)

        self.declare_parameter("base_sign", 1.0)
        self.declare_parameter("arm_sign", 1.0)
        self.declare_parameter("gripper_sign", 1.0)
        self.declare_parameter("conveyor_sign", 1.0)

        self.declare_parameter("base_zero_offset_rad", 0.0)
        self.declare_parameter("arm_zero_offset_rad", 0.0)

        # Calibrate these from the actual open and closed encoder readings.
        self.declare_parameter("gripper_motor_open_deg", 0.0)
        self.declare_parameter("gripper_motor_closed_deg", 90.0)
        self.declare_parameter("gripper_sim_open", 0.5)
        self.declare_parameter("gripper_sim_closed", 0.0)

        self.port = int(self.get_parameter("port").value)

        self.joint_names = [
            str(self.get_parameter("base_joint").value),
            str(self.get_parameter("arm_joint").value),
            str(self.get_parameter("gripper_joint").value),
            str(self.get_parameter("conveyor_joint").value),
        ]

        self.base_gear = float(self.get_parameter("base_gear_ratio").value)
        self.arm_gear = float(self.get_parameter("arm_gear_ratio").value)
        self.conveyor_gear = float(
            self.get_parameter("conveyor_gear_ratio").value
        )

        self.base_sign = float(self.get_parameter("base_sign").value)
        self.arm_sign = float(self.get_parameter("arm_sign").value)
        self.gripper_sign = float(self.get_parameter("gripper_sign").value)
        self.conveyor_sign = float(
            self.get_parameter("conveyor_sign").value
        )

        self.base_zero = float(
            self.get_parameter("base_zero_offset_rad").value
        )
        self.arm_zero = float(
            self.get_parameter("arm_zero_offset_rad").value
        )

        self.gripper_motor_open = float(
            self.get_parameter("gripper_motor_open_deg").value
        )
        self.gripper_motor_closed = float(
            self.get_parameter("gripper_motor_closed_deg").value
        )
        self.gripper_sim_open = float(
            self.get_parameter("gripper_sim_open").value
        )
        self.gripper_sim_closed = float(
            self.get_parameter("gripper_sim_closed").value
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            "/digital_twin/joint_states",
            20,
        )
        self.event_pub = self.create_publisher(
            String,
            "/digital_twin/events",
            20,
        )
        self.connected_pub = self.create_publisher(
            Bool,
            "/digital_twin/connected",
            1,
        )
        self.conveyor_velocity_pub = self.create_publisher(
            Float64,
            "/digital_twin/conveyor_velocity",
            20,
        )
        self.raw_state_pub = self.create_publisher(
            String,
            "/digital_twin/raw_state",
            20,
        )

        self.line_queue: queue.Queue[tuple[str, Optional[str]]] = queue.Queue()
        self.shutdown_event = threading.Event()
        self.server: Optional[socket.socket] = None
        self.client: Optional[socket.socket] = None

        self.listener_thread = threading.Thread(
            target=self.listen_loop,
            daemon=True,
        )
        self.listener_thread.start()

        self.create_timer(0.01, self.drain_queue)
        self.publish_connection(False)

    @staticmethod
    def motor_deg_to_joint_rad(
        motor_deg: float,
        gear_ratio: float,
        sign: float,
        zero_offset_rad: float = 0.0,
    ) -> float:
        if gear_ratio == 0.0:
            raise ValueError("gear_ratio must not be zero")

        return sign * math.radians(motor_deg / gear_ratio) + zero_offset_rad

    @staticmethod
    def motor_dps_to_joint_rad_s(
        motor_dps: float,
        gear_ratio: float,
        sign: float,
    ) -> float:
        if gear_ratio == 0.0:
            raise ValueError("gear_ratio must not be zero")

        return sign * math.radians(motor_dps / gear_ratio)

    def map_gripper_position(self, motor_deg: float) -> float:
        denominator = self.gripper_motor_closed - self.gripper_motor_open

        if abs(denominator) < 1e-9:
            raise ValueError(
                "gripper_motor_open_deg and gripper_motor_closed_deg "
                "must be different"
            )

        fraction = (motor_deg - self.gripper_motor_open) / denominator
        fraction = max(0.0, min(1.0, fraction))

        value = self.gripper_sim_open + fraction * (
            self.gripper_sim_closed - self.gripper_sim_open
        )
        return self.gripper_sign * value

    def map_gripper_velocity(self, motor_dps: float) -> float:
        denominator = self.gripper_motor_closed - self.gripper_motor_open

        if abs(denominator) < 1e-9:
            return 0.0

        slope = (
            self.gripper_sim_closed - self.gripper_sim_open
        ) / denominator
        return self.gripper_sign * motor_dps * slope

    def publish_connection(self, connected: bool) -> None:
        msg = Bool()
        msg.data = connected
        self.connected_pub.publish(msg)

    def listen_loop(self) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", self.port))
        self.server.listen(1)
        self.server.settimeout(1.0)

        while rclpy.ok() and not self.shutdown_event.is_set():
            self.get_logger().info(
                "Awaiting EV3 state stream on TCP port {}...".format(
                    self.port
                )
            )

            try:
                client, address = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.client = client

            try:
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                client.settimeout(5.0)
                reader = client.makefile("r", encoding="utf-8", newline="\n")

                handshake = reader.readline().strip()

                if handshake != "EV3_CONNECT_REQUEST":
                    raise RuntimeError(
                        "Invalid handshake token: {!r}".format(handshake)
                    )

                client.sendall(b"ROS_CONNECT_ACCEPT\n")
                client.settimeout(None)
                self.line_queue.put(("CONNECTED", str(address)))

                for raw_line in reader:
                    line = raw_line.strip()
                    if line:
                        self.line_queue.put(("LINE", line))

            except (OSError, RuntimeError) as exc:
                self.line_queue.put(("ERROR", str(exc)))

            finally:
                try:
                    client.close()
                except OSError:
                    pass

                self.client = None
                self.line_queue.put(("DISCONNECTED", None))

    def drain_queue(self) -> None:
        while True:
            try:
                kind, payload = self.line_queue.get_nowait()
            except queue.Empty:
                return

            if kind == "CONNECTED":
                self.publish_connection(True)
                self.get_logger().info("EV3 connected from {}".format(payload))
            elif kind == "DISCONNECTED":
                self.publish_connection(False)
                self.get_logger().warning("EV3 disconnected")
            elif kind == "ERROR":
                self.get_logger().error("EV3 TCP error: {}".format(payload))
            elif kind == "LINE" and payload is not None:
                self.process_line(payload)

    def process_line(self, line: str) -> None:
        if line.startswith("STATE|"):
            self.process_state(line)
            return

        if line.startswith("EVENT|"):
            msg = String()
            msg.data = line
            self.event_pub.publish(msg)
            self.get_logger().info("EV3 event: {}".format(line))
            return

        self.get_logger().warning("Unknown EV3 packet: {}".format(line))

    def process_state(self, line: str) -> None:
        parts = line.split("|")

        if len(parts) != 15:
            self.get_logger().warning(
                "Malformed STATE packet: expected 15 fields, got {}: {}".format(
                    len(parts),
                    line,
                )
            )
            return

        try:
            _sequence = int(parts[1])
            _ev3_ms = int(parts[2])

            base_pos_deg = float(parts[3])
            base_vel_dps = float(parts[4])
            arm_pos_deg = float(parts[5])
            arm_vel_dps = float(parts[6])
            gripper_pos_deg = float(parts[7])
            gripper_vel_dps = float(parts[8])
            conveyor_pos_deg = float(parts[9])
            conveyor_vel_dps = float(parts[10])

            _arm_home = bool(int(parts[11]))
            _base_home = bool(int(parts[12]))
            _color = parts[13]
            _action = parts[14]
        except ValueError as exc:
            self.get_logger().warning(
                "Could not parse STATE packet: {} ({})".format(line, exc)
            )
            return

        base_pos = self.motor_deg_to_joint_rad(
            base_pos_deg,
            self.base_gear,
            self.base_sign,
            self.base_zero,
        )
        arm_pos = self.motor_deg_to_joint_rad(
            arm_pos_deg,
            self.arm_gear,
            self.arm_sign,
            self.arm_zero,
        )
        gripper_pos = self.map_gripper_position(gripper_pos_deg)
        conveyor_pos = self.motor_deg_to_joint_rad(
            conveyor_pos_deg,
            self.conveyor_gear,
            self.conveyor_sign,
        )

        base_vel = self.motor_dps_to_joint_rad_s(
            base_vel_dps,
            self.base_gear,
            self.base_sign,
        )
        arm_vel = self.motor_dps_to_joint_rad_s(
            arm_vel_dps,
            self.arm_gear,
            self.arm_sign,
        )
        gripper_vel = self.map_gripper_velocity(gripper_vel_dps)
        conveyor_vel = self.motor_dps_to_joint_rad_s(
            conveyor_vel_dps,
            self.conveyor_gear,
            self.conveyor_sign,
        )

        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(self.joint_names)
        state.position = [
            base_pos,
            arm_pos,
            gripper_pos,
            conveyor_pos,
        ]
        state.velocity = [
            base_vel,
            arm_vel,
            gripper_vel,
            conveyor_vel,
        ]
        self.joint_state_pub.publish(state)

        conveyor_msg = Float64()
        conveyor_msg.data = conveyor_vel
        self.conveyor_velocity_pub.publish(conveyor_msg)

        raw = String()
        raw.data = line
        self.raw_state_pub.publish(raw)

    def destroy_node(self) -> None:
        self.shutdown_event.set()

        for sock_obj in (self.client, self.server):
            if sock_obj is not None:
                try:
                    sock_obj.close()
                except OSError:
                    pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ev3StateBridge()

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