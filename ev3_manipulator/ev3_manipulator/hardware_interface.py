#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import socket
import threading


class Ev3HardwareInterface(Node):
    def __init__(self):
        super().__init__('ev3_hardware_interface')

        # ---------------- EV3 -> ROS2 publishers ----------------
        self.start_homing_pub = self.create_publisher(String, '/hw/start_homing', 10)
        self.ev3_homed_pub = self.create_publisher(String, '/hw/ev3_homed', 10)
        self.ev3_gripper_open_pub = self.create_publisher(String, '/hw/ev3_gripper_open', 10)

        self.detect_pub = self.create_publisher(String, '/hw/ball_detected', 10)
        self.ready_pickup_pub = self.create_publisher(String, '/hw/ready_pickup', 10)
        self.start_pick_pub = self.create_publisher(String, '/hw/start_pick', 10)
        self.ready_place_pub = self.create_publisher(String, '/hw/ready_place', 10)
        self.start_place_pub = self.create_publisher(String, '/hw/start_place', 10)
        
        self.theta1_pub = self.create_publisher(Float64, '/hw/theta1', 10)
        self.theta2_pub = self.create_publisher(Float64, '/hw/theta2', 10)

        # ---------------- ROS2 sim -> EV3 subscribers ----------------
        self.sim_homed_sub = self.create_subscription(
            String, '/sim/homed', self.sim_homed_callback, 10)

        self.sim_gripper_open_sub = self.create_subscription(
            String, '/sim/gripper_open', self.sim_gripper_open_callback, 10)

        self.spawn_sub = self.create_subscription(
            String, '/sim/spawn_confirmed', self.spawn_confirmed_callback, 10)

        self.sim_ready_pickup_sub = self.create_subscription(
            String, '/sim/ready_pickup', self.sim_ready_pickup_callback, 10)

        self.sim_ready_place_sub = self.create_subscription(
            String, '/sim/ready_place', self.sim_ready_place_callback, 10)

        self.sim_cycle_done_sub = self.create_subscription(
            String, '/sim/cycle_done', self.sim_cycle_done_callback, 10)

        # ---------------- Socket setup ----------------
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', 5005))
        self.server.listen(1)

        self.ev3_client = None
        self.client_lock = threading.Lock()

        threading.Thread(target=self.listen_to_brick, daemon=True).start()

    # ==================================================
    # Utility helpers
    # ==================================================

    def publish_string(self, publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def publish_float(self, publisher, value):
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    def send_to_ev3(self, token):
        with self.client_lock:
            if not self.ev3_client:
                self.get_logger().warn(f"Cannot send [{token}], EV3 is not connected.")
                return

            try:
                self.ev3_client.sendall((token + "\n").encode("utf-8"))
                self.get_logger().info(f"-> EV3 [{token}]")
            except socket.error as e:
                self.get_logger().error(f"Failed to send [{token}] to EV3: {e}")
                self.ev3_client = None

    # ==================================================
    # Socket listener
    # ==================================================

    def listen_to_brick(self):
        while rclpy.ok():
            self.get_logger().info('Awaiting physical layer connection from EV3...')

            client_sock, addr = self.server.accept()
            self.get_logger().info(f'TCP transport layer connected from {addr}')

            # ---------------- Application handshake ----------------
            try:
                client_sock.settimeout(5.0)
                handshake_msg = client_sock.recv(1024).decode('utf-8').strip()

                if handshake_msg == "EV3_CONNECT_REQUEST":
                    self.get_logger().info("Received [EV3_CONNECT_REQUEST].")
                    client_sock.sendall(b"ROS_CONNECT_ACCEPT\n")
                    self.get_logger().info("Sent [ROS_CONNECT_ACCEPT]. Handshake complete.")

                    client_sock.settimeout(None)

                    with self.client_lock:
                        self.ev3_client = client_sock

                else:
                    self.get_logger().error(
                        f"Handshake rejected. Invalid token: '{handshake_msg}'"
                    )
                    client_sock.close()
                    continue

            except (socket.error, socket.timeout) as e:
                self.get_logger().error(f"Application handshake failed: {e}")
                client_sock.close()
                continue

            # ---------------- Operational telemetry phase ----------------
            buffer = ""

            while rclpy.ok():
                with self.client_lock:
                    active_client = self.ev3_client

                if not active_client:
                    break

                try:
                    data = active_client.recv(1024).decode('utf-8')

                    if not data:
                        break

                    buffer += data

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        if line:
                            self.process_ev3_line(line)

                except socket.error as e:
                    self.get_logger().error(f"Socket read error: {e}")
                    break

            with self.client_lock:
                self.ev3_client = None

            try:
                client_sock.close()
            except socket.error:
                pass

            self.get_logger().warn('EV3 disconnected. Resetting server pipeline.')

    # ==================================================
    # EV3 -> ROS2 parser
    # ==================================================

    def process_ev3_line(self, line):
        self.get_logger().info(f"<- EV3 [{line}]")

        if line == "START_HOMING":
            self.publish_string(self.start_homing_pub, "start")

        elif line == "EV3_HOMED":
            self.publish_string(self.ev3_homed_pub, "done")

        elif line == "EV3_GRIPPER_OPEN":
            self.publish_string(self.ev3_gripper_open_pub, "done")

        elif line.startswith("DETECTED:"):
            color = line.split(":", 1)[1].strip().lower()
            self.publish_string(self.detect_pub, color)
            self.get_logger().info(f"Published ball detection: {color}")

        elif line == "READY_PICKUP":
            self.publish_string(self.ready_pickup_pub, "ready")

        elif line == "START_PICK":
            self.publish_string(self.start_pick_pub, "start")

        elif line == "READY_PLACE":
            self.publish_string(self.ready_place_pub, "ready")
        
        elif line == "START_PLACE":
            self.publish_string(self.start_place_pub, "start")

        elif line.startswith("THETA1:"):
            try:
                val = float(line.split(":", 1)[1])
                self.publish_float(self.theta1_pub, val)
            except ValueError:
                self.get_logger().warn(f"Invalid THETA1 packet: {line}")

        elif line.startswith("THETA2:"):
            try:
                val = float(line.split(":", 1)[1])
                self.publish_float(self.theta2_pub, val)
            except ValueError:
                self.get_logger().warn(f"Invalid THETA2 packet: {line}")

        elif line == "EV3_DONE":
            self.get_logger().info("EV3 reported final shutdown.")

        else:
            self.get_logger().warn(f"Unknown EV3 token: {line}")

    # ==================================================
    # ROS2 sim -> EV3 callbacks
    # ==================================================

    def sim_homed_callback(self, msg):
        self.send_to_ev3("ROS_HOMED")

    def sim_gripper_open_callback(self, msg):
        self.send_to_ev3("ROS_GRIPPER_OPEN")

    def spawn_confirmed_callback(self, msg):
        self.send_to_ev3("ACK_SPAWN")

    def sim_ready_pickup_callback(self, msg):
        self.send_to_ev3("ROS_READY_PICKUP")

    def sim_ready_place_callback(self, msg):
        self.send_to_ev3("ROS_READY_PLACE")

    def sim_cycle_done_callback(self, msg):
        self.send_to_ev3("ROS_CYCLE_DONE")


def main(args=None):
    rclpy.init(args=args)
    node = Ev3HardwareInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with node.client_lock:
            if node.ev3_client:
                node.ev3_client.close()

        node.server.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
