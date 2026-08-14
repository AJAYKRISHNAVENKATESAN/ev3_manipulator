#!/usr/bin/env python3

"""Minimal TCP <-> ROS 2 bridge for stage-synchronised EV3 execution."""

import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Ev3HardwareInterface(Node):
    def __init__(self):
        super().__init__('ev3_hardware_interface')

        # Every operational EV3 line is forwarded unchanged to the sim node.
        self.stage_event_pub = self.create_publisher(
            String,
            '/hw/stage_event',
            20,
        )

        # Every simulation acknowledgement is forwarded unchanged to the EV3.
        self.stage_sync_sub = self.create_subscription(
            String,
            '/sim/stage_sync',
            self.stage_sync_callback,
            20,
        )

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', 5005))
        self.server.listen(1)
        self.server.settimeout(1.0)

        self.ev3_client = None
        self.client_lock = threading.Lock()
        self.shutdown_event = threading.Event()

        self.listener_thread = threading.Thread(
            target=self.listen_to_brick,
            daemon=True,
        )
        self.listener_thread.start()

    # ==================================================
    # ROS helpers
    # ==================================================

    def publish_stage_event(self, text):
        msg = String()
        msg.data = text
        self.stage_event_pub.publish(msg)

    def stage_sync_callback(self, msg):
        self.send_to_ev3(msg.data.strip())

    # ==================================================
    # Socket helpers
    # ==================================================

    def send_to_ev3(self, token):
        with self.client_lock:
            client = self.ev3_client

        if client is None:
            self.get_logger().warn(
                f"Cannot send [{token}]: EV3 is not connected."
            )
            return

        try:
            client.sendall((token + '\n').encode('utf-8'))
            self.get_logger().info(f"-> EV3 [{token}]")
        except OSError as exc:
            self.get_logger().error(
                f"Failed to send [{token}] to EV3: {exc}"
            )
            self.close_active_client()

    def close_active_client(self):
        with self.client_lock:
            client = self.ev3_client
            self.ev3_client = None

        if client is not None:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

            try:
                client.close()
            except OSError:
                pass

    # ==================================================
    # Listener and parser
    # ==================================================

    def listen_to_brick(self):
        while rclpy.ok() and not self.shutdown_event.is_set():
            self.get_logger().info(
                'Awaiting physical layer connection from EV3...'
            )

            try:
                client_sock, addr = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.get_logger().info(
                f'TCP transport layer connected from {addr}'
            )

            try:
                client_sock.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1,
                )
                client_sock.settimeout(5.0)

                handshake = self.read_one_line(client_sock)

                if handshake != 'EV3_CONNECT_REQUEST':
                    self.get_logger().error(
                        f"Handshake rejected. Invalid token: '{handshake}'"
                    )
                    client_sock.close()
                    continue

                client_sock.sendall(b'ROS_CONNECT_ACCEPT\n')
                client_sock.settimeout(None)

                with self.client_lock:
                    self.ev3_client = client_sock

                self.get_logger().info(
                    'Application handshake completed.'
                )

                self.receive_operational_lines(client_sock)

            except (OSError, socket.timeout, RuntimeError) as exc:
                self.get_logger().error(
                    f'EV3 connection ended with error: {exc}'
                )

            finally:
                self.close_active_client()
                self.get_logger().warn(
                    'EV3 disconnected. Resetting server pipeline.'
                )

    @staticmethod
    def read_one_line(client_sock):
        chars = []

        while True:
            data = client_sock.recv(1)

            if not data:
                raise RuntimeError('Socket closed while reading a line.')

            char = data.decode('utf-8')

            if char == '\n':
                return ''.join(chars).strip()

            chars.append(char)

    def receive_operational_lines(self, client_sock):
        buffer = ''

        while rclpy.ok() and not self.shutdown_event.is_set():
            data = client_sock.recv(1024)

            if not data:
                return

            buffer += data.decode('utf-8')

            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()

                if line:
                    self.process_ev3_line(line)

    def process_ev3_line(self, line):
        self.get_logger().info(f"<- EV3 [{line}]")

        if line.startswith('STAGE_START|'):
            self.publish_stage_event(line)
            return

        if line.startswith('STAGE_HW_DONE|'):
            self.publish_stage_event(line)
            return

        if line == 'EV3_DONE':
            self.publish_stage_event(line)
            self.get_logger().info('EV3 reported final shutdown.')
            return

        self.get_logger().warn(f'Unknown EV3 token: {line}')

    def destroy_node(self):
        self.shutdown_event.set()
        self.close_active_client()

        try:
            self.server.close()
        except OSError:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ev3HardwareInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()