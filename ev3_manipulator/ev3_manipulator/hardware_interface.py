import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import socket
import threading

class Ev3HardwareInterface(Node):
    def __init__(self):
        super().__init__('ev3_hardware_interface')

        # ROS2 Communications
        self.detect_pub = self.create_publisher(String, '/hw/ball_detected', 10)
        self.theta1_pub = self.create_publisher(Float64, '/hw/theta1', 10)
        self.theta2_pub = self.create_publisher(Float64, '/hw/theta2', 10)
        self.spawn_sub = self.create_subscription(
            String, '/sim/spawn_confirmed', self.spawn_confirmed_callback, 10)

        # Socket setup
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', 5005))
        self.server.listen(1)
        self.ev3_client = None

        threading.Thread(target=self.listen_to_brick, daemon=True).start()

    def listen_to_brick(self):
        while rclpy.ok():
            self.ev3_client, addr = self.server.accept()
            self.get_logger().info(f'EV3 connected from {addr}')
            buffer = ""
            while self.ev3_client:
                try:
                    data = self.ev3_client.recv(1024).decode('utf-8')
                    if not data:
                        break
                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        if line.startswith("DETECTED:"):
                            color = line.split(":", 1)[1].strip()
                            msg = String()
                            msg.data = color
                            self.detect_pub.publish(msg)
                            self.get_logger().info(f'Ball detected: {color}')
                        elif line.startswith("THETA1:"):
                            try:
                                val = float(line.split(":", 1)[1])
                                msg = Float64()
                                msg.data = val
                                self.theta1_pub.publish(msg)
                                self.get_logger().info(f'theta1 = {val:.2f} deg')
                            except ValueError:
                                pass
                        elif line.startswith("THETA2:"):
                            try:
                                val = float(line.split(":", 1)[1])
                                msg = Float64()
                                msg.data = val
                                self.theta2_pub.publish(msg)
                                self.get_logger().info(f'theta2 = {val:.2f} deg')
                            except ValueError:
                                pass
                except socket.error:
                    break
            self.ev3_client = None
            self.get_logger().warn('EV3 disconnected')

    def spawn_confirmed_callback(self, msg):
        """ When simulation node says ball is successfully created, unblock EV3 """
        if self.ev3_client:
            try:
                self.ev3_client.sendall(b"ACK_SPAWN\n")
                self.get_logger().info("Simulation spawn confirmed. Unblocking physical EV3 brick.")
            except socket.error:
                self.get_logger().error("Failed to send sync ACK to EV3.")

def main(args=None):
    rclpy.init(args=args)
    node = Ev3HardwareInterface()
    rclpy.spin(node)
    rclpy.shutdown()