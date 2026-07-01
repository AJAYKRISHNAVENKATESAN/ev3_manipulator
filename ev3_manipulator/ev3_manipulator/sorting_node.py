#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64, Float64MultiArray
import subprocess
import threading
import time
import socket

# ── Geometry & Spatial Anchors ────────────────────────────────────────────────
SPAWN_X    = -0.14824
SPAWN_Y    =  0.29075
SPAWN_Z    =  0.09232
PICKUP_Z   =  SPAWN_Z + 0.022 

PICKUP_X    = -0.015
TRANSPORT_Z = SPAWN_Z + 0.005  
BELT_END_R  =  0.17
BELT_END_L  = -0.17

STEP_SIZE   =  0.008   
STEP_DELAY  =  0.090   

THETA1_MIN = -math.pi / 2
THETA1_MAX =  math.pi / 2
THETA2_MIN = -0.55             
THETA2_MAX =  math.pi / 3

ARM_HOME   = [0.0, 0.2]        

BALL_RGB   = {
    'red':   (1,    0,    0   ),
    'blue':  (0,    0,    1   ),
    'black': (0.05, 0.05, 0.05),
    'green': (0,    0.8,  0   ),
}

BALL_SDF = """<sdf version='1.7'>
  <model name='{name}'>
    <link name='link'>
      <inertial>
        <mass>0.05</mass>
        <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
      </inertial>
      <collision name='col'>
        <geometry><sphere><radius>0.0185</radius></sphere></geometry>
        <surface>
          <friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction>
        </surface>
      </collision>
      <visual name='vis'>
        <geometry><sphere><radius>0.0185</radius></sphere></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

def solve_ik_sim(x, y, z_target):
    theta1 = math.atan2(x, y)
    Z0 = 0.220995          
    R_ARM = 0.226963  
    sin_t2 = (z_target - Z0) / R_ARM
    sin_t2 = max(-1.0, min(1.0, sin_t2))  
    theta2 = math.asin(sin_t2)
    return max(THETA1_MIN, min(THETA1_MAX, theta1)), max(THETA2_MIN, min(THETA2_MAX, theta2))

class SortingNode(Node):
    def __init__(self):
        super().__init__('sorting1_node')
        self._arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self._gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self._belt_vel_pub = self.create_publisher(Float64, '/conveyor_belt_vel', 10)
        
        self.ball_count  = 0
        
        # Thread Synchronization Tools for HIL Interlocks
        self._detected_color = None
        self._color_event = threading.Event()
        self._pickup_event = threading.Event()
        self._place_event = threading.Event()

        # Socket Server Setup binding to all system interfaces
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', 5005))
        self.server.listen(1)
        self.ev3_client = None

        threading.Thread(target=self._start, daemon=True).start()

    def _start(self):
        time.sleep(2.0)
        self._arm_client.wait_for_server()
        self._send_trajectory([ARM_HOME], [2.0])
        self._grip(0.0)
        
        self.get_logger().info('Listening for EV3 on port 5005...')
        client_sock, addr = self.server.accept()
        self.get_logger().info(f'TCP transport connection established from {addr}')
        
        try:
            client_sock.settimeout(5.0)
            handshake_msg = client_sock.recv(1024).decode('utf-8').strip()
            if handshake_msg == "EV3_CONNECT_REQUEST":
                client_sock.sendall(b"ROS_CONNECT_ACCEPT\n")
                self.get_logger().info('Application Handshake Completed Successfully! ✓')
                client_sock.settimeout(None)
                self.ev3_client = client_sock
                
                # Start background hardware listener thread
                threading.Thread(target=self._listen_to_ev3, daemon=True).start()
            else:
                self.get_logger().error("Handshake rejected. Invalid header token.")
                client_sock.close()
                return
        except (socket.error, socket.timeout) as e:
            self.get_logger().error(f"Handshake pipeline failure: {e}")
            client_sock.close()
            return

        self._main_loop()

    def _listen_to_ev3(self):
        """ Background thread parser that isolates streaming socket I/O from execution timelines """
        network_buffer = ""
        while rclpy.ok() and self.ev3_client:
            try:
                data = self.ev3_client.recv(1024).decode('utf-8')
                if not data:
                    self.get_logger().error("Physical EV3 connection dropped cleanly by remote host.")
                    self.ev3_client = None
                    break
                
                network_buffer += data
                while "\n" in network_buffer:
                    line, network_buffer = network_buffer.split("\n", 1)
                    line = line.strip()
                    self._process_incoming_line(line)
            except socket.error as e:
                self.get_logger().error(f"Socket transport read fault: {e}")
                self.ev3_client = None
                break

    def _process_incoming_line(self, line):
        if line.startswith("DETECTED:"):
            self._detected_color = line.split(":", 1)[1].strip()
            self._color_event.set()
        elif line == "READY_PICKUP":
            self.get_logger().info("Received [READY_PICKUP] confirmation from hardware.")
            self._pickup_event.set()
        elif line == "READY_PLACE":
            self.get_logger().info("Received [READY_PLACE] confirmation from hardware.")
            self._place_event.set()
        elif line.startswith("THETA1:"):
            try:
                val = float(line.split(":", 1)[1])
                self.get_logger().info(f"[EV3 Telemetry] Joint 1 Space: {val:.2f}°")
            except ValueError: pass
        elif line.startswith("THETA2:"):
            try:
                val = float(line.split(":", 1)[1])
                self.get_logger().info(f"[EV3 Telemetry] Joint 2 Space: {val:.2f}°")
            except ValueError: pass

    def _main_loop(self):
        self.start_conveyor(0.05)
        while self.ball_count < 4 and self.ev3_client:
            self.get_logger().info('Awaiting hardware sensor trigger stream...')
            
            # Block safely until background listener receives a color string
            self._color_event.wait()
            if not self.ev3_client: break
            color = self._detected_color
            self._color_event.clear()

            self.get_logger().info(f'=== Ball {self.ball_count + 1}: {color.upper()} ===')

            name = self._spawn(color)
            time.sleep(0.1)

            # Release EV3 feeder arm obstacle block
            self.ev3_client.sendall(b"ACK_SPAWN\n")

            if color == 'red':
                self._move_ball(name, SPAWN_X, PICKUP_X)
                self._pick_place_sequence(left_side=True)
            elif color == 'blue':
                self._move_ball(name, SPAWN_X, PICKUP_X)
                self._pick_place_sequence(left_side=False)
            elif color in ['black', 'green']:
                self._fall(name, direction=-1 if color == 'black' else 1)
                time.sleep(1.5)

        self.stop_conveyor()
        self._send_trajectory([ARM_HOME], [2.0])
        self._grip(0.0)
        self.get_logger().info('=== Execution Cycle Complete ===')

    def _pick_place_sequence(self, left_side=True):
        target_yaw = -1.5708 if left_side else 1.5708
        _, pick_pitch = solve_ik_sim(PICKUP_X, SPAWN_Y, PICKUP_Z)

        # 1. Pre-open fingers and descend to pickup location
        self._grip(0.5)
        time.sleep(0.3)
        self._send_trajectory(positions=[[0.0, pick_pitch]], durations=[0.8])
        time.sleep(0.9)

        # ── INTERLOCK POINT 1: PICKUP POSTURE CONFIRMATION ────────────────
        self.get_logger().info("Holding pose. Waiting for physical EV3 to match position...")
        self._pickup_event.wait()
        self._pickup_event.clear()
        
        # Simultaneously trigger physical grab and virtual twin closure
        self.ev3_client.sendall(b"ROS_READY_PICKUP\n")
        self.get_logger().info("Sent [ROS_READY_PICKUP]. Closing gripper jaws.")
        self._grip(0.0)
        time.sleep(0.5)
        
        # Lift and rotate toward sortation bins
        self._send_trajectory(positions=[[0.0, 0.2]], durations=[0.8])
        time.sleep(0.9)
        self._send_trajectory(positions=[[target_yaw, 0.2]], durations=[1.5])
        time.sleep(1.6)

        # Descend to drop height
        self._send_trajectory(positions=[[target_yaw, -0.45]], durations=[1.5])
        time.sleep(1.1)
        self._grip(0.15) # Center crack
        time.sleep(0.4)

        # ── INTERLOCK POINT 2: PLACE POSTURE CONFIRMATION ─────────────────
        self.get_logger().info("Holding drop pose. Waiting for physical EV3 alignment...")
        self._place_event.wait()
        self._place_event.clear()
        
        # Release object on both targets simultaneously
        self.ev3_client.sendall(b"ROS_READY_PLACE\n")
        self.get_logger().info("Sent [ROS_READY_PLACE]. Releasing target payload.")
        self._grip(0.5)
        time.sleep(2.0)

        # 6. Recoil upward wide open, clear sorting boundary, wrap home
        self._send_trajectory(positions=[[target_yaw, 0.2]], durations=[1.2])
        time.sleep(1.3)
        self._grip(0.0)
        time.sleep(0.6)
        self._send_trajectory(positions=[ARM_HOME], durations=[1.5])
        time.sleep(1.6)

    def _send_trajectory(self, positions, durations):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['arm_1_base_link_joint', 'arm_2_left_arm_linkage_joint']
        for pos, t in zip(positions, durations):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in pos]
            p.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            goal.trajectory.points.append(p)

        future = self._arm_client.send_goal_async(goal)
        while not future.done(): time.sleep(0.01)
        result_future = future.result().get_result_async()
        while not result_future.done(): time.sleep(0.01)

    def _move_ball(self, name, x_start, x_end):
        x = x_start
        while x < x_end:
            x = min(x + STEP_SIZE, x_end)
            self._set_pose(name, x, SPAWN_Y, TRANSPORT_Z)
            time.sleep(STEP_DELAY)

    def _fall(self, name, direction):
        x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
        x, z = SPAWN_X, TRANSPORT_Z
        step = STEP_SIZE * direction
        while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
            x += step
            if (direction > 0 and x > BELT_END_R) or (direction < 0 and x < BELT_END_L):
                z = max(z - 0.006, -0.05)
            self._set_pose(name, x, SPAWN_Y, z)
            time.sleep(STEP_DELAY)

    def _set_pose(self, name, x, y, z):
        cmd = [
            'ign', 'service', '-s', '/world/empty/set_pose',
            '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '150', '--req',
            f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
        ]
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _grip(self, position):
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self._gripper_pub.publish(msg)

    def _spawn(self, color, retries=3):
        self.ball_count += 1
        name    = f'{color}_ball_{self.ball_count}'
        r, g, b = BALL_RGB.get(color, (0.5, 0.5, 0.5))
        sdf     = BALL_SDF.format(name=name, r=r, g=g, b=b)
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', name, '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z), '-string', sdf
        ]
        for attempt in range(retries):
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                if result.returncode == 0: return name
            except subprocess.TimeoutExpired: time.sleep(1.0)
        return name

    def start_conveyor(self, speed=0.05):
        msg = Float64()
        msg.data = float(speed)
        self._belt_vel_pub.publish(msg)

    def stop_conveyor(self):
        self.start_conveyor(0.0)

def main(args=None):
    rclpy.init(args=args)
    node = SortingNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_conveyor()
        rclpy.shutdown()

if __name__ == '__main__':
    main()