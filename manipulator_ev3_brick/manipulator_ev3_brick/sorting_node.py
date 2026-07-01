#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray
import subprocess
import threading
import time

# ── Geometry ─────────────────────────────────────────────────────────────────
SPAWN_X    = -0.14824
SPAWN_Y    =  0.29075
SPAWN_Z    =  0.09232
PICKUP_X   = -0.020    # tune: 5mm back from gripper centre
BELT_END_R =  0.17
BELT_END_L = -0.17
STEP_SIZE  =  0.004
STEP_DELAY =  0.04

BALL_CYCLE = ['red', 'blue', 'black', 'green']
BALL_RGB   = {
    'red':   (1, 0, 0),
    'blue':  (0, 0, 1),
    'black': (0.05, 0.05, 0.05),
    'green': (0, 0.8, 0),
}

BALL_SDF = """<sdf version='1.7'>
  <model name='{name}'>
    <link name='link'>
      <inertial>
        <mass>0.05</mass>
        <inertia><ixx>8e-06</ixx><iyy>8e-06</iyy><izz>8e-06</izz></inertia>
      </inertial>
      <collision name='col'>
        <geometry><sphere><radius>0.02</radius></sphere></geometry>
        <surface>
          <friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction>
          <contact><ode><kp>1e5</kp><kd>10</kd></ode></contact>
        </surface>
      </collision>
      <visual name='vis'>
        <geometry><sphere><radius>0.02</radius></sphere></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class SortingNode(Node):
    def __init__(self):
        super().__init__('sorting_node')
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)
        self.ball_count  = 0
        self.cycle_index = 0
        threading.Thread(target=self._start, daemon=True).start()

    # ── Startup ───────────────────────────────────────────────────────────────
    def _start(self):
        time.sleep(2.0)
        self.get_logger().info('Waiting for arm controller...')
        self._arm_client.wait_for_server()
        self.get_logger().info('Ready — starting sort loop')
        self._main_loop()

    # ── Main loop ─────────────────────────────────────────────────────────────
    def _main_loop(self):
        self._arm([0.0, 0.0], secs=2)
        self._grip(0.0)

        while rclpy.ok():
            color = BALL_CYCLE[self.cycle_index % 4]
            self.cycle_index += 1
            self.get_logger().info(f'=== Ball {self.cycle_index}: {color.upper()} ===')

            name = self._spawn(color)
            time.sleep(0.5)

            if color == 'red':
                self._move_ball(name, SPAWN_X, PICKUP_X)
                self._pick_place_left()
            elif color == 'blue':
                self._move_ball(name, SPAWN_X, PICKUP_X)
                self._pick_place_right()
            elif color == 'black':
                self._fall(name, direction=-1)
            elif color == 'green':
                self._fall(name, direction=1)

            time.sleep(0.3)

    # ══════════════════════════════════════════════════════════════════════════
    # PICK & PLACE — RED (LEFT)
    # Sequence: home → pick → grip → home → rotate_left → pitch_down
    #           → open → pitch_up → close → home
    # ══════════════════════════════════════════════════════════════════════════
    def _pick_place_left(self):
        self.get_logger().info('Pick → Place LEFT')

        # 1. Home
        self._arm([0.0, 0.0], secs=1)

        # 2. Lower arm_2 to pick height (arm_1 stays 0)
        self._arm([0.0, -0.17], secs=1)

        # 3. Close gripper
        self._grip(0.19)

        # 4. Back to home (lift arm_2 back up)
        self._arm([0.0, 0.0], secs=1)

        # 5. Rotate arm_1 anticlockwise to -1.5708 (arm_2 stays 0)
        self._arm([-1.5708, 0.0], secs=2)
        time.sleep(0.5)  # brief pause at rotate position

        # 6. Pitch arm_2 down to -0.45
        self._arm([-1.5708, -0.45], secs=1)

        # 7. Open gripper (release ball)
        self._grip(0.0)

        # 8. Pitch arm_2 up (return to 0)
        self._arm([-1.5708, 0.0], secs=1)

        # 9. Close gripper fingers (safe travel position)
        self._grip(0.19)

        # 10. Return arm_1 to home
        self._arm([0.0, 0.0], secs=2)

        # 11. Open gripper at home
        self._grip(0.0)
        self.get_logger().info('Place LEFT complete ✓')

    # ══════════════════════════════════════════════════════════════════════════
    # PICK & PLACE — BLUE (RIGHT)
    # Mirror of left: arm_1 goes to +1.5708
    # ══════════════════════════════════════════════════════════════════════════
    def _pick_place_right(self):
        self.get_logger().info('Pick → Place RIGHT')

        # 1. Home
        self._arm([0.0, 0.0], secs=1)

        # 2. Lower arm_2 to pick height
        self._arm([0.0, -0.17], secs=1)

        # 3. Close gripper
        self._grip(0.19)

        # 4. Back to home
        self._arm([0.0, 0.0], secs=1)

        # 5. Rotate arm_1 clockwise to +1.5708
        self._arm([1.5708, 0.0], secs=2)
        time.sleep(0.5)

        # 6. Pitch arm_2 down to -0.45
        self._arm([1.5708, -0.45], secs=1)

        # 7. Open gripper
        self._grip(0.0)

        # 8. Pitch arm_2 up
        self._arm([1.5708, 0.0], secs=1)

        # 9. Close gripper for travel
        self._grip(0.19)

        # 10. Return arm_1 to home
        self._arm([0.0, 0.0], secs=2)

        # 11. Open at home
        self._grip(0.0)
        self.get_logger().info('Place RIGHT complete ✓')

    # ── Ball movement ─────────────────────────────────────────────────────────
    def _move_ball(self, name, x_start, x_end):
        x = x_start
        while x < x_end:
            x = min(x + STEP_SIZE, x_end)
            self._set_pose(name, x, SPAWN_Y, SPAWN_Z)
            time.sleep(STEP_DELAY)
        self.get_logger().info(f'{name} at pickup zone x={x_end:.4f}')

    def _fall(self, name, direction):
        side = 'LEFT' if direction < 0 else 'RIGHT'
        self.get_logger().info(f'{name} → falling off {side} end')
        x_end = BELT_END_L - 0.06 if direction < 0 else BELT_END_R + 0.06
        x = SPAWN_X
        z = SPAWN_Z
        step = STEP_SIZE * direction
        while (direction > 0 and x < x_end) or (direction < 0 and x > x_end):
            x += step
            if (direction > 0 and x > BELT_END_R) or \
               (direction < 0 and x < BELT_END_L):
                z = max(z - 0.006, -0.05)
            self._set_pose(name, x, SPAWN_Y, z)
            time.sleep(STEP_DELAY)
        self.get_logger().info(f'{name} fell off {side} ✓ (stays in world)')

    def _set_pose(self, name, x, y, z):
        cmd = [
            'ign', 'service', '-s', '/world/empty/set_pose',
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '200',
            '--req',
            f'name: "{name}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}}'
        ]
        try:
            subprocess.run(cmd, capture_output=True, timeout=0.3)
        except subprocess.TimeoutExpired:
            pass

    # ── Arm helpers ───────────────────────────────────────────────────────────
    def _arm(self, positions, secs=2):
        """Send a single-point trajectory and BLOCK until complete."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'arm_1_base_link_joint',
            'left_arm_linkage_arm_2_joint'
        ]
        p = JointTrajectoryPoint()
        p.positions       = [float(v) for v in positions]
        p.velocities      = [0.0, 0.0]
        p.time_from_start = Duration(sec=int(secs))
        goal.trajectory.points = [p]

        future = self._arm_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.02)
        result_future = future.result().get_result_async()
        while not result_future.done():
            time.sleep(0.02)

    def _grip(self, position):
        """Publish gripper command and wait for it to physically move."""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self._gripper_pub.publish(msg)
        time.sleep(0.8)   # physical settle time

    # ── Spawn ─────────────────────────────────────────────────────────────────
    def _spawn(self, color):
        self.ball_count += 1
        name = f'{color}_ball_{self.ball_count}'
        r, g, b = BALL_RGB[color]
        sdf = BALL_SDF.format(name=name, r=r, g=g, b=b)
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create',
               '-name', name,
               '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
               '-string', sdf]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            self.get_logger().info(f'Spawned {name}')
        else:
            self.get_logger().error(f'Spawn failed: {result.stderr[:80]}')
        return name


def main(args=None):
    rclpy.init(args=args)
    node = SortingNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
