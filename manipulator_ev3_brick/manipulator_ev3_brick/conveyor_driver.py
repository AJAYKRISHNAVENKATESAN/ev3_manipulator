# #!/usr/bin/env python3
# """
# conveyor_driver.py
# Applies continuous force to balls on the belt to simulate belt surface motion.
# Subscribe to /conveyor_belt_vel → push registered balls in belt direction (Y axis).
# """
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64, String
# from ros_gz_interfaces.msg import EntityWrench
# from geometry_msgs.msg import Wrench


# BALL_MASS   = 0.05   # kg - must match your SDF
# BELT_DIR_Y  = 1.0    # +Y is belt travel direction in world frame


# class ConveyorDriver(Node):
#     def __init__(self):
#         super().__init__('conveyor_driver')

#         self.belt_vel  = 0.0          # m/s linear belt surface speed
#         self.ball_list = []           # names of balls currently on belt

#         # ── Subscriptions ───────────────────────────────────────────
#         self.create_subscription(
#             Float64, '/conveyor_belt_vel', self._vel_cb, 10)

#         # Publish ball name here to register it: e.g. "red_ball_1"
#         self.create_subscription(
#             String, '/conveyor/add_ball', self._add_ball_cb, 10)

#         # Publish ball name here to remove it when gripper picks it up
#         self.create_subscription(
#             String, '/conveyor/remove_ball', self._remove_ball_cb, 10)

#         # ── Wrench publisher (Ignition Gazebo) ──────────────────────
#         self.wrench_pub = self.create_publisher(
#             EntityWrench, '/world/empty/wrench', 10)

#         # ── Timer: push every 50ms ───────────────────────────────────
#         self.create_timer(0.05, self._push_balls)

#         self.get_logger().info('ConveyorDriver ready. '
#                                'Publish ball name to /conveyor/add_ball to register.')

#     # ── Callbacks ────────────────────────────────────────────────────
#     def _vel_cb(self, msg: Float64):
#         # Belt joint is continuous, pulley radius ≈ 0.025 m
#         # linear_speed = angular_vel × radius
#         self.belt_vel = msg.data * 0.025
#         self.get_logger().debug(f'Belt linear speed: {self.belt_vel:.3f} m/s')

#     def _add_ball_cb(self, msg: String):
#         name = msg.data.strip()
#         if name not in self.ball_list:
#             self.ball_list.append(name)
#             self.get_logger().info(f'Tracking ball: {name}')

#     def _remove_ball_cb(self, msg: String):
#         name = msg.data.strip()
#         if name in self.ball_list:
#             self.ball_list.remove(name)
#             self.get_logger().info(f'Removed ball: {name}')

#     # ── Force application ─────────────────────────────────────────────
#     def _push_balls(self):
#         if not self.ball_list or self.belt_vel == 0.0:
#             return

#         # Force = mass × acceleration
#         # We want the ball to reach belt_vel quickly → use a proportional push
#         # This acts like a friction force from the moving surface
#         force_y = BALL_MASS * self.belt_vel * 10.0  # tunable gain

#         for ball_name in self.ball_list:
#             msg = EntityWrench()
#             msg.entity.name  = ball_name
#             msg.entity.type  = 2          # 2 = MODEL
#             msg.wrench.force.x = 0.0
#             msg.wrench.force.y = force_y * BELT_DIR_Y
#             msg.wrench.force.z = 0.0
#             self.wrench_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ConveyorDriver()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
