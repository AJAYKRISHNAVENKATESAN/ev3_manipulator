#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.tools import wait, StopWatch
import math
import socket as socket

# ==================================================
# INITIALIZATION
# ==================================================

ev3 = EV3Brick()
count = 0

ROS2_SERVER_IP = "169.254.35.229"
PORT = 5005

USE_ROS2_SYNC = True
MAX_BALLS = 4

# Motors
gripper  = Motor(Port.A)
arm      = Motor(Port.B)   # arm2 / pitch
base     = Motor(Port.C)   # arm1 / base yaw
conveyer = Motor(Port.D)

# Sensors
arm_home     = TouchSensor(Port.S3)
base_home    = TouchSensor(Port.S1)
color_sensor = ColorSensor(Port.S4)

VALID_COLORS = [Color.RED, Color.BLUE, Color.BLACK, Color.GREEN]

COLOR_MAP = {
    Color.RED: "red",
    Color.BLUE: "blue",
    Color.BLACK: "black",
    Color.GREEN: "green",
}


# ==================================================
# GEOMETRY / CALIBRATION
# ==================================================

L0 = 40.0
L1 = 50.0
L2 = 95.0
L3 = 185.0

L12 = math.sqrt(L1**2 + L2**2 - 2 * L1 * L2 * math.cos(math.radians(135)))
L_ARM = L12 + L3

GEAR_BASE = 36 / 12       # 3:1
GEAR_ARM  = 40 / 8        # 5:1

# Tuned pickup-center offset.
# Homing:
#   touch base switch
#   move away by 112 world degrees
#   reset encoder to 0
# That final pose becomes pickup center.
BASE_HOME_OFFSET_WORLD_DEG = 112
BASE_HOME_OFFSET_MOTOR_DEG = BASE_HOME_OFFSET_WORLD_DEG * GEAR_BASE

Z_CLEARANCE = 50
PICK_XZ     = (-110, -230)
PLACE_XZ    = (-200, -250)

# Gripper calibration:
#   open-zero = 0
#   wide-open = +10
#   close inward = -60
GRIPPER_OPEN_TARGET = 0
GRIPPER_WIDE_OPEN_TARGET = 10
GRIPPER_CLOSE_TARGET = -60
GRIPPER_SPEED = 60

gripper_state = None
sock = None


# ==================================================
# NETWORK UTILITIES
# ==================================================

def send_line(text):
    global sock

    if USE_ROS2_SYNC and sock:
        print("TX:", text)
        sock.send((text + "\n").encode("utf-8"))


def read_socket_line():
    global sock

    line = ""

    while True:
        char = sock.recv(1).decode("utf-8")

        if char == "\n" or not char:
            break

        line += char

    return line.strip()


def wait_for(token):
    print("Waiting for:", token)

    while True:
        line = read_socket_line()

        if line:
            print("RX:", line)

        if token in line:
            print("Matched:", token)
            return line

        wait(10)


def connect_and_handshake(max_attempts=20):
    global sock

    ev3.speaker.say("Connecting")

    attempt = 0

    while attempt < max_attempts:
        try:
            print("Connection attempt:", attempt + 1)

            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ROS2_SERVER_IP, PORT))

            send_line("EV3_CONNECT_REQUEST")

            response = read_socket_line()
            print("Handshake response:", response)

            if "ROS_CONNECT_ACCEPT" in response:
                ev3.speaker.say("Connected")
                return True

            try:
                sock.close()
            except:
                pass

        except Exception as e:
            print("Connection failed:", e)

            try:
                if sock:
                    sock.close()
            except:
                pass

            wait(1000)

        attempt += 1

    ev3.speaker.say("Connection failed")
    return False


# ==================================================
# KINEMATICS
# ==================================================

def inverse_base(x, y):
    return math.degrees(math.atan2(y, x))


def inverse_arm(x, z):
    dz = z - L0
    dz = max(min(dz, L_ARM), -L_ARM)

    theta = math.asin(dz / L_ARM)

    return -math.degrees(theta)


def move_base_to(x, y, speed=150):
    deg = inverse_base(x, y)

    print("theta1 target for", x, y, "=", deg, "deg")

    if USE_ROS2_SYNC:
        send_line("THETA1:" + str(deg))

    target = deg * GEAR_BASE

    base.run_angle(
        speed,
        target - base.angle(),
        Stop.HOLD
    )

    print("[BASE] encoder angle:", base.angle())


def move_base_yaw(world_deg, speed=150):
    print("theta1 target yaw =", world_deg, "deg")

    if USE_ROS2_SYNC:
        send_line("THETA1:" + str(world_deg))

    target = world_deg * GEAR_BASE

    base.run_angle(
        speed,
        target - base.angle(),
        Stop.HOLD
    )

    print("[BASE] encoder angle:", base.angle())


def move_arm_to(x, z, speed=100):
    deg = inverse_arm(x, z)

    print("theta2 target for", x, z, "=", deg, "deg")

    if USE_ROS2_SYNC:
        send_line("THETA2:" + str(deg))

    target = deg * GEAR_ARM

    arm.run_angle(
        speed,
        target - arm.angle(),
        Stop.HOLD
    )

    print("[ARM2] encoder angle:", arm.angle())


def move_arm_clearance():
    move_arm_to(0, Z_CLEARANCE)


# ==================================================
# SENSOR HELPERS
# ==================================================

def sensor_debounced(sensor):
    if not sensor.pressed():
        return False

    wait(50)

    return sensor.pressed()


# ==================================================
# HOMING
# ==================================================

def home_arm2(speed=60, timeout_ms=8000):
    print("========== HOME ARM2 ==========")
    print("[ARM2 HOME] start angle:", arm.angle())

    ev3.speaker.say("Arm home")

    watch = StopWatch()
    arm.run(-speed)

    while not sensor_debounced(arm_home):
        if watch.time() > timeout_ms:
            arm.stop(Stop.BRAKE)
            ev3.speaker.say("Arm timeout")
            raise SystemExit("ARM2 homing timeout")

        wait(10)

    arm.stop(Stop.BRAKE)
    print("[ARM2 HOME] hit switch angle:", arm.angle())

    arm.run_angle(speed, 15, Stop.HOLD)
    arm.reset_angle(0)

    print("[ARM2 HOME] reset angle:", arm.angle())
    wait(300)


def home_base_to_pickup_center(speed=90, timeout_ms=20000):
    print("========== HOME BASE ==========")
    print("[BASE HOME] start angle:", base.angle())
    print("[BASE HOME] offset world deg:", BASE_HOME_OFFSET_WORLD_DEG)
    print("[BASE HOME] offset motor deg:", BASE_HOME_OFFSET_MOTOR_DEG)

    ev3.speaker.say("Base home")

    watch = StopWatch()
    base.run(speed)

    while not sensor_debounced(base_home):
        if watch.time() > timeout_ms:
            base.stop(Stop.BRAKE)
            ev3.speaker.say("Base timeout")
            raise SystemExit("Base homing timeout")

        wait(10)

    base.stop(Stop.BRAKE)
    print("[BASE HOME] hit switch angle:", base.angle())

    wait(200)

    base.run_angle(-speed, BASE_HOME_OFFSET_MOTOR_DEG, Stop.HOLD)
    base.reset_angle(0)

    print("[BASE HOME] pickup center reset angle:", base.angle())
    wait(300)

    if USE_ROS2_SYNC:
        send_line("THETA1:0.0")


def execute_system_homing():
    home_arm2()
    home_base_to_pickup_center()


# ==================================================
# GRIPPER
# ==================================================

def calibrate_gripper_open_zero():
    global gripper_state

    ev3.speaker.say("Set gripper open")

    print("========== GRIPPER OPEN-ZERO CALIBRATION ==========")
    print("Manually put gripper in OPEN pickup state.")
    print("This state will become gripper encoder angle 0.")

    for i in [5, 4, 3, 2, 1]:
        print("Starting in", i)
        wait(1000)

    gripper.reset_angle(0)
    gripper_state = "open"

    print("[GRIPPER] open-zero set. angle:", gripper.angle())
    wait(300)


def gripper_open():
    global gripper_state

    print("[GRIPPER] open target:", GRIPPER_OPEN_TARGET, "current:", gripper.angle())

    gripper.run_target(
        GRIPPER_SPEED,
        GRIPPER_OPEN_TARGET,
        then=Stop.HOLD
    )

    gripper_state = "open"

    print("[GRIPPER] open done:", gripper.angle())
    wait(300)


def gripper_wide_open():
    global gripper_state

    print("[GRIPPER] wide open target:", GRIPPER_WIDE_OPEN_TARGET, "current:", gripper.angle())

    gripper.run_target(
        GRIPPER_SPEED,
        GRIPPER_WIDE_OPEN_TARGET,
        then=Stop.HOLD
    )

    gripper_state = "open"

    print("[GRIPPER] wide open done:", gripper.angle())
    wait(300)


def gripper_close_for_pickup():
    global gripper_state

    print("[GRIPPER] close target:", GRIPPER_CLOSE_TARGET, "current:", gripper.angle())

    # gripper.run_target(
    #     GRIPPER_SPEED,
    #     GRIPPER_CLOSE_TARGET,
    #     then=Stop.HOLD
    # )

    gripper.run_time(
        -60,
        900,
        then=Stop.HOLD,
        wait=True
    )

    gripper_state = "closed"

    print("[GRIPPER] timed close done:", gripper.angle())
    wait(300)


# ==================================================
# STARTUP + PER-CYCLE HOMING SYNC
# ==================================================

def synchronized_startup():
    """
    Startup only connects to ROS2.
    Homing is done inside each ball cycle.
    """
    if USE_ROS2_SYNC:
        if not connect_and_handshake():
            raise SystemExit("Network connection failed.")

    ev3.speaker.say("Connected")


def synchronized_homing_cycle(first_cycle=False):
    """
    EV3 and sim home before every ball.

    Sequence:
      EV3 -> START_HOMING
      EV3 homes hardware
      EV3 -> EV3_HOMED
      sim homes and replies ROS_HOMED
      EV3 opens gripper
      EV3 -> EV3_GRIPPER_OPEN
      sim opens gripper and replies ROS_GRIPPER_OPEN
    """

    if USE_ROS2_SYNC:
        send_line("START_HOMING")

    ev3.speaker.say("EV3 homing")
    execute_system_homing()
    ev3.speaker.say("EV3 homed")

    if USE_ROS2_SYNC:
        send_line("EV3_HOMED")
        wait_for("ROS_HOMED")

    if first_cycle:
        calibrate_gripper_open_zero()

    gripper_wide_open()

    if USE_ROS2_SYNC:
        send_line("EV3_GRIPPER_OPEN")
        wait_for("ROS_GRIPPER_OPEN")

    ev3.speaker.say("Homing done")


# ==================================================
# COLOR DETECTION
# ==================================================

def wait_for_ball_color():
    while True:
        color = color_sensor.color()

        if color in VALID_COLORS:
            wait(100)

            if color_sensor.color() == color:
                return color

        wait(50)


# ==================================================
# PICKUP STATE
# ==================================================

def move_to_pickup_state_without_rehome():
    move_base_yaw(0)
    move_arm_clearance()
    gripper_wide_open()


# ==================================================
# PICK AND PLACE - OLD STABLE COARSE SYNC
# ==================================================

def pick_and_place(bin_x, bin_y):
    """
    Old stable coarse-sync sequence.

    Sync points:
      READY_PICKUP
      READY_PLACE
      ROS_CYCLE_DONE

    No fine-sync tokens.
    """

    # ---------------- PICKUP READY ----------------
    move_to_pickup_state_without_rehome()

    if USE_ROS2_SYNC:
        send_line("READY_PICKUP")
        wait_for("ROS_READY_PICKUP")

    # ---------------- PICK ----------------
    ev3.speaker.say("Pick")

    move_arm_to(PICK_XZ[0], PICK_XZ[1])
    gripper_close_for_pickup()
    move_arm_clearance()

    # ---------------- PLACE READY ----------------
    if USE_ROS2_SYNC:
        send_line("READY_PLACE")
        wait_for("ROS_READY_PLACE")

    # ---------------- PLACE ----------------
    ev3.speaker.say("Place")

    move_base_to(bin_x, bin_y)
    move_arm_to(PLACE_XZ[0], PLACE_XZ[1])
    gripper_wide_open()
    move_arm_clearance()


def wait_for_ros_cycle_done():
    if USE_ROS2_SYNC:
        wait_for("ROS_CYCLE_DONE")


# ==================================================
# MAIN
# ==================================================

synchronized_startup()

while count < MAX_BALLS:
    # Home both EV3 and sim for every ball cycle.
    synchronized_homing_cycle(first_cycle=(count == 0))

    move_to_pickup_state_without_rehome()

    ev3.speaker.say("Place ball")

    color = wait_for_ball_color()
    color_name = COLOR_MAP[color]

    conveyer.stop()
    ev3.speaker.say(color_name)

    # Send detection immediately after color detection.
    if USE_ROS2_SYNC:
        send_line("DETECTED:" + color_name)
        wait_for("ACK_SPAWN")

    if color == Color.RED:
        # Real ball moves to pickup.
        conveyer.run_angle(80, -280, Stop.HOLD)

        # Red = -90.
        pick_and_place(0, -50)

        wait_for_ros_cycle_done()
        count += 1

    elif color == Color.BLUE:
        # Real ball moves to pickup.
        conveyer.run_angle(80, -280, Stop.HOLD)

        # Blue = +90.
        pick_and_place(0, 50)

        wait_for_ros_cycle_done()
        count += 1

    elif color == Color.BLACK:
        # Black runs down the conveyor.
        conveyer.run_angle(80, -650, Stop.HOLD)

        wait_for_ros_cycle_done()
        count += 1

    elif color == Color.GREEN:
        # Green travels the other way / falls near sensor side.
        conveyer.run_angle(80, 100, Stop.HOLD)

        wait_for_ros_cycle_done()
        count += 1


# ==================================================
# SHUTDOWN
# ==================================================

conveyer.stop()

if USE_ROS2_SYNC:
    send_line("EV3_DONE")

    try:
        sock.close()
    except:
        pass

ev3.speaker.say("All balls sorted")
print("All balls sorted")