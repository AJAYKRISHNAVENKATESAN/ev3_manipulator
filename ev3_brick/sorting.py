#!/usr/bin/env pybricks-micropython
"""Stage-synchronised EV3 sorting controller.

- Full homing once at startup.
- RED / BLUE: pick-and-place, then full arm/base homing.
- GREEN / BLACK: conveyor route only; remain centred.
- Colours may arrive in any order.
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.tools import wait, StopWatch
import math
import socket


# ==================================================
# CONFIGURATION
# ==================================================

ROS2_SERVER_IP = "169.254.35.229"
PORT = 5005
USE_ROS2_SYNC = True
SOCKET_TIMEOUT_SECONDS = 60
MAX_BALLS = 4

ev3 = EV3Brick()

gripper = Motor(Port.A)
arm = Motor(Port.B)
base = Motor(Port.C)
conveyor = Motor(Port.D)

arm_home = TouchSensor(Port.S3)
base_home = TouchSensor(Port.S1)
color_sensor = ColorSensor(Port.S4)

VALID_COLORS = (
    Color.RED,
    Color.BLUE,
    Color.BLACK,
    Color.GREEN,
)

COLOR_MAP = {
    Color.RED: "red",
    Color.BLUE: "blue",
    Color.BLACK: "black",
    Color.GREEN: "green",
}


# ==================================================
# GEOMETRY AND CALIBRATION
# ==================================================

L0 = 40.0
L1 = 50.0
L2 = 95.0
L3 = 185.0

L12 = math.sqrt(
    L1 ** 2
    + L2 ** 2
    - 2 * L1 * L2 * math.cos(math.radians(135))
)
L_ARM = L12 + L3

GEAR_BASE = 3.0
GEAR_ARM = 5.0

# Physical calibration:
# switch -> pickup centre = 11 world degrees = 354 motor degrees.
BASE_HOME_OFFSET_WORLD_DEG = 110
BASE_HOME_OFFSET_MOTOR_DEG = (
    BASE_HOME_OFFSET_WORLD_DEG * GEAR_BASE
)

Z_CLEARANCE = 50
PICK_XZ = (-110, -230)
PLACE_XZ = (-200, -250)

BASE_SPEED = 300
ARM_SPEED = 200
BASE_HOME_SPEED = 170
ARM_HOME_SPEED = 75

RED_BIN_WORLD_DEG = -90
BLUE_BIN_WORLD_DEG = 90
BASE_TARGET_TOLERANCE_MOTOR_DEG = 8

# Positive direction closes; negative relative movement releases.
GRIP_CLOSE_SPEED = 180
GRIP_CLOSE_DUTY_LIMIT = 45
GRIP_RETRY_SPEED = 200
GRIP_RETRY_DUTY_LIMIT = 50
GRIP_RELEASE_SPEED = -90
GRIP_RELEASE_ROTATION_DEG = 90
GRIP_RECOVERY_ROTATION_DEG = 25
MIN_VALID_GRIP_TRAVEL_DEG = 15
GRIP_SETTLE_MS = 350

sock = None


# ==================================================
# NETWORK
# ==================================================

def stage_message(kind, cycle_id, sequence_id, stage):
    return "{}|{}|{}|{}".format(
        kind,
        cycle_id,
        sequence_id,
        stage,
    )


def send_line(text):
    if not USE_ROS2_SYNC:
        return

    if sock is None:
        raise RuntimeError("ROS socket is not connected.")

    sock.sendall((text + "\n").encode("utf-8"))


def read_socket_line():
    chars = []

    while True:
        data = sock.recv(1)

        if not data:
            raise RuntimeError("ROS socket disconnected.")

        char = data.decode("utf-8")

        if char == "\n":
            return "".join(chars).strip()

        chars.append(char)


def wait_for_stage_sync(cycle_id, sequence_id, stage):
    expected = stage_message(
        "STAGE_SYNC_DONE",
        cycle_id,
        sequence_id,
        stage,
    )
    failed_prefix = stage_message(
        "STAGE_SYNC_FAILED",
        cycle_id,
        sequence_id,
        stage,
    )

    while True:
        line = read_socket_line()

        if line == expected:
            return

        if line.startswith(failed_prefix):
            raise RuntimeError(line)


def connect_and_handshake(max_attempts=20):
    global sock

    ev3.speaker.say("Connecting to ROS")

    for attempt in range(max_attempts):
        try:
            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM,
            )
            sock.settimeout(5.0)

            try:
                sock.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1,
                )
            except Exception:
                pass

            sock.connect((ROS2_SERVER_IP, PORT))
            send_line("EV3_CONNECT_REQUEST")

            if read_socket_line() == "ROS_CONNECT_ACCEPT":
                sock.settimeout(SOCKET_TIMEOUT_SECONDS)
                print("[NET] connected")
                ev3.speaker.say("Connected")
                return True

        except Exception as exc:
            print(
                "[NET] attempt {} failed: {}"
                .format(attempt + 1, exc)
            )

        try:
            if sock is not None:
                sock.close()
        except Exception:
            pass

        sock = None
        wait(1000)

    return False


def run_synced_stage(
    cycle_id,
    sequence_id,
    stage,
    hardware_action,
):
    print(
        "[STAGE] {}:{} {} START"
        .format(cycle_id, sequence_id, stage)
    )

    timer = StopWatch()

    if USE_ROS2_SYNC:
        send_line(
            stage_message(
                "STAGE_START",
                cycle_id,
                sequence_id,
                stage,
            )
        )

    hardware_action()
    hardware_ms = timer.time()

    print(
        "[STAGE] {}:{} {} HARDWARE_DONE hardware_ms={}ms"
        .format(cycle_id, sequence_id, stage, hardware_ms)
    )

    if USE_ROS2_SYNC:
        send_line(
            stage_message(
                "STAGE_HW_DONE",
                cycle_id,
                sequence_id,
                stage,
            )
        )
        wait_for_stage_sync(
            cycle_id,
            sequence_id,
            stage,
        )
        total_ms = timer.time()

        print(
            "[STAGE] {}:{} {} SYNC_DONE total_ms={}ms"
            .format(cycle_id, sequence_id, stage, total_ms)
        )

    return sequence_id + 1


# ==================================================
# MOTION
# ==================================================

def inverse_arm(x, z):
    dz = max(min(z - L0, L_ARM), -L_ARM)
    return -math.degrees(math.asin(dz / L_ARM))


def move_base_yaw(world_deg, speed=BASE_SPEED):
    motor_target = world_deg * GEAR_BASE

    base.run_target(
        abs(speed),
        motor_target,
        then=Stop.HOLD,
    )

    motor_actual = base.angle()
    error = motor_actual - motor_target

    print(
        "[BASE] world={} motor_target={} motor_actual={}"
        .format(
            world_deg,
            motor_target,
            motor_actual,
        )
    )

    if abs(error) > BASE_TARGET_TOLERANCE_MOTOR_DEG:
        raise RuntimeError(
            "Base target error: target={} actual={} error={}"
            .format(
                motor_target,
                motor_actual,
                error,
            )
        )


def move_arm_world(world_deg, speed=ARM_SPEED):
    arm.run_target(
        abs(speed),
        world_deg * GEAR_ARM,
        then=Stop.HOLD,
    )


def move_arm_to(x, z, speed=ARM_SPEED):
    move_arm_world(
        inverse_arm(x, z),
        speed=speed,
    )


def move_arm_clearance():
    move_arm_to(0, Z_CLEARANCE)


def move_to_pickup_ready():
    move_arm_clearance()
    move_base_yaw(0)


def move_to_center_hold():
    move_arm_clearance()
    move_base_yaw(0)


def pick_down():
    move_arm_to(PICK_XZ[0], PICK_XZ[1])


def place_down():
    move_arm_to(PLACE_XZ[0], PLACE_XZ[1])


def rotate_red_bin():
    move_base_yaw(RED_BIN_WORLD_DEG)


def rotate_blue_bin():
    move_base_yaw(BLUE_BIN_WORLD_DEG)


# ==================================================
# HOMING
# ==================================================

def sensor_debounced(sensor):
    if not sensor.pressed():
        return False

    wait(50)
    return sensor.pressed()


def home_arm2(timeout_ms=8000):
    watch = StopWatch()
    arm.run(-ARM_HOME_SPEED)

    while not sensor_debounced(arm_home):
        if watch.time() > timeout_ms:
            arm.stop(Stop.BRAKE)
            raise RuntimeError("Arm homing timeout.")

        wait(10)

    arm.stop(Stop.BRAKE)

    arm.run_angle(
        ARM_HOME_SPEED,
        15,
        then=Stop.HOLD,
    )
    arm.reset_angle(0)


def home_base_to_center(timeout_ms=20000):
    watch = StopWatch()
    base.run(BASE_HOME_SPEED)

    while not sensor_debounced(base_home):
        if watch.time() > timeout_ms:
            base.stop(Stop.BRAKE)
            raise RuntimeError("Base homing timeout.")

        wait(10)

    base.stop(Stop.BRAKE)
    wait(150)

    base.run_angle(
        -BASE_HOME_SPEED,
        BASE_HOME_OFFSET_MOTOR_DEG,
        then=Stop.HOLD,
    )
    base.reset_angle(0)


def execute_system_homing():
    home_arm2()
    home_base_to_center()
    print("[HOME] centred; offset=118 world deg / 354 motor deg")


def execute_initial_homing():
    execute_system_homing()


def execute_red_homing():
    execute_system_homing()


def execute_blue_homing():
    execute_system_homing()


# ==================================================
# GRIPPER
# ==================================================

def initialize_gripper():
    # The gripper is already physically open.
    # No countdown, no manual setup prompt, and no encoder reset.
    pass


def gripper_ready_no_motion():
    pass


def grip_attempt(speed, duty_limit):
    start_angle = gripper.angle()

    stall_angle = gripper.run_until_stalled(
        speed,
        then=Stop.HOLD,
        duty_limit=duty_limit,
    )

    return stall_angle - start_angle


def grip_ball_until_stalled():
    travel = grip_attempt(
        GRIP_CLOSE_SPEED,
        GRIP_CLOSE_DUTY_LIMIT,
    )

    if abs(travel) < MIN_VALID_GRIP_TRAVEL_DEG:
        # The mechanism may be sitting at an over-centre/open hard stop.
        # Move slightly in the release direction and retry with more torque.
        gripper.run_angle(
            GRIP_RELEASE_SPEED,
            GRIP_RECOVERY_ROTATION_DEG,
            then=Stop.HOLD,
        )
        wait(200)

        travel = grip_attempt(
            GRIP_RETRY_SPEED,
            GRIP_RETRY_DUTY_LIMIT,
        )

    if abs(travel) < MIN_VALID_GRIP_TRAVEL_DEG:
        raise RuntimeError(
            "Gripper did not close; travel={} deg."
            .format(travel)
        )

    print("[GRIP] closed, travel={} deg".format(travel))
    wait(GRIP_SETTLE_MS)


def release_ball():
    gripper.run_angle(
        GRIP_RELEASE_SPEED,
        GRIP_RELEASE_ROTATION_DEG,
        then=Stop.HOLD,
    )
    wait(GRIP_SETTLE_MS)


# ==================================================
# BALL DETECTION AND CONVEYOR
# ==================================================

def wait_for_ball_color():
    while True:
        color = color_sensor.color()

        if color in VALID_COLORS:
            wait(100)

            if color_sensor.color() == color:
                return color

        wait(50)


def no_hardware_action():
    pass


def conveyor_to_pickup():
    # Increased speed to better keep red/blue balls moving while the arm
    # performs the pickup and placement sequence.
    conveyor.run_angle(
        100,
        -280,
        then=Stop.HOLD,
    )


def conveyor_black():
    conveyor.run_angle(
        120, # speed changed from 100 to 120 to improve black ball movement
        -650,
        then=Stop.HOLD,
    )


def conveyor_green():
    conveyor.run_angle(
        90,
        100,
        then=Stop.HOLD,
    )


# ==================================================
# STATE MACHINE
# ==================================================

def run_initial_home():
    ev3.speaker.say("Initial homing")

    run_synced_stage(
        0,
        1,
        "HOME_INITIAL",
        execute_initial_homing,
    )


def execute_ball_cycle(cycle_id):
    sequence_id = 1

    ev3.speaker.say("Place a ball")

    color = wait_for_ball_color()
    color_name = COLOR_MAP[color]

    ev3.speaker.say(color_name + " ball")

    print(
        "[BALL] cycle={} color={}"
        .format(cycle_id, color_name)
    )

    sequence_id = run_synced_stage(
        cycle_id,
        sequence_id,
        "SPAWN_" + color_name.upper(),
        no_hardware_action,
    )

    if color in (Color.RED, Color.BLUE):
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "CONVEYOR_TO_PICKUP",
            conveyor_to_pickup,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "PICKUP_READY",
            move_to_pickup_ready,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "GRIPPER_READY",
            gripper_ready_no_motion,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "PICK_DOWN",
            pick_down,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "GRIP_CLOSE",
            grip_ball_until_stalled,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "PICK_UP",
            move_arm_clearance,
        )

        if color == Color.RED:
            rotate_stage = "ROTATE_RED"
            rotate_action = rotate_red_bin
            home_stage = "HOME_AFTER_RED"
            home_action = execute_red_homing
        else:
            rotate_stage = "ROTATE_BLUE"
            rotate_action = rotate_blue_bin
            home_stage = "HOME_AFTER_BLUE"
            home_action = execute_blue_homing

        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            rotate_stage,
            rotate_action,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "PLACE_DOWN",
            place_down,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "GRIP_RELEASE",
            release_ball,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "PLACE_UP",
            move_arm_clearance,
        )

        if color == Color.RED:
            ev3.speaker.say("Homing after red")
        else:
            ev3.speaker.say("Homing after blue")

        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            home_stage,
            home_action,
        )

    elif color == Color.GREEN:
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "CONVEYOR_GREEN",
            conveyor_green,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "CENTER_HOLD",
            move_to_center_hold,
        )

    else:
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "CONVEYOR_BLACK",
            conveyor_black,
        )
        sequence_id = run_synced_stage(
            cycle_id,
            sequence_id,
            "CENTER_HOLD",
            move_to_center_hold,
        )

    run_synced_stage(
        cycle_id,
        sequence_id,
        "CYCLE_COMPLETE",
        no_hardware_action,
    )


# ==================================================
# MAIN
# ==================================================

def main():
    if USE_ROS2_SYNC and not connect_and_handshake():
        raise SystemExit("Could not connect to ROS 2.")

    try:
        initialize_gripper()
        run_initial_home()

        for cycle_id in range(1, MAX_BALLS + 1):
            execute_ball_cycle(cycle_id)

        conveyor.stop()

        if USE_ROS2_SYNC:
            send_line("EV3_DONE")

        ev3.speaker.say("All balls sorted")
        print("[DONE] all cycles complete")

    except Exception as exc:
        conveyor.stop()
        arm.stop(Stop.BRAKE)
        base.stop(Stop.BRAKE)
        gripper.stop(Stop.BRAKE)

        print("[ERROR]", exc)
        raise

    finally:
        try:
            if sock is not None:
                sock.close()
        except Exception:
            pass


main()