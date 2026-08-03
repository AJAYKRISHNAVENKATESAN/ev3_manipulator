#!/usr/bin/env pybricks-micropython
"""EV3 sorter controller with continuous state streaming.

The EV3 remains the task controller and source of truth. It no longer waits
for Gazebo to finish a duplicated stage. Instead, it streams measured motor
encoder states and semantic events to ROS 2. Gazebo mirrors those states.
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
NETWORK_REQUIRED_AT_START = True
TELEMETRY_PERIOD_MS = 50  # 20 Hz
SOCKET_TIMEOUT_SECONDS = 2
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

# IMPORTANT: verify this value physically. The older file had conflicting
# comments/logs for 110, 118, and 354 motor degrees.
BASE_HOME_OFFSET_WORLD_DEG = 110.0
BASE_HOME_OFFSET_MOTOR_DEG = BASE_HOME_OFFSET_WORLD_DEG * GEAR_BASE

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

GRIP_CLOSE_SPEED = 180
GRIP_CLOSE_DUTY_LIMIT = 45
GRIP_RETRY_SPEED = 200
GRIP_RETRY_DUTY_LIMIT = 50
GRIP_RELEASE_SPEED = -90
GRIP_RELEASE_ROTATION_DEG = 90
GRIP_RECOVERY_ROTATION_DEG = 25
MIN_VALID_GRIP_TRAVEL_DEG = 15
GRIP_SETTLE_MS = 350


# ==================================================
# NETWORK AND TELEMETRY
# ==================================================

sock = None
telemetry_clock = StopWatch()
last_telemetry_ms = -TELEMETRY_PERIOD_MS
state_sequence = 0
active_action = "IDLE"
current_color_name = "none"


def close_socket():
    global sock

    if sock is None:
        return

    try:
        sock.close()
    except Exception:
        pass

    sock = None


def send_line(text):
    """Best-effort send; hardware motion does not wait for the simulator."""
    global sock

    if sock is None:
        return False

    try:
        sock.sendall((text + "\n").encode("utf-8"))
        return True
    except Exception as exc:
        print("[NET] send failed: {}".format(exc))
        close_socket()
        return False


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


def connect_and_handshake(max_attempts=20):
    global sock

    ev3.speaker.say("Connecting to ROS")

    for attempt in range(max_attempts):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)

            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except Exception:
                pass

            sock.connect((ROS2_SERVER_IP, PORT))
            sock.sendall(b"EV3_CONNECT_REQUEST\n")

            if read_socket_line() == "ROS_CONNECT_ACCEPT":
                sock.settimeout(SOCKET_TIMEOUT_SECONDS)
                print("[NET] connected")
                ev3.speaker.say("Connected")
                return True

        except Exception as exc:
            print(
                "[NET] attempt {} failed: {}".format(
                    attempt + 1,
                    exc,
                )
            )

        close_socket()
        wait(1000)

    return False


def color_name_or_none():
    color = color_sensor.color()
    return COLOR_MAP.get(color, "none")


def publish_state(force=False):
    """Send measured motor state, sensor state, and current action.

    Wire format:
      STATE|seq|ev3_ms|
      base_pos_deg|base_vel_dps|
      arm_pos_deg|arm_vel_dps|
      gripper_pos_deg|gripper_vel_dps|
      conveyor_pos_deg|conveyor_vel_dps|
      arm_home|base_home|color|action
    """
    global last_telemetry_ms, state_sequence

    now_ms = telemetry_clock.time()

    if not force and now_ms - last_telemetry_ms < TELEMETRY_PERIOD_MS:
        return

    last_telemetry_ms = now_ms
    state_sequence += 1

    line = (
        "STATE|{}|{}|{:.3f}|{:.3f}|{:.3f}|{:.3f}|"
        "{:.3f}|{:.3f}|{:.3f}|{:.3f}|{}|{}|{}|{}"
    ).format(
        state_sequence,
        now_ms,
        base.angle(),
        base.speed(),
        arm.angle(),
        arm.speed(),
        gripper.angle(),
        gripper.speed(),
        conveyor.angle(),
        conveyor.speed(),
        1 if arm_home.pressed() else 0,
        1 if base_home.pressed() else 0,
        current_color_name,
        active_action,
    )

    send_line(line)


def send_event(cycle_id, event_name, value=""):
    safe_value = str(value).replace("|", "/").replace("\n", " ")
    send_line(
        "EVENT|{}|{}|{}".format(
            cycle_id,
            event_name,
            safe_value,
        )
    )


def wait_with_telemetry(duration_ms):
    timer = StopWatch()

    while timer.time() < duration_ms:
        publish_state()
        wait(min(TELEMETRY_PERIOD_MS, duration_ms - timer.time()))

    publish_state(force=True)


def wait_for_motor_done(motor, timeout_ms, label):
    timer = StopWatch()

    while not motor.control.done():
        publish_state()

        if timer.time() > timeout_ms:
            motor.brake()
            raise RuntimeError("{} motion timeout.".format(label))

        wait(TELEMETRY_PERIOD_MS)

    publish_state(force=True)


def run_target_streamed(
    motor,
    speed,
    target_angle,
    label,
    timeout_ms=20000,
    then=Stop.HOLD,
):
    motor.run_target(
        abs(speed),
        target_angle,
        then=then,
        wait=False,
    )
    wait_for_motor_done(motor, timeout_ms, label)


def run_angle_streamed(
    motor,
    speed,
    rotation_angle,
    label,
    timeout_ms=20000,
    then=Stop.HOLD,
):
    motor.run_angle(
        speed,
        rotation_angle,
        then=then,
        wait=False,
    )
    wait_for_motor_done(motor, timeout_ms, label)


def run_action(cycle_id, action_name, hardware_action):
    """Execute hardware and emit observability events; never wait for Gazebo."""
    global active_action

    active_action = action_name
    send_event(cycle_id, "ACTION_START", action_name)
    publish_state(force=True)

    timer = StopWatch()

    try:
        hardware_action()
    except Exception as exc:
        send_event(cycle_id, "ACTION_FAILED", "{}:{}".format(action_name, exc))
        raise
    finally:
        publish_state(force=True)

    elapsed_ms = timer.time()
    send_event(
        cycle_id,
        "ACTION_DONE",
        "{}:{}".format(action_name, elapsed_ms),
    )

    active_action = "IDLE"
    publish_state(force=True)


# ==================================================
# MOTION
# ==================================================


def inverse_arm(x, z):
    dz = max(min(z - L0, L_ARM), -L_ARM)
    return -math.degrees(math.asin(dz / L_ARM))


def move_base_yaw(world_deg, speed=BASE_SPEED):
    motor_target = world_deg * GEAR_BASE

    run_target_streamed(
        base,
        speed,
        motor_target,
        "base target",
    )

    motor_actual = base.angle()
    error = motor_actual - motor_target

    print(
        "[BASE] world={} motor_target={} motor_actual={}".format(
            world_deg,
            motor_target,
            motor_actual,
        )
    )

    if abs(error) > BASE_TARGET_TOLERANCE_MOTOR_DEG:
        raise RuntimeError(
            "Base target error: target={} actual={} error={}".format(
                motor_target,
                motor_actual,
                error,
            )
        )


def move_arm_world(world_deg, speed=ARM_SPEED):
    run_target_streamed(
        arm,
        speed,
        world_deg * GEAR_ARM,
        "arm target",
    )


def move_arm_to(x, z, speed=ARM_SPEED):
    move_arm_world(inverse_arm(x, z), speed=speed)


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

    wait_with_telemetry(50)
    return sensor.pressed()


def home_arm2(timeout_ms=8000):
    watch = StopWatch()
    arm.run(-ARM_HOME_SPEED)

    while not sensor_debounced(arm_home):
        publish_state()

        if watch.time() > timeout_ms:
            arm.brake()
            raise RuntimeError("Arm homing timeout.")

        wait(10)

    arm.brake()

    run_angle_streamed(
        arm,
        ARM_HOME_SPEED,
        15,
        "arm home release",
    )
    arm.reset_angle(0)
    publish_state(force=True)


def home_base_to_center(timeout_ms=20000):
    watch = StopWatch()
    base.run(BASE_HOME_SPEED)

    while not sensor_debounced(base_home):
        publish_state()

        if watch.time() > timeout_ms:
            base.brake()
            raise RuntimeError("Base homing timeout.")

        wait(10)

    base.brake()
    wait_with_telemetry(150)

    run_angle_streamed(
        base,
        -BASE_HOME_SPEED,
        BASE_HOME_OFFSET_MOTOR_DEG,
        "base home offset",
    )
    base.reset_angle(0)
    publish_state(force=True)


def execute_system_homing():
    home_arm2()
    home_base_to_center()
    print(
        "[HOME] centred; configured offset={} world deg / {} motor deg".format(
            BASE_HOME_OFFSET_WORLD_DEG,
            BASE_HOME_OFFSET_MOTOR_DEG,
        )
    )


# ==================================================
# GRIPPER
# ==================================================


def initialize_gripper():
    # This assumes the gripper is physically open at startup. A deterministic
    # encoder zero is necessary for state-to-joint conversion in the twin.
    gripper.reset_angle(0)
    conveyor.reset_angle(0)
    publish_state(force=True)


def gripper_ready_no_motion():
    publish_state(force=True)


def grip_attempt(speed, duty_limit):
    start_angle = gripper.angle()
    publish_state(force=True)

    stall_angle = gripper.run_until_stalled(
        speed,
        then=Stop.HOLD,
        duty_limit=duty_limit,
    )

    publish_state(force=True)
    return stall_angle - start_angle


def grip_ball_until_stalled():
    travel = grip_attempt(
        GRIP_CLOSE_SPEED,
        GRIP_CLOSE_DUTY_LIMIT,
    )

    if abs(travel) < MIN_VALID_GRIP_TRAVEL_DEG:
        run_angle_streamed(
            gripper,
            GRIP_RELEASE_SPEED,
            GRIP_RECOVERY_ROTATION_DEG,
            "gripper recovery",
        )
        wait_with_telemetry(200)

        travel = grip_attempt(
            GRIP_RETRY_SPEED,
            GRIP_RETRY_DUTY_LIMIT,
        )

    if abs(travel) < MIN_VALID_GRIP_TRAVEL_DEG:
        raise RuntimeError(
            "Gripper did not close; travel={} deg.".format(travel)
        )

    print("[GRIP] closed, travel={} deg".format(travel))
    wait_with_telemetry(GRIP_SETTLE_MS)


def release_ball():
    run_angle_streamed(
        gripper,
        GRIP_RELEASE_SPEED,
        GRIP_RELEASE_ROTATION_DEG,
        "gripper release",
    )
    wait_with_telemetry(GRIP_SETTLE_MS)


# ==================================================
# BALL DETECTION AND CONVEYOR
# ==================================================


def wait_for_ball_color():
    global current_color_name

    while True:
        publish_state()
        color = color_sensor.color()

        if color in VALID_COLORS:
            wait_with_telemetry(100)

            if color_sensor.color() == color:
                current_color_name = COLOR_MAP[color]
                publish_state(force=True)
                return color

        wait(50)


def no_hardware_action():
    publish_state(force=True)


def conveyor_to_pickup():
    run_angle_streamed(
        conveyor,
        100,
        -280,
        "conveyor to pickup",
    )


def conveyor_black():
    run_angle_streamed(
        conveyor,
        120,
        -650,
        "conveyor black",
    )


def conveyor_green():
    run_angle_streamed(
        conveyor,
        90,
        100,
        "conveyor green",
    )


# ==================================================
# STATE MACHINE
# ==================================================


def run_initial_home():
    ev3.speaker.say("Initial homing")
    run_action(0, "HOME_INITIAL", execute_system_homing)


def execute_ball_cycle(cycle_id):
    global current_color_name

    ev3.speaker.say("Place a ball")

    color = wait_for_ball_color()
    color_name = COLOR_MAP[color]

    ev3.speaker.say(color_name + " ball")
    print("[BALL] cycle={} color={}".format(cycle_id, color_name))

    send_event(cycle_id, "BALL_DETECTED", color_name)

    if color in (Color.RED, Color.BLUE):
        run_action(cycle_id, "CONVEYOR_TO_PICKUP", conveyor_to_pickup)
        run_action(cycle_id, "PICKUP_READY", move_to_pickup_ready)
        run_action(cycle_id, "GRIPPER_READY", gripper_ready_no_motion)
        run_action(cycle_id, "PICK_DOWN", pick_down)
        run_action(cycle_id, "GRIP_CLOSE", grip_ball_until_stalled)
        send_event(cycle_id, "GRIPPER_CLOSED", color_name)
        run_action(cycle_id, "PICK_UP", move_arm_clearance)

        if color == Color.RED:
            run_action(cycle_id, "ROTATE_RED", rotate_red_bin)
        else:
            run_action(cycle_id, "ROTATE_BLUE", rotate_blue_bin)

        run_action(cycle_id, "PLACE_DOWN", place_down)
        run_action(cycle_id, "GRIP_RELEASE", release_ball)
        send_event(cycle_id, "GRIPPER_OPENED", color_name)
        run_action(cycle_id, "PLACE_UP", move_arm_clearance)

        if color == Color.RED:
            run_action(cycle_id, "HOME_AFTER_RED", execute_system_homing)
        else:
            run_action(cycle_id, "HOME_AFTER_BLUE", execute_system_homing)

    elif color == Color.GREEN:
        run_action(cycle_id, "CONVEYOR_GREEN", conveyor_green)
        run_action(cycle_id, "CENTER_HOLD", move_to_center_hold)

    else:
        run_action(cycle_id, "CONVEYOR_BLACK", conveyor_black)
        run_action(cycle_id, "CENTER_HOLD", move_to_center_hold)

    send_event(cycle_id, "CYCLE_COMPLETE", color_name)
    current_color_name = "none"
    publish_state(force=True)


# ==================================================
# MAIN
# ==================================================


def main():
    connected = connect_and_handshake()

    if NETWORK_REQUIRED_AT_START and not connected:
        raise SystemExit("Could not connect to ROS 2.")

    try:
        initialize_gripper()
        run_initial_home()

        for cycle_id in range(1, MAX_BALLS + 1):
            execute_ball_cycle(cycle_id)

        conveyor.stop()
        send_event(0, "TASK_COMPLETE", MAX_BALLS)
        publish_state(force=True)

        ev3.speaker.say("All balls sorted")
        print("[DONE] all cycles complete")

    except Exception as exc:
        conveyor.brake()
        arm.brake()
        base.brake()
        gripper.brake()

        send_event(0, "FAULT", exc)
        publish_state(force=True)

        print("[ERROR]", exc)
        raise

    finally:
        close_socket()


main()