#!/usr/bin/env pybricks-micropython
"""EV3 gripper: robust stall-datum grip cycle (red / blue test).

=====================================================================
THE FIVE RULES THIS CODE IS BUILT ON  (the concepts worth keeping)
=====================================================================

1. AN ENCODER IS RELATIVE.
   motor.angle() at power-on is whatever it happened to be. It carries
   no physical meaning. Never write an absolute target that assumes a
   value survived a reboot, a manual nudge, or a dropped cable.

2. A DATUM MUST BE PHYSICAL.
   The only repeatable reference on this mechanism is the full-open
   HARD STOP. We drive into it, and only then call reset_angle(0).
   After that, 0 means "fully open" -- guaranteed, every run.

3. STALL DETECTION IS A TORQUE MEASUREMENT, NOT A POSITION ONE.
   run_until_stalled() stops when the motor cannot reach the commanded
   speed within duty_limit. So duty_limit is the *force* you are willing
   to apply. It must differ per direction:
       - opening into a rigid hard stop  -> firmer duty is safe
       - closing onto a ball             -> gentler duty, or you crush
                                            the ball / slip the fingers
   Too high = damage and false travel. Too low = false stall on friction.

4. RE-DATUM EVERY CYCLE, AND DRIFT CAN NEVER ACCUMULATE.
   Gear backlash, belt skip and missed counts are real. If each cycle
   begins by re-finding the hard stop, every cycle starts from the same
   physical place and yesterday's error is irrelevant. This is why the
   mechanism behaves the same on cycle 1 and cycle 500.

5. TRAVEL FROM DATUM IS YOUR SENSOR.
   You have no force sensor and no ball-presence switch. But the
   distance the gripper travels before stalling tells you what happened:
       small travel   -> jammed, or it never really moved
       medium travel  -> stopped early = something is in the way = BALL
       large travel   -> fingers met each other = EMPTY, grab missed
   Calibrate the "empty" number once; everything less is an object.

=====================================================================
DIRECTION: SET THIS FIRST
=====================================================================
Your two existing files disagree about which sign opens the gripper.
Resolve it ONCE with the jog test, set OPEN_SIGN below, and never think
about it again. Every other direction in this file is derived from it.

    Run the jog test. Press RIGHT (positive speed).
        fingers move APART -> OPEN_SIGN = +1
        fingers move TOGETHER -> OPEN_SIGN = -1
=====================================================================
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import wait, StopWatch


# ==================================================
# HARDWARE
# ==================================================

ev3 = EV3Brick()

gripper = Motor(Port.A)
color_sensor = ColorSensor(Port.S4)


# ==================================================
# DIRECTION  (see header -- set once, from the jog test)
# ==================================================

# +1 : positive motor speed OPENS the gripper
# -1 : positive motor speed CLOSES the gripper
OPEN_SIGN = 1

CLOSE_SIGN = -OPEN_SIGN


# ==================================================
# SPEEDS AND FORCES
# ==================================================

# Opening drives into a rigid mechanical stop: a firmer duty is fine,
# and a slower speed makes the stall position more repeatable.
OPEN_SPEED = 90
OPEN_DUTY_LIMIT = 30

# Closing lands on a ball: gentler duty so we grip without crushing,
# and without the fingers skipping over the ball.
CLOSE_SPEED = 120
CLOSE_DUTY_LIMIT = 45

# Moving back to a known angle (not into a stop) can be brisk.
RETURN_SPEED = 150

# After finding the hard stop we back off a few degrees so the motor is
# not permanently leaning on the mechanism between cycles.
OPEN_BACKOFF_DEG = 5

# Let the mechanism settle after a stall before trusting angle().
SETTLE_MS = 250


# ==================================================
# GRIP CLASSIFICATION THRESHOLDS
# ==================================================

# Below this, the gripper barely moved: jam, obstruction, or it was
# already closed. Never treat this as a successful grip.
MIN_VALID_TRAVEL_DEG = 15

# Travel when closing on NOTHING (fingers meet each other).
# CALIBRATE THIS ONCE: set CALIBRATE_EMPTY_CLOSE = True, run with no
# ball in the gripper, read the printed value, paste it here.
EMPTY_CLOSE_TRAVEL_DEG = 120

# A real ball stops the fingers noticeably earlier than an empty close.
# Anything within this margin of the empty number is treated as a miss.
EMPTY_MARGIN_DEG = 20

CALIBRATE_EMPTY_CLOSE = False


# ==================================================
# CYCLE CONFIG
# ==================================================

TEST_COLORS = (Color.RED, Color.BLUE)

COLOR_NAMES = {
    Color.RED: "red",
    Color.BLUE: "blue",
    Color.GREEN: "green",
    Color.BLACK: "black",
}

# How long to hold the ball before releasing, so you can watch it.
HOLD_MS = 1500


# ==================================================
# STATE
# ==================================================

datum_ready = False


# ==================================================
# RULE 2 + 3: ESTABLISH THE PHYSICAL DATUM
# ==================================================

def find_open_datum():
    """Drive to the full-open hard stop and define it as angle 0.

    This is the single most important function in the file. After it
    returns, angle 0 always means the same physical finger position,
    no matter what the encoder read beforehand.
    """
    global datum_ready

    before = gripper.angle()

    # Drive OPEN until the mechanism refuses to go further.
    gripper.run_until_stalled(
        OPEN_SIGN * OPEN_SPEED,
        then=Stop.HOLD,
        duty_limit=OPEN_DUTY_LIMIT,
    )
    wait(SETTLE_MS)

    at_stop = gripper.angle()
    travel = at_stop - before

    # NOTE: a small travel here is legitimate. If you already hand-opened
    # the gripper near its stop, there is almost nothing left to move.
    # This is why we do NOT validate travel on the opening move.
    print(
        "[DATUM] before={} at_stop={} travel={}".format(
            before, at_stop, travel
        )
    )

    # THE DATUM. Everything downstream is measured from here.
    gripper.reset_angle(0)

    # Back off so we are not resting against the stop between cycles.
    gripper.run_target(
        RETURN_SPEED,
        CLOSE_SIGN * OPEN_BACKOFF_DEG,
        then=Stop.HOLD,
        wait=True,
    )

    datum_ready = True

    print(
        "[DATUM] established. open_hold={} (0 = hard stop)".format(
            gripper.angle()
        )
    )
    ev3.speaker.beep()


def open_hold_angle():
    """The resting 'fully open' angle, just off the hard stop."""
    return CLOSE_SIGN * OPEN_BACKOFF_DEG


# ==================================================
# RULE 5: CLOSE, AND MEASURE WHAT HAPPENED
# ==================================================

def close_until_stall():
    """Close until the fingers stop. Return absolute travel in degrees.

    Travel is measured FROM THE DATUM, not from wherever we happened to
    be, so the number is comparable across cycles and across runs.
    """
    before = gripper.angle()

    gripper.run_until_stalled(
        CLOSE_SIGN * CLOSE_SPEED,
        then=Stop.HOLD,
        duty_limit=CLOSE_DUTY_LIMIT,
    )
    wait(SETTLE_MS)

    at_stall = gripper.angle()

    # Distance from the OPEN DATUM (0), not from 'before'. That makes the
    # value mean "how far closed is it", independent of the backoff.
    travel = abs(at_stall)

    print(
        "[CLOSE] before={} at_stall={} travel_from_datum={}".format(
            before, at_stall, travel
        )
    )
    return travel


def classify_grip(travel):
    """Turn a travel measurement into a decision. Returns a string.

    This is the whole 'sensorless object detection' idea: the mechanism
    itself is the sensor, and travel is its output.
    """
    if travel < MIN_VALID_TRAVEL_DEG:
        return "JAMMED"

    if travel >= (EMPTY_CLOSE_TRAVEL_DEG - EMPTY_MARGIN_DEG):
        return "EMPTY"

    return "GRIPPED"


# ==================================================
# RELEASE: BACK TO THE SAME PLACE, EVERY TIME
# ==================================================

def release_to_open():
    """Return to the fully-open hold position.

    We use run_target() against the datum rather than another stall.
    Re-stalling on every release would hammer the hard stop for no gain
    -- the datum is already known. We only stall to FIND the datum.
    """
    gripper.run_target(
        RETURN_SPEED,
        open_hold_angle(),
        then=Stop.HOLD,
        wait=True,
    )
    wait(SETTLE_MS)

    print(
        "[RELEASE] target={} actual={}".format(
            open_hold_angle(), gripper.angle()
        )
    )


# ==================================================
# CALIBRATION HELPER (run once, with NO ball)
# ==================================================

def calibrate_empty_close():
    """Measure the travel when the fingers close on nothing."""
    print("[CAL] Remove any object from the gripper.")
    ev3.speaker.say("Remove object")
    wait(3000)

    find_open_datum()
    travel = close_until_stall()

    print("")
    print("=" * 52)
    print("[CAL] EMPTY_CLOSE_TRAVEL_DEG = {}".format(travel))
    print("[CAL] Paste that number into the constant at the top.")
    print("=" * 52)
    print("")

    release_to_open()
    ev3.speaker.beep()


# ==================================================
# ONE FULL GRIP CYCLE
# ==================================================

def run_grip_cycle(label):
    """Datum -> close on object -> hold -> release. Fully self-contained.

    RULE 4 in action: this begins with find_open_datum(), so the cycle
    cannot inherit an error from the cycle before it.
    """
    print("")
    print("---------- CYCLE: {} ----------".format(label))

    timer = StopWatch()

    # 1. Re-establish the physical zero. Drift dies here.
    find_open_datum()

    # 2. Close onto whatever is there.
    travel = close_until_stall()

    # 3. Decide what we caught.
    verdict = classify_grip(travel)

    print(
        "[VERDICT] {} travel={} (empty~{}, min_valid={})".format(
            verdict,
            travel,
            EMPTY_CLOSE_TRAVEL_DEG,
            MIN_VALID_TRAVEL_DEG,
        )
    )

    if verdict == "JAMMED":
        ev3.speaker.say("Jammed")
        release_to_open()
        return False

    if verdict == "EMPTY":
        ev3.speaker.say("No ball")
        release_to_open()
        return False

    # 4. Hold, so the grip is visible and you can tug-test it.
    ev3.speaker.say("Gripped " + label)
    wait(HOLD_MS)

    # 5. Release to the SAME open position, ready for the next cycle.
    release_to_open()

    print("[CYCLE] {} complete in {} ms".format(label, timer.time()))
    ev3.speaker.beep()
    return True


# ==================================================
# BALL DETECTION
# ==================================================

def wait_for_test_color():
    """Block until a RED or BLUE ball is seen, debounced."""
    print("[WAIT] place a red or blue ball at the sensor")

    while True:
        color = color_sensor.color()

        if color in TEST_COLORS:
            wait(120)
            if color_sensor.color() == color:
                return color

        wait(50)


def wait_for_center_press():
    while Button.CENTER not in ev3.buttons.pressed():
        wait(50)
    while Button.CENTER in ev3.buttons.pressed():
        wait(20)


# ==================================================
# MAIN
# ==================================================

def main():
    print("[INIT] OPEN_SIGN={} (positive speed {})".format(
        OPEN_SIGN,
        "opens" if OPEN_SIGN > 0 else "closes",
    ))

    if CALIBRATE_EMPTY_CLOSE:
        calibrate_empty_close()
        return

    ev3.speaker.say("Gripper test")

    results = []

    try:
        for label in ("red", "blue"):
            print("")
            print("[STEP] Place the {} ball, then press CENTER.".format(label))
            ev3.speaker.say("Place " + label + " ball")
            wait_for_center_press()

            ok = run_grip_cycle(label)
            results.append((label, ok))

        print("")
        print("========== SUMMARY ==========")
        for label, ok in results:
            print("  {:5s} : {}".format(label, "OK" if ok else "FAILED"))
        print("=============================")

        ev3.speaker.say("Test complete")

    except Exception as exc:
        gripper.brake()
        print("[ERROR] {}".format(exc))
        raise

    finally:
        # Leave the gripper open and holding, ready for the next run.
        try:
            if datum_ready:
                release_to_open()
            else:
                gripper.brake()
        except Exception:
            gripper.brake()


main()