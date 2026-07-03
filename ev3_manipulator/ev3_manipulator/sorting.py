#!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
# from pybricks.parameters import Port, Stop, Color
# from pybricks.tools import wait
# import math
# import socket as socket

# # ==================================================
# # INITIALIZATION & NETWORK SETUP
# # ==================================================
# ev3 = EV3Brick()
# count = 0

# # Network Parameters mapped directly from Ubuntu System Settings Screen
# ROS2_SERVER_IP = "169.254.35.229"  
# PORT = 5005

# # Translation dictionary to map Pybricks Enums to raw lowercase ROS2 keys
# COLOR_MAP = {
#     Color.RED: "red",
#     Color.BLUE: "blue",
#     Color.BLACK: "black",
#     Color.GREEN: "green"
# }

# # Motors
# gripper  = Motor(Port.A)
# arm      = Motor(Port.B)
# base     = Motor(Port.C)
# conveyer = Motor(Port.D)

# # Sensors
# arm_home    = TouchSensor(Port.S3)
# base_home   = TouchSensor(Port.S1)
# color_sensor = ColorSensor(Port.S4)

# # ==================================================
# # ROBOT GEOMETRY & SAFETY CONFIGURATIONS
# # ==================================================
# L0 = 40.0                      
# L1 = 50.0
# L2 = 95.0
# L3 = 185.0
# L4 = 110.0 

# L12   = math.sqrt(L1**2 + L2**2 - 2*L1*L2*math.cos(math.radians(135)))
# L_ARM = L12 + L3

# GEAR_BASE = 36 / 12
# GEAR_ARM  = 40 / 8

# # Absolute Hardware Safety Boundaries
# THETA1_MIN = -180.0
# THETA1_MAX = 180.0

# # Live Operational Telemetry Observers
# max_theta2_observed = -999.0
# min_theta2_observed = 999.0

# # ==================================================
# # NETWORK TRANSPORT UTILITIES
# # ==================================================
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# def connect_and_handshake():
#     ev3.speaker.say("Connecting to server node")
#     try:
#         sock.connect((ROS2_SERVER_IP, PORT))
#         sock.send(b"EV3_CONNECT_REQUEST\n")
        
#         response = read_socket_line()
#         if "ROS_CONNECT_ACCEPT" in response:
#             ev3.speaker.say("Handshake verified")
#             return True
#         else:
#             ev3.speaker.say("Handshake rejected")
#             return False
#     except Exception as e:
#         print("Network error:", e)
#         ev3.speaker.say("Connection failed")
#         return False

# def read_socket_line():
#     line = ""
#     while True:
#         char = sock.recv(1).decode('utf-8')
#         if char == '\n' or not char:
#             break
#         line += char
#     return line.strip()

# # ==================================================
# # HOMING & SAFETY ENFORCEMENT
# # ==================================================
# def home_joints():
#     ev3.speaker.say("Homing sequence active")
#     home_joint(arm,  arm_home,  -1, 15)
#     home_joint(base, base_home,  1, 120 * GEAR_BASE)
#     ev3.speaker.say("Homing complete")

# def home_joint(motor, sensor, direction, offset, speed=100):
#     motor.run(direction * speed)
#     while not sensor.pressed():
#         wait(10)
#     motor.stop(Stop.BRAKE)
#     motor.run_angle(-direction * speed, offset, Stop.HOLD)
#     motor.reset_angle(0)

# def trigger_safety_recovery(violated_joint, bad_value):
#     base.stop(Stop.BRAKE)
#     arm.stop(Stop.BRAKE)
#     conveyer.stop(Stop.BRAKE)
#     print("CRITICAL VIOLATION ON " + violated_joint + ": " + str(bad_value))
#     ev3.speaker.say("Safety boundary limit reached")
#     wait(3000) 
#     home_joints()

# # ==================================================
# # KINEMATICS WITH INTEGRATED REAL-TIME TELEMETRY
# # ==================================================
# def inverse_base(x, y):
#     return math.degrees(math.atan2(y, x))

# def inverse_arm(x, z):
#     dz = z - L0
#     dz = max(min(dz, L_ARM), -L_ARM)
#     theta = math.asin(dz / L_ARM)
#     return -math.degrees(theta)

# def move_base_to(x, y, speed=150):
#     t1 = inverse_base(x, y)
#     if t1 < THETA1_MIN or t1 > THETA1_MAX:
#         trigger_safety_recovery("THETA1", t1)
#         return False
    
#     sock.send(("THETA1:" + str(t1) + "\n").encode('utf-8'))
#     target = t1 * GEAR_BASE
#     base.run_angle(speed, target - base.angle(), Stop.HOLD)
#     return True

# def move_arm_to(x, z, speed=100):
#     global max_theta2_observed, min_theta2_observed
#     t2 = inverse_arm(x, z)
    
#     if t2 > max_theta2_observed:
#         max_theta2_observed = t2
#     if t2 < min_theta2_observed:
#         min_theta2_observed = t2
        
#     print("Live Theta2: " + str(t2) + " | Observed Range: [" + str(min_theta2_observed) + ", " + str(max_theta2_observed) + "]")
#     sock.send(("THETA2:" + str(t2) + "\n").encode('utf-8'))
    
#     target = t2 * GEAR_ARM
#     arm.run_angle(speed, target - arm.angle(), Stop.HOLD)
#     return True

# def open_gripper():
#     gripper.run_angle(90, 90, Stop.HOLD, False)
#     gripper.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
#     wait(500)

# def close_gripper():
#     gripper.run_angle(-90, 90, Stop.HOLD, False)
#     gripper.run_until_stalled(-200, then=Stop.HOLD, duty_limit=60)
#     wait(500)

# # ==================================================
# # RUN SYSTEM INITIALIZATION
# # ==================================================
# home_joints()
# if not connect_and_handshake():
#     raise SystemExit("Network connection failed.")

# # ==================================================
# # MAIN COORDINATED SYSTEM LOOP
# # ==================================================
# while count < 4:
#     # Start conveyor running continuously looking for a target
#     conveyer.run(80) 
    
#     color = None
#     while color not in COLOR_MAP:
#         color = color_sensor.color()
#         wait(20)
        
#     # Ball found! Immediately halt conveyor edge line to analyze identity
#     conveyer.stop(Stop.BRAKE)
    
#     color_str = COLOR_MAP[color]
#     ev3.speaker.say(color_str)  # Audibly announces detected color profile
#     print("Processing detected ball color: " + color_str)
    
#     # ── ROUTINE A: BYPASS BALLS (BLACK / GREEN) ───────────────────────────
#     if color_str in ["black", "green"]:
#         dist = -650 if color_str == "black" else 100
#         conveyer.run_angle(80, dist, Stop.HOLD)
#         count += 1
#         print("Finished processing bypass ball cycle: " + str(count) + "/4")

#     # ── ROUTINE B: MANIPULATOR SORTING BALLS (RED / BLUE) ──────────────────
#     elif color_str in ["red", "blue"]:
#         # 1. First move conveyor to shift ball to absolute physical position
#         conveyer.run_angle(80, -300, Stop.HOLD)
        
#         # 2. Only say "ready" now that the ball has arrived at the position
#         ev3.speaker.say("ready")
        
#         # 3. Inform ROS2 server to generate digital twin model
#         sock.send(("DETECTED:" + color_str + "\n").encode('utf-8'))
        
#         # 4. Wait for Gazebo node spawn acknowledgement
#         response = read_socket_line()
#         if response == "ACK_SPAWN":
#             print("ROS2 confirmed spawn. Deploying manipulator assembly...")
            
#             # 5. Bring mechanical arm down to initial approach hovering posture
#             move_arm_to(-110, -180)
#             open_gripper()
            
#             # 6. Audio cue and interlock broadcast for pickup confirmation
#             ev3.speaker.say("ready for pickup")
#             sock.send(b"READY_PICKUP\n")
#             print("Holding at pickup pose. Waiting for simulation confirmation...")
            
#             while True:
#                 sync_msg = read_socket_line()
#                 if "ROS_READY_PICKUP" in sync_msg:
#                     break
            
#             # 7. Execute synchronized drop-descent and physical squeeze
#             print("Lockstep verified. Dropping to grab target.")
#             move_arm_to(-200, -250)
#             close_gripper()
#             wait(200)
            
#             # 8. Retract arm back up to clearance space and announce achievement
#             move_arm_to(-110, -180)
#             ev3.speaker.say("picked up")
            
#             # 9. Deliver payload to corresponding collection bin locations
#             if color_str == "red":
#                 move_base_to(-120, 150) 
#                 move_arm_to(-200, -250)
#             else:
#                 move_base_to(120, 150)
#                 move_arm_to(-200, -250)
            
#             # 10. Sync timing for drop-off sequence execution
#             sock.send(b"READY_PLACE\n")
#             print("Holding at drop posture. Waiting for simulation confirmation...")
            
#             while True:
#                 sync_msg = read_socket_line()
#                 if "ROS_READY_PLACE" in sync_msg:
#                     break
            
#             # 11. Finalize release sequence in precise lockstep alignment
#             print("Lockstep verified. Actuating open release jaw.")
#             open_gripper()
#             wait(500)
            
#             # 12. Output strict validation receipt message as requested
#             print("MESSAGE: The " + color_str + " ball has been dropped.")
#             ev3.speaker.say("dropped")
            
#             # 13. Reset kinematic postures back to clean absolute origins
#             home_joints()
#             count += 1
#             print("Finished processing sorted ball cycle: " + str(count) + "/4")

# # Post-execution wrap-up termination rules
# conveyer.stop(Stop.COAST)
# ev3.speaker.say("Sorting process complete")
# print("All 4 balls processed successfully.")






#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.tools import wait
import math
import usocket as socket

# ==================================================
# INITIALIZATION & NETWORK SETUP
# ==================================================
ev3 = EV3Brick()
count = int(0)

ROS2_SERVER_IP = "169.254.35.229"  
PORT = 5005

# Motors
gripper  = Motor(Port.A)
arm      = Motor(Port.B)
base     = Motor(Port.C)
conveyer = Motor(Port.D)

# Sensors
arm_home   = TouchSensor(Port.S3)
base_home  = TouchSensor(Port.S1)
color_sensor = ColorSensor(Port.S4)

COLOR_MAP = {
    Color.RED: "red",
    Color.BLUE: "blue",
    Color.BLACK: "black",
    Color.GREEN: "green"
}

# ROBOT GEOMETRY (mm)
L0 = 40.0                      
L1 = 50.0
L2 = 95.0
L3 = 185.0
L4 = 110.0 
L12   = math.sqrt(L1**2 + L2**2 - 2*L1*L2*math.cos(math.radians(135)))
L_ARM = L12 + L3

GEAR_BASE = 36 / 12
GEAR_ARM  = 40 / 8

# Network Utilities
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def connect_and_handshake():
    ev3.speaker.say("Connecting")
    try:
        sock.connect((ROS2_SERVER_IP, PORT))
        sock.send(b"EV3_CONNECT_REQUEST\n")
        response = read_socket_line()
        if "ROS_CONNECT_ACCEPT" in response:
            ev3.speaker.say("Connected")
            return True
        return False
    except:
        return False

def read_socket_line():
    line = ""
    while True:
        char = sock.recv(1).decode('utf-8')
        if char == '\n' or not char:
            break
        line += char
    return line.strip()

# ==================================================
# HOMING & KINEMATICS (INITIAL ONLY)
# ==================================================
def home_joint(motor, sensor, direction, offset, speed=100):
    motor.run(direction * speed)
    while not sensor.pressed():
        wait(10)
    motor.stop(Stop.BRAKE)
    motor.run_angle(-direction * speed, offset, Stop.HOLD)
    motor.reset_angle(0)

def execute_system_homing():
    home_joint(arm,  arm_home,  -1, 15)
    home_joint(base, base_home,  1, 120 * GEAR_BASE)

# Run initial homing sequence once before cycles begin
ev3.speaker.say("Starting homing sequence")
execute_system_homing()
ev3.speaker.say("Homing complete")

if not connect_and_handshake():
    raise SystemExit("Network connection failed.")

def inverse_base(x, y):
    return math.degrees(math.atan2(y, x))

def inverse_arm(x, z):
    dz = z - L0
    dz = max(min(dz, L_ARM), -L_ARM)   
    theta = math.asin(dz / L_ARM)
    return -math.degrees(theta)

def move_base_to(x, y, speed=150):
    deg = inverse_base(x, y)
    sock.send(("THETA1:" + str(deg) + "\n").encode('utf-8'))
    target = deg * GEAR_BASE
    base.run_angle(speed, target - base.angle(), Stop.HOLD)

def move_arm_to(x, z, speed=100):
    deg = inverse_arm(x, z)
    sock.send(("THETA2:" + str(deg) + "\n").encode('utf-8'))
    target = deg * GEAR_ARM
    arm.run_angle(speed, target - arm.angle(), Stop.HOLD)

def open_gripper():
    gripper.run_angle(90, 90, Stop.HOLD, False) 
    gripper.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
    wait(500)

def close_gripper():
    gripper.run_angle(-90, 90, Stop.HOLD, False) 
    wait(500)

# ==================================================
# MAIN COORDINTATED LOOP
# ==================================================
while (count < 4):
    ev3.speaker.say("Place a ball on conveyor")
    wait(1000)

    color = color_sensor.color()

    if color == Color.BLACK:
        conveyer.stop()
        ev3.speaker.say("Black")
        conveyer.run_angle(80, -650, Stop.HOLD)
        count += 1

    elif color == Color.GREEN:
        conveyer.stop()
        ev3.speaker.say("Green")
        conveyer.run_angle(80, 100, Stop.HOLD)
        count += 1

    elif color == Color.RED:
        conveyer.stop()
        ev3.speaker.say("Red")
        conveyer.run_angle(80, -300, Stop.HOLD)

        sock.send(b"DETECTED:red\n")
        while True:
            if "ACK_SPAWN" in read_socket_line(): break

        # Approach/Drop Station Location
        move_arm_to(-110, -180)
        open_gripper()
        wait(300)

        # Clearance Lift and Swivel to Pickup (-90 deg)
        move_arm_to(50, 50)
        move_base_to(0, -50)

        sock.send(b"READY_PICKUP\n")
        while True:
            if "ROS_READY_PICKUP" in read_socket_line(): break

        # Plunge and Grab
        move_arm_to(-200, -250)
        close_gripper()
        wait(300)

        # Post-Grab Vertical Clearance Ascent
        move_arm_to(50, 50)

        sock.send(b"READY_PLACE\n")
        while True:
            if "ROS_READY_PLACE" in read_socket_line(): break

        # Direct Return Sweep (+90 deg rotation back to center)
        move_base_to(100, 0)
        count += 1

    elif color == Color.BLUE:
        conveyer.stop()
        ev3.speaker.say("Blue")
        conveyer.run_angle(80, -300, Stop.HOLD)

        sock.send(b"DETECTED:blue\n")
        while True:
            if "ACK_SPAWN" in read_socket_line(): break

        # Approach/Drop Station Location
        move_arm_to(-110, -180)
        open_gripper()
        wait(300)

        # Clearance Lift and Swivel to Pickup (+90 deg)
        move_arm_to(50, 50)
        move_base_to(0, 50)

        sock.send(b"READY_PICKUP\n")
        while True:
            if "ROS_READY_PICKUP" in read_socket_line(): break

        # Plunge and Grab
        move_arm_to(-200, -250)
        close_gripper()
        wait(300)

        # Unique Blue Vertical Clearance Ascent
        move_arm_to(50, 30)

        sock.send(b"READY_PLACE\n")
        while True:
            if "ROS_READY_PLACE" in read_socket_line(): break

        # Direct Return Sweep (-90 deg rotation back to center)
        move_base_to(100, 0)
        count += 1

conveyer.stop()
sock.close()
ev3.speaker.say("All balls sorted in respective stations. Process complete")