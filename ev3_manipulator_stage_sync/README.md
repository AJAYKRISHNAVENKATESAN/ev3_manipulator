# ev3_manipulator_stage_sync

ROS 2 package that runs the same sort-cycle logic both in an Ignition Gazebo
simulation and on a physical LEGO Mindstorms EV3 brick, kept in lockstep by a
TCP handshake at every stage of the cycle.

## Layout
URDF/xacro, meshes, Gazebo sim launch, `ros2_control`, dev `tools/`, and the
`sorting_node` / `hardware_interface` nodes that drive the sim and talk to
the physical brick. `ev3_manipulator_stage_sync_firmware/` holds the
`pybricks-micropython` counterpart that runs on the EV3 brick itself — **not
a ROS 2 package** (colcon-ignored).

## Architecture
```mermaid
flowchart LR
    subgraph ROS 2
        SN["sorting_node<br/>(sort-cycle logic)"]
        HI["hardware_interface<br/>(TCP bridge)"]
        CM["ros2_control<br/>controller_manager"]
    end
    SIM["Gazebo sim<br/>(arm model)"]
    BRICK["EV3 brick<br/>ev3_manipulator_stage_sync_firmware/sorting.py<br/>(pybricks-micropython)"]

    SN -- "FollowJointTrajectory<br/>action" --> CM --> SIM
    SN <-- "stage_event / stage_sync<br/>topics" --> HI
    HI <-- "TCP handshake<br/>(homing, ready-to-pick,<br/>ready-to-place, cycle-done)" --> BRICK
```
`sorting_node` drives the sim arm directly via `ros2_control` and stays in
sync with the physical arm through `hardware_interface`, which owns the TCP
handshake link to the EV3 brick.

Each stage above belongs to a per-cycle state machine — one pass through
this for every ball/brick fed onto the conveyor:

```mermaid
stateDiagram-v2
    [*] --> InitialHome
    InitialHome --> WaitBall
    WaitBall --> Spawn : colour detected
    Spawn --> ConveyorToPickup : RED / BLUE
    Spawn --> ConveyorEject : BLACK / GREEN

    ConveyorToPickup --> PickupReady
    PickupReady --> PickDown
    PickDown --> GripClose
    GripClose --> PickUp
    PickUp --> Rotate
    Rotate --> PlaceDown
    PlaceDown --> Release
    Release --> PlaceUp
    PlaceUp --> HomeAfterPick
    HomeAfterPick --> CycleComplete

    ConveyorEject --> CenterHold
    CenterHold --> CycleComplete

    CycleComplete --> WaitBall : more balls
    CycleComplete --> [*] : all balls sorted
```

## Status
**Active work.** Both the sim and the physical brick run their own sort
cycle correctly in isolation. `sorting_node.py` and the brick's firmware
`sorting.py` handshake at each stage of a sort cycle (homing, ready-to-pick,
ready-to-place, cycle-done) over the TCP link owned by
`hardware_interface.py`. Getting their timing to line up stage-for-stage
over that handshake is the remaining work — this sync is still being
fine-tuned, not a finished/stable protocol yet.
