# ev3_manipulator

Two independent ROS 2 packages for a 2.5-DOF EV3 LEGO pick-and-place robot,
each keeping a Gazebo digital twin in sync with the physical hardware a
different way:

- **`ev3_manipulator_stage_sync`** — the same sort-cycle logic runs both in
  an Ignition Gazebo simulation and on a physical LEGO Mindstorms EV3 brick,
  kept in lockstep by TCP interlocks at every stage.
- **`ev3_manipulator_live_sync`** — a pure digital twin: the sim continuously
  mirrors the physical brick's live encoder telemetry over TCP, with no stage
  barrier or handshake.
- **Stack:** ROS 2 Humble, URDF/xacro modeling, `ros2_control`, Ignition
  Gazebo simulation, and a custom protocol talking to embedded
  `pybricks-micropython` on the EV3 hardware.
- **Context:** used as a git submodule in
  [`project-drishti`](https://github.com/PavanSandaka/project-drishti) at
  `bots/ev3_manipulator`.
- **Why:** built as a master's project for my mechatronics professor, to get
  hands-on with ROS 2 and sim-to-real robotics — modeling a real arm,
  controlling it in simulation, and closing the loop with actual hardware
  over two different sync strategies.

![EV3 manipulator hardware showing the homing switches for the base and pickup-arm encoders](docs/images/manipulator_ev3.png)
*Homing switches for the base and pickup-arm motors — pressing one gives that motor's encoder a known zero point to center its angle from.*

https://github.com/user-attachments/assets/a11ed067-34a3-43b9-aa6a-01087d70825e

## Architecture

## Tech stack
- **ROS 2** (Humble by default, Jazzy supported) — `ros2_control`, URDF/xacro
- **Gazebo** (Fortress/Ignition, or Harmonic on Jazzy) for simulation; **Isaac Sim 5.1** as an alternate sim backend
- **Python** — each package's `sorting_node` and `hardware_interface` nodes
- **pybricks-micropython** — runs on the physical EV3 brick; talks to `hardware_interface` over TCP (a stage handshake for `stage_sync`, a continuous telemetry stream for `live_sync`)
- **MoveIt 2** — scaffolded, not yet integrated (see Status below)
- **Docker** — containerized, GPU-accelerated dev environments for every stack above

## Layout
- **`ev3_manipulator_stage_sync/`** — see its own
  [README](ev3_manipulator_stage_sync/README.md) for architecture, layout,
  and status.
- **`ev3_manipulator_live_sync/`** — see its own
  [README](ev3_manipulator_live_sync/README.md) for architecture, layout,
  and status.
- **`ev3_manipulator_moveit/`** — see its own
  [README](ev3_manipulator_moveit/README.md) for architecture, layout, and
  status.
- **`conveyor_belt/`** — vendored third-party Gazebo-ROS2 conveyor belt
  plugin ([IFRA-Cranfield/IFRA_ConveyorBelt](https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt)),
  used by both packages' simulations.

## Quickstart
```bash
git clone git@github.com:AJAYKRISHNAVENKATESAN/ev3_manipulator.git
cd ev3_manipulator
```
Then see [Development environment (Docker)](#development-environment-docker)
below to build and launch the sim — requires a native Ubuntu host with an
NVIDIA GPU and Docker (+
[nvidia-container-toolkit](https://github.com/NVIDIA/nvidia-container-toolkit)).

To run against real EV3 hardware instead of (or alongside) the sim, flash the
relevant package's firmware `sorting.py` to the brick — see the Layout
section above for what each package's firmware does.

## Status / Roadmap
- **`ev3_manipulator_stage_sync`** — see its
  [README](ev3_manipulator_stage_sync/README.md#status) for current status.
- **`ev3_manipulator_live_sync`** — see its
  [README](ev3_manipulator_live_sync/README.md#status) for current status.
- **`ev3_manipulator_moveit`** — see its
  [README](ev3_manipulator_moveit/README.md#status) for current status.

## Development environment (Docker)

Self-contained envs for a **native Ubuntu host with an NVIDIA GPU**.

### Default: ROS 2 Humble + Gazebo Fortress (Ignition)
```bash
xhost +local:root                                      # once: allow GUI
docker compose -f docker/docker-compose.yml up -d --build
docker compose -f docker/docker-compose.yml exec ev3-manipulator-dev bash
# inside:  cb   (colcon build)   then   cs   (source)
```
This is the stack the manipulator sim and EV3 hardware sync were authored
against — use this one unless you have a specific reason not to. The repo is
mounted at `/workspace/src/ev3_manipulator`.

### Alternate: ROS 2 Jazzy + Gazebo Harmonic
See [`docker/jazzy/README.md`](docker/jazzy/README.md) — a separate env with its
own container/volumes, for tracking the newer Jazzy/Harmonic stack.

### Isaac Sim 5.1 (headless + WebRTC)
See [`docker/isaac-sim/README.md`](docker/isaac-sim/README.md). On Blackwell
(RTX 50-series) the host needs driver **580** — see
[`docker/isaac-sim/DRIVER_DOWNGRADE.md`](https://github.com/PavanSandaka/project-drishti/blob/main/docker/isaac-sim/DRIVER_DOWNGRADE.md).

## Author
**Ajaykrishna Venkatesan** — [github.com/AJAYKRISHNAVENKATESAN](https://github.com/AJAYKRISHNAVENKATESAN) · aj.grizzy@gmail.com
