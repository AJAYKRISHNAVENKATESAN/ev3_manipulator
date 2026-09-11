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

![EV3 manipulator hardware showing the homing switches for the base and pickup-arm encoders](docs/images/manipulator_ev3.png)
*Homing switches for the base and pickup-arm motors — pressing one gives that motor's encoder a known zero point to center its angle from.*

## Tech stack
- **ROS 2** (Humble by default, Jazzy supported) — `ros2_control`, URDF/xacro
- **Gazebo** (Fortress/Ignition, or Harmonic on Jazzy) for simulation; **Isaac Sim 5.1** as an alternate sim backend
- **Python** — each package's `sorting_node` and `hardware_interface` nodes
- **pybricks-micropython** — runs on the physical EV3 brick; talks to `hardware_interface` over TCP (a stage handshake for `stage_sync`, a continuous telemetry stream for `live_sync`)
- **MoveIt 2** — scaffolded, not yet integrated
- **Docker** — containerized, GPU-accelerated dev environments for every stack above

## Packages
- [`ev3_manipulator_stage_sync`](ev3_manipulator_stage_sync/README.md)
- [`ev3_manipulator_live_sync`](ev3_manipulator_live_sync/README.md)
- [`ev3_manipulator_moveit`](ev3_manipulator_moveit/README.md) (🚧 work in progress)
- [`conveyor_belt/`](conveyor_belt/) — vendored third-party Gazebo-ROS2 conveyor
  belt plugin
  ([IFRA-Cranfield/IFRA_ConveyorBelt](https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt))

## Quickstart
```bash
git clone git@github.com:AJAYKRISHNAVENKATESAN/ev3_manipulator.git
cd ev3_manipulator
xhost +local:root                                      # once: allow GUI
docker compose -f docker/docker-compose.yml up -d --build
docker compose -f docker/docker-compose.yml exec ev3-manipulator-dev bash
# inside:  cb   (colcon build)   then   cs   (source)
```
Requires a native Ubuntu host with an NVIDIA GPU and Docker (+
[nvidia-container-toolkit](https://github.com/NVIDIA/nvidia-container-toolkit)).
For ROS 2 Jazzy/Gazebo Harmonic or Isaac Sim instead, see
[docs/development.md](docs/development.md).

To run against real EV3 hardware instead of (or alongside) the sim, flash the
relevant package's firmware `sorting.py` to the brick — see each package's
README for what its firmware does.

## Author
**Ajaykrishna Venkatesan** — [github.com/AJAYKRISHNAVENKATESAN](https://github.com/AJAYKRISHNAVENKATESAN) · aj.grizzy@gmail.com
