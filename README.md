# ev3_manipulator

ROS 2 packages for the **EV3-brick Lego manipulator** — a robot arm that
picks/sorts objects off a conveyor, synchronized between a Gazebo sim and the
physical LEGO EV3 hardware. Used as a git submodule in
[`project-drishti`](https://github.com/PavanSandaka/project-drishti) at
`bots/ev3_manipulator`.

## Layout
- **`ev3_manipulator/`** — ROS 2 package: URDF/xacro, meshes, Gazebo sim launch,
  `ros2_control`, and the `sorting_node` / `hardware_interface` nodes that drive
  the sim and talk to the physical brick over TCP.
- **`ev3_brick/`** — **not a ROS 2 package.** `pybricks-micropython` that runs on
  the physical EV3 brick itself; the hardware-side counterpart to
  `hardware_interface.py`. See [`ev3_brick/README.md`](ev3_brick/README.md) for
  the protocol and how to flash it.
- **`ev3_manipulator_moveit/`** — MoveIt 2 config. **Experimental / unused** —
  scaffolding for future MoveIt-based control of the sim and hardware; not
  currently wired into `sorting_node`/`hardware_interface`, and its config still
  targets an older URDF. Explored as future work, not part of the current
  sorting pipeline.

## Build
```bash
colcon build && source install/setup.bash
```

## Status / Roadmap
- **Sim ↔ EV3 stage synchronization — active work.** `sorting_node.py` and the
  brick's `ev3_brick/sorting.py` handshake at each stage of a sort cycle
  (homing, ready-to-pick, ready-to-place, cycle-done) over the TCP link owned
  by `hardware_interface.py`. This sync is still being fine-tuned — timing and
  stage boundaries between the sim and the physical arm are an ongoing area of
  tuning, not a finished/stable protocol yet. Both the sim and the physical
  brick run their own sort cycle correctly in isolation; getting their timing
  to line up stage-for-stage over the TCP handshake is the remaining work.
- **MoveIt 2 — to be explored in the near future.** `ev3_manipulator_moveit/`
  is experimental scaffolding, not yet wired into the sim/hardware sync above.

<img width="1177" height="928" alt="image" src="https://github.com/user-attachments/assets/af0eb69b-a9ec-4078-b755-072b7889044f" />


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
