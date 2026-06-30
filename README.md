# ev3_manipulator

ROS 2 (Jazzy) packages for the **EV3-brick Lego manipulator** — a robot arm that
picks/sorts objects off a conveyor. Used as a git submodule in
[`project-drishti`](https://github.com/PavanSandaka/project-drishti) at
`bots/ev3_manipulator`.

## Packages
- **`ev3_manipulator`** — the arm: URDF/xacro, meshes, gz (Gazebo Harmonic) sim
  launch, `ros2_control`, and the sorting / hardware-interface nodes.
- **`ev3_manipulator_moveit`** — MoveIt 2 motion-planning config.

## Build
```bash
colcon build && source install/setup.bash
```
Work in progress -> to do 
->Current Tasks
**Hardware Synchronization**: Establish real-time communication and state synchronization between the virtual Gazebo model and the physical LEGO EV3 manipulator.
**CI/CD & Deployment**: Containerize the ROS 2 environment into a unified `Dockerfile` for seamless deployment across laboratory workstations.
**Explore MoveIt 2 Integration in the future**: 

> Note: the MoveIt config currently targets the older `manipulator_ev3_brick`
> URDF; re-pointing it to this arm's URDF is a follow-up.



https://github.com/user-attachments/assets/9ebfe38d-4cc8-4826-ae7d-aa0d116ae9a8

## Development environment (Docker)

Self-contained envs for a **native Ubuntu host with an NVIDIA GPU**.

### Gazebo / ROS 2 (Jazzy + Gazebo Harmonic)
```bash
xhost +local:docker                                   # once: allow GUI
docker compose -f docker/docker-compose.yml up -d --build
docker compose -f docker/docker-compose.yml exec ev3-manipulator-dev bash
# inside:  cb   (colcon build)   then   cs   (source)
```
Brings up ROS 2 Jazzy + Gazebo Harmonic with `ros_gz`, `gz_ros2_control`,
`ros2_control(lers)`, and MoveIt 2. The repo is mounted at
`/workspace/src/ev3_manipulator`.

### Isaac Sim 5.1 (headless + WebRTC)
See [`docker/isaac-sim/README.md`](docker/isaac-sim/README.md). On Blackwell
(RTX 50-series) the host needs driver **580** — see
[`docker/isaac-sim/DRIVER_DOWNGRADE.md`](docker/isaac-sim/DRIVER_DOWNGRADE.md).
