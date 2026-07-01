# ROS 2 Humble + Gazebo Fortress (Ignition) environment

The manipulator sim was originally authored against **ROS 2 Humble + Ignition
Fortress** (`ign-gazebo-6`). This environment runs that original setup unmodified.
The top-level [`docker/`](../) env is the newer **ROS 2 Jazzy + Gazebo Harmonic** stack.

## Usage (native Ubuntu host with an NVIDIA GPU)
```bash
xhost +local:root                                       # once: allow GUI
docker compose -f docker/humble/docker-compose.yml up -d --build
docker compose -f docker/humble/docker-compose.yml exec ev3-manipulator-humble bash
# inside:  cb   (colcon build)   then   cs   (source)   then ros2 launch ...
```
Uses separate build volumes and a separate container name (`ev3-manipulator-humble`),
so it coexists with the Jazzy env without clashing.
