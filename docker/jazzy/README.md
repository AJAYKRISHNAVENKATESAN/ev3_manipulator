# ROS 2 Jazzy + Gazebo Harmonic environment

The default [`docker/`](../) env is **ROS 2 Humble + Gazebo Fortress** (Ignition),
matching what the manipulator sim and EV3 hardware sync were originally authored
against. This env tracks the newer **ROS 2 Jazzy + Gazebo Harmonic** stack instead.

## Usage (native Ubuntu host with an NVIDIA GPU)
```bash
xhost +local:docker                                     # once: allow GUI
docker compose -f docker/jazzy/docker-compose.yml up -d --build
docker compose -f docker/jazzy/docker-compose.yml exec ev3-manipulator-jazzy bash
# inside:  cb   (colcon build)   then   cs   (source)   then ros2 launch ...
```
Uses separate build volumes and a separate container name (`ev3-manipulator-jazzy`),
so it coexists with the default Humble env without clashing.
