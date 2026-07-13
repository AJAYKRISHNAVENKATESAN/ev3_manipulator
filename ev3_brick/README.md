# ev3_brick

`sorting.py` runs **on the physical EV3 brick itself** — it's `pybricks-micropython`,
not a ROS 2 node, and is not part of any colcon package in this repo (colcon never
builds it, and it's excluded from the `ev3_manipulator` package's `ament` lint tests).

It's the hardware-side counterpart to `ev3_manipulator/ev3_manipulator/hardware_interface.py`:
it opens a TCP socket to the ROS 2 side, does the same handshake/token protocol
(`EV3_CONNECT_REQUEST`, `START_HOMING`, `THETA1:`/`THETA2:`, `READY_PICKUP`,
`ROS_CYCLE_DONE`, etc.), and drives the brick's motors/sensors directly via the
Pybricks API.

## Deploying to the brick
Flash it onto the EV3 with the [Pybricks VS Code extension](https://code.visualstudio.com/)
(or `pybricksdev`) — this is a manual step done from your host machine, separate
from the ROS 2/Gazebo dev container. Before running it, update `ROS2_SERVER_IP` at
the top of the file to the IP of the machine running `hardware_interface`.
