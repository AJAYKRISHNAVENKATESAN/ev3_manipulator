# ev3_manipulator — ROS 2 Jazzy + Gazebo Harmonic development environment
# The default env (docker/) is ROS 2 Humble + Gazebo Fortress, matching what the
# manipulator sim / EV3 hardware sync was authored against. This env tracks the
# newer Jazzy + Harmonic stack.
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE_ROOT=/workspace
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl unzip \
    python3-pip python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    # Gazebo Harmonic (gz) integration + ros2_control bridge
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    # ros2_control
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    # MoveIt 2
    ros-${ROS_DISTRO}-moveit \
    # description / utilities
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    vim nano tmux htop mesa-utils dos2unix \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${WORKSPACE_ROOT}/src

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "alias cb='cd ${WORKSPACE_ROOT} && colcon build --symlink-install'" >> ~/.bashrc && \
    echo "alias cs='source ${WORKSPACE_ROOT}/install/setup.bash'" >> ~/.bashrc && \
    echo "alias ws='cd ${WORKSPACE_ROOT}'" >> ~/.bashrc

WORKDIR ${WORKSPACE_ROOT}
COPY docker/jazzy/entrypoint.sh /entrypoint.sh
RUN dos2unix /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
