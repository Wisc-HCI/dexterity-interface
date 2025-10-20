FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive

ARG ROS_DISTRO=jazzy

# System packages
RUN apt-get update && apt-get install -y \
    curl \
    git \
    python3-pip \
    build-essential \
    cmake \
    lsb-release \
 && rm -rf /var/lib/apt/lists/*

# ROS tools
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*


# Pybind Dependencies
# Compatible with 3.11
# RUN apt-get update && apt-get install -y \
#     python3.11-dev \
#     pybind11-dev \
#     libeigen3-dev 

# Compatible with 3.12
RUN apt-get install -y \
    python3-dev \ 
    pybind11-dev \
    libeigen3-dev


############## Pinocchio Install ##############
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        -o /etc/apt/keyrings/robotpkg.asc && \
     echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
        > /etc/apt/sources.list.d/robotpkg.list

RUN apt-get update && apt-get install -y \
    robotpkg-py3*-pinocchio \
 && rm -rf /var/lib/apt/lists/*


ENV PATH="/opt/openrobots/bin:${PATH:-}" \
    PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH:-}" \
    LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH:-}" \
    PYTHONPATH="/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH:-}" \
    CMAKE_PREFIX_PATH="/opt/openrobots:${CMAKE_PREFIX_PATH:-}"




############## Python Install ##############

COPY ./libs /workspace/libs
WORKDIR /workspace

RUN pip install -e libs/robot_motion  --break-system-packages
#     # && pip install -e libs/robot_motion_interface \
#     # && pip install -e libs/isaacsim_ui_interface \
#     # && pip install -e libs/sensor_interface/sensor_interface_py \
#     # && pip install -e libs/primitives/primitives_py \
#     # && pip install -e libs/planning/planning_py


############## Environment Niceties ##############
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Add python alias to python3 
RUN ln -s /usr/bin/python3 /usr/bin/python


CMD ["bash"]
