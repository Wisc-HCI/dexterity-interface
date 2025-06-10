# Use an official ROS Noetic image with desktop-full
FROM osrf/ros:noetic-desktop-full

# Set environment variable to avoid user prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV PULSE_SERVER=unix:/run/user/1000/pulse/native

# Remove all existing ROS sources to prevent GPG errors
RUN rm -f /etc/apt/sources.list.d/ros*.list

# Install curl and gnupg (without ROS sources)
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release

# Add ROS GPG key and source
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list

# Update apt and install all required packages
RUN chmod 1777 /tmp && \
    apt-get update && \
    apt-get install -y --fix-missing \
    git \
    python3-pip \
    build-essential \
    cmake \
    nano \
    libeigen3-dev \
    libpoco-dev \
    python3-rosdep \
    python3-catkin-tools \
    mesa-utils \
    pulseaudio \
    pulseaudio-utils \
    alsa-base \
    alsa-utils \
    libasound2 \
    libasound2-plugins \
    ros-noetic-libfranka \
    ros-noetic-franka-ros \
    ros-noetic-urdfdom-py \
    ros-noetic-kdl-parser-py \
    ros-noetic-kdl-conversions \
    ros-noetic-spacenav-node \
    ros-noetic-pr2-description \
    ros-noetic-rosbridge-server \
    ros-noetic-tf2-web-republisher \
    ros-noetic-interactive-marker-tutorials \
    ros-noetic-interactive-markers \
    ros-noetic-tf2-tools \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rospy-message-converter \
    ros-noetic-effort-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-moveit \
    ros-noetic-moveit-commander \
    ros-noetic-moveit-visual-tools \
    ros-noetic-rgbd-launch \
    usbutils \
    python3-tk \
    v4l-utils

# Microsoft GPG key (safe version)
RUN curl -fsSL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor -o /usr/share/keyrings/microsoft.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/ubuntu/18.04/prod bionic main" > /etc/apt/sources.list.d/microsoft-prod.list

# Azure Kinect SDK
RUN apt-get update && \
    ACCEPT_EULA=Y apt-get install -y \
    k4a-tools \
    libk4a1.4 \
    libk4a1.4-dev


# Add python alias to python3
RUN ln -s /usr/bin/python3 /usr/bin/python

COPY . /workspace
WORKDIR /workspace

WORKDIR /workspace
RUN pip install -r requirements.txt 

# PulseAudio volume
VOLUME ["/run/user/1000/pulse"]

# Default command
CMD ["bash"]