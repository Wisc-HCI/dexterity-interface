# Use an official ROS Noetic image with desktop-full
FROM osrf/ros:noetic-desktop-full

# Set environment variable to avoid user prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV PULSE_SERVER=unix:/run/user/1000/pulse/native

# Update apt and install all required packages
RUN chmod 1777 /tmp && \
    apt-get update && \
    apt-get install -y --fix-missing \
    curl \
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

# Azure Kinect SDK dependencies
RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \
    echo "deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main" > /etc/apt/sources.list.d/microsoft-prod.list && \
    apt-get update && \
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
