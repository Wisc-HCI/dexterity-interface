# Use an official Ubuntu 20.04 LTS as a parent image
FROM osrf/ros:noetic-desktop-full

# Set noninteractive to avoid prompts during the build
ENV DEBIAN_FRONTEND=noninteractive

# Update apt package list and install general packages
RUN chmod 1777 /tmp
RUN apt-get update && \
    apt-get install -y --fix-missing \
    curl\
    python3-pip\
    build-essential\ 
    cmake\
    nano \
    libeigen3-dev\
    python3-catkin-tools \
    ros-noetic-libfranka ros-noetic-franka-ros \
    ros-noetic-urdfdom-py \
    ros-noetic-kdl-parser-py \
    ros-noetic-kdl-conversions\
    ros-noetic-spacenav-node \
    # ros-noetic-foxglove-bridge \ 
    ros-noetic-pr2-description \
    ros-noetic-rosbridge-server \
    ros-noetic-tf2-web-republisher \
    ros-noetic-interactive-marker-tutorials \
    ros-noetic-interactive-markers \ 
    ros-noetic-tf2-tools \
    usbutils

# Update apt package list and install general packages
RUN apt-get update && \
    apt-get install -y --fix-missing \
    curl\
    python3-pip\
    build-essential\ 
    cmake\
    libpoco-dev\ 
    libeigen3-dev\
    python3-rosdep\
    mesa-utils\
    nano\
    pulseaudio\
    pulseaudio-utils\
    alsa-base\
    alsa-utils\
    libasound2\
    libasound2-plugins\
    pulseaudio\
    python3-catkin-tools\ 
    ros-noetic-gazebo-ros-control\
    ros-noetic-rospy-message-converter\
    ros-noetic-effort-controllers\
    ros-noetic-joint-state-controller\
    ros-noetic-moveit\
    ros-noetic-moveit-commander\
    ros-noetic-moveit-visual-tools\
    ros-noetic-rgbd-launch\
    v4l-utils\
    usbutils\
    python3-tk\
    v4l-utils


# Install Azure Kinect SDK dependencies
RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \
    echo "deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main" > /etc/apt/sources.list.d/microsoft-prod.list && \
    apt-get update && \
    ACCEPT_EULA=Y DEBIAN_FRONTEND=noninteractive apt-get install -y k4a-tools libk4a1.4 libk4a1.4-dev


# Install python packages
RUN pip install future\
    PyYaml\
    urdf-parser-py\
    panda_robot\
    numpy==1.21


# Install model workflow packages
RUN pip install \
    opencv-python \
    transformers \
    torch \
    matplotlib \
    pillow \
    requests \
    python-dotenv \
    azure-cognitiveservices-speech \
    pyk4a \
    einops \
    timm


# Add python alias to python3
RUN ln -s /usr/bin/python3 /usr/bin/python


# Setup spacenavd (can't use apt-get bc that version is too old)
COPY . /workspace
WORKDIR /workspace/

# WORKDIR /workspace/panda-primitives-control/spacenavd
# RUN ./configure
# RUN make install
WORKDIR /workspace/

# Install python packages
RUN pip install -r requirements.txt

# Set the default command to execute
CMD ["bash"]