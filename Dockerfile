# Use an official Ubuntu 20.04 LTS as a parent image
FROM ubuntu:20.04

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

# Update apt package list and install necessary packages
RUN apt-get update && \
    apt-get install -y --fix-missing \
    sudo \
    python3-pip \
    ffmpeg \
    libcairo2-dev \
    python3-pyaudio \
    alsa-utils \
    pulseaudio \
    portaudio19-dev \
    nano

RUN    rm -rf /var/lib/apt/lists/*

# # Create a non-root user (required for pulseaudio) with access to suo and audio
# RUN useradd -m -s /bin/bash user
# RUN usermod -aG sudo user
# RUN usermod -aG audio user
# RUN usermod -aG pulse,pulse-access user
# RUN echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


COPY . /workspace
# RUN chown -R user:user /workspace

# # Switch to non-root user
# USER user

WORKDIR /workspace/

RUN pip install -r requirements.txt


CMD ["bash"]
