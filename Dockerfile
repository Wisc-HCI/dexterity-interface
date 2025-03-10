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


COPY . /workspace

WORKDIR /workspace/

RUN pip install -r requirements.txt


CMD ["bash"]
