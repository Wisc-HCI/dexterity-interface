# Use an official Ubuntu 20.04 LTS as a parent image
FROM ubuntu:20.04

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

# Update apt package list and install general packages
RUN apt-get update && \
    apt-get install -y --fix-missing \
    python3-pip \
    ffmpeg \
    python3-pyaudio \
    alsa-utils \
    pulseaudio \
    portaudio19-dev



COPY . /workspace
WORKDIR /workspace/

# Install python packages
RUN pip install -r requirements.txt


# Set the default command to execute
CMD ["bash"]



