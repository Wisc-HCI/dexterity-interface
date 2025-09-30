FROM osrf/ros:jazzy-desktop-full

# Set environment variable to avoid user prompts during build
ENV DEBIAN_FRONTEND=noninteractive


################# Install Dependencies #################

# Required for PPA
RUN apt-get update && apt-get install -y \
    software-properties-common 

# Required for Python3.11
RUN add-apt-repository ppa:deadsnakes/ppa

RUN apt update && apt install -y libeigen3-dev python3.11 python3-pip python3.11-venv

RUN python3.11 -m venv /venv
ENV PATH="/venv/bin:$PATH"
RUN pip install --upgrade pip && \
    pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128 && \
    pip install isaaclab[isaacsim,all]==2.2.0 --extra-index-url https://pypi.nvidia.com


################# Workspace setup #################

# Add python alias to python3
RUN ln -s /usr/bin/python3 /usr/bin/python 

ENV OMNI_KIT_ALLOW_ROOT=1


COPY . /workspace
WORKDIR /workspace


# Default command
CMD ["bash"]