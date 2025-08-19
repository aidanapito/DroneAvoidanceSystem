FROM ubuntu:22.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set timezone
ENV TZ=UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install system dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

# Install PX4 SITL dependencies
RUN apt-get update && apt-get install -y \
    ninja-build \
    genromfs \
    gperf \
    ccache \
    dfu-util \
    zip \
    unzip \
    openocd \
    libc6-dev-arm-none-eabi \
    libnewlib-arm-none-eabi \
    && rm -rf /var/lib/apt/lists/*

# Install MAVSDK
RUN pip3 install mavsdk

# Install additional Python dependencies
RUN pip3 install numpy scipy matplotlib

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Create workspace directory
WORKDIR /workspace

# Copy the swarm system code
COPY . /workspace/

# Build the workspace
RUN source /opt/ros/humble/setup.bash && \
    cd /workspace && \
    colcon build --symlink-install

# Source the workspace
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Expose ports for MAVSDK connections
EXPOSE 14540 14541 14542

# Set the default command
CMD ["/bin/bash"]
