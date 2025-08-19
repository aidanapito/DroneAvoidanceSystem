#!/bin/bash
# Ubuntu Setup Script for Drone Swarm System

set -e

echo "Setting up Ubuntu environment for Drone Swarm System..."

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
echo "Installing ROS 2 Humble..."
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y

# Install Gazebo
echo "Installing Gazebo..."
sudo apt install gazebo libgazebo-dev -y

# Install PX4 SITL dependencies
echo "Installing PX4 SITL dependencies..."
sudo apt install ninja-build cmake build-essential genromfs ninja-build gperf -y
sudo apt install ccache dfu-util wget python3 python3-pip python3-setuptools -y
sudo apt install python3-dev python3-wheel git zip unzip openocd -y
sudo apt install libc6-dev-arm-none-eabi libnewlib-arm-none-eabi -y

# Install MAVSDK
echo "Installing MAVSDK..."
pip3 install mavsdk

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install numpy scipy matplotlib

# Install development tools
echo "Installing development tools..."
sudo apt install build-essential cmake git -y

# Source ROS 2
echo "Sourcing ROS 2..."
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "Setup complete! Please restart your terminal or run: source ~/.bashrc"
