#!/bin/bash
# macOS Setup Script for Drone Swarm System Development

set -e

echo "Setting up macOS environment for Drone Swarm System Development..."

# Check if Homebrew is installed
if ! command -v brew &> /dev/null; then
    echo "Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    
    # Add Homebrew to PATH
    echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zshrc
    eval "$(/opt/homebrew/bin/brew shellenv)"
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install numpy scipy matplotlib

# Install MAVSDK for testing
echo "Installing MAVSDK..."
pip3 install mavsdk

# Install development tools
echo "Installing development tools..."
brew install cmake git python3

# Install additional Python packages
echo "Installing additional Python packages..."
pip3 install pytest pytest-cov black flake8 mypy

# Install ROS 2 (if available for macOS)
echo "Checking for ROS 2 macOS support..."
if command -v ros2 &> /dev/null; then
    echo "ROS 2 is already installed"
else
    echo "Note: ROS 2 Humble is not officially supported on macOS"
    echo "For full functionality, consider using:"
    echo "1. Docker container with Ubuntu 22.04"
    echo "2. Virtual Machine with Ubuntu 22.04"
    echo "3. Dual boot with Ubuntu 22.04"
fi

# Create virtual environment for development
echo "Creating Python virtual environment..."
python3 -m venv venv
source venv/bin/activate

# Install requirements in virtual environment
echo "Installing requirements in virtual environment..."
pip install -r src/swarm_core/requirements.txt

echo ""
echo "macOS development environment setup complete!"
echo ""
echo "To activate the virtual environment, run:"
echo "  source venv/bin/activate"
echo ""
echo "For full ROS 2 functionality, consider using Docker:"
echo "  docker run -it --name swarm_dev -v \$(pwd):/workspace ubuntu:22.04 bash"
echo ""
echo "Then inside Docker:"
echo "  cd /workspace && ./scripts/setup_ubuntu.sh"
