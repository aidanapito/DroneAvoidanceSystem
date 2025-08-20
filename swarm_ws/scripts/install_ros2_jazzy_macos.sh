#!/bin/bash

echo "🚀 Installing ROS 2 Jazzy on macOS"
echo "=================================="

# Check if Homebrew is installed
if ! command -v brew &> /dev/null; then
    echo "❌ Error: Homebrew not found. Please install Homebrew first."
    echo "   Visit: https://brew.sh"
    exit 1
fi

# Check if Python 3.11+ is available
python_version=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
required_version="3.11"

if [ "$(printf '%s\n' "$required_version" "$python_version" | sort -V | head -n1)" != "$required_version" ]; then
    echo "❌ Error: Python 3.11+ required, found $python_version"
    echo "   ROS 2 Jazzy needs Python 3.11 or higher"
    exit 1
fi

echo "✅ Python $python_version detected"

# Install system dependencies
echo "📦 Installing system dependencies..."
brew install cmake pkg-config cppcheck python@3.11

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
pip3 install colcon-common-extensions

# Create ROS 2 workspace
echo "🏗️  Creating ROS 2 workspace..."
mkdir -p ~/ros2_jazzy_ws/src
cd ~/ros2_jazzy_ws

# Clone ROS 2 Jazzy source
echo "📥 Cloning ROS 2 Jazzy source..."
git clone https://github.com/ros2/ros2.git -b jazzy src/ros2

# Install additional dependencies
echo "🔧 Installing additional dependencies..."
brew install asio tinyxml2 tinyxml eigen pcre log4cxx

echo "✅ ROS 2 Jazzy dependencies installed!"
echo ""
echo "🚀 Next steps:"
echo "1. cd ~/ros2_jazzy_ws"
echo "2. rosdep install --from-paths src --ignore-src -r -y"
echo "3. colcon build --symlink-install"
echo "4. source install/setup.zsh"
echo ""
echo "📚 For detailed instructions, visit:"
echo "   https://docs.ros.org/en/jazzy/Installation.html"
