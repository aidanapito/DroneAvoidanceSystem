#!/bin/bash
# macOS Setup Script for ROS 2 and Gazebo Formation Flying

set -e

echo "🍎 Setting up macOS environment for ROS 2 and Gazebo Formation Flying..."
echo "=" * 60

# Check if Homebrew is installed
if ! command -v brew &> /dev/null; then
    echo "❌ Homebrew not found. Installing Homebrew first..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    
    # Add Homebrew to PATH for this session
    eval "$(/opt/homebrew/bin/brew shellenv)"
    
    echo "✅ Homebrew installed successfully"
else
    echo "✅ Homebrew found"
fi

# Update Homebrew
echo "📦 Updating Homebrew packages..."
brew update

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
brew install python@3.11
brew install python@3.11-pip

# Install ROS 2 Humble (macOS compatible)
echo "🤖 Installing ROS 2 Humble for macOS..."
brew install ros-humble

# Install Gazebo
echo "🌍 Installing Gazebo..."
brew install gazebo

# Install additional ROS 2 packages
echo "📚 Installing additional ROS 2 packages..."
brew install ros-humble-gazebo-ros-pkgs
brew install ros-humble-rviz2
brew install ros-humble-geometry-msgs
brew install ros-humble-nav-msgs
brew install ros-humble-std-msgs
brew install ros-humble-visualization-msgs
brew install ros-humble-std-srvs

# Install development tools
echo "🔧 Installing development tools..."
brew install cmake
brew install ninja
brew install git
brew install wget
brew install curl

# Install Python packages
echo "🐍 Installing Python packages..."
pip3 install numpy matplotlib

# Create ROS 2 environment setup
echo "📝 Setting up ROS 2 environment..."
ROS2_SETUP_FILE="$HOME/.zshrc"

if ! grep -q "source /opt/homebrew/opt/ros/humble/setup.zsh" "$ROS2_SETUP_FILE"; then
    echo "" >> "$ROS2_SETUP_FILE"
    echo "# ROS 2 Humble Setup" >> "$ROS2_SETUP_FILE"
    echo "source /opt/homebrew/opt/ros/humble/setup.zsh" >> "$ROS2_SETUP_FILE"
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/homebrew/share/gazebo-11/models" >> "$ROS2_SETUP_FILE"
    echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:/opt/homebrew/share/gazebo-11" >> "$ROS2_SETUP_FILE"
    echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:/opt/homebrew/lib/gazebo-11/plugins" >> "$ROS2_SETUP_FILE"
    echo "✅ ROS 2 environment added to ~/.zshrc"
else
    echo "✅ ROS 2 environment already configured"
fi

# Install colcon build tools
echo "🔨 Installing colcon build tools..."
pip3 install colcon-common-extensions

# Create workspace setup
echo "📁 Setting up workspace..."
WORKSPACE_DIR="$PWD"
WORKSPACE_SETUP="$WORKSPACE_DIR/setup_workspace.sh"

cat > "$WORKSPACE_SETUP" << 'EOF'
#!/bin/bash
# Workspace setup script for macOS

echo "🚀 Setting up ROS 2 workspace environment..."

# Source ROS 2
source /opt/homebrew/opt/ros/humble/setup.zsh

# Set Gazebo paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/homebrew/share/gazebo-11/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/homebrew/share/gazebo-11
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/homebrew/lib/gazebo-11/plugins

# Set ROS domain ID (useful for multiple users)
export ROS_DOMAIN_ID=0

echo "✅ Workspace environment ready!"
echo "   ROS 2: $(ros2 --version)"
echo "   Gazebo: $(gazebo --version)"
echo "   Python: $(python3 --version)"
echo ""
echo "🚁 Ready for formation flying simulation!"
EOF

chmod +x "$WORKSPACE_SETUP"

# Test installations
echo "🧪 Testing installations..."
echo ""

# Test ROS 2
if command -v ros2 &> /dev/null; then
    echo "✅ ROS 2 installed: $(ros2 --version)"
else
    echo "❌ ROS 2 installation failed"
    exit 1
fi

# Test Gazebo
if command -v gazebo &> /dev/null; then
    echo "✅ Gazebo installed: $(gazebo --version)"
else
    echo "❌ Gazebo installation failed"
    exit 1
fi

# Test Python
if command -v python3 &> /dev/null; then
    echo "✅ Python installed: $(python3 --version)"
else
    echo "❌ Python installation failed"
    exit 1
fi

echo ""
echo "🎉 Setup completed successfully!"
echo "=" * 60
echo ""
echo "📋 Next steps:"
echo "1. Restart your terminal or run: source ~/.zshrc"
echo "2. Navigate to your workspace: cd $WORKSPACE_DIR"
echo "3. Set up the workspace: source setup_workspace.sh"
echo "4. Build the workspace: colcon build --packages-select swarm_core"
echo "5. Run formation flying: python3 scripts/run_gazebo_formation.py"
echo ""
echo "🔧 Troubleshooting:"
echo "- If ROS 2 commands not found, restart terminal or source ~/.zshrc"
echo "- If Gazebo issues, check: brew doctor"
echo "- For build issues, run: rosdep install --from-paths src --ignore-src -r -y"
echo ""
echo "🚁 Happy Formation Flying! 🚁"
