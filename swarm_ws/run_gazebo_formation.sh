#!/bin/bash

echo "🚁 Starting Gazebo Formation Flying Simulation"
echo "=============================================="

# Check if Gazebo is installed
if ! command -v gazebo &> /dev/null; then
    echo "❌ Error: Gazebo not found. Please install Gazebo first."
    echo "   Try: conda install -c conda-forge gazebo"
    exit 1
fi

# Check if ROS is available
if ! command -v roscore &> /dev/null; then
    echo "⚠️  Warning: ROS not found. The formation controller won't work without ROS."
    echo "   You can still run Gazebo to see the drones, but they won't move automatically."
    echo ""
    echo "   To install ROS on macOS, try:"
    echo "   brew install ros-humble"
    echo ""
    read -p "Continue without ROS? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORLD_FILE="$SCRIPT_DIR/worlds/formation_flying.world"

# Check if world file exists
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ Error: World file not found: $WORLD_FILE"
    exit 1
fi

echo "📍 World file: $WORLD_FILE"
echo "🎮 Starting Gazebo with formation world..."
echo ""

# Start Gazebo in the background
gazebo "$WORLD_FILE" &
GAZEBO_PID=$!

# Wait for Gazebo to start
echo "⏳ Waiting for Gazebo to initialize..."
sleep 5

# Check if Gazebo is running
if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "❌ Error: Gazebo failed to start"
    exit 1
fi

echo "✅ Gazebo started successfully (PID: $GAZEBO_PID)"
echo ""

# If ROS is available, start the formation controller
if command -v roscore &> /dev/null; then
    echo "🤖 Starting ROS formation controller..."
    echo "   This will control the drones automatically."
    echo ""
    
    # Start ROS core in background
    roscore &
    ROSCORE_PID=$!
    sleep 3
    
    # Start the formation controller
    echo "🎯 Starting formation controller..."
    python3 "$SCRIPT_DIR/gazebo_formation_controller.py" &
    CONTROLLER_PID=$!
    
    echo ""
    echo "🎉 Formation flying simulation is now running!"
    echo "=============================================="
    echo "📱 Gazebo GUI: Control camera view, pause/resume simulation"
    echo "🤖 Formation Controller: Automatically changes formations and moves drones"
    echo "📊 Status: Check terminal for formation changes and movement logs"
    echo ""
    echo "🛑 To stop: Press Ctrl+C in this terminal"
    echo ""
    
    # Wait for user to stop
    trap "echo '🛑 Stopping simulation...'; kill $GAZEBO_PID $ROSCORE_PID $CONTROLLER_PID 2>/dev/null; exit" INT
    wait
    
else
    echo "🎮 Gazebo is running with the formation world!"
    echo "=============================================="
    echo "📱 Use the Gazebo GUI to:"
    echo "   - Move the camera around (right-click + drag)"
    echo "   - Zoom in/out (scroll wheel)"
    echo "   - Pause/resume simulation (play button)"
    echo "   - Reset simulation (reset button)"
    echo ""
    echo "🛑 To stop Gazebo: Close the Gazebo window or press Ctrl+C"
    echo ""
    
    # Wait for user to stop
    trap "echo '🛑 Stopping Gazebo...'; kill $GAZEBO_PID 2>/dev/null; exit" INT
    wait
fi
