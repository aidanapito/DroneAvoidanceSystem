#!/bin/bash

echo "🚁 Starting Enhanced Formation Flying Simulator"
echo "==============================================="
echo "🎯 This is a high-quality alternative to Gazebo"
echo "   with full 3D visualization and physics simulation"
echo ""

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Error: Python 3 not found"
    exit 1
fi

# Check if required Python packages are available
echo "🔍 Checking Python dependencies..."
python3 -c "import matplotlib, numpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️  Warning: Some Python packages may be missing"
    echo "   Required: matplotlib, numpy"
    echo "   Install with: pip install matplotlib numpy"
    echo ""
    read -p "Continue anyway? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "📍 Script directory: $SCRIPT_DIR"
echo "🎮 Starting enhanced formation simulator..."
echo ""

# Check if the enhanced simulator exists
if [ -f "$SCRIPT_DIR/formation_flying_simulator.py" ]; then
    echo "✅ Found enhanced simulator - launching with full features!"
    python3 "$SCRIPT_DIR/formation_flying_simulator.py"
else
    echo "⚠️  Enhanced simulator not found, using basic version..."
    if [ -f "$SCRIPT_DIR/formation_simulator.py" ]; then
        python3 "$SCRIPT_DIR/formation_simulator.py"
    else
        echo "❌ Error: No formation simulator found"
        echo "   Please run the setup script first"
        exit 1
    fi
fi

echo ""
echo "🎉 Formation simulator completed!"
