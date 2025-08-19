#!/bin/bash
# Docker workspace setup script

echo "🚀 Setting up Docker ROS 2 workspace environment..."

# Check if containers are running
if ! docker ps | grep -q ros2_gazebo_formation; then
    echo "❌ ROS 2 + Gazebo container not running. Start with:"
    echo "   docker-compose -f docker-compose-ros2-gazebo.yml up -d"
    exit 1
fi

# Set environment variables
export ROS_DOMAIN_ID=0
export DISPLAY=:0

echo "✅ Docker workspace environment ready!"
echo "   ROS 2: Available in ros2_gazebo_formation container"
echo "   Gazebo: Available in ros2_gazebo_formation container"
echo "   Formation Controller: Available in formation_controller container"
echo ""
echo "🚁 Ready for formation flying simulation!"
echo ""
echo "📋 Next steps:"
echo "1. Start containers: docker-compose -f docker-compose-ros2-gazebo.yml up -d"
echo "2. Access ROS 2: docker exec -it ros2_gazebo_formation bash"
echo "3. Access controller: docker exec -it formation_controller bash"
echo "4. Build workspace: colcon build --packages-select swarm_core"
echo "5. Run formation flying: ros2 launch swarm_core formation_flying.launch.py"
