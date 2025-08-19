#!/bin/bash
# Quick start script for Docker-based formation flying

echo "🚁 Starting Formation Flying Simulation with Docker..."
echo "=" * 50

# Check if Docker is running
if ! docker info &> /dev/null; then
    echo "❌ Docker is not running. Please start Docker Desktop."
    exit 1
fi

# Start containers
echo "🐳 Starting ROS 2 + Gazebo containers..."
docker-compose -f docker-compose-ros2-gazebo.yml up -d

# Wait for containers to be ready
echo "⏳ Waiting for containers to be ready..."
sleep 10

# Check container status
echo "📊 Container status:"
docker-compose -f docker-compose-ros2-gazebo.yml ps

echo ""
echo "🎯 Containers are running!"
echo ""
echo "📋 To access the simulation:"
echo "1. Open Gazebo: docker exec -it ros2_gazebo_formation gazebo"
echo "2. Access ROS 2: docker exec -it ros2_gazebo_formation bash"
echo "3. Run formation flying:"
echo "   docker exec -it formation_controller bash"
echo "   source /opt/ros/humble/setup.bash"
echo "   cd /workspace"
echo "   colcon build --packages-select swarm_core"
echo "   ros2 launch swarm_core formation_flying.launch.py"
echo ""
echo "🛑 To stop: docker-compose -f docker-compose-ros2-gazebo.yml down"
