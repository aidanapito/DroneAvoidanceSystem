#!/bin/bash
# Script to run the swarm system in Docker

set -e

echo "🚀 Starting Drone Swarm System in Docker..."

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop first."
    exit 1
fi

# Check if docker-compose is available
if ! command -v docker-compose &> /dev/null; then
    echo "❌ docker-compose not found. Please install Docker Compose."
    exit 1
fi

# Navigate to the docker directory
cd "$(dirname "$0")/../docker"

# Build and start the development environment
echo "🔨 Building Docker image (this may take a while on first run)..."
docker-compose build swarm_dev

echo "🚀 Starting development container..."
docker-compose up -d swarm_dev

echo "📦 Container started successfully!"
echo ""
echo "To access the container:"
echo "  docker exec -it swarm_development bash"
echo ""
echo "Inside the container, you can:"
echo "  1. Build the workspace: colcon build"
echo "  2. Source the environment: source install/setup.bash"
echo "  3. Run the system: ros2 launch swarm_core three_uavs.launch.py"
echo ""
echo "To stop the container:"
echo "  docker-compose down"
echo ""
echo "To view logs:"
echo "  docker-compose logs -f swarm_dev"
