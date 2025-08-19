#!/bin/bash
# Simple PX4 SITL setup script (alternative to full Docker setup)

set -e

echo "🚁 Setting up PX4 SITL instances..."

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found. Please install Docker Desktop first:"
    echo "   https://www.docker.com/products/docker-desktop"
    echo ""
    echo "Alternative: Use PX4 SITL directly if you have it installed"
    exit 1
fi

echo "✅ Docker found. Setting up PX4 SITL instances..."

# Create a simple docker-compose for PX4 SITL using official images
cat > docker-compose-px4.yml << 'EOF'
version: '3.8'

services:
  px4_sitl_1:
    image: px4io/px4-dev-ros2:latest
    container_name: px4_sitl_1
    environment:
      - ROS_DOMAIN_ID=0
    ports:
      - "14540:14540"
    volumes:
      - .:/workspace
    command: >
      bash -c "
        cd /workspace &&
        echo 'Starting PX4 SITL instance 1 on port 14540...' &&
        sleep 5 &&
        echo 'PX4 SITL 1 ready on port 14540'
      "

  px4_sitl_2:
    image: px4io/px4-dev-ros2:latest
    container_name: px4_sitl_2
    environment:
      - ROS_DOMAIN_ID=0
    ports:
      - "14541:14541"
    volumes:
      - .:/workspace
    command: >
      bash -c "
        cd /workspace &&
        echo 'Starting PX4 SITL instance 2 on port 14541...' &&
        sleep 5 &&
        echo 'PX4 SITL 2 ready on port 14541'
      "

  px4_sitl_3:
    image: px4io/px4-dev-ros2:latest
    container_name: px4_sitl_3
    environment:
      - ROS_DOMAIN_ID=0
    ports:
      - "14542:14542"
    volumes:
      - .:/workspace
    command: >
      bash -c "
        cd /workspace &&
        echo 'Starting PX4 SITL instance 3 on port 14542...' &&
        sleep 5 &&
        echo 'PX4 SITL 3 ready on port 14542'
      "
EOF

echo "📝 Created docker-compose-px4.yml"
echo ""
echo "🚀 To start PX4 SITL instances:"
echo "   docker-compose -f docker-compose-px4.yml up -d"
echo ""
echo "📊 To check status:"
echo "   docker-compose -f docker-compose-px4.yml ps"
echo ""
echo "📋 To view logs:"
echo "   docker-compose -f docker-compose-px4.yml logs -f px4_sitl_1"
echo ""
echo "🛑 To stop:"
echo "   docker-compose -f docker-compose-px4.yml down"
echo ""
echo "🔌 MAVSDK connection strings:"
echo "   UAV 1: udp://:14540"
echo "   UAV 2: udp://:14541"
echo "   UAV 3: udp://:14542"
echo ""
echo "💡 Note: These are placeholder containers for now."
echo "   We'll add actual PX4 SITL functionality in the next step."
