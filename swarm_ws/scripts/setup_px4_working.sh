#!/bin/bash
# Working PX4 SITL setup script using standard Ubuntu images

set -e

echo "🚁 Setting up PX4 SITL instances with working Docker images..."

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found. Please install Docker Desktop first:"
    echo "   https://www.docker.com/products/docker-desktop"
    exit 1
fi

echo "✅ Docker found. Setting up PX4 SITL instances..."

# Create a working docker-compose using standard Ubuntu images
cat > docker-compose-px4.yml << 'EOF'
version: '3.8'

services:
  px4_sitl_1:
    image: ubuntu:22.04
    container_name: px4_sitl_1
    environment:
      - ROS_DOMAIN_ID=0
    ports:
      - "14540:14540"
    volumes:
      - .:/workspace
    command: >
      bash -c "
        echo 'Starting PX4 SITL instance 1 on port 14540...' &&
        echo 'Container ready - PX4 SITL will be installed in next step' &&
        tail -f /dev/null
      "

  px4_sitl_2:
    image: ubuntu:22.04
    container_name: px4_sitl_2
    environment:
      - ROS_DOMAIN_ID=0
    ports:
      - "14541:14541"
    volumes:
      - .:/workspace
    command: >
      bash -c "
        echo 'Starting PX4 SITL instance 2 on port 14541...' &&
        echo 'Container ready - PX4 SITL will be installed in next step' &&
        tail -f /dev/null
      "

  px4_sitl_3:
    image: ubuntu:22.04
    container_name: px4_sitl_3
    environment:
      - ROS_DOMAIN_ID=0
    ports:
      - "14542:14542"
    volumes:
      - .:/workspace
    command: >
      bash -c "
        echo 'Starting PX4 SITL instance 3 on port 14542...' &&
        echo 'Container ready - PX4 SITL will be installed in next step' &&
        tail -f /dev/null
      "
EOF

echo "📝 Created docker-compose-px4.yml with Ubuntu 22.04 images"
echo ""
echo "🚀 To start the containers:"
echo "   docker-compose -f docker-compose-px4.yml up -d"
echo ""
echo "📊 To check status:"
echo "   docker-compose -f docker-compose-px4.yml ps"
echo ""
echo "🔌 To access containers:"
echo "   docker exec -it px4_sitl_1 bash"
echo "   docker exec -it px4_sitl_2 bash"
echo "   docker exec -it px4_sitl_3 bash"
echo ""
echo "🛑 To stop:"
echo "   docker-compose -f docker-compose-px4.yml down"
echo ""
echo "💡 Next step: Install PX4 SITL inside each container"
