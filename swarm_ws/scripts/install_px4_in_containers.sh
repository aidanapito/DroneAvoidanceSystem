#!/bin/bash
# Script to install PX4 SITL inside the running Docker containers

set -e

echo "🔧 Installing PX4 SITL inside Docker containers..."

# Check if containers are running
if ! docker ps | grep -q px4_sitl_1; then
    echo "❌ PX4 SITL containers are not running. Start them first:"
    echo "   docker-compose -f docker-compose-px4.yml up -d"
    exit 1
fi

echo "✅ Containers are running. Installing PX4 SITL..."

# Function to install PX4 in a container
install_px4_in_container() {
    local container_name=$1
    local port=$2
    
    echo "📦 Installing PX4 SITL in $container_name (port $port)..."
    
    # Update package list and install dependencies
    docker exec $container_name bash -c "
        apt-get update &&
        apt-get install -y \
            git \
            build-essential \
            cmake \
            ninja-build \
            genromfs \
            gperf \
            ccache \
            dfu-util \
            zip \
            unzip \
            openocd \
            libc6-dev-arm-none-eabi \
            libnewlib-arm-none-eabi \
            python3 \
            python3-pip \
            python3-dev \
            python3-setuptools \
            python3-wheel \
            wget \
            curl \
            software-properties-common
    "
    
    # Clone PX4 repository
    docker exec $container_name bash -c "
        cd /workspace &&
        if [ ! -d 'PX4-Autopilot' ]; then
            git clone https://github.com/PX4/PX4-Autopilot.git
        fi
    "
    
    # Build PX4 SITL
    echo "🔨 Building PX4 SITL in $container_name..."
    docker exec $container_name bash -c "
        cd /workspace/PX4-Autopilot &&
        make px4_sitl gz_x500
    "
    
    echo "✅ PX4 SITL installed in $container_name"
}

# Install PX4 in all three containers
echo "🚀 Installing PX4 SITL in all containers..."
install_px4_in_container "px4_sitl_1" "14540"
install_px4_in_container "px4_sitl_2" "14541" 
install_px4_in_container "px4_sitl_3" "14542"

echo ""
echo "🎉 PX4 SITL installation complete!"
echo ""
echo "📊 Container status:"
docker-compose -f docker-compose-px4.yml ps
echo ""
echo "🔌 MAVSDK connection strings:"
echo "   UAV 1: udp://:14540"
echo "   UAV 2: udp://:14541"
echo "   UAV 3: udp://:14542"
echo ""
echo "🧪 Test connection:"
echo "   python3 -c \"from mavsdk import System; print('MAVSDK imported successfully')\""
echo ""
echo "🚁 Next step: Test MAVSDK connections and basic UAV control"
