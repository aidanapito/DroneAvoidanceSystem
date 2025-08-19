#!/bin/bash
# Test MAVSDK connection to the containers

set -e

echo "🧪 Testing MAVSDK connections to containers..."

# Check if containers are running
if ! docker ps | grep -q px4_sitl_1; then
    echo "❌ PX4 SITL containers are not running. Start them first:"
    echo "   docker-compose -f docker-compose-px4.yml up -d"
    exit 1
fi

echo "✅ Containers are running. Testing connections..."

# Test if we can connect to the ports
echo "🔌 Testing port connectivity..."

# Test port 14540
if nc -z localhost 14540 2>/dev/null; then
    echo "✅ Port 14540 (UAV 1) is accessible"
else
    echo "❌ Port 14540 (UAV 1) is not accessible"
fi

# Test port 14541
if nc -z localhost 14541 2>/dev/null; then
    echo "✅ Port 14541 (UAV 2) is accessible"
else
    echo "❌ Port 14541 (UAV 2) is not accessible"
fi

# Test port 14542
if nc -z localhost 14542 2>/dev/null; then
    echo "✅ Port 14542 (UAV 3) is accessible"
else
    echo "❌ Port 14542 (UAV 3) is not accessible"
fi

echo ""
echo "📊 Container status:"
docker-compose -f docker-compose-px4.yml ps

echo ""
echo "🔍 Container logs:"
echo "UAV 1 logs:"
docker logs px4_sitl_1 --tail 5
echo ""
echo "UAV 2 logs:"
docker logs px4_sitl_2 --tail 5
echo ""
echo "UAV 3 logs:"
docker logs px4_sitl_3 --tail 5

echo ""
echo "💡 Next steps:"
echo "1. Install PX4 SITL in containers (takes 30-60 minutes)"
echo "2. Test actual MAVSDK connections"
echo "3. Implement basic UAV control"
echo ""
echo "🚀 To install PX4 SITL:"
echo "   ./scripts/install_px4_in_containers.sh"
echo ""
echo "🛑 To stop containers:"
echo "   docker-compose -f docker-compose-px4.yml down"
