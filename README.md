# Drone Swarm Collision Avoidance System

A ROS 2-based multi-UAV system with ORCA collision avoidance and grid coverage task allocation.

## 🎯 Project Overview

This system demonstrates coordinated flight of 3 simulated quadcopters in Gazebo/PX4 SITL with:
- **Collision Avoidance**: ORCA-based velocity planning for safe navigation
- **Task Allocation**: Grid coverage optimization with shared occupancy mapping
- **Zero Collisions**: Guaranteed safety margins between all vehicles
- **High Coverage**: ≥95% area coverage with efficient path planning

## 🏗️ Architecture

- **Simulation**: Gazebo + PX4 SITL (x500 model)
- **Control**: MAVSDK Python bindings for offboard control
- **Coordination**: ROS 2 nodes for swarm management
- **Avoidance**: ORCA algorithm for collision-free trajectories
- **Coverage**: Occupancy grid with A* path planning

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo (Fortress/Garden)
- PX4 SITL
- MAVSDK Python

### Installation

1. **Clone and setup workspace:**
   ```bash
   cd ~/swarm_ws
   colcon build
   source install/setup.bash
   ```

2. **Launch the system:**
   ```bash
   ros2 launch swarm_core three_uavs.launch.py world:=cluttered
   ```

3. **Monitor performance:**
   ```bash
   # In another terminal
   ros2 topic echo /swarm/coverage_map
   ros2 topic echo /swarm/status
   ```

### Demo Scenarios

- **Open World**: `world:=open` - Basic coverage in empty environment
- **Cluttered World**: `world:=cluttered` - Coverage with obstacles
- **Formation Mode**: `world:=formation` - Coordinated formation flight

## 📊 Performance Metrics

- **Safety**: Minimum pairwise distance ≥ safety radius
- **Coverage**: ≥95% area coverage within time budget
- **Efficiency**: Path length optimization and idle time minimization
- **Robustness**: Completion under agent failure scenarios

## 🧪 Testing

```bash
# Run unit tests
colcon test --packages-select swarm_core

# Run integration tests
ros2 launch swarm_core test_scenarios.launch.py

# Generate evaluation report
python3 scripts/eval.py --rosbag latest_run.bag
```

## 📈 Evaluation

The system automatically generates performance reports including:
- Coverage curves over time
- Separation distance histograms
- Path efficiency metrics
- Collision avoidance statistics

## 🔧 Configuration

Key parameters can be tuned in `config/params.yaml`:
- ORCA safety radius and time horizon
- Grid resolution and coverage thresholds
- Control loop frequencies
- Safety margins and limits

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- PX4 Autopilot team for the simulation framework
- ROS 2 community for the robotics middleware
- ORCA algorithm authors for collision avoidance theory
