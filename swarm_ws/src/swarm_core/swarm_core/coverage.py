#!/usr/bin/env python3
"""
Grid Coverage Node

Handles grid coverage task allocation, maintains occupancy grid,
and assigns coverage targets to individual UAVs.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import math
from typing import Dict, List, Optional, Tuple, Set
from dataclasses import dataclass
from enum import Enum
import heapq

# ROS 2 messages
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import Float32, Bool, Int32
from visualization_msgs.msg import MarkerArray, Marker

# Local imports
from .utils.transforms import TransformUtils


class CoverageState(Enum):
    """Enumeration of coverage states."""
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"


@dataclass
class GridCell:
    """Data class for a grid cell."""
    x: int
    y: int
    visited: bool = False
    assigned_to: Optional[str] = None
    cost: float = 1.0
    timestamp: float = 0.0


@dataclass
class CoverageTarget:
    """Data class for a coverage target."""
    uav_id: str
    position: Point
    priority: int
    timestamp: float
    completed: bool = False


class AStarPlanner:
    """A* path planner for grid-based navigation."""
    
    def __init__(self, grid: np.ndarray, resolution: float):
        """
        Initialize A* planner.
        
        Args:
            grid: 2D occupancy grid (0 = free, 100 = occupied)
            resolution: Grid resolution in meters
        """
        self.grid = grid
        self.resolution = resolution
        self.height, self.width = grid.shape
        
        # Movement directions (8-connected)
        self.directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
    
    def world_to_grid(self, world_pos: Point) -> Tuple[int, int]:
        """
        Convert world position to grid coordinates.
        
        Args:
            world_pos: Position in world coordinates
            
        Returns:
            Grid coordinates (x, y)
        """
        grid_x = int(world_pos.x / self.resolution)
        grid_y = int(world_pos.y / self.resolution)
        
        # Clamp to grid bounds
        grid_x = max(0, min(grid_x, self.width - 1))
        grid_y = max(0, min(grid_y, self.height - 1))
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Point:
        """
        Convert grid coordinates to world position.
        
        Args:
            grid_x: Grid x coordinate
            grid_y: Grid y coordinate
            
        Returns:
            World position
        """
        world_pos = Point()
        world_pos.x = grid_x * self.resolution
        world_pos.y = grid_y * self.resolution
        world_pos.z = 0.0
        return world_pos
    
    def is_valid_cell(self, grid_x: int, grid_y: int) -> bool:
        """
        Check if grid cell is valid and free.
        
        Args:
            grid_x: Grid x coordinate
            grid_y: Grid y coordinate
            
        Returns:
            True if cell is valid and free
        """
        if (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return self.grid[grid_y, grid_x] < 50  # Free space threshold
        return False
    
    def heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """
        Calculate heuristic distance between two grid cells.
        
        Args:
            x1, y1: First cell coordinates
            x2, y2: Second cell coordinates
            
        Returns:
            Heuristic distance
        """
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        return math.sqrt(dx*dx + dy*dy)
    
    def find_path(self, start_pos: Point, goal_pos: Point) -> Optional[Path]:
        """
        Find path from start to goal using A*.
        
        Args:
            start_pos: Start position in world coordinates
            goal_pos: Goal position in world coordinates
            
        Returns:
            Path message or None if no path found
        """
        start_x, start_y = self.world_to_grid(start_pos)
        goal_x, goal_y = self.world_to_grid(goal_pos)
        
        if not self.is_valid_cell(start_x, start_y) or not self.is_valid_cell(goal_x, goal_y):
            return None
        
        # Priority queue for open set
        open_set = []
        heapq.heappush(open_set, (0, start_x, start_y))
        
        # Came from and cost maps
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.heuristic(start_x, start_y, goal_x, goal_y)}
        
        while open_set:
            current_f, current_x, current_y = heapq.heappop(open_set)
            
            if current_x == goal_x and current_y == goal_y:
                # Reconstruct path
                return self._reconstruct_path(came_from, current_x, current_y, start_pos, goal_pos)
            
            # Check all neighbors
            for dx, dy in self.directions:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy
                
                if not self.is_valid_cell(neighbor_x, neighbor_y):
                    continue
                
                # Calculate movement cost
                movement_cost = math.sqrt(dx*dx + dy*dy)
                tentative_g = g_score.get((current_x, current_y), float('inf')) + movement_cost
                
                if tentative_g < g_score.get((neighbor_x, neighbor_y), float('inf')):
                    # This path is better
                    came_from[(neighbor_x, neighbor_y)] = (current_x, current_y)
                    g_score[(neighbor_x, neighbor_y)] = tentative_g
                    f_score[(neighbor_x, neighbor_y)] = tentative_g + self.heuristic(neighbor_x, neighbor_y, goal_x, goal_y)
                    
                    # Add to open set
                    heapq.heappush(open_set, (f_score[(neighbor_x, neighbor_y)], neighbor_x, neighbor_y))
        
        return None  # No path found
    
    def _reconstruct_path(self, came_from: Dict, goal_x: int, goal_y: int, 
                         start_pos: Point, goal_pos: Point) -> Path:
        """
        Reconstruct path from came_from map.
        
        Args:
            came_from: Map of cell predecessors
            goal_x, goal_y: Goal cell coordinates
            start_pos: Start position in world coordinates
            goal_pos: Goal position in world coordinates
            
        Returns:
            Path message
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rclpy.time.Time().to_msg()
        
        # Reconstruct path backwards
        current_x, current_y = goal_x, goal_y
        path_points = []
        
        while (current_x, current_y) in came_from:
            world_pos = self.grid_to_world(current_x, current_y)
            pose = PoseStamped()
            pose.pose.position = world_pos
            pose.pose.orientation.w = 1.0
            path_points.append(pose)
            
            current_x, current_y = came_from[(current_x, current_y)]
        
        # Add start position
        start_pose = PoseStamped()
        start_pose.pose.position = start_pos
        start_pose.pose.orientation.w = 1.0
        path_points.append(start_pose)
        
        # Reverse to get correct order
        path.poses = list(reversed(path_points))
        
        return path


class GridCoverage(Node):
    """
    Grid coverage node.
    
    Responsibilities:
    - Maintain occupancy grid of the world
    - Allocate coverage tasks to UAVs
    - Plan paths using A* algorithm
    - Track coverage progress
    - Publish coverage map and targets
    """
    
    def __init__(self):
        """Initialize the grid coverage node."""
        super().__init__('grid_coverage')
        
        # Initialize parameters
        self._init_parameters()
        
        # Initialize state
        self.coverage_state = CoverageState.IDLE
        self.occupancy_grid: Optional[np.ndarray] = None
        self.grid_resolution = 0.5  # meters
        self.world_bounds = {
            'x_min': -20.0, 'x_max': 20.0,
            'y_min': -20.0, 'y_max': 20.0
        }
        
        # Coverage tracking
        self.visited_cells: Set[Tuple[int, int]] = set()
        self.total_cells = 0
        self.coverage_percentage = 0.0
        
        # UAV assignments
        self.uav_positions: Dict[str, Point] = {}
        self.coverage_targets: Dict[str, List[CoverageTarget]] = {}
        self.current_targets: Dict[str, CoverageTarget] = {}
        
        # Path planning
        self.planner: Optional[AStarPlanner] = None
        
        # Performance tracking
        self.coverage_start_time = 0.0
        self.coverage_history: List[Tuple[float, float]] = []
        
        # Initialize ROS 2 components
        self._init_ros_components()
        
        # Initialize grid
        self._init_occupancy_grid()
        
        # Start main control loop
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('coverage.target_selection.update_frequency').value,
            self._control_loop_callback
        )
        
        self.get_logger().info("Grid Coverage initialized")
    
    def _init_parameters(self):
        """Initialize node parameters from config file."""
        # Coverage parameters
        self.declare_parameter('coverage.grid_resolution', 0.5)
        self.declare_parameter('coverage.min_coverage_threshold', 0.95)
        self.declare_parameter('coverage.target_selection.method', 'round_robin')
        self.declare_parameter('coverage.target_selection.update_frequency', 1.0)
        
        # World bounds
        self.declare_parameter('coverage.world_bounds.x_min', -20.0)
        self.declare_parameter('coverage.world_bounds.x_max', 20.0)
        self.declare_parameter('coverage.world_bounds.y_min', -20.0)
        self.declare_parameter('coverage.world_bounds.y_max', 20.0)
        
        # UAV parameters
        self.declare_parameter('uav.count', 3)
        self.declare_parameter('uav.safety_radius', 1.5)
    
    def _init_ros_components(self):
        """Initialize ROS 2 publishers, subscribers, and services."""
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        fast_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.coverage_map_pub = self.create_publisher(
            OccupancyGrid, '/swarm/coverage_map', reliable_qos
        )
        
        self.coverage_status_pub = self.create_publisher(
            Float32, '/swarm/coverage_percentage', fast_qos
        )
        
        self.coverage_targets_pub = self.create_publisher(
            MarkerArray, '/swarm/coverage_targets', fast_qos
        )
        
        self.uav_paths_pub = self.create_publisher(
            Path, '/swarm/uav_paths', fast_qos
        )
        
        # Subscribers
        self.odom_subscribers = {}
        self._setup_odometry_subscribers()
        
        # Callback groups for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
    
    def _setup_odometry_subscribers(self):
        """Set up odometry subscribers for all UAVs."""
        uav_count = self.get_parameter('uav.count').value
        
        for i in range(uav_count):
            uav_name = f"uav{i+1}"
            topic_name = f"/{uav_name}/odometry"
            subscriber = self.create_subscription(
                Odometry, topic_name,
                lambda msg, name=uav_name: self._odometry_callback(msg, name),
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)
            )
            self.odom_subscribers[uav_name] = subscriber
    
    def _odometry_callback(self, msg: Odometry, uav_id: str):
        """Handle odometry updates from UAVs."""
        try:
            position = msg.pose.pose.position
            self.uav_positions[uav_id] = position
            
            # Update visited cells
            self._update_visited_cells(uav_id, position)
            
        except Exception as e:
            self.get_logger().error(f"Error processing odometry from {uav_id}: {e}")
    
    def _init_occupancy_grid(self):
        """Initialize the occupancy grid."""
        # Calculate grid dimensions
        x_size = int((self.world_bounds['x_max'] - self.world_bounds['x_min']) / self.grid_resolution)
        y_size = int((self.world_bounds['y_max'] - self.world_bounds['y_min']) / self.grid_resolution)
        
        # Create empty grid (0 = free space)
        self.occupancy_grid = np.zeros((y_size, x_size), dtype=np.uint8)
        self.total_cells = x_size * y_size
        
        # Initialize planner
        self.planner = AStarPlanner(self.occupancy_grid, self.grid_resolution)
        
        self.get_logger().info(f"Initialized occupancy grid: {x_size}x{y_size} cells")
    
    def _update_visited_cells(self, uav_id: str, position: Point):
        """Update visited cells based on UAV position."""
        if not self.planner:
            return
        
        # Convert position to grid coordinates
        grid_x, grid_y = self.planner.world_to_grid(position)
        
        # Mark cell as visited
        if (grid_x, grid_y) not in self.visited_cells:
            self.visited_cells.add((grid_x, grid_y))
            
            # Update coverage percentage
            self.coverage_percentage = len(self.visited_cells) / self.total_cells
            
            # Add to history
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.coverage_history.append((current_time, self.coverage_percentage))
            
            # Keep only recent history
            if len(self.coverage_history) > 1000:
                self.coverage_history = self.coverage_history[-1000:]
    
    def _control_loop_callback(self):
        """Main control loop callback."""
        try:
            # Update coverage state
            self._update_coverage_state()
            
            # Execute current state
            self._execute_coverage_state()
            
            # Publish updates
            self._publish_updates()
            
        except Exception as e:
            self.get_logger().error(f"Error in coverage control loop: {e}")
    
    def _update_coverage_state(self):
        """Update coverage state based on current conditions."""
        if self.coverage_state == CoverageState.IDLE:
            # Check if we have UAVs and should start coverage
            if len(self.uav_positions) > 0:
                self.coverage_state = CoverageState.PLANNING
                self.coverage_start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info("Starting coverage planning")
                
        elif self.coverage_state == CoverageState.PLANNING:
            # Check if planning is complete
            if self._planning_complete():
                self.coverage_state = CoverageState.EXECUTING
                self.get_logger().info("Coverage planning complete, starting execution")
                
        elif self.coverage_state == CoverageState.EXECUTING:
            # Check if coverage is complete
            if self.coverage_percentage >= self.get_parameter('coverage.min_coverage_threshold').value:
                self.coverage_state = CoverageState.COMPLETED
                self.get_logger().info(f"Coverage complete: {self.coverage_percentage:.1%}")
    
    def _execute_coverage_state(self):
        """Execute actions for the current coverage state."""
        if self.coverage_state == CoverageState.PLANNING:
            self._execute_planning()
            
        elif self.coverage_state == CoverageState.EXECUTING:
            self._execute_coverage()
            
        elif self.coverage_state == CoverageState.COMPLETED:
            self._execute_completion()
    
    def _execute_planning(self):
        """Execute coverage planning phase."""
        # Generate coverage targets for each UAV
        self._generate_coverage_targets()
        
        # Plan paths to targets
        self._plan_paths_to_targets()
    
    def _execute_coverage(self):
        """Execute coverage execution phase."""
        # Update target assignments based on current UAV positions
        self._update_target_assignments()
        
        # Replan paths if needed
        self._replan_paths_if_needed()
    
    def _execute_completion(self):
        """Execute coverage completion phase."""
        # Coverage is complete, maintain current positions
        pass
    
    def _generate_coverage_targets(self):
        """Generate coverage targets for all UAVs."""
        if not self.occupancy_grid:
            return
        
        # Clear existing targets
        self.coverage_targets.clear()
        
        # Get unvisited cells
        unvisited_cells = []
        for y in range(self.occupancy_grid.shape[0]):
            for x in range(self.occupancy_grid.shape[1]):
                if (x, y) not in self.visited_cells:
                    world_pos = self.planner.grid_to_world(x, y)
                    unvisited_cells.append((x, y, world_pos))
        
        # Assign targets to UAVs
        uav_ids = list(self.uav_positions.keys())
        if not uav_ids:
            return
        
        # Round-robin assignment
        for i, (x, y, world_pos) in enumerate(unvisited_cells):
            uav_id = uav_ids[i % len(uav_ids)]
            
            if uav_id not in self.coverage_targets:
                self.coverage_targets[uav_id] = []
            
            target = CoverageTarget(
                uav_id=uav_id,
                position=world_pos,
                priority=i,
                timestamp=self.get_clock().now().nanoseconds / 1e9
            )
            
            self.coverage_targets[uav_id].append(target)
        
        self.get_logger().info(f"Generated {len(unvisited_cells)} coverage targets")
    
    def _plan_paths_to_targets(self):
        """Plan paths from UAVs to their coverage targets."""
        if not self.planner:
            return
        
        for uav_id, targets in self.coverage_targets.items():
            if not targets:
                continue
            
            # Get current UAV position
            current_pos = self.uav_positions.get(uav_id)
            if not current_pos:
                continue
            
            # Plan path to first target
            first_target = targets[0]
            path = self.planner.find_path(current_pos, first_target.position)
            
            if path:
                # Publish path for visualization
                self.uav_paths_pub.publish(path)
                
                # Set current target
                self.current_targets[uav_id] = first_target
    
    def _update_target_assignments(self):
        """Update target assignments based on current UAV positions."""
        for uav_id, targets in self.coverage_targets.items():
            if not targets:
                continue
            
            current_pos = self.uav_positions.get(uav_id)
            if not current_pos:
                continue
            
            # Check if current target is reached
            current_target = self.current_targets.get(uav_id)
            if current_target:
                distance = TransformUtils.calculate_2d_distance(current_pos, current_target.position)
                tolerance = self.grid_resolution * 0.5
                
                if distance < tolerance:
                    # Target reached, mark as completed
                    current_target.completed = True
                    
                    # Remove from targets list
                    if current_target in targets:
                        targets.remove(current_target)
                    
                    # Assign next target
                    if targets:
                        next_target = targets[0]
                        self.current_targets[uav_id] = next_target
                        
                        # Plan path to next target
                        path = self.planner.find_path(current_pos, next_target.position)
                        if path:
                            self.uav_paths_pub.publish(path)
                    else:
                        # No more targets for this UAV
                        if uav_id in self.current_targets:
                            del self.current_targets[uav_id]
    
    def _replan_paths_if_needed(self):
        """Replan paths if UAVs have deviated significantly."""
        # This could be implemented to check if UAVs have deviated
        # from their planned paths and replan if necessary
        pass
    
    def _planning_complete(self) -> bool:
        """Check if coverage planning is complete."""
        return len(self.coverage_targets) > 0
    
    def _publish_updates(self):
        """Publish coverage updates."""
        # Publish coverage map
        self._publish_coverage_map()
        
        # Publish coverage status
        status_msg = Float32()
        status_msg.data = self.coverage_percentage
        self.coverage_status_pub.publish(status_msg)
        
        # Publish coverage targets visualization
        self._publish_targets_visualization()
    
    def _publish_coverage_map(self):
        """Publish the occupancy grid as an OccupancyGrid message."""
        if not self.occupancy_grid:
            return
        
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set grid metadata
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width = self.occupancy_grid.shape[1]
        grid_msg.info.height = self.occupancy_grid.shape[0]
        
        # Set origin (bottom-left corner)
        grid_msg.info.origin.position.x = self.world_bounds['x_min']
        grid_msg.info.origin.position.y = self.world_bounds['y_min']
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Set grid data
        grid_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.coverage_map_pub.publish(grid_msg)
    
    def _publish_targets_visualization(self):
        """Publish coverage targets for visualization in RViz."""
        marker_array = MarkerArray()
        
        # Publish current targets
        for uav_id, target in self.current_targets.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"target_{uav_id}"
            marker.id = hash(uav_id) % 1000
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position = target.position
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.8
            
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        # Publish all targets
        for uav_id, targets in self.coverage_targets.items():
            for i, target in enumerate(targets):
                if target.completed:
                    continue
                
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"all_targets_{uav_id}"
                marker.id = hash(f"{uav_id}_{i}") % 1000
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                marker.pose.position = target.position
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.6
                
                marker_array.markers.append(marker)
        
        self.coverage_targets_pub.publish(marker_array)
    
    def get_coverage_stats(self) -> Dict[str, float]:
        """Get coverage statistics."""
        return {
            'coverage_percentage': self.coverage_percentage,
            'visited_cells': len(self.visited_cells),
            'total_cells': self.total_cells,
            'uav_count': len(self.uav_positions),
            'active_targets': len(self.current_targets)
        }


def main(args=None):
    """Main function for the grid coverage node."""
    rclpy.init(args=args)
    
    try:
        coverage_node = GridCoverage()
        rclpy.spin(coverage_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in grid coverage: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
