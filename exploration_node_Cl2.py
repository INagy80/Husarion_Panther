#!/usr/bin/env python3
"""
Enhanced Exploration Node - Fast, Smooth, Robust Navigation
Key improvements:
1. Smart goal selection - finds nearest free space to frontier
2. Improved A* with fallback goal finding
3. Fast, smooth path following with adaptive lookahead
4. Better frontier detection and blacklist management
5. Robust stuck detection and recovery
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, TwistStamped, PoseArray, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
import math
import time
import os
import yaml
from collections import deque
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set
import heapq


# Cost values (similar to m-explore)
FREE_SPACE = 0
LETHAL_OBSTACLE = 254
INSCRIBED_OBSTACLE = 253
NO_INFORMATION = 255


class State(Enum):
    WAIT_MAP = 0
    IDLE = 1
    DETECT_FRONTIERS = 2
    PLAN_PATH = 3
    MOVE_ALONG_PATH = 4
    STUCK_RECOVERY = 5
    DONE = 6


@dataclass
class Frontier:
    """Frontier representation (like m-explore)"""
    cells: List[Tuple[int, int]]
    centroid: Tuple[float, float]
    middle: Tuple[float, float]
    initial: Tuple[float, float]
    size: int
    min_distance: float
    cost: float = 0.0
    reachable_goal: Optional[Tuple[int, int]] = None  # Nearest free space cell


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        
        # Parameters
        self.declare_parameter('robot_width', 0.85)
        self.declare_parameter('robot_length', 0.81)
        self.declare_parameter('occupied_threshold', 30)
        self.declare_parameter('inflation_radius_cells', 6)  # Reduced for less conservative planning
        
        # Frontier search parameters
        self.declare_parameter('min_frontier_size', 0.3)  # Smaller to find more frontiers
        self.declare_parameter('potential_scale', 0.001)
        self.declare_parameter('gain_scale', 1.0)
        
        # Fast, smooth control parameters
        self.declare_parameter('max_angular_vel', 0.5)  # Increased for faster turns
        self.declare_parameter('max_linear_vel', 0.6)  # Increased for faster movement
        self.declare_parameter('min_linear_vel', 0.1)  # Higher minimum for smoother motion
        self.declare_parameter('k_angular', 2.5)  # Increased for more responsive turns
        self.declare_parameter('k_linear', 1.2)  # Increased for faster response
        
        # Rotation to heading parameters (Nav2-style)
        self.declare_parameter('rotate_to_heading_min_angle', 0.4)  # Rotate in place if angle > this (rad)
        self.declare_parameter('rotate_to_heading_angular_vel', 0.5)  # Angular vel when rotating in place
        self.declare_parameter('use_rotate_to_heading', True)  # Enable rotate-then-move behavior
        
        # Smooth motion parameters
        self.declare_parameter('angular_smoothing_factor', 0.5)  # Less smoothing for faster response
        self.declare_parameter('linear_smoothing_factor', 0.6)
        self.declare_parameter('max_angular_accel', 2.5)  # Faster acceleration
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('angular_deadband', 0.05)
        
        # Path following parameters - hybrid approach
        self.declare_parameter('waypoint_tolerance', 0.15)  # When to advance to next waypoint
        self.declare_parameter('lookahead_dist', 0.8)  # Lookahead for smooth following
        self.declare_parameter('min_lookahead_dist', 0.5)
        self.declare_parameter('max_lookahead_dist', 2.0)
        self.declare_parameter('adaptive_lookahead', True)
        
        # Navigation parameters
        self.declare_parameter('final_goal_tolerance', 0.25)
        self.declare_parameter('stuck_timeout', 8.0)  # Reduced timeout
        self.declare_parameter('stuck_distance_threshold', 0.2)
        self.declare_parameter('control_rate', 30.0)
        
        # Safety parameters
        self.declare_parameter('collision_check_distance', 1.0)
        self.declare_parameter('emergency_stop_distance', 0.2)
        self.declare_parameter('progress_timeout', 60.0)  # Longer timeout
        
        # Map saving parameters
        self.declare_parameter('map_save_dir', '/home/inagy/my_robot/src/husarion_ugv_ros/husarion_ugv_gazebo/maps')
        self.declare_parameter('map_filename', 'exploredmap')
        
        # Get parameters
        self.robot_width = self.get_parameter('robot_width').value
        self.robot_length = self.get_parameter('robot_length').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.inflation_radius_cells = self.get_parameter('inflation_radius_cells').value
        
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.potential_scale = self.get_parameter('potential_scale').value
        self.gain_scale = self.get_parameter('gain_scale').value
        
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.k_angular = self.get_parameter('k_angular').value
        self.k_linear = self.get_parameter('k_linear').value
        
        self.rotate_to_heading_min_angle = self.get_parameter('rotate_to_heading_min_angle').value
        self.rotate_to_heading_angular_vel = self.get_parameter('rotate_to_heading_angular_vel').value
        self.use_rotate_to_heading = self.get_parameter('use_rotate_to_heading').value
        
        self.angular_smoothing_factor = self.get_parameter('angular_smoothing_factor').value
        self.linear_smoothing_factor = self.get_parameter('linear_smoothing_factor').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.angular_deadband = self.get_parameter('angular_deadband').value
        
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.min_lookahead_dist = self.get_parameter('min_lookahead_dist').value
        self.max_lookahead_dist = self.get_parameter('max_lookahead_dist').value
        self.adaptive_lookahead = self.get_parameter('adaptive_lookahead').value
        
        self.final_goal_tolerance = self.get_parameter('final_goal_tolerance').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').value
        
        self.collision_check_distance = self.get_parameter('collision_check_distance').value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        control_rate = self.get_parameter('control_rate').value
        
        self.map_save_dir = self.get_parameter('map_save_dir').value
        self.map_filename = self.get_parameter('map_filename').value
        
        # State variables
        self.state = State.WAIT_MAP
        self.map_data = None
        self.map_grid = None
        self.map_info = None
        self.costmap = None
        self.robot_pose = None
        self.robot_yaw = None
        
        self.frontiers = []
        self.current_goal = None
        self.current_path = []
        self.current_waypoint_idx = 0
        
        self.blacklist = []
        self.blacklist_timestamps = {}  # Track when goals were blacklisted
        self.blacklist_timeout = 120.0  # Unblacklist after 2 minutes
        self.prev_goal = None
        self.last_progress_time = time.time()
        self.prev_distance = 0.0
        
        self.stuck_count = 0
        self.stuck_timer_start = None
        self.last_pose_for_stuck = None
        self.recovery_step = 0
        self.recovery_start_time = None
        
        # Smooth control variables
        self.prev_angular_vel = 0.0
        self.prev_linear_vel = 0.0
        self.control_dt = 1.0 / control_rate
        
        # Velocity command smoothing history
        self.angular_vel_history = deque(maxlen=3)  # Reduced for faster response
        self.linear_vel_history = deque(maxlen=3)
        
        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.frontier_goals_pub = self.create_publisher(PoseArray, '/frontier_goals', 10)
        self.frontier_markers_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self.current_path_pub = self.create_publisher(Path, '/current_path', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/exploration_costmap', 10)
        self.current_subgoal_pub = self.create_publisher(Marker, '/current_subgoal_marker', 10)
        
        # Control timer
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)
        
        self.get_logger().info('=== Enhanced Fast Exploration Navigation ===')
        self.get_logger().info(f'  Control rate: {control_rate} Hz')
        self.get_logger().info(f'  Max linear vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max angular vel: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Map saving: ONLY on completion')
    
    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map"""
        self.map_data = msg     
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.map_grid = data
        
        self.create_costmap()
        
        if self.state == State.WAIT_MAP and self.robot_pose is not None:
            self.state = State.DETECT_FRONTIERS
            self.get_logger().info('Map ready, starting exploration')
    
    def odom_callback(self, msg: Odometry):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
    
    def create_costmap(self):
        """Create costmap with proper cost translation"""
        if self.map_grid is None:
            return
        
        height, width = self.map_grid.shape
        self.costmap = np.zeros((height, width), dtype=np.uint8)
        
        for gy in range(height):
            for gx in range(width):
                cell_value = self.map_grid[gy, gx]
                
                if cell_value == -1:
                    self.costmap[gy, gx] = NO_INFORMATION
                elif cell_value >= self.occupied_threshold:
                    self.costmap[gy, gx] = LETHAL_OBSTACLE
                elif cell_value == 0:
                    self.costmap[gy, gx] = FREE_SPACE
                else:
                    self.costmap[gy, gx] = int((cell_value * 252) / self.occupied_threshold)
        
        self.inflate_costmap()
        self.publish_costmap()
    
    def inflate_costmap(self):
        """Inflate obstacles in costmap"""
        if self.costmap is None:
            return
        
        height, width = self.costmap.shape
        inflation_r = self.inflation_radius_cells
        
        obstacles = np.argwhere(self.costmap == LETHAL_OBSTACLE)
        inflated = self.costmap.copy()
        
        for gy, gx in obstacles:
            for dy in range(-inflation_r, inflation_r + 1):
                for dx in range(-inflation_r, inflation_r + 1):
                    dist_sq = dx*dx + dy*dy
                    if dist_sq <= inflation_r * inflation_r:
                        ny, nx = gy + dy, gx + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            if inflated[ny, nx] == FREE_SPACE:
                                inflated[ny, nx] = INSCRIBED_OBSTACLE
        
        self.costmap = inflated
    
    def publish_costmap(self):
        """Publish costmap for visualization"""
        if self.costmap is None or self.map_info is None:
            return
        
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info = self.map_info
        
        data = self.costmap.astype(np.int8)
        msg.data = data.flatten().tolist()
        
        self.costmap_pub.publish(msg)
    
    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        gx = int(math.floor((wx - origin_x) / resolution))
        gy = int(math.floor((wy - origin_y) / resolution))
        return (gx, gy)
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        wx = origin_x + (gx + 0.5) * resolution
        wy = origin_y + (gy + 0.5) * resolution
        return (wx, wy)
    
    def is_valid_cell(self, gx: int, gy: int) -> bool:
        if self.costmap is None:
            return False
        height, width = self.costmap.shape
        return 0 <= gx < width and 0 <= gy < height
    
    def get_neighbors_4(self, gx: int, gy: int) -> List[Tuple[int, int]]:
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = gx + dx, gy + dy
            if self.is_valid_cell(nx, ny):
                neighbors.append((nx, ny))
        return neighbors
    
    def get_neighbors_8(self, gx: int, gy: int) -> List[Tuple[int, int]]:
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = gx + dx, gy + dy
                if self.is_valid_cell(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors
    
    def is_new_frontier_cell(self, gx: int, gy: int, frontier_flags: np.ndarray) -> bool:
        if not self.is_valid_cell(gx, gy):
            return False
        
        idx = gy * self.costmap.shape[1] + gx
        
        if self.costmap[gy, gx] != NO_INFORMATION or frontier_flags[idx]:
            return False
        
        for nx, ny in self.get_neighbors_4(gx, gy):
            if self.costmap[ny, nx] == FREE_SPACE:
                return True
        
        return False
    
    def find_nearest_free_space(self, start_gx: int, start_gy: int, max_radius: int = 20) -> Optional[Tuple[int, int]]:
        """Find nearest free space cell to a given point (for goal finding)"""
        if self.is_valid_cell(start_gx, start_gy) and self.costmap[start_gy, start_gx] == FREE_SPACE:
            return (start_gx, start_gy)
        
        for radius in range(1, max_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) + abs(dy) != radius:
                        continue
                    nx, ny = start_gx + dx, start_gy + dy
                    if self.is_valid_cell(nx, ny) and self.costmap[ny, nx] == FREE_SPACE:
                        return (nx, ny)
        return None
    
    def build_frontier(self, start_gx: int, start_gy: int, 
                      robot_gx: int, robot_gy: int,
                      frontier_flags: np.ndarray) -> Optional[Frontier]:
        """Build frontier cluster using BFS"""
        
        width = self.costmap.shape[1]
        initial_wx, initial_wy = self.grid_to_world(start_gx, start_gy)
        robot_wx, robot_wy = self.grid_to_world(robot_gx, robot_gy)
        
        cells = []
        centroid_x = 0.0
        centroid_y = 0.0
        min_distance = float('inf')
        middle_point = (initial_wx, initial_wy)
        closest_free_cell = None
        closest_free_dist = float('inf')
        
        queue = deque([(start_gx, start_gy)])
        frontier_flags[start_gy * width + start_gx] = True
        
        while queue:
            gx, gy = queue.popleft()
            wx, wy = self.grid_to_world(gx, gy)
            
            cells.append((gx, gy))
            centroid_x += wx
            centroid_y += wy
            
            distance = math.sqrt((robot_wx - wx)**2 + (robot_wy - wy)**2)
            if distance < min_distance:
                min_distance = distance
                middle_point = (wx, wy)
            
            # Find closest free space cell for path planning
            for nx, ny in self.get_neighbors_8(gx, gy):
                if self.is_valid_cell(nx, ny) and self.costmap[ny, nx] == FREE_SPACE:
                    dist_to_robot = math.sqrt((nx - robot_gx)**2 + (ny - robot_gy)**2)
                    if dist_to_robot < closest_free_dist:
                        closest_free_dist = dist_to_robot
                        closest_free_cell = (nx, ny)
            
            for nx, ny in self.get_neighbors_8(gx, gy):
                if self.is_new_frontier_cell(nx, ny, frontier_flags):
                    frontier_flags[ny * width + nx] = True
                    queue.append((nx, ny))
        
        size = len(cells)
        
        if size * self.map_info.resolution < self.min_frontier_size:
            return None
        
        centroid = (centroid_x / size, centroid_y / size)
        
        # Find reachable goal near centroid
        centroid_gx, centroid_gy = self.world_to_grid(centroid[0], centroid[1])
        reachable_goal = self.find_nearest_free_space(centroid_gx, centroid_gy, max_radius=15)
        if reachable_goal is None and closest_free_cell is not None:
            reachable_goal = closest_free_cell
        
        return Frontier(
            cells=cells,
            centroid=centroid,
            middle=middle_point,
            initial=(initial_wx, initial_wy),
            size=size,
            min_distance=min_distance,
            reachable_goal=reachable_goal
        )
    
    def search_frontiers(self) -> List[Frontier]:
        """Search for frontiers using BFS - improved to find more frontiers"""
        if self.costmap is None or self.robot_pose is None:
            return []
        
        robot_gx, robot_gy = self.world_to_grid(self.robot_pose[0], self.robot_pose[1])
        
        if not self.is_valid_cell(robot_gx, robot_gy):
            self.get_logger().error('Robot outside map bounds')
            return []
        
        height, width = self.costmap.shape
        
        frontier_flags = np.zeros(height * width, dtype=bool)
        visited_flags = np.zeros(height * width, dtype=bool)
        
        start_pos = robot_gy * width + robot_gx
        
        if self.costmap[robot_gy, robot_gx] != FREE_SPACE:
            for radius in range(1, 20):
                found = False
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        nx, ny = robot_gx + dx, robot_gy + dy
                        if self.is_valid_cell(nx, ny) and self.costmap[ny, nx] == FREE_SPACE:
                            robot_gx, robot_gy = nx, ny
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            
            start_pos = robot_gy * width + robot_gx
        
        queue = deque([start_pos])
        visited_flags[start_pos] = True
        
        frontiers = []
        
        while queue:
            pos = queue.popleft()
            gx = pos % width
            gy = pos // width
            
            for nx, ny in self.get_neighbors_4(gx, gy):
                npos = ny * width + nx
                
                if self.costmap[ny, nx] <= self.costmap[gy, gx] and not visited_flags[npos]:
                    visited_flags[npos] = True
                    queue.append(npos)
                
                elif self.is_new_frontier_cell(nx, ny, frontier_flags):
                    frontier_flags[npos] = True
                    frontier = self.build_frontier(nx, ny, robot_gx, robot_gy, frontier_flags)
                    if frontier:
                        frontiers.append(frontier)
        
        resolution = self.map_info.resolution
        for frontier in frontiers:
            frontier.cost = (self.potential_scale * frontier.min_distance * resolution - 
                           self.gain_scale * frontier.size * resolution)
        
        frontiers.sort(key=lambda f: f.cost)
        
        self.get_logger().info(f'Found {len(frontiers)} frontiers')
        return frontiers
    
    def is_goal_on_blacklist(self, goal: Tuple[float, float]) -> bool:
        """Check if goal is blacklisted, with timeout"""
        tolerance = 5 * self.map_info.resolution
        current_time = time.time()
        
        # Clean old blacklist entries
        expired = [k for k, v in self.blacklist_timestamps.items() 
                   if current_time - v > self.blacklist_timeout]
        for key in expired:
            self.blacklist.remove(key)
            del self.blacklist_timestamps[key]
        
        for blacklisted in self.blacklist:
            dx = abs(goal[0] - blacklisted[0])
            dy = abs(goal[1] - blacklisted[1])
            if dx < tolerance and dy < tolerance:
                return True
        
        return False
    
    def select_frontier(self) -> Optional[Frontier]:
        """Select best frontier that's not blacklisted"""
        for frontier in self.frontiers:
            if not self.is_goal_on_blacklist(frontier.centroid):
                if frontier.reachable_goal is not None:
                    return frontier
        return None
    
    def update_waypoint_index(self):
        """Update current waypoint index based on proximity"""
        if not self.current_path or self.robot_pose is None:
            return
        
        # Advance through waypoints as we get close
        while self.current_waypoint_idx < len(self.current_path) - 1:
            gx, gy = self.current_path[self.current_waypoint_idx]
            wx, wy = self.grid_to_world(gx, gy)
            d = math.hypot(wx - self.robot_pose[0], wy - self.robot_pose[1])
            
            if d <= self.waypoint_tolerance:
                self.current_waypoint_idx += 1
            else:
                break
    
    def get_lookahead_waypoint(self) -> Tuple[float, float]:
        """Get lookahead point on path for smooth following"""
        if not self.current_path or self.robot_pose is None:
            return None
        
        lookahead_dist = self.lookahead_dist
        if self.adaptive_lookahead:
            current_speed = abs(self.prev_linear_vel)
            lookahead_dist = self.min_lookahead_dist + current_speed * 1.5
            lookahead_dist = np.clip(lookahead_dist, self.min_lookahead_dist, self.max_lookahead_dist)
        
        # Start from current waypoint
        start_idx = max(0, self.current_waypoint_idx)
        
        # Find point at lookahead distance
        for i in range(start_idx, len(self.current_path)):
            gx, gy = self.current_path[i]
            wx, wy = self.grid_to_world(gx, gy)
            dist = math.hypot(wx - self.robot_pose[0], wy - self.robot_pose[1])
            
            if dist >= lookahead_dist:
                return (wx, wy)
        
        # If we're near the end, return final waypoint
        final_gx, final_gy = self.current_path[-1]
        return self.grid_to_world(final_gx, final_gy)
    
    def is_path_valid(self) -> bool:
        """Check if remaining path is still valid"""
        if not self.current_path or self.costmap is None:
            return True
        
        start_idx = max(0, self.current_waypoint_idx - 2)  # Check a bit behind too
        
        for i in range(start_idx, len(self.current_path)):
            gx, gy = self.current_path[i]
            if not self.is_valid_cell(gx, gy):
                return False
            
            if self.costmap[gy, gx] >= INSCRIBED_OBSTACLE:
                return False
        
        return True
    
    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* path planning with improved goal handling"""
        if not self.is_valid_cell(start[0], start[1]) or not self.is_valid_cell(goal[0], goal[1]):
            return None
        
        if self.costmap is None:
            return None
        
        # Ensure start is free
        if self.costmap[start[1], start[0]] != FREE_SPACE:
            start = self.find_nearest_free_space(start[0], start[1], max_radius=5)
            if start is None:
                return None
        
        # Ensure goal is free
        if self.costmap[goal[1], goal[0]] != FREE_SPACE:
            goal = self.find_nearest_free_space(goal[0], goal[1], max_radius=10)
            if goal is None:
                return None
        
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        closed_set = set()
        max_iterations = 10000  # Prevent infinite loops
        
        iterations = 0
        while open_set and iterations < max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                self.get_logger().info(f'A* success: {len(path)} waypoints')
                return path
            
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if neighbor in closed_set:
                    continue
                
                if not self.is_valid_cell(neighbor[0], neighbor[1]):
                    continue
                
                if self.costmap[neighbor[1], neighbor[0]] != FREE_SPACE:
                    continue
                
                move_cost = 1.414 if abs(dx) + abs(dy) == 2 else 1.0
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
        
        return None
    
    def check_collision_ahead(self) -> Tuple[bool, float]:
        """Check for obstacles ahead"""
        if self.robot_pose is None or self.costmap is None:
            return False, float('inf')
        
        resolution = self.map_info.resolution
        steps = int(self.collision_check_distance / resolution)
        
        for i in range(1, steps + 1):
            check_dist = i * resolution
            check_x = self.robot_pose[0] + check_dist * math.cos(self.robot_yaw)
            check_y = self.robot_pose[1] + check_dist * math.sin(self.robot_yaw)
            
            gx, gy = self.world_to_grid(check_x, check_y)
            
            if not self.is_valid_cell(gx, gy):
                return True, check_dist
            
            if self.costmap[gy, gx] >= INSCRIBED_OBSTACLE:
                return True, check_dist
        
        return False, float('inf')
    
    def smooth_velocity_command(self, desired_linear: float, desired_angular: float) -> Tuple[float, float]:
        """Apply smoothing for smooth motion"""
        
        # Acceleration limiting
        linear_diff = desired_linear - self.prev_linear_vel
        max_linear_change = self.max_linear_accel * self.control_dt
        
        if abs(linear_diff) > max_linear_change:
            linear = self.prev_linear_vel + np.sign(linear_diff) * max_linear_change
        else:
            linear = desired_linear
        
        angular_diff = desired_angular - self.prev_angular_vel
        max_angular_change = self.max_angular_accel * self.control_dt
        
        if abs(angular_diff) > max_angular_change:
            angular = self.prev_angular_vel + np.sign(angular_diff) * max_angular_change
        else:
            angular = desired_angular
        
        # Low-pass filtering
        linear = (self.linear_smoothing_factor * linear + 
                 (1 - self.linear_smoothing_factor) * self.prev_linear_vel)
        
        angular = (self.angular_smoothing_factor * angular + 
                  (1 - self.angular_smoothing_factor) * self.prev_angular_vel)
        
        # Update history
        self.prev_linear_vel = linear
        self.prev_angular_vel = angular
        
        return linear, angular
    
    def publish_frontier_markers(self):
        """Publish frontier visualization markers"""
        marker_array = MarkerArray()
        
        for i, frontier in enumerate(self.frontiers[:20]):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'frontier_points'
            marker.id = i * 2
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            
            if self.is_goal_on_blacklist(frontier.centroid):
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 0.8
            
            for gx, gy in frontier.cells:
                wx, wy = self.grid_to_world(gx, gy)
                p = Pose()
                p.position.x = wx
                p.position.y = wy
                p.position.z = 0.0
                marker.points.append(p.position)
            
            marker_array.markers.append(marker)
            
            marker2 = Marker()
            marker2.header = marker.header
            marker2.ns = 'frontier_centroids'
            marker2.id = i * 2 + 1
            marker2.type = Marker.SPHERE
            marker2.action = Marker.ADD
            marker2.pose.position.x = frontier.centroid[0]
            marker2.pose.position.y = frontier.centroid[1]
            marker2.pose.position.z = 0.2
            marker2.scale.x = 0.3
            marker2.scale.y = 0.3
            marker2.scale.z = 0.3
            marker2.color.r = 0.0
            marker2.color.g = 1.0
            marker2.color.b = 0.0
            marker2.color.a = 0.9
            marker_array.markers.append(marker2)
        
        self.frontier_markers_pub.publish(marker_array)
    
    def publish_current_subgoal(self, wx: float, wy: float):
        """Publish current active subgoal marker"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'current_subgoal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = wx
        marker.pose.position.y = wy
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.8
        
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        self.current_subgoal_pub.publish(marker)
    
    def clear_current_subgoal(self):
        """Clear current subgoal marker"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'current_subgoal'
        marker.id = 0
        marker.action = Marker.DELETE
        self.current_subgoal_pub.publish(marker)
    
    def publish_path(self, path: List[Tuple[int, int]]):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        for gx, gy in path:
            pose = PoseStamped()
            pose.header = msg.header
            wx, wy = self.grid_to_world(gx, gy)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        self.current_path_pub.publish(msg)
    
    def publish_cmd_vel(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_vel_pub.publish(msg)
    
    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def save_map(self):
        """Save explored map to PGM and YAML - ONLY when exploration is complete"""
        if self.map_data is None or self.map_info is None:
            self.get_logger().warn('No map data to save')
            return
        
        try:
            os.makedirs(self.map_save_dir, exist_ok=True)
            
            pgm_path = os.path.join(self.map_save_dir, f'{self.map_filename}.pgm')
            yaml_path = os.path.join(self.map_save_dir, f'{self.map_filename}.yaml')
            
            width = self.map_info.width
            height = self.map_info.height
            resolution = self.map_info.resolution
            origin = self.map_info.origin
            
            img_data = np.zeros((height, width), dtype=np.uint8)
            
            for y in range(height):
                for x in range(width):
                    cell = self.map_grid[y, x]
                    if cell == -1:
                        img_data[y, x] = 205
                    elif cell == 0:
                        img_data[y, x] = 254
                    elif cell >= self.occupied_threshold:
                        img_data[y, x] = 0
                    else:
                        img_data[y, x] = int(254 - (cell * 254 / self.occupied_threshold))
            
            with open(pgm_path, 'wb') as f:
                f.write(b'P5\n')
                f.write(f'{width} {height}\n'.encode())
                f.write(b'255\n')
                img_data_flipped = np.flipud(img_data)
                f.write(img_data_flipped.tobytes())
            
            yaml_data = {
                'image': f'{self.map_filename}.pgm',
                'resolution': float(resolution),
                'origin': [float(origin.position.x), float(origin.position.y), float(origin.position.z)],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
                'mode': 'trinary'
            }
            
            with open(yaml_path, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False)
            
            self.get_logger().info(f'✓ Map saved successfully to {pgm_path}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {str(e)}')
    
    def control_loop(self):
        if self.state == State.WAIT_MAP:
            return
        
        if self.state == State.IDLE:
            self.state = State.DETECT_FRONTIERS
        
        if self.state == State.DETECT_FRONTIERS:
            self.frontiers = self.search_frontiers()
            self.publish_frontier_markers()
            
            if not self.frontiers:
                # Check if we really explored everything - wait a bit before declaring done
                self.get_logger().warn('No frontiers found, waiting 5s before declaring complete...')
                time.sleep(5.0)
                self.frontiers = self.search_frontiers()
                if not self.frontiers:
                    self.state = State.DONE
                    self.publish_cmd_vel(0.0, 0.0)
                    self.clear_current_subgoal()
                    self.get_logger().info('=== EXPLORATION COMPLETE ===')
                    self.save_map()
                    return
            
            self.state = State.PLAN_PATH
        
        if self.state == State.PLAN_PATH:
            frontier = self.select_frontier()
            
            if frontier is None:
                # Clear some blacklist entries and try again
                self.get_logger().warn('No valid frontiers, clearing old blacklist entries...')
                current_time = time.time()
                expired = [k for k, v in self.blacklist_timestamps.items() 
                          if current_time - v > self.blacklist_timeout / 2]
                for key in expired:
                    self.blacklist.remove(key)
                    del self.blacklist_timestamps[key]
                
                self.state = State.DETECT_FRONTIERS
                return
            
            target_position = frontier.centroid
            
            same_goal = (self.prev_goal is not None and 
                        abs(target_position[0] - self.prev_goal[0]) < 0.1 and 
                        abs(target_position[1] - self.prev_goal[1]) < 0.1)
            
            if not same_goal:
                self.prev_goal = target_position
                self.last_progress_time = time.time()
                self.prev_distance = frontier.min_distance
            
            if time.time() - self.last_progress_time > self.progress_timeout:
                self.blacklist.append(target_position)
                self.blacklist_timestamps[target_position] = time.time()
                self.get_logger().warn('No progress, blacklisting goal')
                self.state = State.DETECT_FRONTIERS
                return
            
            # Don't replan same goal
            if same_goal:
                return
            
            robot_gx, robot_gy = self.world_to_grid(self.robot_pose[0], self.robot_pose[1])
            
            # Use reachable goal from frontier
            if frontier.reachable_goal is None:
                goal_gx, goal_gy = self.world_to_grid(target_position[0], target_position[1])
                goal_gx, goal_gy = self.find_nearest_free_space(goal_gx, goal_gy, max_radius=15)
                if goal_gx is None:
                    self.blacklist.append(target_position)
                    self.blacklist_timestamps[target_position] = time.time()
                    self.get_logger().warn('Cannot find reachable goal, blacklisting')
                    self.state = State.DETECT_FRONTIERS
                    return
            else:
                goal_gx, goal_gy = frontier.reachable_goal
            
            path = self.astar((robot_gx, robot_gy), (goal_gx, goal_gy))
            
            if path is None:
                self.blacklist.append(target_position)
                self.blacklist_timestamps[target_position] = time.time()
                self.get_logger().warn('A* failed, blacklisting')
                self.state = State.DETECT_FRONTIERS
                return
            
            self.current_goal = target_position
            self.current_path = path
            self.current_waypoint_idx = 0
            self.publish_path(path)
            
            self.state = State.MOVE_ALONG_PATH
            self.stuck_timer_start = time.time()
            self.last_pose_for_stuck = self.robot_pose
            self.get_logger().info(f'→ Following path to frontier (cost={frontier.cost:.3f}, {len(path)} waypoints)')
        
        if self.state == State.MOVE_ALONG_PATH:
            if self.robot_pose is None:
                self.publish_cmd_vel(0.0, 0.0)
                return
            
            # Check if path is still valid
            if not self.is_path_valid():
                self.get_logger().warn('⚠ Path invalidated by obstacles, replanning')
                self.state = State.PLAN_PATH
                return
            
            # Emergency collision check
            is_collision, obstacle_dist = self.check_collision_ahead()
            if is_collision and obstacle_dist < self.emergency_stop_distance:
                self.get_logger().error(f'🛑 EMERGENCY STOP! Obstacle at {obstacle_dist:.2f}m')
                self.publish_cmd_vel(0.0, 0.0)
                self.state = State.STUCK_RECOVERY
                self.recovery_step = 0
                self.recovery_start_time = time.time()
                return
            
            # Update waypoint index
            self.update_waypoint_index()
            
            # Check if we reached final goal
            final_gx, final_gy = self.current_path[-1]
            final_wx, final_wy = self.grid_to_world(final_gx, final_gy)
            distance_to_final = math.hypot(final_wx - self.robot_pose[0], final_wy - self.robot_pose[1])
            
            if distance_to_final <= self.final_goal_tolerance:
                self.publish_cmd_vel(0.0, 0.0)
                self.clear_current_subgoal()
                self.get_logger().info('✓ Final goal reached! Searching for next frontier...')
                self.state = State.DETECT_FRONTIERS
                return
            
            # Get lookahead target for smooth following
            target_wx, target_wy = self.get_lookahead_waypoint()
            self.publish_current_subgoal(target_wx, target_wy)
            
            # Calculate control
            dx = target_wx - self.robot_pose[0]
            dy = target_wy - self.robot_pose[1]
            distance = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_yaw - self.robot_yaw)
            
            # Stuck detection
            if self.last_pose_for_stuck:
                elapsed = time.time() - self.stuck_timer_start
                moved = math.hypot(self.robot_pose[0] - self.last_pose_for_stuck[0], 
                                   self.robot_pose[1] - self.last_pose_for_stuck[1])
                
                if moved < self.stuck_distance_threshold and elapsed > self.stuck_timeout:
                    self.publish_cmd_vel(0.0, 0.0)
                    self.state = State.STUCK_RECOVERY
                    self.recovery_step = 0
                    self.recovery_start_time = time.time()
                    self.stuck_count += 1
                    self.get_logger().warn(f'⚠ STUCK detected!')
                    return
                elif moved > 0.3:
                    self.stuck_timer_start = time.time()
                    self.last_pose_for_stuck = self.robot_pose
            
            # Nav2-style control: Rotate in place if misaligned, then move forward
            abs_angle_error = abs(angle_error)
            
            # Check if we should rotate in place first (Nav2-style)
            should_rotate_in_place = (self.use_rotate_to_heading and 
                                     abs_angle_error > self.rotate_to_heading_min_angle)
            
            if should_rotate_in_place:
                # Rotate in place - no forward movement
                desired_linear = 0.0
                sign = 1.0 if angle_error > 0.0 else -1.0
                desired_angular = sign * self.rotate_to_heading_angular_vel
                
                # Limit angular acceleration
                max_angular_change = self.max_angular_accel * self.control_dt
                angular_diff = desired_angular - self.prev_angular_vel
                if abs(angular_diff) > max_angular_change:
                    desired_angular = self.prev_angular_vel + np.sign(angular_diff) * max_angular_change
                
                # Slow down to avoid overshooting
                max_vel_to_stop = math.sqrt(2 * self.max_angular_accel * abs_angle_error)
                if abs(desired_angular) > max_vel_to_stop:
                    desired_angular = sign * max_vel_to_stop
                
                desired_angular = np.clip(desired_angular, -self.max_angular_vel, self.max_angular_vel)
                
            else:
                # Move forward with angular correction (aligned enough)
                # Apply deadband
                if abs_angle_error < self.angular_deadband:
                    angle_error = 0.0
                    abs_angle_error = 0.0
                
                # Calculate linear velocity based on distance
                desired_linear = self.k_linear * distance
                desired_linear = np.clip(desired_linear, 0.0, self.max_linear_vel)
                
                # Reduce speed near obstacles
                if is_collision:
                    desired_linear *= 0.7
                
                # Maintain minimum speed when moving
                if desired_linear > 0.01:
                    desired_linear = max(desired_linear, self.min_linear_vel)
                
                # Calculate angular velocity based on curvature (Nav2-style)
                # For pure pursuit: angular_vel = linear_vel * curvature
                # Curvature = 2 * sin(angle_error) / distance
                if abs_angle_error > 0.01 and distance > 0.01:
                    # Calculate curvature (1/radius of curvature)
                    curvature = 2.0 * math.sin(angle_error) / distance
                    # Limit curvature to prevent excessive turning
                    max_curvature = 1.5  # Maximum curvature (1/m)
                    curvature = np.clip(curvature, -max_curvature, max_curvature)
                    
                    # Angular velocity = linear_vel * curvature (Nav2-style)
                    desired_angular = desired_linear * curvature
                else:
                    # Very small angle - use proportional control for fine adjustment
                    desired_angular = self.k_angular * angle_error
                
                # Limit angular velocity
                desired_angular = np.clip(desired_angular, -self.max_angular_vel, self.max_angular_vel)
            
            # Apply smoothing
            linear, angular = self.smooth_velocity_command(desired_linear, desired_angular)
            
            self.publish_cmd_vel(linear, angular)
        
        if self.state == State.STUCK_RECOVERY:
            elapsed = time.time() - self.recovery_start_time
            
            if self.recovery_step == 0:
                if elapsed < 1.5:
                    self.publish_cmd_vel(-0.15, 0.0)
                else:
                    self.recovery_step = 1
                    self.recovery_start_time = time.time()
            
            elif self.recovery_step == 1:
                if elapsed < 0.3:
                    self.publish_cmd_vel(0.0, 0.0)
                else:
                    self.recovery_step = 2
                    self.recovery_start_time = time.time()
            
            elif self.recovery_step == 2:
                if elapsed < 1.5:
                    direction = 1.0 if (self.stuck_count % 2 == 0) else -1.0
                    self.publish_cmd_vel(0.0, direction * 0.8)
                else:
                    self.recovery_step = 3
            
            elif self.recovery_step == 3:
                self.publish_cmd_vel(0.0, 0.0)
                
                if self.current_goal:
                    self.blacklist.append(self.current_goal)
                    self.blacklist_timestamps[self.current_goal] = time.time()
                    self.get_logger().info('Goal blacklisted after recovery')
                
                # Reset velocity history
                self.prev_angular_vel = 0.0
                self.prev_linear_vel = 0.0
                self.angular_vel_history.clear()
                self.linear_vel_history.clear()
                
                self.state = State.DETECT_FRONTIERS
                self.get_logger().info('Recovery complete, searching for new path')
        
        if self.state == State.DONE:
            self.publish_cmd_vel(0.0, 0.0)
            self.clear_current_subgoal()


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt - shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

