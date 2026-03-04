"""
Path Planner Module

Provides A* path planning for obstacle avoidance on a grid map.
Replaces the previous Vector Field implementation.
Map size: 10m x 10m
Cell size: 5cm x 5cm (200x200 grid)
"""

import numpy as np
import math
import heapq
from typing import Tuple, List, Optional, Dict

class PathPlanner:
    """
    A* Path Planner for global path planning.
    
    Attributes:
        map_size: Tuple (width, height) in meters
        cell_size: Size of each grid cell in meters
        grid_size: Tuple (cols, rows) - number of cells
        obstacle_map: 2D boolean array marking obstacles
    """
    
    def __init__(self, x_range: Tuple[float, float] = (0.0, 10.0), 
                 y_range: Tuple[float, float] = (0.0, 10.0),
                 cell_size: float = 0.05,
                 robot_radius: float = 0.25): # Default robot radius 25cm (includes buffer)
        """
        Initialize the Path Planner.
        
        Args:
            x_range: Tuple (min, max) for X axis in meters.
            y_range: Tuple (min, max) for Y axis in meters.
            cell_size: Size of each grid cell in meters.
            robot_radius: Radius of the robot for Configuration Space expansion (meters).
        """
        self.x_range = x_range
        self.y_range = y_range
        self.cell_size = cell_size
        self.robot_radius = robot_radius
        
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range
        
        # Calculate grid dimensions
        self.grid_cols = int((self.x_max - self.x_min) / cell_size)
        self.grid_rows = int((self.y_max - self.y_min) / cell_size)
        
        # Initialize obstacle map (False = free, True = obstacle)
        self.obstacle_map = np.zeros((self.grid_rows, self.grid_cols), dtype=bool)
        
        # Obstacle storage for visualization/reconstruction
        self.obstacles = []  # List of dicts: {type, params}
    
    # ========================================
    # Coordinate Conversion
    # ========================================
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid cell indices."""
        rel_x = x - self.x_min
        rel_y = y - self.y_min
        
        col = int(rel_x / self.cell_size)
        row = int(rel_y / self.cell_size)
        
        # Clamp to valid range
        col = max(0, min(col, self.grid_cols - 1))
        row = max(0, min(row, self.grid_rows - 1))
        
        return (col, row)
    
    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Convert grid cell indices to world coordinates (center of cell)."""
        rel_x = (col + 0.5) * self.cell_size
        rel_y = (row + 0.5) * self.cell_size
        
        x = rel_x + self.x_min
        y = rel_y + self.y_min
        
        return (x, y)
    
    def is_valid_position(self, x: float, y: float) -> bool:
        """Check if world position is within map bounds."""
        return (self.x_min <= x < self.x_max) and (self.y_min <= y < self.y_max)
    
    def is_valid_cell(self, col: int, row: int) -> bool:
        """Check if grid cell indices are valid."""
        return (0 <= col < self.grid_cols) and (0 <= row < self.grid_rows)
    
    # ========================================
    # Obstacle Management
    # ========================================
    
    def add_obstacle(self, obstacle_type: str, *args):
        """Add an obstacle (circle or rectangle)."""
        if obstacle_type == 'circle':
            self.add_circular_obstacle(*args)
        elif obstacle_type == 'rectangle':
            self.add_rectangular_obstacle(*args)
            
    def add_circular_obstacle(self, cx: float, cy: float, radius: float):
        """
        Add circular obstacle.
        NOTE: Visualizes with actual radius, plans with (radius + robot_radius).
        """
        self.obstacles.append({'type': 'circle', 'cx': cx, 'cy': cy, 'radius': radius})
        
        # Use inflated radius for Grid Map (Configuration Space)
        map_radius = radius + self.robot_radius
        
        center_col, center_row = self.world_to_grid(cx, cy)
        radius_cells = int(map_radius / self.cell_size) + 1
        
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                r, c = center_row + dr, center_col + dc
                if self.is_valid_cell(c, r):
                    wx, wy = self.grid_to_world(c, r)
                    if (wx - cx)**2 + (wy - cy)**2 <= map_radius**2:
                        self.obstacle_map[r, c] = True

    def add_rectangular_obstacle(self, x1: float, y1: float, x2: float, y2: float):
        """
        Add rectangular obstacle.
        NOTE: Visualizes with actual bounds, plans with bounds inflated by robot_radius.
        """
        self.obstacles.append({'type': 'rectangle', 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})
        
        # Inflate bounds for Grid Map
        min_x = min(x1, x2) - self.robot_radius
        max_x = max(x1, x2) + self.robot_radius
        min_y = min(y1, y2) - self.robot_radius
        max_y = max(y1, y2) + self.robot_radius
        
        col1, row1 = self.world_to_grid(min_x, min_y)
        col2, row2 = self.world_to_grid(max_x, max_y)
        
        c_min, c_max = min(col1, col2), max(col1, col2)
        r_min, r_max = min(row1, row2), max(row1, row2)
        
        self.obstacle_map[r_min:r_max+1, c_min:c_max+1] = True

    def clear_obstacles(self):
        """Clear all obstacles."""
        self.obstacle_map.fill(False)
        self.obstacles.clear()

    def is_obstacle(self, x: float, y: float) -> bool:
        """Check if world position is in obstacle."""
        if not self.is_valid_position(x, y):
            return True
        col, row = self.world_to_grid(x, y)
        return self.obstacle_map[row, col]

    def remove_last_obstacle(self):
        """Remove last added obstacle."""
        if self.obstacles:
            self.obstacles.pop()
            # Rebuild map
            self.obstacle_map.fill(False)
            for obs in self.obstacles:
                if obs['type'] == 'circle':
                    self._readd_circle(obs)
                elif obs['type'] == 'rectangle':
                    self._readd_rect(obs)

    def _readd_circle(self, obs):
        self._add_obs_circle_raw(obs['cx'], obs['cy'], obs['radius'])

    def _readd_rect(self, obs):
        self._add_obs_rect_raw(obs['x1'], obs['y1'], obs['x2'], obs['y2'])
        
    def _add_obs_circle_raw(self, cx, cy, radius):
        """Internal helper without adding to list."""
        # Fix: Must apply inflation during rebuild too!
        map_radius = radius + self.robot_radius
        
        center_col, center_row = self.world_to_grid(cx, cy)
        radius_cells = int(map_radius / self.cell_size) + 1
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                r, c = center_row + dr, center_col + dc
                if self.is_valid_cell(c, r):
                    wx, wy = self.grid_to_world(c, r)
                    if (wx - cx)**2 + (wy - cy)**2 <= map_radius**2:
                        self.obstacle_map[r, c] = True

    def _add_obs_rect_raw(self, x1, y1, x2, y2):
        """Internal helper without adding to list."""
        # Fix: Must apply inflation during rebuild too!
        min_x = min(x1, x2) - self.robot_radius
        max_x = max(x1, x2) + self.robot_radius
        min_y = min(y1, y2) - self.robot_radius
        max_y = max(y1, y2) + self.robot_radius
        
        col1, row1 = self.world_to_grid(min_x, min_y)
        col2, row2 = self.world_to_grid(max_x, max_y)
        c_min, c_max = min(col1, col2), max(col1, col2)
        r_min, r_max = min(row1, row2), max(row1, row2)
        self.obstacle_map[r_min:r_max+1, c_min:c_max+1] = True

    # ========================================
    # A* Path Planning
    # ========================================
    
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Find path from start to goal using A*.
        
        Args:
            start: (x, y) start position in meters.
            goal: (x, y) goal position in meters.
            
        Returns:
            List of (x, y) points or None if no path found.
        """
        start_c, start_r = self.world_to_grid(*start)
        goal_c, goal_r = self.world_to_grid(*goal)
        
        # If start or goal is obstacle, try to find nearest free cell?
        # For now, just return direct line if trivial, or fail.
        if self.obstacle_map[start_r, start_c]:
            print(f"[PathPlanner] WARNING: Start position {start} is in Obstacle/Inflation Zone! (Cell {start_c},{start_r})")
        if self.obstacle_map[goal_r, goal_c]:
            print(f"[PathPlanner] WARNING: Goal position {goal} is in Obstacle/Inflation Zone! (Cell {goal_c},{goal_r})") 
        
        open_set = []
        heapq.heappush(open_set, (0, (start_c, start_r)))
        
        came_from = {}
        g_score = { (start_c, start_r): 0 }
        f_score = { (start_c, start_r): self._heuristic((start_c, start_r), (goal_c, goal_r)) }
        
        dirs = [
            (0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1), # Cardinal
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414) # Diagonal
        ]
        
        path_found = False
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == (goal_c, goal_r):
                path_found = True
                break
            
            cx, cy = current
            current_g = g_score[current]
            
            for dx, dy, cost in dirs:
                neighbor = (cx + dx, cy + dy)
                nx, ny = neighbor
                
                # Check bounds
                if not self.is_valid_cell(nx, ny):
                    continue
                
                # Check obstacle
                if self.obstacle_map[ny, nx]:
                    continue
                
                # Diagonal check: prevent cutting corners through walls
                if dx != 0 and dy != 0:
                    if self.obstacle_map[cy, nx] or self.obstacle_map[ny, cx]:
                        continue
                
                tentative_g = current_g + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, (goal_c, goal_r))
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        
        if path_found:
            # Reconstruct path
            path = []
            curr = (goal_c, goal_r)
            while curr in came_from:
                path.append(self.grid_to_world(*curr))
                curr = came_from[curr]
            path.append(self.grid_to_world(*curr)) # Add start
            path.reverse()
            
            # Smooth
            return self.smooth_path(path)
        else:
            # Fallback: Straight line if A* fails (optional, good for robustness)
            # Or return None
            return None

    def _heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def smooth_path(self, path: List[Tuple[float, float]], window_size: int = 5) -> List[Tuple[float, float]]:
        """
        Simple moving average smoothing.
        
        WARNING: Default smoothing can pull the path into obstacles (cutting corners).
        Disabling for now to ensure strict collision avoidance.
        """
        # Return raw path to guarantee safety (zigzag is better than crash)
        return path

    def get_obstacles(self) -> List[Dict]:
        """Return list of obstacles for visualization."""
        return self.obstacles

# Singleton
_default_planner = None

def get_path_planner(x_range=(0.0, 10.0), y_range=(0.0, 10.0), cell_size=0.05) -> PathPlanner:
    global _default_planner
    if _default_planner is None:
        _default_planner = PathPlanner(x_range, y_range, cell_size)
    return _default_planner
