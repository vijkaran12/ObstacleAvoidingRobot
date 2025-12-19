"""
Occupancy grid mapping module for building environment representation.
"""

import numpy as np
import pybullet as p
import math
from config import *


class OccupancyGrid:
    """Occupancy grid map for environment representation."""
    
    def __init__(self, width, height, resolution):
        """
        Initialize occupancy grid.
        
        Args:
            width: Width in cells
            height: Height in cells
            resolution: Meters per cell
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        
        # Initialize grid with unknown (0.5 probability)
        self.grid = np.ones((height, width)) * 0.5
        
        # Origin offset (to handle negative coordinates)
        self.origin_x = 0
        self.origin_y = -AREA_DEPTH / 2
        
        self.update_count = 0
        
    def world_to_grid(self, x, y):
        """
        Convert world coordinates to grid indices.
        
        Args:
            x, y: World coordinates in meters
            
        Returns:
            Tuple of (grid_x, grid_y) or None if out of bounds
        """
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return (grid_x, grid_y)
        return None
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates."""
        x = grid_x * self.resolution + self.origin_x
        y = grid_y * self.resolution + self.origin_y
        return (x, y)
    
    def update_from_sensors(self, robot_pos, sensor_readings):
        """
        Update occupancy grid based on sensor readings.
        
        Args:
            robot_pos: Robot position (x, y)
            sensor_readings: Dictionary of sensor readings
        """
        self.update_count += 1
        
        # Only update every N frames for performance
        if self.update_count % 10 != 0:
            return
        
        robot_grid = self.world_to_grid(robot_pos[0], robot_pos[1])
        if robot_grid is None:
            return
        
        # Process each sensor reading
        for sensor_name, distance in sensor_readings.items():
            angle = SENSOR_ANGLES[sensor_name]
            
            # Calculate obstacle position in world coordinates
            obs_x = robot_pos[0] + distance * math.cos(angle)
            obs_y = robot_pos[1] + distance * math.sin(angle)
            
            obs_grid = self.world_to_grid(obs_x, obs_y)
            if obs_grid is None:
                continue
            
            # Ray tracing: mark cells along ray as free
            cells = self._bresenham_line(robot_grid, obs_grid)
            
            for cell in cells[:-1]:  # All cells except last
                cx, cy = cell
                if 0 <= cx < self.width and 0 <= cy < self.height:
                    # Decrease occupancy probability (mark as free)
                    self.grid[cy, cx] = max(0.0, self.grid[cy, cx] - 0.1)
            
            # Mark endpoint as occupied if obstacle detected
            if distance < RAY_LENGTH * 0.95:  # Not max range
                ox, oy = obs_grid
                if 0 <= ox < self.width and 0 <= oy < self.height:
                    # Increase occupancy probability
                    self.grid[oy, ox] = min(1.0, self.grid[oy, ox] + 0.3)
    
    def _bresenham_line(self, start, end):
        """
        Bresenham's line algorithm for ray tracing.
        
        Returns list of grid cells along the line from start to end.
        """
        x0, y0 = start
        x1, y1 = end
        
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            cells.append((x0, y0))
            
            if x0 == x1 and y0 == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        
        return cells
    
    def is_occupied(self, grid_x, grid_y):
        """Check if grid cell is occupied."""
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return self.grid[grid_y, grid_x] > OCCUPIED_THRESHOLD
        return True  # Out of bounds = occupied
    
    def is_free(self, grid_x, grid_y):
        """Check if grid cell is free."""
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return self.grid[grid_y, grid_x] < FREE_THRESHOLD
        return False
    
    def visualize(self):
        """Visualize occupancy grid in PyBullet."""
        if not DRAW_GRID:
            return
        
        # Draw occupied cells
        for gy in range(0, self.height, 2):  # Skip some for performance
            for gx in range(0, self.width, 2):
                if self.grid[gy, gx] > OCCUPIED_THRESHOLD:
                    wx, wy = self.grid_to_world(gx, gy)
                    
                    # Draw small cube at occupied cell
                    p.addUserDebugLine(
                        [wx, wy, 0.05],
                        [wx, wy, 0.15],
                        [0.5, 0.5, 0.5],
                        lineWidth=2,
                        lifeTime=0.5
                    )
    
    def get_grid(self):
        """Return the occupancy grid array."""
        return self.grid.copy()