"""
A* global path planner for optimal path generation.
"""

import numpy as np
import heapq
import math
import pybullet as p
from config import *


class AStarPlanner:
    """A* algorithm for global path planning on occupancy grid."""
    
    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.path = []
        
    def plan(self, start_pos, goal_pos):
        
        # Convert to grid coordinates
        start_grid = self.grid.world_to_grid(start_pos[0], start_pos[1])
        goal_grid = self.grid.world_to_grid(goal_pos[0], goal_pos[1])
        
        if start_grid is None or goal_grid is None:
           
            return None
        
        # Run A* search
        grid_path = self._astar_search(start_grid, goal_grid)
        
        if grid_path is None:
           
            return None
        
        # Convert grid path to world coordinates
        self.path = []
        for grid_pos in grid_path:
            world_pos = self.grid.grid_to_world(grid_pos[0], grid_pos[1])
            self.path.append(world_pos)
        
       
        return self.path
    
    def _astar_search(self, start, goal):
       
        # Priority queue: (f_score, counter, node)
        open_set = []
        counter = 0
        heapq.heappush(open_set, (0, counter, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        open_set_hash = {start}
        
        while open_set:
            current = heapq.heappop(open_set)[2]
            open_set_hash.remove(current)
            
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            # Check 8-connected neighbors
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Skip if occupied or out of bounds
                if self.grid.is_occupied(neighbor[0], neighbor[1]):
                    continue
                
                # Cost: diagonal moves cost sqrt(2), others cost 1
                move_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        counter += 1
                        heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                        open_set_hash.add(neighbor)
        
        return None  # No path found
    
    def _heuristic(self, pos1, pos2):
        """Euclidean distance heuristic."""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def _reconstruct_path(self, came_from, current):
        """Reconstruct path by backtracking through came_from."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def get_next_waypoint(self, current_pos, lookahead_distance=2.0):
    
        if not self.path:
            return None
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        
        for i, waypoint in enumerate(self.path):
            dist = math.sqrt(
                (waypoint[0] - current_pos[0])**2 +
                (waypoint[1] - current_pos[1])**2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Look ahead from closest point
        for i in range(closest_idx, len(self.path)):
            waypoint = self.path[i]
            dist = math.sqrt(
                (waypoint[0] - current_pos[0])**2 +
                (waypoint[1] - current_pos[1])**2
            )
            
            if dist >= lookahead_distance:
                return waypoint
        
        # Return goal if no waypoint at lookahead distance
        return self.path[-1] if self.path else None
    
    def visualize(self):
        """Visualize planned path in PyBullet."""
        if not DRAW_PLAN or not self.path:
            return
        
        # Draw path as line segments
        for i in range(len(self.path) - 1):
            p1 = [self.path[i][0], self.path[i][1], 0.2]
            p2 = [self.path[i+1][0], self.path[i+1][1], 0.2]
            p.addUserDebugLine(p1, p2, [1, 1, 0], lineWidth=3, lifeTime=0.5)