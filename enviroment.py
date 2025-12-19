"""
Environment module for creating and managing the simulation world.
Handles obstacle generation and goal placement.
"""

import pybullet as p
import random
import math
from config import *


class Environment:
    """Manages the simulation environment including obstacles and goals."""
    
    def __init__(self):
        self.obstacles = []
        self.goal_marker = None
        self.goal_position = None
        
    def setup_world(self):
        """Initialize the simulation world with ground plane."""
        p.loadURDF("plane.urdf")
        print("✅ World initialized")
        
    def generate_obstacles(self):
        """Generate randomly placed obstacles with intelligent spacing."""
        box_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3])
        box_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.3, 0.3, 0.3],
            rgbaColor=[1, 0, 0, 1]
        )
        
        self.obstacles = []
        
        for _ in range(NUM_OBSTACLES):
            attempts = 0
            while attempts < 50:
                # Generate random position
                x = random.uniform(3, AREA_WIDTH)
                y = random.uniform(-AREA_DEPTH/2, AREA_DEPTH/2)
                
                # Check clearance from origin (robot spawn)
                dist_to_origin = math.sqrt(x**2 + y**2)
                if dist_to_origin < ROBOT_CLEAR_RADIUS:
                    attempts += 1
                    continue
                
                # Check spacing from existing obstacles
                valid = True
                for obs_pos in self.obstacles:
                    dist = math.sqrt((x - obs_pos[0])**2 + (y - obs_pos[1])**2)
                    if dist < MIN_OBSTACLE_SPACING:
                        valid = False
                        break
                
                if valid:
                    pos = [x, y, 0.3]
                    self.obstacles.append(pos)
                    p.createMultiBody(
                        baseMass=0,
                        baseCollisionShapeIndex=box_col,
                        baseVisualShapeIndex=box_vis,
                        basePosition=pos
                    )
                    break
                
                attempts += 1
        
        print(f"🏗️  Generated {len(self.obstacles)} obstacles")
        return self.obstacles
    
    def create_goal(self):
        """Create the goal marker at the far end of the course."""
        self.goal_position = [AREA_WIDTH - 5, random.uniform(-5, 5), 0.4]
        
        goal_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        goal_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.5, 0.5, 0.5],
            rgbaColor=[0, 1, 0, 0.7]
        )
        
        self.goal_marker = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=goal_col,
            baseVisualShapeIndex=goal_vis,
            basePosition=self.goal_position
        )
        
        print(f"🎯 Goal placed at: ({self.goal_position[0]:.1f}, {self.goal_position[1]:.1f})")
        return self.goal_position
    
    def get_obstacle_positions(self):
        """Return list of obstacle positions."""
        return self.obstacles.copy()
    
    def get_goal_position(self):
        """Return goal position."""
        return self.goal_position
    
    def get_goal_marker_id(self):
        """Return goal marker body ID."""
        return self.goal_marker