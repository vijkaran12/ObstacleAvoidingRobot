"""
Sensor module implementing ray-based distance sensing for obstacle detection.
"""

import pybullet as p
import math
from config import *


class RaySensor:
    """Ray-based distance sensor for obstacle detection."""
    
    def __init__(self, robot_id, goal_marker_id=None):
        self.robot_id = robot_id
        self.goal_marker_id = goal_marker_id
        self.sensor_readings = {}
        
    def read_distance(self, angle, length=RAY_LENGTH):
        """
        Cast a ray at given angle relative to robot's heading.
        
        Args:
            angle: Angle in radians relative to robot's forward direction
            length: Maximum ray length in meters
            
        Returns:
            Distance to nearest obstacle, or length if nothing detected
        """
        try:
            pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        except p.error:
            return length
        
        yaw = p.getEulerFromQuaternion(orn)[2]
        
        # Calculate ray start and end points
        start = [pos[0], pos[1], pos[2] + 0.05]
        end = [
            start[0] + length * math.cos(yaw + angle),
            start[1] + length * math.sin(yaw + angle),
            start[2]
        ]
        
        # Perform raycast
        hit = p.rayTest(start, end)[0]
        hit_id = hit[0]
        hit_fraction = hit[2]
        
        # Visualize ray (red if obstacle detected, green if clear)
        is_obstacle = (hit_id != -1 and 
                      hit_id != self.robot_id and 
                      hit_id != 0 and 
                      hit_id != self.goal_marker_id)
        color = [1, 0, 0] if is_obstacle else [0, 1, 0]
        p.addUserDebugLine(start, end, color, lifeTime=0.1)
        
        # Ignore self-hits, ground, and goal marker
        if hit_id == -1 or hit_id == self.robot_id or hit_id == 0 or hit_id == self.goal_marker_id:
            return length
        
        return hit_fraction * length
    
    def read_all_sensors(self):
        """
        Read all configured sensors.
        
        Returns:
            Dictionary of sensor readings {name: distance}
        """
        self.sensor_readings = {}
        for name, angle in SENSOR_ANGLES.items():
            self.sensor_readings[name] = self.read_distance(angle)
        
        return self.sensor_readings
    
    def get_sensor_reading(self, sensor_name):
        """Get reading from specific sensor."""
        return self.sensor_readings.get(sensor_name, RAY_LENGTH)