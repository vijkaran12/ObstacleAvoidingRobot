"""
Odometry module for robot localization using wheel encoders.
"""

import pybullet as p
import math
from config import *


class Odometry:
    """Odometry-based localization for tracking robot pose."""
    
    def __init__(self, robot_id, joint_ids):
        self.robot_id = robot_id
        self.joint_ids = joint_ids
        
        # Initialize pose estimate
        self.x = ROBOT_START_POS[0]
        self.y = ROBOT_START_POS[1]
        self.theta = 0.0
        
        # Previous wheel positions for velocity calculation
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.prev_time = 0.0
        
        # Ground truth (for error calculation)
        self.true_x = ROBOT_START_POS[0]
        self.true_y = ROBOT_START_POS[1]
        self.true_theta = 0.0
        
    def update(self, dt):
        """
        Update odometry estimate based on wheel encoder readings.
        
        Args:
            dt: Time step in seconds
        """
        # Get left and right wheel positions (average of front and rear)
        left_pos = (p.getJointState(self.robot_id, self.joint_ids['FL'])[0] +
                   p.getJointState(self.robot_id, self.joint_ids['RL'])[0]) / 2
        right_pos = (p.getJointState(self.robot_id, self.joint_ids['FR'])[0] +
                    p.getJointState(self.robot_id, self.joint_ids['RR'])[0]) / 2
        
        # Calculate wheel displacements
        delta_left = (left_pos - self.prev_left_pos) * WHEEL_RADIUS
        delta_right = (right_pos - self.prev_right_pos) * WHEEL_RADIUS
        
        # Update previous positions
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        
        # Calculate robot displacement and rotation (differential drive kinematics)
        delta_s = (delta_right + delta_left) / 2  # Forward displacement
        delta_theta = (delta_right - delta_left) / WHEEL_BASE  # Angular displacement
        
        # Update pose estimate
        self.theta += delta_theta
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)
        
        # Normalize theta to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Update ground truth for comparison
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        self.true_x = pos[0]
        self.true_y = pos[1]
        self.true_theta = p.getEulerFromQuaternion(orn)[2]
    
    def get_pose(self):
        """
        Get current pose estimate.
        
        Returns:
            Tuple of (x, y, theta)
        """
        return (self.x, self.y, self.theta)
    
    def get_position(self):
        """Get current position estimate (x, y)."""
        return (self.x, self.y)
    
    def get_heading(self):
        """Get current heading estimate (theta)."""
        return self.theta
    
    def get_localization_error(self):
        """
        Calculate localization error compared to ground truth.
        
        Returns:
            Dictionary with position and heading errors
        """
        pos_error = math.sqrt((self.x - self.true_x)**2 + (self.y - self.true_y)**2)
        heading_error = abs(self.theta - self.true_theta)
        
        # Normalize heading error
        if heading_error > math.pi:
            heading_error = 2 * math.pi - heading_error
        
        return {
            'position_error': pos_error,
            'heading_error': math.degrees(heading_error),
            'x_error': abs(self.x - self.true_x),
            'y_error': abs(self.y - self.true_y)
        }