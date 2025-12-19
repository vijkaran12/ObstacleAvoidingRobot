"""
Performance metrics tracking and evaluation module.
"""

import time
import csv
import math
from config import *


class PerformanceMetrics:
    """Track and evaluate navigation performance metrics."""
    
    def __init__(self, trial_number=1):
        self.trial_number = trial_number
        self.start_time = time.time()
        
        # Metrics
        self.total_distance = 0.0
        self.collision_count = 0
        self.replans = 0
        self.goal_reached = False
        self.completion_time = 0.0
        
        # Tracking
        self.prev_position = None
        self.position_history = []
        self.localization_errors = []
        
    def update(self, robot_pos, robot_vel, collision_detected=False):
        
        # Track distance traveled
        if self.prev_position:
            dx = robot_pos[0] - self.prev_position[0]
            dy = robot_pos[1] - self.prev_position[1]
            self.total_distance += math.sqrt(dx**2 + dy**2)
        
        self.prev_position = robot_pos
        self.position_history.append(robot_pos)
        
        # Track collisions
        if collision_detected:
            self.collision_count += 1
    
    def record_localization_error(self, error_dict):
        """Record odometry localization error."""
        self.localization_errors.append(error_dict)
    
    def mark_goal_reached(self):
        """Mark goal as reached and record completion time."""
        if not self.goal_reached:
            self.goal_reached = True
            self.completion_time = time.time() - self.start_time
    
    def calculate_efficiency(self, goal_pos, start_pos):
        
        if self.total_distance == 0:
            return 0.0
        
        optimal_distance = math.sqrt(
            (goal_pos[0] - start_pos[0])**2 +
            (goal_pos[1] - start_pos[1])**2
        )
        
        return optimal_distance / self.total_distance
    
    def get_summary(self, goal_pos, start_pos):
       
        efficiency = self.calculate_efficiency(goal_pos, start_pos)
        
        # Calculate average localization error
        avg_pos_error = 0.0
        avg_heading_error = 0.0
        if self.localization_errors:
            avg_pos_error = sum(e['position_error'] for e in self.localization_errors) / len(self.localization_errors)
            avg_heading_error = sum(e['heading_error'] for e in self.localization_errors) / len(self.localization_errors)
        
        summary = {
            'trial': self.trial_number,
            'goal_reached': self.goal_reached,
            'completion_time': self.completion_time,
            'total_distance': self.total_distance,
            'collision_count': self.collision_count,
            'efficiency': efficiency,
            'avg_localization_error': avg_pos_error,
            'avg_heading_error': avg_heading_error,
            'num_waypoints': len(self.position_history)
        }
        
        return summary
    
    def print_summary(self, goal_pos, start_pos):
        """Print formatted performance summary."""
        summary = self.get_summary(goal_pos, start_pos)
        
       
    
    def save_to_csv(self, goal_pos, start_pos, filename=RESULTS_FILE):
        """Save metrics to CSV file."""
        if not SAVE_RESULTS:
            return
        
        summary = self.get_summary(goal_pos, start_pos)
        
        # Check if file exists to determine if we need headers
        import os
        file_exists = os.path.isfile(filename)
        
        with open(filename, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=summary.keys())
            
            if not file_exists:
                writer.writeheader()
            
            writer.writerow(summary)
        
        