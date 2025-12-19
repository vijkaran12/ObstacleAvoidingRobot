"""
State machine controller integrating global planning and local reactive control.
"""

import pybullet as p
import math
from config import *


class RobotController:
    """Hybrid controller combining global A* planning with reactive local avoidance."""
    
    # State definitions
    STATE_FORWARD = 0
    STATE_TURN = 1
    STATE_REACHED = 2
    
    def __init__(self, robot_id, joint_ids, planner):
        self.robot_id = robot_id
        self.joint_ids = joint_ids
        self.planner = planner
        
        self.state = self.STATE_FORWARD
        self.turn_steps = 0
        self.committed_turn_dir = 0
        
        # Path following
        self.current_waypoint = None
        self.replan_counter = 0
        
    def compute_control(self, sensor_readings, robot_pos, robot_heading, goal_pos):
        
        # Extract sensor readings
        front = sensor_readings.get('front', RAY_LENGTH)
        front_left = sensor_readings.get('front_left', RAY_LENGTH)
        front_right = sensor_readings.get('front_right', RAY_LENGTH)
        left = sensor_readings.get('left', RAY_LENGTH)
        right = sensor_readings.get('right', RAY_LENGTH)
        
        # Check if goal reached
        dist_to_goal = math.sqrt(
            (robot_pos[0] - goal_pos[0])**2 +
            (robot_pos[1] - goal_pos[1])**2
        )
        
        if dist_to_goal < GOAL_REACH_DISTANCE and self.state != self.STATE_REACHED:
            self.state = self.STATE_REACHED
            return (0, 0, "REACHED")
        
        # Get target direction (from A* path or direct to goal)
        target_angle = self._get_target_angle(robot_pos, goal_pos, robot_heading)
        
        # Calculate angle difference
        angle_diff = target_angle - robot_heading
        angle_diff = self._normalize_angle(angle_diff)
        
        vx = 0
        vtheta = 0
        
        # ========== STATE MACHINE ==========
        
        if self.state == self.STATE_REACHED:
            # Goal reached - stop
            vx = 0
            vtheta = 0
            state_name = "REACHED"
            
        elif self.state == self.STATE_FORWARD:
            # Check for obstacles (reactive layer)
            obstacle_detected = (
                front < SAFE_DIST or
                front_left < SAFE_DIST * 0.9 or
                front_right < SAFE_DIST * 0.9
            )
            
            if not obstacle_detected:
                # Path is clear - follow global plan with steering correction
                vx = FORWARD_SPEED
                
                # Proportional steering toward target
                if abs(angle_diff) > 0.1:
                    vtheta = 5 * angle_diff
                    vtheta = max(-TURN_SPEED, min(TURN_SPEED, vtheta))
                
                state_name = "FORWARD"
            else:
                # Obstacle detected - switch to reactive avoidance
                self.state = self.STATE_TURN
                self.turn_steps = 0
                
                # Calculate clearance scores
                left_clearance = left + front_left
                right_clearance = right + front_right
                
                # Bias toward target direction
                goal_bias = 3.0 * (1 if angle_diff > 0 else -1)
                left_clearance += goal_bias if angle_diff > 0 else 0
                right_clearance += -goal_bias if angle_diff < 0 else 0
                
                # Commit to turn direction
                self.committed_turn_dir = 1 if left_clearance > right_clearance else -1
                
              
                
                vx = 0
                vtheta = 0
                state_name = "TURN"
        
        elif self.state == self.STATE_TURN:
            # Execute committed turn
            vtheta = TURN_SPEED * self.committed_turn_dir
            self.turn_steps += 1
            
            # Check if path is clear
            path_clear = (
                front >= CLEAR_DIST and
                front_left >= CLEAR_DIST * 0.8 and
                front_right >= CLEAR_DIST * 0.8
            )
            
            if self.turn_steps >= MIN_TURN_DURATION and path_clear:
                self.state = self.STATE_FORWARD
               
            elif self.turn_steps >= MAX_TURN_DURATION:
                self.state = self.STATE_FORWARD
                
            
            state_name = "TURN"
        
        else:
            state_name = "UNKNOWN"
        
        # Convert to wheel speeds (differential drive)
        left_speed = vx - vtheta
        right_speed = vx + vtheta
        
        return (left_speed, right_speed, state_name)
    
    def _get_target_angle(self, robot_pos, goal_pos, robot_heading):
      
        # Replan periodically or if no path
        self.replan_counter += 1
        if self.replan_counter > 480 or not self.current_waypoint:  # Every 2 seconds
            waypoint = self.planner.get_next_waypoint(robot_pos, lookahead_distance=3.0)
            if waypoint:
                self.current_waypoint = waypoint
            self.replan_counter = 0
        
        # Use waypoint if available, otherwise direct to goal
        target_pos = self.current_waypoint if self.current_waypoint else goal_pos
        
        return math.atan2(
            target_pos[1] - robot_pos[1],
            target_pos[0] - robot_pos[0]
        )
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def apply_control(self, left_speed, right_speed):
        """Apply computed wheel velocities to robot motors."""
        p.setJointMotorControl2(
            self.robot_id, self.joint_ids['FL'],
            p.VELOCITY_CONTROL,
            targetVelocity=left_speed,
            force=MOTOR_FORCE
        )
        p.setJointMotorControl2(
            self.robot_id, self.joint_ids['FR'],
            p.VELOCITY_CONTROL,
            targetVelocity=right_speed,
            force=MOTOR_FORCE
        )
        p.setJointMotorControl2(
            self.robot_id, self.joint_ids['RL'],
            p.VELOCITY_CONTROL,
            targetVelocity=left_speed,
            force=MOTOR_FORCE
        )
        p.setJointMotorControl2(
            self.robot_id, self.joint_ids['RR'],
            p.VELOCITY_CONTROL,
            targetVelocity=right_speed,
            force=MOTOR_FORCE
        )