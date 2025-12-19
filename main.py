

import pybullet as p
import pybullet_data
import time

from config import *
from environment import Environment
from sensors import RaySensor
from odometry import Odometry
from mapping import OccupancyGrid
from planner import AStarPlanner
from controller import RobotController
from metrics import PerformanceMetrics


class AutonomousRobot:
    """Main class orchestrating autonomous navigation."""
    
    def __init__(self, trial_number=1):
        self.trial = trial_number
        self.robot_id = None
        self.joint_ids = {}
        
        # Initialize modules
        self.env = Environment()
        self.sensor = None
        self.odometry = None
        self.grid = None
        self.planner = None
        self.controller = None
        self.metrics = PerformanceMetrics(trial_number)
        
    def setup_simulation(self):
        """Initialize PyBullet and load world."""
        print(f"\n{'='*60}")
        print(f"🚀 TRIAL {self.trial} - INITIALIZING SIMULATION")
        print(f"{'='*60}\n")
        
        # Connect to physics engine
        physicsClient = p.connect(p.GUI)
        if physicsClient < 0:
            raise RuntimeError("Failed to connect to physics server")
        
        p.setGravity(0, 0, GRAVITY)
        p.setTimeStep(TIME_STEP)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Setup world
        self.env.setup_world()
        self.env.generate_obstacles()
        goal_pos = self.env.create_goal()
        
        return goal_pos
    
    def setup_robot(self):
        """Load and configure robot."""
        self.robot_id = p.loadURDF(
            ROBOT_URDF,
            ROBOT_START_POS,
            useFixedBase=False
        )
        
        # Disable default motors
        for i in range(p.getNumJoints(self.robot_id)):
            p.setJointMotorControl2(
                self.robot_id, i,
                p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0
            )
        
        # Map joint names to IDs
        for i in range(p.getNumJoints(self.robot_id)):
            name = p.getJointInfo(self.robot_id, i)[1].decode()
            for key, joint_name in JOINT_NAMES.items():
                if name == joint_name:
                    self.joint_ids[key] = i
        
        print(f"🤖 Robot loaded with joints: {self.joint_ids}")
        
        # Initialize sensors and modules
        goal_marker_id = self.env.get_goal_marker_id()
        self.sensor = RaySensor(self.robot_id, goal_marker_id)
        self.odometry = Odometry(self.robot_id, self.joint_ids)
        self.grid = OccupancyGrid(GRID_WIDTH, GRID_HEIGHT, GRID_RESOLUTION)
        self.planner = AStarPlanner(self.grid)
        self.controller = RobotController(self.robot_id, self.joint_ids, self.planner)
        
    
    
    def run_navigation(self, goal_pos, max_steps=10000):
        
        print(f"\n🏃 Starting navigation to goal ({goal_pos[0]:.1f}, {goal_pos[1]:.1f})")
        print(f"📡 Sensors: {len(SENSOR_ANGLES)} rays | 🗺️  Grid: {GRID_WIDTH}x{GRID_HEIGHT}")
        print(f"🎯 Goal threshold: {GOAL_REACH_DISTANCE}m\n")
        
        step_count = 0
        path_planned = False
        
        try:
            while p.isConnected() and step_count < max_steps:
                step_count += 1
                
                # Update odometry
                self.odometry.update(TIME_STEP)
                odom_pos = self.odometry.get_position()
                odom_heading = self.odometry.get_heading()
                
                # Read sensors
                sensor_readings = self.sensor.read_all_sensors()
                
                # Update occupancy grid
                self.grid.update_from_sensors(odom_pos, sensor_readings)
                
                # Plan global path (once at start, or replan periodically)
                if not path_planned or step_count % 480 == 0:  # Every 2 seconds
                    self.planner.plan(odom_pos, (goal_pos[0], goal_pos[1]))
                    path_planned = True
                
                # Get control commands (hybrid: global planning + reactive avoidance)
                left_speed, right_speed, state = self.controller.compute_control(
                    sensor_readings,
                    odom_pos,
                    odom_heading,
                    (goal_pos[0], goal_pos[1])
                )
                
                # Apply control
                self.controller.apply_control(left_speed, right_speed)
                
                # Update metrics
                true_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
                velocity, _ = p.getBaseVelocity(self.robot_id)
                self.metrics.update((true_pos[0], true_pos[1]), velocity)
                
                # Track localization error
                if step_count % 240 == 0:  # Every second
                    loc_error = self.odometry.get_localization_error()
                    self.metrics.record_localization_error(loc_error)
                
                # Visualizations
                if step_count % 20 == 0:  # Update visualizations
                    self.grid.visualize()
                    self.planner.visualize()
                
                # Debug output
                if step_count % DEBUG_INTERVAL == 0:
                    dist_to_goal = ((odom_pos[0] - goal_pos[0])**2 + 
                                   (odom_pos[1] - goal_pos[1])**2) ** 0.5
                    loc_error = self.odometry.get_localization_error()
                    print(f"Step {step_count:5d} | Pos: ({odom_pos[0]:5.1f}, {odom_pos[1]:5.1f}) | "
                          f"Goal: {dist_to_goal:5.1f}m | State: {state:8s} | "
                          f"Loc Error: {loc_error['position_error']:.3f}m")
                
                # Check if goal reached
                if state == "REACHED":
                    self.metrics.mark_goal_reached()
                    print(f"\n🎉 Goal reached at step {step_count}!")
                    time.sleep(2)  # Pause to see result
                    break
                
                # Step simulation
                p.stepSimulation()
                time.sleep(TIME_STEP)
        
        except KeyboardInterrupt:
            print("\n⚠️ Simulation interrupted by user")
        except Exception as e:
            print(f"\n❌ Error during simulation: {e}")
            import traceback
            traceback.print_exc()
    
    def cleanup(self, goal_pos):
        """Print results and cleanup."""
        self.metrics.print_summary((goal_pos[0], goal_pos[1]), ROBOT_START_POS[:2])
        self.metrics.save_to_csv((goal_pos[0], goal_pos[1]), ROBOT_START_POS[:2])
        
        if p.isConnected():
            p.disconnect()
        
        print("✅ Simulation ended cleanly\n")


def main():
    """Main entry point."""
    print("""
╔═══════════════════════════════════════════════════════════════╗
║   AUTONOMOUS MOBILE ROBOT NAVIGATION SYSTEM                   ║
║   - Dynamic Obstacle Generation                               ║
║   - Ray-based Sensing (5 sensors)                             ║
║   - Occupancy Grid Mapping                                    ║
║   - A* Global Path Planning                                   ║
║   - Reactive Local Obstacle Avoidance                         ║
║   - Odometry-based Localization                               ║
║   - Performance Evaluation                                    ║
╚═══════════════════════════════════════════════════════════════╝
    """)
    
    # Run single trial (can be extended for multiple trials)
    trial_number = 1
    
    robot_system = AutonomousRobot(trial_number)
    
    try:
        goal_pos = robot_system.setup_simulation()
        robot_system.setup_robot()
        robot_system.run_navigation(goal_pos)
    finally:
        robot_system.cleanup(goal_pos)


if __name__ == "__main__":
    main()