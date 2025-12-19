"""
Configuration parameters for the autonomous mobile robot navigation system.
"""

# ================== SIMULATION PARAMETERS ==================
SIMULATION_HZ = 240
TIME_STEP = 1 / SIMULATION_HZ
GRAVITY = -9.8

# ================== ROBOT PARAMETERS ==================
ROBOT_START_POS = [0, 0, 0.1]
ROBOT_URDF = "obstacle_bot.urdf"

# Joint names
JOINT_NAMES = {
    'FL': 'front_left_joint',
    'FR': 'front_right_joint',
    'RL': 'rear_left_joint',
    'RR': 'rear_right_joint'
}

# ================== CONTROL PARAMETERS ==================
FORWARD_SPEED = 15      # rad/s
TURN_SPEED = 12         # rad/s
MOTOR_FORCE = 250       # Newtons

# ================== SENSOR PARAMETERS ==================
RAY_LENGTH = 2.5        # meters
SENSOR_ANGLES = {
    'front': 0,
    'front_left': 0.4,
    'front_right': -0.4,
    'left': 1.0,
    'right': -1.0
}

# ================== NAVIGATION PARAMETERS ==================
SAFE_DIST = 1.5         # Distance to trigger obstacle avoidance (m)
CLEAR_DIST = 2.0        # Distance required to resume forward motion (m)
MIN_TURN_DURATION = 90  # frames
MAX_TURN_DURATION = 300 # frames
GOAL_REACH_DISTANCE = 1.5  # Distance to consider goal reached (m)

# ================== ENVIRONMENT PARAMETERS ==================
AREA_WIDTH = 30         # meters
AREA_DEPTH = 20         # meters
NUM_OBSTACLES = 20
MIN_OBSTACLE_SPACING = 2.5  # meters
ROBOT_CLEAR_RADIUS = 2.0    # meters

# ================== MAPPING PARAMETERS ==================
GRID_RESOLUTION = 0.5   # meters per cell
GRID_WIDTH = int(AREA_WIDTH / GRID_RESOLUTION)
GRID_HEIGHT = int(AREA_DEPTH / GRID_RESOLUTION)

# Occupancy grid thresholds
OCCUPIED_THRESHOLD = 0.65
FREE_THRESHOLD = 0.35

# ================== ODOMETRY PARAMETERS ==================
WHEEL_RADIUS = 0.1      # meters (adjust based on your URDF)
WHEEL_BASE = 0.4        # meters (distance between left and right wheels)

# ================== VISUALIZATION PARAMETERS ==================
DRAW_PATH = False       # Set True to visualize robot path
DRAW_GRID = True        # Set True to visualize occupancy grid
DRAW_PLAN = True        # Set True to visualize A* path
DEBUG_INTERVAL = 480    # frames (print debug info every N frames)

# ================== PERFORMANCE TRACKING ==================
TRACK_METRICS = True
SAVE_RESULTS = True
RESULTS_FILE = "navigation_results.csv"