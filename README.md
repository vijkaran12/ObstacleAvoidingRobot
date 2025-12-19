# 🤖 Autonomous Mobile Robot Navigation

A modular autonomous navigation system that combines A* global path planning with reactive obstacle avoidance for mobile robots in PyBullet simulation.

## 📋 Overview

This project implements an autonomous robot that navigates from a start position to a goal while avoiding randomly generated obstacles. The system uses a hybrid control architecture combining deliberative planning (A* algorithm) with reactive sensor-based control.

## ✨ Key Features

- **Occupancy Grid Mapping**: Real-time probabilistic map building from sensor data
- **A* Path Planning**: Optimal global path computation on occupancy grid
- **Reactive Obstacle Avoidance**: Immediate response to sensor readings via state machine
- **Ray-based Sensing**: 5-sensor LIDAR-like array for obstacle detection
- **Odometry Localization**: Position estimation using wheel encoder kinematics
- **Performance Evaluation**: Comprehensive metrics tracking (time, distance, efficiency, localization error)

## 🏗️ Architecture

The project uses a modular design with 9 Python files:

- `config.py` - Centralized configuration parameters
- `environment.py` - Random obstacle generation and world setup
- `sensors.py` - Ray-based distance sensing (5 sensors)
- `odometry.py` - Wheel encoder-based localization
- `mapping.py` - Occupancy grid mapping with Bresenham ray tracing
- `planner.py` - A* global path planning algorithm
- `controller.py` - Hybrid state machine control (FORWARD, TURN, REACHED states)
- `metrics.py` - Performance tracking and CSV export
- `main.py` - Main simulation orchestrator

## 🚀 Installation

```bash
# Install dependencies
pip install pybullet numpy

# Clone repository
git clone https://github.com/yourusername/autonomous-robot-navigation.git
cd autonomous-robot-navigation

# Run simulation
python main.py
```

## 🎮 Usage

Run the simulation:
```bash
python main.py
```

The robot will:
1. Start at origin (0, 0)
2. Navigate through 20 randomly placed obstacles
3. Reach the green goal marker at the far end
4. Display performance metrics and save to CSV

**Customize parameters** in `config.py`:
```python
FORWARD_SPEED = 15          # Robot speed (rad/s)
NUM_OBSTACLES = 20          # Number of obstacles
SAFE_DIST = 1.5            # Avoidance trigger distance (m)
DRAW_GRID = True           # Visualize occupancy grid
DRAW_PLAN = True           # Visualize A* path
```

## 🧠 How It Works

**Control Loop (240 Hz)**:
1. **Sense**: Read 5 ray sensors for obstacle distances
2. **Localize**: Update position estimate from wheel encoders
3. **Map**: Update occupancy grid with sensor data
4. **Plan**: Compute optimal path using A* (every 2 seconds)
5. **Control**: Hybrid decision-making
   - Global layer: Steer toward A* waypoints
   - Reactive layer: Immediately avoid obstacles when detected
6. **Act**: Apply wheel velocities (differential drive)
7. **Evaluate**: Track performance metrics

## 📊 Performance Metrics

Tracked metrics:
- Goal reached (success/failure)
- Completion time (seconds)
- Total distance traveled (meters)
- Path efficiency (optimal/actual distance ratio)
- Collision count
- Localization error (position and heading)

Results saved to `navigation_results.csv`

## 📁 Project Structure

```
autonomous-robot-navigation/
├── config.py              # Configuration parameters
├── main.py                # Main simulation loop
├── environment.py         # World and obstacle generation
├── sensors.py             # Ray-based sensing
├── odometry.py            # Localization
├── mapping.py             # Occupancy grid
├── planner.py             # A* algorithm
├── controller.py          # Hybrid control
├── metrics.py             # Performance evaluation
├── obstacle_bot.urdf      # Robot model
└── README.md
```

## 🎯 Technical Highlights

- **Modular Architecture**: Clean separation of concerns across 9 modules
- **Hybrid Control**: Combines global planning with reactive avoidance
- **Differential Drive Kinematics**: Accurate wheel-level velocity control
- **Probabilistic Mapping**: Bayesian occupancy grid updates
- **8-Connected A* Search**: Optimal pathfinding with diagonal movement
- **State Machine Design**: Robust control flow (Forward → Turn → Reached)

## 📝 License

MIT License - feel free to use for educational and research purposes.

## 🙏 Acknowledgments

Built with PyBullet physics simulation and inspired by modern robotics navigation systems.