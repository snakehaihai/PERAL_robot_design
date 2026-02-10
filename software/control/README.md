# Control Software - PERAL Robot

High-level control algorithms for the PERAL spherical robot.

## Overview

This module contains the high-level control algorithms including:
- Motion planning and control
- LiDAR excitation strategies (perception-aware motion)
- Sensor fusion and state estimation
- Path planning and navigation

## Architecture

```
control/
├── motion_control/         # Low-level motion control
│   ├── pid_controller.py
│   ├── motion_planner.py
│   └── trajectory_gen.py
├── lidar_excitation/       # LiDAR excitation algorithms
│   ├── perception_aware.py
│   ├── excitation_planner.py
│   └── optimization.py
├── perception/             # Perception and estimation
│   ├── state_estimator.py
│   ├── sensor_fusion.py
│   └── mapping.py
├── utils/                  # Utility functions
│   ├── math_utils.py
│   ├── communication.py
│   └── visualization.py
├── config/                 # Configuration files
│   ├── robot_params.yaml
│   └── control_params.yaml
├── main.py                 # Main control loop
└── README.md               # This file
```

## Installation

### Prerequisites
- Python 3.8 or higher
- pip package manager

### Setup
```bash
cd software/control

# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Dependencies
Create `requirements.txt`:
```
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.4.0
pyyaml>=5.4.0
pyserial>=3.5
```

Optional dependencies:
```
opencv-python>=4.5.0    # For visualization
transforms3d>=0.4.0     # For 3D transformations
```

## Modules

### Motion Control

**Purpose:** Generate motor commands for desired robot motion

**Key Components:**
- **PID Controller:** Speed and position control
- **Motion Planner:** High-level trajectory planning
- **Trajectory Generator:** Smooth path generation

**Example Usage:**
```python
from motion_control.pid_controller import PIDController

# Create controller
controller = PIDController(kp=1.0, ki=0.1, kd=0.05)

# Control loop
while running:
    current_state = get_robot_state()
    desired_state = target_trajectory[t]
    
    control_output = controller.update(desired_state, current_state)
    send_motor_commands(control_output)
```

### LiDAR Excitation

**Purpose:** Implement perception-aware motion for optimal LiDAR reflection

**Key Algorithms:**
- **Perception-Aware Planning:** Optimize motion for LiDAR visibility
- **Excitation Strategy:** Maximize LiDAR point cloud density
- **Trajectory Optimization:** Balance motion and perception goals

**Example Usage:**
```python
from lidar_excitation.perception_aware import PerceptionAwarePlanner

planner = PerceptionAwarePlanner()

# Plan motion considering LiDAR scanner position
lidar_pose = get_lidar_scanner_pose()
environment_map = get_environment_map()

optimal_trajectory = planner.plan(
    current_pose=robot_pose,
    lidar_pose=lidar_pose,
    environment=environment_map
)
```

### Perception

**Purpose:** State estimation and sensor fusion

**Key Components:**
- **State Estimator:** Kalman filter or complementary filter
- **Sensor Fusion:** Combine IMU, encoders, and other sensors
- **Mapping:** Build environment representation

**Example Usage:**
```python
from perception.state_estimator import StateEstimator

estimator = StateEstimator()

# Sensor fusion loop
while running:
    imu_data = read_imu()
    encoder_data = read_encoders()
    
    estimated_state = estimator.update(imu_data, encoder_data)
```

## Configuration

### Robot Parameters (`config/robot_params.yaml`)
```yaml
robot:
  radius: 0.15  # meters
  mass: 2.0     # kg
  inertia: 0.05 # kg*m^2
  
motors:
  max_speed: 100  # RPM
  max_torque: 1.0 # N*m
  
sensors:
  imu_rate: 100   # Hz
  encoder_resolution: 360  # pulses per revolution
```

### Control Parameters (`config/control_params.yaml`)
```yaml
pid:
  kp: 1.0
  ki: 0.1
  kd: 0.05
  
motion:
  max_velocity: 0.5    # m/s
  max_acceleration: 1.0 # m/s^2
  
lidar_excitation:
  optimization_weight: 0.7
  lookahead_time: 2.0  # seconds
```

## Running the Control Software

### Basic Operation
```bash
python main.py
```

### With Configuration
```bash
python main.py --config config/robot_params.yaml
```

### Simulation Mode
```bash
python main.py --simulate
```

## Main Control Loop

```python
# main.py
import yaml
from motion_control import MotionController
from lidar_excitation import ExcitationPlanner
from perception import StateEstimator
from utils.communication import RobotInterface

def main():
    # Load configuration
    with open('config/robot_params.yaml') as f:
        config = yaml.safe_load(f)
    
    # Initialize components
    robot = RobotInterface(port='/dev/ttyUSB0')
    estimator = StateEstimator(config)
    motion_ctrl = MotionController(config)
    excitation = ExcitationPlanner(config)
    
    # Main control loop
    while True:
        # Get current state
        sensor_data = robot.read_sensors()
        state = estimator.update(sensor_data)
        
        # Plan motion
        target = excitation.plan(state)
        
        # Generate control
        commands = motion_ctrl.compute(state, target)
        
        # Send to robot
        robot.send_commands(commands)
        
        # Logging and visualization
        log_data(state, target, commands)

if __name__ == '__main__':
    main()
```

## Testing

### Unit Tests
```bash
python -m pytest tests/
```

### Integration Tests
```bash
python -m pytest tests/integration/
```

### Simulation
```bash
python simulation/test_control.py
```

## Visualization

### Real-time Plotting
```python
from utils.visualization import RealTimePlotter

plotter = RealTimePlotter()
plotter.add_plot('position')
plotter.add_plot('velocity')

while running:
    state = get_state()
    plotter.update('position', state.position)
    plotter.update('velocity', state.velocity)
```

### Trajectory Visualization
```python
import matplotlib.pyplot as plt

plt.plot(trajectory[:, 0], trajectory[:, 1])
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('Robot Trajectory')
plt.show()
```

## Debugging

### Logging
```python
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

logger.debug('Current state: %s', state)
logger.info('Target reached')
logger.warning('High velocity detected')
logger.error('Communication lost')
```

### Performance Profiling
```bash
python -m cProfile -o profile.stats main.py
python -m pstats profile.stats
```

## API Reference

### MotionController
```python
class MotionController:
    def __init__(self, config):
        """Initialize motion controller with configuration"""
        
    def compute(self, current_state, target_state):
        """Compute motor commands"""
        return motor_commands
```

### ExcitationPlanner
```python
class ExcitationPlanner:
    def __init__(self, config):
        """Initialize LiDAR excitation planner"""
        
    def plan(self, robot_state, lidar_state):
        """Plan optimal trajectory for LiDAR excitation"""
        return trajectory
```

## Future Improvements

- [ ] Implement advanced optimization algorithms
- [ ] Add machine learning for adaptive control
- [ ] Integrate ROS2 for better modularity
- [ ] Add real-time visualization dashboard
- [ ] Implement multi-robot coordination
- [ ] Add reinforcement learning for motion planning

## Performance Metrics

- Control loop frequency: 50-100 Hz
- Planning horizon: 2-5 seconds
- Trajectory update rate: 10 Hz
- State estimation latency: <10 ms

## Troubleshooting

**Control is unstable:**
- Tune PID parameters
- Check sensor noise levels
- Verify motor response time

**High computational load:**
- Reduce planning horizon
- Use simpler optimization methods
- Profile and optimize critical paths

**Communication errors:**
- Check serial connection
- Verify baud rate settings
- Add error recovery logic

## References

- ICRA 2026 Paper: PERAL perception-aware motion control
- Control theory textbooks
- Spherical robot dynamics papers
- LiDAR perception research

