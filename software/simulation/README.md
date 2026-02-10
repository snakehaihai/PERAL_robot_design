# Simulation

This directory contains simulation tools and models for the PERAL robot.

## Overview

Simulation allows testing control algorithms and validating designs before physical implementation.

## Contents

### Models (`models/`)
Robot models for various simulators:
- URDF/XACRO files for ROS/Gazebo
- Physics parameters
- Collision meshes
- Visual meshes
- Sensor models

### Scripts (`scripts/`)
Simulation scripts including:
- Environment setup
- Test scenarios
- Data collection
- Visualization

## Simulators

### Gazebo (ROS Integration)
```bash
# Launch simulation
roslaunch peral_sim robot_sim.launch
```

### Custom Python Simulator
```bash
# Run custom simulation
cd simulation
python simulator.py --config config/sim_params.yaml
```

## Features

- **Physics Simulation**: Accurate dynamics of spherical robot
- **Sensor Simulation**: IMU, encoders, LiDAR (if applicable)
- **Environment**: Customizable test environments
- **Visualization**: Real-time 3D visualization
- **Data Logging**: Record simulation data for analysis

## Installation

```bash
# For ROS/Gazebo
sudo apt-get install ros-noetic-gazebo-ros

# For Python simulator
pip install numpy scipy matplotlib

# Optional: For 3D visualization
pip install pygame  # or pyvista
```

## Usage

### Basic Simulation
```python
from simulation import RobotSimulator

# Create simulator
sim = RobotSimulator()

# Set initial conditions
sim.set_initial_pose(x=0, y=0, theta=0)

# Run simulation
for t in range(1000):
    # Generate control commands
    commands = controller.compute(sim.get_state())
    
    # Step simulation
    sim.step(commands, dt=0.01)
    
    # Visualize
    sim.render()
```

### Testing Control Algorithms
```bash
python test_controller.py --controller pid --duration 60
```

## Configuration

Edit `config/sim_params.yaml`:
```yaml
robot:
  mass: 2.0
  radius: 0.15
  inertia: 0.05

simulation:
  timestep: 0.001
  realtime_factor: 1.0
  
physics:
  gravity: -9.81
  friction: 0.3
```

## Validation

Use simulation to validate:
- Control algorithm stability
- Motion trajectories
- Sensor noise effects
- Fault scenarios
- Performance metrics

## Visualization

### Real-time 3D View
Shows robot moving in environment with:
- Robot model
- Trajectory trace
- Sensor data overlay
- Performance metrics

### Data Plots
Plot simulation results:
```python
import matplotlib.pyplot as plt

# Load simulation data
data = load_simulation_data('results/test_1.npz')

# Plot trajectory
plt.plot(data['x'], data['y'])
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Trajectory')
plt.show()
```

## Examples

### Example 1: Point-to-Point Motion
```python
# Navigate to target position
sim = RobotSimulator()
controller = PointToPointController()
controller.set_target(x=5.0, y=3.0)

while not controller.reached_goal():
    state = sim.get_state()
    cmd = controller.compute(state)
    sim.step(cmd)
```

### Example 2: LiDAR Excitation
```python
# Simulate perception-aware motion
sim = RobotSimulator()
lidar_sim = LiDARSimulator(position=[10, 0, 2])
excitation = ExcitationPlanner()

for t in range(1000):
    state = sim.get_state()
    trajectory = excitation.plan(state, lidar_sim.get_pose())
    cmd = controller.compute(state, trajectory)
    sim.step(cmd)
```

## Performance Testing

Test robot performance in simulation:
```bash
# Run benchmark suite
python benchmark.py

# Test specific scenario
python test_scenario.py --scenario hill_climb
```

## Comparison with Real Robot

Validate simulation accuracy:
1. Run same test in simulation and on real robot
2. Compare trajectories and performance
3. Tune simulation parameters to match reality
4. Document differences and limitations

## To-Do

- [ ] Create URDF model of robot
- [ ] Implement physics simulation
- [ ] Add visualization
- [ ] Create test scenarios
- [ ] Validate against real robot
- [ ] Add Monte Carlo testing

## Troubleshooting

**Simulation runs slowly:**
- Reduce visualization detail
- Increase timestep (if stable)
- Use faster integrator
- Disable unnecessary features

**Simulation unstable:**
- Reduce timestep
- Check physics parameters
- Verify model correctness
- Add damping

## References

- Gazebo documentation
- ROS/URDF tutorials
- Physics engine documentation
- Simulation best practices

