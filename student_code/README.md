# Student Code

This directory contains code contributions from students working on the PERAL robot project.

## Directory Structure

### `control/`
Place motion control algorithms and controllers here:
- PID controllers
- Motion planning algorithms
- Trajectory generation
- State machines

**Example files**: `pid_controller.py`, `motion_planner.cpp`, `trajectory_optimizer.m`

### `perception/`
Place sensor processing and perception code here:
- LiDAR data processing
- Sensor fusion
- Localization algorithms
- Point cloud processing

**Example files**: `lidar_processor.py`, `sensor_fusion.cpp`, `localization.py`

### `simulation/`
Place simulation scripts and testing code here:
- Gazebo/ROS simulation files
- MATLAB/Simulink models
- Test scenarios
- Validation scripts

**Example files**: `robot_sim.py`, `test_motion.m`, `gazebo_world.launch`

### `hardware/`
Place hardware integration code here:
- Motor drivers
- Sensor interfaces
- Microcontroller code
- Hardware abstraction layers

**Example files**: `motor_driver.cpp`, `sensor_interface.py`, `arduino_code.ino`

## Contribution Guidelines

1. **Organize your code**: Place files in the appropriate subdirectory
2. **Use descriptive names**: Make file names clear and meaningful
3. **Add comments**: Document your code thoroughly
4. **Include dependencies**: List required libraries in a requirements.txt or similar
5. **Test your code**: Ensure code is tested before pushing

## Getting Started

1. Clone the repository
2. Navigate to the appropriate subdirectory
3. Create your code files
4. Test your implementation
5. Commit and push your changes

For questions, contact the project supervisor.
