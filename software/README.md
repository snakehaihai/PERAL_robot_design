# Software - PERAL Spherical Robot Control

This directory contains all software and firmware for the PERAL spherical robot.

## Directory Structure

```
software/
├── firmware/           # Microcontroller firmware
│   ├── src/           # Source code
│   ├── include/       # Header files
│   ├── lib/           # Libraries
│   ├── platformio.ini # PlatformIO config (if using)
│   └── README.md      # Firmware documentation
├── control/           # High-level control algorithms
│   ├── motion_control/    # Motion planning and control
│   ├── lidar_excitation/  # LiDAR excitation algorithms
│   ├── perception/        # Perception algorithms
│   └── README.md          # Control documentation
└── simulation/        # Simulation and testing
    ├── models/        # Robot models
    ├── scripts/       # Simulation scripts
    └── README.md      # Simulation documentation
```

## System Architecture

```
┌─────────────────────────────────────────────┐
│         High-Level Control (PC/RPi)         │
│  - Motion Planning                          │
│  - LiDAR Excitation Strategy                │
│  - Data Collection                          │
└──────────────┬──────────────────────────────┘
               │ WiFi/Serial
┌──────────────┴──────────────────────────────┐
│      Firmware (Microcontroller)             │
│  - Motor Control                            │
│  - IMU Reading                              │
│  - Low-level Motion                         │
└──────────────┬──────────────────────────────┘
               │ I2C, PWM, GPIO
┌──────────────┴──────────────────────────────┐
│         Hardware Layer                      │
│  - Motors & Drivers                         │
│  - IMU Sensor                               │
│  - Power Management                         │
└─────────────────────────────────────────────┘
```

## Firmware Components

### Core Modules
1. **Motor Control**: PWM generation, direction control, speed regulation
2. **Sensor Interface**: IMU data acquisition and filtering
3. **Communication**: Serial/WiFi command interface
4. **State Machine**: Robot behavior state management
5. **Safety**: Emergency stop, battery monitoring

### Communication Protocol
- Command format: JSON or binary protocol
- Status updates: Periodic telemetry
- Error handling: Fault codes and recovery

## Control Algorithms

### Motion Control
- **Objective**: Navigate the spherical robot in desired directions
- **Methods**: 
  - Pendulum control
  - Reaction wheel control
  - Hybrid approaches
- **Inputs**: Desired velocity/direction, current IMU state
- **Outputs**: Motor commands

### LiDAR Excitation
- **Objective**: Optimize robot motion for passive LiDAR reflection
- **Methods**: Perception-aware trajectory planning
- **Inputs**: LiDAR scanner position, environment map
- **Outputs**: Optimal motion trajectory

## Development Setup

### Prerequisites
- PlatformIO or Arduino IDE (for firmware)
- Python 3.8+ (for control algorithms)
- ROS/ROS2 (optional, for integration)

### Installation
```bash
# For firmware development
cd software/firmware
pio init  # or setup your preferred IDE

# For control software
cd software/control
pip install -r requirements.txt
```

### Build and Upload
```bash
# Firmware
cd software/firmware
pio run --target upload

# Control software
cd software/control
python main.py
```

## Testing

### Unit Tests
- Motor control functions
- Sensor reading accuracy
- Communication protocols

### Integration Tests
- Full motion sequences
- Sensor-control loop
- Emergency scenarios

### Simulation
- Gazebo or custom simulator
- Physics validation
- Algorithm verification

## Dependencies

### Firmware
- ESP32/Arduino Core
- IMU library (e.g., MPU9250)
- Motor control library

### Control Software
- NumPy, SciPy (numerical computation)
- Matplotlib (visualization)
- PySerial (communication)
- ROS2 (optional, for robotics integration)

## Configuration

Configuration files should be stored in `config/` directory:
- `robot_params.yaml`: Physical parameters
- `control_params.yaml`: Control gains and limits
- `network_config.yaml`: Communication settings

## Safety Features

1. **Emergency Stop**: Immediate motor shutdown
2. **Battery Monitor**: Low voltage protection
3. **Timeout**: Communication loss handling
4. **Limit Checks**: Speed and acceleration bounds
5. **Fault Detection**: Sensor failure detection

## Performance Metrics

- Motion accuracy: ±X degrees
- Response time: <Y ms
- Battery life: Z minutes
- Communication latency: <W ms

## To-Do

- [ ] Implement basic motor control firmware
- [ ] Add IMU sensor integration
- [ ] Develop motion control algorithms
- [ ] Create LiDAR excitation strategy
- [ ] Set up simulation environment
- [ ] Write comprehensive tests
- [ ] Document API and protocols

## References

- [ICRA 2026 Paper] PERAL: perception-aware motion control for passive LiDAR excitation in spherical robots
- ESP32 documentation
- Robot motion control literature

