# Usage Guide - PERAL Robot

This guide covers the basic operation of the PERAL spherical robot after assembly.

## Prerequisites

Before operating the robot, ensure:
- [ ] Robot is fully assembled (see [Assembly Instructions](../assembly/README.md))
- [ ] Battery is fully charged
- [ ] Firmware is uploaded to microcontroller
- [ ] Control software is installed (if using high-level control)
- [ ] All connections are secure
- [ ] Initial testing is complete

## Safety Guidelines

⚠️ **Always follow these safety rules:**

1. **Battery Safety**
   - Never leave charging unattended
   - Check battery voltage before use
   - Disconnect if battery is hot or swollen
   - Have fire extinguisher accessible

2. **Operation Safety**
   - Test in safe, open area first
   - Keep away from stairs and ledges
   - Emergency stop should be easily accessible
   - Supervise during operation

3. **Maintenance Safety**
   - Always disconnect battery before maintenance
   - Use proper tools
   - Check for loose parts regularly

## Initial Setup

### 1. Battery Installation

1. Ensure robot is powered off
2. Install fully charged battery
3. Connect battery to power connector (observe polarity!)
4. Secure battery in holder
5. Close access panel

### 2. Power On Sequence

1. Verify emergency stop is accessible
2. Turn on main power switch
3. Wait for startup sequence:
   - Power LED should illuminate
   - Status LED may blink during initialization
   - Listen for any unusual sounds

4. Check status:
   - LED indicators show normal operation
   - No error beeps or flashing
   - Firmware initialization complete

### 3. Communication Setup

#### Serial Connection (USB)
```bash
# Linux/Mac
screen /dev/ttyUSB0 115200

# Or use PlatformIO
pio device monitor

# Windows
# Use PuTTY or Arduino Serial Monitor
```

#### WiFi Connection (if equipped)
1. Robot creates access point: `PERAL_Robot`
2. Connect to network
3. Default IP: `192.168.4.1`
4. Use web interface or API

## Basic Operation

### Manual Control

#### Using Serial Commands
Send commands via serial terminal:

```json
// Move forward
{"cmd":"move","params":{"left_speed":50,"right_speed":50}}

// Turn right
{"cmd":"move","params":{"left_speed":50,"right_speed":20}}

// Stop
{"cmd":"stop"}

// Request status
{"cmd":"status"}
```

#### Using Control Software
```bash
cd software/control
python main.py

# Follow on-screen prompts for control
```

### Monitoring

#### View Robot Status
Request status to see:
- Battery voltage
- IMU orientation (roll, pitch, yaw)
- Motor speeds
- Error codes (if any)

Example response:
```json
{
  "status": "ok",
  "battery": 11.8,
  "imu": {
    "roll": 0.5,
    "pitch": -2.1,
    "yaw": 45.3
  },
  "motors": {
    "left": 50,
    "right": 50
  }
}
```

#### Real-time Monitoring
Use the visualization tools:
```bash
cd software/control
python utils/monitor.py --port /dev/ttyUSB0
```

## Advanced Features

### LiDAR Excitation Mode

For research experiments with LiDAR:

1. **Setup**
   - Position LiDAR scanner in known location
   - Configure scanner parameters in config file
   - Start control software with excitation mode

2. **Operation**
   ```bash
   python main.py --mode lidar_excitation --config config/lidar_params.yaml
   ```

3. **Monitor**
   - Robot will autonomously plan motion
   - Optimize for LiDAR visibility
   - Log data for analysis

### Autonomous Navigation

If implementing autonomous features:

1. Load map or environment model
2. Set goal position
3. Start autonomous mode
4. Monitor progress

```python
from control.navigation import Navigator

nav = Navigator()
nav.set_goal(x=5.0, y=3.0)
nav.start()
```

### Data Logging

Enable logging for research:

```bash
python main.py --log data/experiment_1.log
```

Log includes:
- Timestamps
- Robot state (position, orientation, velocity)
- Sensor readings
- Control commands
- Events and errors

## Common Operations

### Charging the Battery

1. Power off the robot
2. Disconnect battery (optional) or use charging port
3. Connect charger (use appropriate LiPo charger)
4. Set charging current (typically 1C = 1x battery capacity)
5. Monitor during charging
6. Stop when fully charged (never overcharge)

### Calibration

#### IMU Calibration
```bash
# Via serial command
{"cmd":"calibrate","params":{"sensor":"imu"}}

# Or use calibration script
cd software/firmware
python scripts/calibrate_imu.py
```

Follow prompts to:
1. Place on level surface
2. Keep still for calibration
3. Slowly rotate through all axes
4. Save calibration values

#### Motor Calibration
```bash
{"cmd":"calibrate","params":{"sensor":"motors"}}
```

This will:
1. Test each motor individually
2. Measure speed response
3. Adjust parameters for uniform performance

### Firmware Updates

To update firmware:

```bash
cd software/firmware

# Build new firmware
pio run

# Upload to robot
pio run --target upload

# Or use OTA (WiFi)
pio run --target upload --upload-port 192.168.4.1
```

### Configuration Changes

Edit configuration files in `software/control/config/`:

```yaml
# robot_params.yaml
robot:
  radius: 0.15  # Adjust as needed
  
motion:
  max_velocity: 0.5  # Reduce for safer operation
```

Reload configuration:
```bash
python main.py --config config/robot_params.yaml
```

## Troubleshooting

### Robot Not Responding

**Symptoms:** No response to commands

**Solutions:**
1. Check battery voltage
2. Verify communication connection
3. Reset microcontroller
4. Check firmware status

### Erratic Motion

**Symptoms:** Unstable or unpredictable movement

**Solutions:**
1. Recalibrate IMU
2. Check motor connections
3. Verify balance and center of gravity
4. Tune control parameters

### Communication Loss

**Symptoms:** Cannot connect or frequent disconnects

**Solutions:**
1. Check cable connections (serial)
2. Verify WiFi signal strength
3. Reset communication module
4. Check for interference

### Low Battery Warning

**Symptoms:** Battery voltage below threshold

**Actions:**
1. Stop operation immediately
2. Return to safe location
3. Power off and charge battery
4. Never operate with low battery

### Emergency Stop

**When to use:**
- Dangerous behavior
- Loss of control
- Component failure
- Any safety concern

**How to activate:**
1. Physical switch (if equipped)
2. Serial command: `{"cmd":"emergency_stop"}`
3. Power off robot

## Maintenance

### Daily (Before Each Use)
- [ ] Check battery voltage
- [ ] Inspect for loose parts
- [ ] Test basic movements
- [ ] Verify communication

### Weekly (Regular Use)
- [ ] Clean shell and components
- [ ] Check all connections
- [ ] Verify sensor accuracy
- [ ] Test emergency stop

### Monthly
- [ ] Deep clean all components
- [ ] Check motor bearings
- [ ] Inspect wiring for wear
- [ ] Update firmware if needed
- [ ] Recalibrate sensors

### As Needed
- Replace worn parts
- Repair damaged components
- Update software
- Modify configuration

## Performance Optimization

### Improving Battery Life
- Reduce maximum speed
- Use efficient motion paths
- Disable unused features
- Optimize control algorithms

### Improving Stability
- Adjust center of gravity
- Tune PID parameters
- Reduce acceleration
- Improve sensor mounting

### Improving Response
- Increase control loop frequency
- Optimize communication
- Reduce latency
- Use faster processors

## Data Analysis

### Extracting Logs
```bash
# Convert log to CSV
python utils/log_converter.py data/experiment_1.log -o results.csv

# Plot trajectory
python utils/plot_trajectory.py data/experiment_1.log
```

### Analyzing Performance
- Motion accuracy
- Response time
- Energy efficiency
- Sensor quality

## Best Practices

1. **Always Start Slow**: Test with low speeds first
2. **Regular Calibration**: Calibrate sensors regularly
3. **Monitor Battery**: Never let battery get too low
4. **Keep Clean**: Clean robot after each use
5. **Document Changes**: Keep notes on modifications
6. **Safety First**: Always have emergency stop ready

## Support Resources

- [Assembly Instructions](../assembly/README.md)
- [Hardware Documentation](../../hardware/README.md)
- [Software Documentation](../../software/README.md)
- [BOM](../../BOM.md)
- GitHub Issues: Report problems
- Community Forum: Ask questions

## Experiment Workflows

### Example: LiDAR Excitation Experiment

1. **Setup Phase**
   - Position LiDAR scanner
   - Calibrate robot
   - Load environment map
   - Configure parameters

2. **Execution Phase**
   - Start data logging
   - Run excitation algorithm
   - Monitor performance
   - Collect LiDAR data

3. **Analysis Phase**
   - Stop logging
   - Extract data
   - Analyze results
   - Generate plots

4. **Iteration**
   - Adjust parameters
   - Repeat experiment
   - Compare results

## Tips for Researchers

- Keep detailed lab notebook
- Version control your configurations
- Document all modifications
- Share results with community
- Contribute improvements back

---

**Happy robot operation!** 

For more help, refer to other documentation or open an issue on GitHub.

