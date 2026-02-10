# Firmware - PERAL Robot

Microcontroller firmware for low-level robot control.

## Overview

This firmware runs on the robot's microcontroller (ESP32/STM32) and handles:
- Motor control (PWM generation, direction)
- Sensor data acquisition (IMU)
- Communication with high-level controller
- Safety and fault handling

## Development Environment

### PlatformIO Setup (Recommended)
```bash
# Install PlatformIO
pip install platformio

# Initialize project
cd firmware
pio init

# Build
pio run

# Upload to board
pio run --target upload

# Open serial monitor
pio device monitor
```

### Arduino IDE Setup
1. Install Arduino IDE
2. Add ESP32 board support
3. Install required libraries
4. Open firmware.ino and upload

## Project Structure

```
firmware/
├── src/
│   ├── main.cpp              # Main program
│   ├── motor_control.cpp     # Motor control functions
│   ├── imu_sensor.cpp        # IMU interface
│   ├── communication.cpp     # Serial/WiFi comm
│   └── safety.cpp            # Safety monitoring
├── include/
│   ├── motor_control.h
│   ├── imu_sensor.h
│   ├── communication.h
│   ├── safety.h
│   └── config.h              # Configuration parameters
├── lib/                      # External libraries
├── test/                     # Unit tests
├── platformio.ini            # PlatformIO configuration
└── README.md                 # This file
```

## Hardware Requirements

- Microcontroller: ESP32 DevKit or STM32F4
- Motor drivers: Connected to PWM pins
- IMU sensor: MPU9250 or similar (I2C)
- Power supply: 3.3V/5V regulated

## Pin Configuration

### ESP32 Example
```cpp
// Motors
#define MOTOR_LEFT_PWM    25
#define MOTOR_LEFT_DIR    26
#define MOTOR_RIGHT_PWM   27
#define MOTOR_RIGHT_DIR   14

// IMU (I2C)
#define IMU_SDA           21
#define IMU_SCL           22

// Communication
#define SERIAL_BAUD       115200
#define WIFI_SSID         "PERAL_Robot"
```

## Core Modules

### 1. Motor Control
```cpp
// Initialize motors
void motorInit();

// Set motor speed (-100 to 100)
void setMotorSpeed(int motor, int speed);

// Emergency stop
void motorStop();
```

### 2. IMU Sensor
```cpp
// Initialize IMU
bool imuInit();

// Read orientation (roll, pitch, yaw)
void getOrientation(float* roll, float* pitch, float* yaw);

// Read accelerometer
void getAccel(float* ax, float* ay, float* az);
```

### 3. Communication
```cpp
// Process incoming commands
void processCommand(String cmd);

// Send status update
void sendStatus();

// WiFi setup (ESP32)
void setupWiFi();
```

### 4. Safety
```cpp
// Monitor battery voltage
float getBatteryVoltage();

// Check for faults
bool checkSafety();

// Handle emergency stop
void emergencyStop();
```

## Communication Protocol

### Command Format (JSON)
```json
{
  "cmd": "move",
  "params": {
    "left_speed": 50,
    "right_speed": 50
  }
}
```

### Status Format (JSON)
```json
{
  "status": "ok",
  "battery": 11.8,
  "imu": {
    "roll": 0.5,
    "pitch": -2.1,
    "yaw": 45.3
  }
}
```

## Building and Flashing

### Using PlatformIO
```bash
# Clean build
pio run --target clean

# Build
pio run

# Upload via USB
pio run --target upload

# Upload via OTA (WiFi)
pio run --target upload --upload-port <IP_ADDRESS>
```

### Using Arduino IDE
1. Select board: ESP32 Dev Module
2. Select port: /dev/ttyUSB0 (Linux) or COM port (Windows)
3. Click Upload

## Configuration

Edit `include/config.h` to customize:
- Pin assignments
- PID controller gains
- Communication settings
- Safety thresholds

## Testing

### Unit Tests
```bash
pio test
```

### Serial Monitor Test
```bash
pio device monitor
```

Send test commands:
```
{"cmd":"move","params":{"left_speed":30,"right_speed":30}}
{"cmd":"stop"}
{"cmd":"status"}
```

## Debugging

### Serial Debug
- Enable debug prints in code
- Monitor via `pio device monitor`
- Use appropriate baud rate (115200)

### LED Indicators
- Power LED: System running
- Status LED: Activity indicator
- Error LED: Fault condition

## Troubleshooting

**Motors not responding:**
- Check wiring and connections
- Verify PWM signal with oscilloscope
- Test motor driver independently

**IMU not detected:**
- Check I2C connections (SDA, SCL)
- Verify I2C address with scanner
- Check pull-up resistors (4.7kΩ)

**WiFi connection issues:**
- Verify SSID and password
- Check WiFi signal strength
- Use serial fallback

## Dependencies

Libraries required:
- Wire (I2C communication)
- WiFi (ESP32 only)
- MPU9250 or Adafruit_MPU6050
- ArduinoJson (for communication)

Install via PlatformIO:
```ini
[env:esp32dev]
lib_deps =
    bblanchon/ArduinoJson@^6.21.0
    adafruit/Adafruit MPU6050@^2.2.4
```

## Performance

- Loop frequency: ~100Hz
- IMU sampling: 100Hz
- PWM frequency: 1kHz
- Serial baud: 115200

## Future Improvements

- [ ] Implement PID control for motors
- [ ] Add motor encoder support
- [ ] Implement sensor fusion for IMU
- [ ] Add OTA firmware updates
- [ ] Optimize power consumption
- [ ] Add data logging to SD card

## License

See main repository LICENSE file.

