# Electronics Design

This directory contains electronic schematics, PCB layouts, and wiring diagrams.

## Contents

### Schematics (`schematics/`)
- Power distribution circuit
- Motor driver circuits
- Microcontroller connections
- Sensor interfaces
- Communication modules

### PCB Layout (`pcb/`)
- Main control board
- Motor driver board (if separate)
- Sensor breakout boards
- Gerber files for manufacturing

## Design Tools

Recommended EDA software:
- **KiCad** (open-source, recommended)
- **Eagle** (Autodesk)
- **EasyEDA** (web-based)
- **Altium Designer** (professional)

## Circuit Blocks

### 1. Power Management
- Battery input (11.1V LiPo)
- 5V regulator for logic
- 3.3V regulator for sensors
- Power switch and fuse protection
- Battery voltage monitoring

### 2. Microcontroller
- ESP32 or STM32
- GPIO for motor control
- I2C for IMU
- UART for debugging
- WiFi/Bluetooth antenna

### 3. Motor Drivers
- Dual H-bridge (L298N, TB6612, or similar)
- PWM inputs from MCU
- Current sensing (optional)
- Protection diodes

### 4. Sensor Interface
- IMU (I2C or SPI)
- Optional: encoders, proximity sensors
- Pull-up resistors as needed

### 5. Communication
- USB-to-serial for programming
- WiFi antenna (if ESP32)
- Status LEDs
- Debug headers

## PCB Specifications

- Layers: 2-layer (or 4-layer for complex designs)
- Board size: [TBD] mm x [TBD] mm
- Mounting holes: M3 at corners
- Thickness: 1.6mm standard

## Component Selection

- Use automotive/industrial grade for reliability
- Consider operating temperature range
- Choose components with good availability
- Keep component count minimal

## Wiring Guidelines

### Power Wiring
- Use adequate wire gauge for motor current
- Keep power and signal wires separated
- Add bypass capacitors near ICs

### Signal Wiring
- Use twisted pairs for encoder signals
- Shield I2C and SPI lines if needed
- Label all connectors clearly

## Manufacturing

### PCB Fabrication
- Services: JLCPCB, PCBWay, OSH Park
- Include BOM and pick-and-place files
- Review: Always order bare PCB first for fit check

### Assembly
- SMD soldering or assembly service
- Through-hole for connectors
- Test points for debugging

## Testing Checklist

- [ ] Power rails voltage check
- [ ] Short circuit test
- [ ] MCU programming test
- [ ] Motor driver functionality
- [ ] Sensor communication
- [ ] Full system integration

## Safety Notes

- Add reverse polarity protection
- Include current limiting
- Use TVS diodes for ESD protection
- Thermal relief on power pads
- Keep high voltage traces isolated

## To-Do

- [ ] Complete schematic design
- [ ] Layout PCB
- [ ] Generate BOM
- [ ] Create assembly drawings
- [ ] Order prototypes
- [ ] Test and iterate

