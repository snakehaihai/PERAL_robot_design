# Bill of Materials (BOM) - PERAL Spherical Robot

This document lists all the components required to build the PERAL spherical robot for passive LiDAR excitation experiments.

## Mechanical Components

| Item | Description | Quantity | Specifications | Notes |
|------|-------------|----------|----------------|-------|
| Spherical Shell | Outer transparent shell | 1 | Diameter: TBD, Material: Polycarbonate/Acrylic | Must be transparent for LiDAR |
| Inner Frame | Support structure | 1 | Material: Aluminum/3D printed | Holds electronics and motors |
| Drive Wheels | Internal drive mechanism | 2-4 | Diameter: TBD, Material: Rubber/TPU | For motion control |
| Bearings | Ball bearings for wheels | 4-8 | Size: TBD | Low friction |
| Fasteners | Screws, nuts, bolts | Various | M3, M4, M5 | Stainless steel |
| Counterweight | Balance weight | 1 | Mass: TBD | For stability |

## Electronics Components

| Item | Description | Quantity | Specifications | Part Number/Link | Notes |
|------|-------------|----------|----------------|------------------|-------|
| Microcontroller | Main processor | 1 | ESP32/Raspberry Pi/STM32 | TBD | Motion control |
| Motor Driver | DC motor controller | 2 | H-bridge, Current: TBD | TBD | For drive motors |
| DC Motors | Drive motors | 2-4 | Voltage: 12V, Torque: TBD | TBD | With encoders preferred |
| IMU | Inertial Measurement Unit | 1 | 9-DOF, I2C/SPI | MPU9250 or similar | For orientation |
| Battery | Power supply | 1 | LiPo, Voltage: 11.1V-14.8V, Capacity: TBD | TBD | Runtime consideration |
| Voltage Regulator | Power regulation | 2-3 | 5V and 3.3V output | TBD | For electronics |
| Power Switch | On/off control | 1 | Current rating: TBD | TBD | Easy access |
| Charging Port | Battery charging | 1 | XT60 or similar | TBD | External access |
| PCB | Custom circuit board | 1 | Design: TBD | TBD | Optional |

## Sensors (Optional)

| Item | Description | Quantity | Specifications | Part Number/Link | Notes |
|------|-------------|----------|----------------|------------------|-------|
| Encoders | Motor position sensing | 2-4 | Resolution: TBD | TBD | If not integrated |
| Proximity Sensors | Obstacle detection | 2-4 | Range: TBD | TBD | Optional |

## Communication (Optional)

| Item | Description | Quantity | Specifications | Part Number/Link | Notes |
|------|-------------|----------|----------------|------------------|-------|
| WiFi Module | Wireless communication | 1 | 2.4GHz/5GHz | Built-in ESP32 | For remote control |
| Bluetooth Module | Short-range comm | 1 | BLE 5.0 | Built-in ESP32 | For debugging |

## Tools Required

- Soldering iron and solder
- Wire strippers and cutters
- Screwdriver set (Phillips and flathead)
- Allen key set
- Multimeter
- 3D printer (if using printed parts)
- Drill and drill bits

## Notes

- **TBD items** need to be specified based on the final design requirements
- All electronic components should be verified for compatibility
- Consider environmental factors (temperature, humidity) for component selection
- Ensure all components fit within the spherical shell dimensions
- Total estimated cost: TBD

## Revision History

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| TBD | 1.0 | Initial BOM | TBD |

