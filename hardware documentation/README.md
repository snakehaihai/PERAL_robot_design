# Hardware Design - PERAL Spherical Robot

This directory contains all hardware design files for the PERAL spherical robot.

## Directory Structure

```
hardware/
├── cad/                 # CAD files for mechanical design
│   ├── shell/          # Spherical shell design
│   ├── frame/          # Internal frame structure
│   ├── assembly/       # Assembly models
│   └── README.md       # CAD file information
├── electronics/        # Electronic schematics and PCB designs
│   ├── schematics/    # Circuit schematics
│   ├── pcb/           # PCB layouts
│   └── README.md      # Electronics documentation
└── mechanical/        # Mechanical specifications and drawings
    ├── drawings/      # Technical drawings
    ├── specs/         # Component specifications
    └── README.md      # Mechanical documentation
```

## Design Overview

The PERAL spherical robot is designed for passive LiDAR excitation research. Key design considerations include:

1. **Transparent Shell**: The outer shell must be transparent to allow LiDAR penetration
2. **Internal Drive Mechanism**: Pendulum-based or hamster-wheel drive system for omnidirectional motion
3. **Sensor Integration**: Mounting points for IMU and other sensors
4. **Power System**: Battery placement for optimal weight distribution
5. **Accessibility**: Easy access for maintenance and component replacement

## Design Files

### CAD Models
- **Software**: SolidWorks, Fusion 360, or FreeCAD recommended
- **Format**: Native files + STEP/STL for compatibility
- **Version Control**: Save major revisions with version numbers

### Electronics
- **Schematic Software**: KiCad, Eagle, or Altium
- **PCB Design**: Include Gerber files for manufacturing
- **Wiring Diagrams**: Clearly labeled connection diagrams

## Design Guidelines

1. **Modularity**: Design components to be easily replaceable
2. **Weight Distribution**: Maintain center of gravity for stable motion
3. **Cable Management**: Plan for internal wiring routes
4. **Thermal Management**: Consider heat dissipation from motors and electronics
5. **Shock Resistance**: Protect sensitive components from impacts

## Manufacturing Notes

- Shell fabrication: CNC machining or thermoforming
- Frame: 3D printing (PLA/PETG) or CNC aluminum
- PCB: Standard PCB manufacturing services
- Assembly: Document assembly sequence

## Testing Considerations

- Structural integrity tests
- Weight balance verification
- Motion range validation
- Sensor mounting accuracy

## To-Do

- [ ] Complete CAD model of spherical shell
- [ ] Design internal frame structure
- [ ] Create PCB layout for motor control
- [ ] Generate assembly instructions
- [ ] Simulate motion dynamics

## References

- [ICRA 2026 Paper] PERAL: perception-aware motion control for passive LiDAR excitation in spherical robots
- Related spherical robot designs and research

