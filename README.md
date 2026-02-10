# PERAL Robot Design

**[ICRA 2026] PERAL: perception-aware motion control for passive LiDAR excitation in spherical robots**

This repository contains the complete hardware design, bill of materials (BOM), and software for the PERAL spherical robot - a research platform for studying perception-aware motion control for passive LiDAR excitation.

## 📋 Repository Contents

This repository is organized into three main sections:

### 🔧 Hardware Design
Complete mechanical and electronic designs for building the robot.
- **Location:** [`hardware/`](hardware/)
- **Contents:** CAD files, PCB schematics, mechanical drawings
- **Documentation:** [Hardware README](hardware/README.md)

### 📦 Bill of Materials (BOM)
Comprehensive list of all components needed to build the robot.
- **Location:** [`BOM.md`](BOM.md)
- **Contents:** Mechanical parts, electronics, sensors, tools required
- **Status:** Template provided - needs customization for your specific design

### 💻 Software & Firmware
All code for robot control and operation.
- **Location:** [`software/`](software/)
- **Contents:** 
  - Microcontroller firmware ([`software/firmware/`](software/firmware/))
  - High-level control algorithms ([`software/control/`](software/control/))
  - Simulation tools ([`software/simulation/`](software/simulation/))
- **Documentation:** [Software README](software/README.md)

### 📚 Documentation
Assembly instructions and usage guides.
- **Location:** [`docs/`](docs/)
- **Contents:**
  - Assembly instructions ([`docs/assembly/`](docs/assembly/))
  - Usage guides ([`docs/usage/`](docs/usage/))

## 🚀 Quick Start

### For Building the Robot

1. **Review the BOM**: Check [`BOM.md`](BOM.md) and order all required components
2. **Hardware Design**: Review [`hardware/`](hardware/) for CAD files and schematics
3. **Fabrication**: Manufacture or 3D print mechanical parts, order PCBs
4. **Assembly**: Follow the step-by-step guide in [`docs/assembly/`](docs/assembly/)
5. **Software Setup**: Install firmware and control software from [`software/`](software/)

### For Software Development

```bash
# Clone the repository
git clone https://github.com/snakehaihai/PERAL_robot_design.git
cd PERAL_robot_design

# For firmware development (microcontroller)
cd software/firmware
# Follow firmware/README.md for setup

# For control software (high-level)
cd software/control
pip install -r requirements.txt
python main.py
```

## 🤖 Robot Overview

### Design Features

- **Spherical Shell**: Transparent outer shell for LiDAR penetration
- **Omnidirectional Motion**: Internal drive mechanism for flexible movement
- **Sensor Suite**: IMU for orientation, optional encoders
- **Wireless Control**: WiFi/Bluetooth connectivity
- **Modular Design**: Easy maintenance and component replacement

### Key Specifications

| Parameter | Value (Example) |
|-----------|----------------|
| Shell Diameter | TBD mm |
| Weight | ~2.0 kg |
| Battery | 11.1V LiPo |
| Runtime | TBD minutes |
| Max Speed | TBD m/s |
| Microcontroller | ESP32/STM32 |

*Note: Specifications marked as TBD need to be filled in based on your final design*

## 📖 Research Context

The PERAL robot is designed for research on:
- **Perception-aware motion planning**: Optimize robot motion for LiDAR visibility
- **Passive LiDAR excitation**: Maximize point cloud density through intelligent motion
- **Spherical robot dynamics**: Study and control of spherical robots
- **Multi-sensor fusion**: Integration of IMU and other sensors

## 🏗️ Project Status

### Current Status
- [x] Repository structure created
- [x] Documentation templates provided
- [ ] Hardware CAD models (to be added)
- [ ] Electronics schematics (to be added)
- [ ] Firmware implementation (to be added)
- [ ] Control algorithms implementation (to be added)
- [ ] Testing and validation (to be added)

### Getting Involved

This is an open research project. Contributions are welcome!

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## 📂 Directory Structure

```
PERAL_robot_design/
├── README.md                 # This file
├── BOM.md                    # Bill of materials
├── LICENSE                   # License file
├── hardware/                 # Hardware design files
│   ├── cad/                 # CAD models
│   ├── electronics/         # Schematics and PCB
│   └── mechanical/          # Technical drawings
├── software/                 # Software and firmware
│   ├── firmware/            # Microcontroller code
│   ├── control/             # High-level algorithms
│   └── simulation/          # Simulation tools
└── docs/                     # Documentation
    ├── assembly/            # Assembly instructions
    └── usage/               # Usage guides
```

## 🛠️ Development Tools

### Hardware
- **CAD Software**: FreeCAD, Fusion 360, SolidWorks
- **EDA Software**: KiCad, Eagle, Altium
- **3D Printing**: PLA/PETG for prototypes

### Software
- **Firmware**: PlatformIO, Arduino IDE
- **Languages**: C++, Python
- **Version Control**: Git
- **Simulation**: Gazebo, custom simulators

## 📝 Documentation

Detailed documentation is available in each subdirectory:
- [Hardware Documentation](hardware/README.md)
- [Software Documentation](software/README.md)
- [Firmware Documentation](software/firmware/README.md)
- [Control Software Documentation](software/control/README.md)
- [Assembly Instructions](docs/assembly/README.md)

## 🔬 Research & Publication

This robot design supports the research presented in:

**PERAL: Perception-aware motion control for passive LiDAR excitation in spherical robots**  
*Submitted to ICRA 2026*

If you use this design in your research, please cite our work (citation details to be added upon publication).

## 🤝 Contributing

We welcome contributions in several areas:
- Hardware design improvements
- Electronics optimization
- Software algorithms
- Documentation enhancements
- Testing and validation
- Bug reports and fixes

Please see our contributing guidelines (to be added) for more information.

## 📧 Contact

For questions, issues, or collaboration:
- Open an issue in this repository
- Contact the maintainers (details to be added)

## 📄 License

This project is licensed under the terms specified in the [LICENSE](LICENSE) file.

## 🙏 Acknowledgments

- Research team and contributors
- Open-source community
- Hardware/software libraries used

## 🔗 Related Resources

- Spherical robot research papers
- LiDAR perception papers
- Motion control literature
- Related open-source robot projects

---

**Note**: This is an active research project. Components marked as "TBD" (To Be Determined) should be customized based on your specific design requirements and research needs.

For the latest updates and more information, visit the [GitHub repository](https://github.com/snakehaihai/PERAL_robot_design).
