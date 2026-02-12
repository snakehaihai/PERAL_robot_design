# PERAL Robot Design

**[ICRA 2026] PERAL: perception-aware motion control for passive LiDAR excitation in spherical robots**

This repository contains the complete hardware design, and software for the PERAL spherical robot - a research platform for studying perception-aware motion control for passive LiDAR excitation.

### For Development

```bash
# Clone the repository
git clone https://github.com/snakehaihai/PERAL_robot_design.git
cd PERAL_robot_design

```

## 📖 Research Context

The PERAL robot is designed for research on:
- **Perception-aware motion planning**: Optimize robot motion for LiDAR visibility
- **Passive LiDAR excitation**: Maximize point cloud density through intelligent motion
- **Spherical robot dynamics**: Study and control of spherical robots
- **Multi-sensor fusion**: Integration of IMU and other sensors

## 🏗️ Project Status

### Current Status
- [x] ROS2 workspace
- [x] Hardware Documentation
- [x] microros_esp32
- [ ] Update README

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
├── LICENSE                   # License file
├── haloball/src              # ROS workspace
│   ├── FAST_LIO/             
│   ├── haloball_bringup/     # Centralised package to manage FAST-LIO and livox_ros_driver2
│   ├── livox_ros_driver2/    # Used by MID-360 LiDAR
│   └── nmpc_acados/          # NMPC
├── hardware documentation/   
│   ├── 3D model/
│   ├── ESP32-C3 SuperMini/
│   ├── Motor and Drivers/
│   ├── Kinematics.png
│   └── haloball_documentation.xlsx # Refer together with Kinematics.png
└── microros_esp32/           # To be flashed on esp32
    ├── include/             
    ├── lib/               
    ├── src/               
    └── README.md            # Refer for instruction
```

## 🛠️ Development Tools

### Hardware
- **CAD Software**: FreeCAD
- **3D Printing**: PLA for prototypes

### Software
- **Firmware**: PlatformIO, ROS2
- **Languages**: C++, Python
- **Version Control**: Git

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
- [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
- [ACADOS](https://github.com/acados/acados)

---

**Note**: This is an active research project. Components marked as "TBD" (To Be Determined) should be customized based on your specific design requirements and research needs.

For the latest updates and more information, visit the [GitHub repository](https://github.com/snakehaihai/PERAL_robot_design).
