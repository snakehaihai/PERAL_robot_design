# PERAL Robot - Quick Reference

Quick reference guide for navigating and using the PERAL robot design repository.

## 📚 Essential Documents

| Document | Purpose | Link |
|----------|---------|------|
| Main README | Project overview and getting started | [README.md](../README.md) |
| Bill of Materials | List of all components needed | [BOM.md](../BOM.md) |
| Contributing Guide | How to contribute to the project | [CONTRIBUTING.md](../CONTRIBUTING.md) |
| Assembly Instructions | Step-by-step assembly guide | [docs/assembly/README.md](assembly/README.md) |
| Usage Guide | How to operate the robot | [docs/usage/README.md](usage/README.md) |

## 🗂️ Repository Structure

```
PERAL_robot_design/
├── README.md              # Start here!
├── BOM.md                 # Parts list
├── CONTRIBUTING.md        # Contribution guidelines
├── .gitignore            # Git ignore rules
├── LICENSE               # Project license
│
├── hardware/             # All hardware design files
│   ├── README.md        # Hardware overview
│   ├── cad/             # CAD models (3D designs)
│   ├── electronics/     # Schematics and PCB
│   └── mechanical/      # Technical drawings
│
├── software/            # All software and code
│   ├── README.md       # Software overview
│   ├── firmware/       # Microcontroller code
│   ├── control/        # High-level control algorithms
│   └── simulation/     # Simulation tools
│
└── docs/               # Documentation
    ├── assembly/       # Assembly instructions
    └── usage/          # Usage guides
```

## 🎯 Common Tasks

### I Want To...

#### Build the Robot
1. Read [BOM.md](../BOM.md) - order all components
2. Review [hardware/README.md](../hardware/README.md) - understand the design
3. Follow [docs/assembly/README.md](assembly/README.md) - assemble step by step
4. Follow [software/firmware/README.md](../software/firmware/README.md) - upload firmware

#### Use the Robot
1. Read [docs/usage/README.md](usage/README.md) - operation guide
2. Review [software/control/README.md](../software/control/README.md) - control software
3. Start with basic commands and gradually increase complexity

#### Contribute to the Project
1. Read [CONTRIBUTING.md](../CONTRIBUTING.md) - contribution guidelines
2. Fork the repository
3. Make your changes
4. Submit a pull request

#### Understand the Hardware
- **Mechanical Design**: See [hardware/cad/README.md](../hardware/cad/README.md)
- **Electronics**: See [hardware/electronics/README.md](../hardware/electronics/README.md)
- **Components**: See [BOM.md](../BOM.md)

#### Understand the Software
- **System Overview**: See [software/README.md](../software/README.md)
- **Firmware**: See [software/firmware/README.md](../software/firmware/README.md)
- **Control Algorithms**: See [software/control/README.md](../software/control/README.md)

## 🚀 Quick Start Paths

### Path 1: Researcher/Student
**Goal**: Build and experiment with the robot
1. Start: [README.md](../README.md)
2. Order parts: [BOM.md](../BOM.md)
3. Build: [docs/assembly/README.md](assembly/README.md)
4. Use: [docs/usage/README.md](usage/README.md)

### Path 2: Hardware Engineer
**Goal**: Improve or customize hardware design
1. Review: [hardware/README.md](../hardware/README.md)
2. Understand: [BOM.md](../BOM.md)
3. Design: Use CAD tools and update files in `hardware/`
4. Contribute: [CONTRIBUTING.md](../CONTRIBUTING.md)

### Path 3: Software Developer
**Goal**: Develop control algorithms or firmware
1. Setup: [software/README.md](../software/README.md)
2. Firmware: [software/firmware/README.md](../software/firmware/README.md)
3. Control: [software/control/README.md](../software/control/README.md)
4. Contribute: [CONTRIBUTING.md](../CONTRIBUTING.md)

### Path 4: Documentation Writer
**Goal**: Improve documentation
1. Read: All README files
2. Identify: Gaps or unclear sections
3. Write: Clear, helpful content
4. Contribute: [CONTRIBUTING.md](../CONTRIBUTING.md)

## 🔍 Key Concepts

### Hardware
- **Spherical Shell**: Transparent outer shell for LiDAR
- **Internal Drive**: Mechanism for omnidirectional motion
- **IMU**: Inertial Measurement Unit for orientation
- **Motors**: DC motors for motion control

### Software
- **Firmware**: Low-level microcontroller code
- **Control Algorithms**: High-level motion planning
- **LiDAR Excitation**: Perception-aware motion strategy
- **State Estimation**: Sensor fusion and localization

### Research
- **Passive LiDAR**: Using external LiDAR scanners
- **Excitation**: Optimizing motion for better perception
- **Perception-aware**: Considering sensor limitations in planning

## 📞 Getting Help

### Documentation Issues
- Check if information exists elsewhere in docs
- Open an issue if documentation is missing/unclear
- Suggest improvements via pull request

### Technical Problems
- **Hardware**: See troubleshooting in [docs/assembly/README.md](assembly/README.md)
- **Software**: See troubleshooting in [docs/usage/README.md](usage/README.md)
- **General**: Open an issue on GitHub

### Questions
- Check existing issues for similar questions
- Open a new issue with "question" label
- Provide context and what you've already tried

## 🔗 Important Links

- **GitHub Repository**: https://github.com/snakehaihai/PERAL_robot_design
- **Main README**: [README.md](../README.md)
- **License**: [LICENSE](../LICENSE)

## 📋 Checklists

### Before Building
- [ ] Read main README
- [ ] Review BOM - verify all parts available
- [ ] Understand assembly process
- [ ] Have all tools ready
- [ ] Understand safety requirements

### Before First Use
- [ ] Assembly complete and tested
- [ ] Firmware uploaded
- [ ] Battery charged
- [ ] Safety checks complete
- [ ] Emergency stop tested

### Before Contributing
- [ ] Read CONTRIBUTING.md
- [ ] Fork repository
- [ ] Create feature branch
- [ ] Test your changes
- [ ] Update documentation

## 💡 Tips

- **Start Simple**: Don't try to understand everything at once
- **Follow Order**: Use the suggested paths above
- **Ask Questions**: Don't hesitate to open issues
- **Document**: Keep notes of your changes and findings
- **Share**: Contribute improvements back to the project

## 🎓 Learning Resources

### Spherical Robots
- Research papers on spherical robot dynamics
- Videos of spherical robots in action
- Related open-source projects

### Control Systems
- PID control tutorials
- State estimation and Kalman filters
- Motion planning algorithms

### Embedded Systems
- Microcontroller programming
- Sensor interfacing
- Communication protocols

### LiDAR and Perception
- LiDAR technology basics
- Point cloud processing
- Perception-aware planning

---

**Need something else?** Check the [main README](../README.md) or open an issue!

