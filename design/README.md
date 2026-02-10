# Design Files

This directory contains all design files for the PERAL spherical robot project.

## Directory Structure

### `mechanical/`
Place mechanical design files here:
- CAD models (SolidWorks, Fusion 360, etc.)
- 3D printing files (STL, STEP)
- Assembly drawings
- Technical specifications

**Supported formats**: `.sldprt`, `.sldasm`, `.f3d`, `.step`, `.stl`, `.dwg`, `.pdf`

**Example files**:
- `robot_shell.sldprt` - Main robot shell design
- `motor_mount.stl` - 3D printable motor mount
- `assembly_drawing.pdf` - Assembly instructions

### `electrical/`
Place electrical design files here:
- Circuit schematics
- PCB layouts
- Wiring diagrams
- Power distribution designs

**Supported formats**: `.sch`, `.brd`, `.kicad_pcb`, `.pdf`, `.png`

**Example files**:
- `main_controller_pcb.kicad_pcb` - Main controller board
- `power_system_schematic.pdf` - Power system diagram
- `wiring_diagram.png` - System wiring overview

### `documentation/`
Place design documentation here:
- Design reports
- Calculation sheets
- Analysis results
- Design iterations and notes

**Supported formats**: `.pdf`, `.docx`, `.md`, `.tex`

**Example files**:
- `mechanical_analysis.pdf` - Structural analysis report
- `design_rationale.md` - Design decisions documentation
- `calculations.xlsx` - Engineering calculations

## Naming Convention

Please use descriptive names with version numbers if applicable:
- `component_name_v1.0.ext`
- `robot_shell_final.stl`
- `pcb_rev2.kicad_pcb`

## File Organization Tips

1. **Version control**: Keep track of design iterations
2. **Export formats**: Include both native and universal formats (STEP, PDF)
3. **Documentation**: Always include a description of what the file contains
4. **File size**: For large files (>100MB), consider using Git LFS or external storage

## Collaboration

When updating designs:
1. Check if anyone else is working on the same component
2. Document your changes in commit messages
3. Include screenshots or renders when appropriate
4. Update documentation to match design changes

For questions about design standards, contact the project supervisor.
