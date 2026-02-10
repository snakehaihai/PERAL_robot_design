# Assembly Instructions - PERAL Spherical Robot

This guide provides step-by-step instructions for assembling the PERAL robot.

## Safety Precautions

⚠️ **Important Safety Notes:**
- Always disconnect battery before working on electronics
- Use appropriate personal protective equipment (safety glasses, etc.)
- Work in a well-ventilated area when soldering
- Handle lithium batteries with care - never short circuit
- Keep a fire extinguisher nearby when working with batteries

## Required Tools

- Soldering iron and solder
- Wire strippers and cutters
- Screwdriver set (Phillips and flathead)
- Allen key set (hex wrenches)
- Multimeter
- Hot glue gun (optional)
- Cable ties
- Heat shrink tubing

## Parts Checklist

Before starting, verify you have all parts from the [BOM](../../BOM.md):

- [ ] Spherical shell (2 hemispheres)
- [ ] Internal frame
- [ ] Motors (2-4 units)
- [ ] Motor drivers
- [ ] Microcontroller board
- [ ] IMU sensor
- [ ] Battery
- [ ] Voltage regulators
- [ ] Wiring and connectors
- [ ] Fasteners (screws, nuts, bolts)
- [ ] Switch and charging port

## Assembly Steps

### Step 1: Prepare the Internal Frame

1. **Inspect the frame** for any defects or required cleanup (if 3D printed)
2. **Install threaded inserts** or heat-set inserts if using 3D printed parts
3. **Test fit** all components before permanent installation

### Step 2: Mount Motors

1. **Attach motor brackets** to the frame
2. **Secure motors** with screws (typically M3)
3. **Check alignment** - motors should rotate freely
4. **Connect wheels/drive mechanism** to motor shafts

**Important:** Ensure motors are balanced and symmetric for stable motion.

### Step 3: Electronics Assembly

#### 3.1 Power System
1. **Mount battery holder** in designated location
2. **Install power switch** in accessible position
3. **Solder voltage regulators**:
   - Input: Battery voltage (11.1V)
   - Output 1: 5V for motor drivers
   - Output 2: 3.3V for microcontroller/sensors
4. **Add fuse** for protection (recommended)
5. **Test voltages** with multimeter before connecting components

#### 3.2 Microcontroller
1. **Mount microcontroller** on frame or PCB
2. **Connect power** (3.3V/5V and GND)
3. **Add decoupling capacitors** near power pins
4. **Install programming header** for easy access

#### 3.3 Motor Drivers
1. **Mount motor driver boards**
2. **Connect to 5V power supply**
3. **Wire PWM control lines** from microcontroller
4. **Connect motor terminals** (observe polarity)
5. **Add flyback diodes** if not included on driver board

#### 3.4 IMU Sensor
1. **Mount IMU** at center of frame (ideal for measurements)
2. **Connect I2C lines** (SDA, SCL)
3. **Add pull-up resistors** (4.7kΩ) if needed
4. **Secure firmly** to avoid vibration affecting readings

### Step 4: Wiring

1. **Create wiring harness** following these guidelines:
   - Use color coding (red=positive, black=ground, etc.)
   - Keep power wires separated from signal wires
   - Use twisted pairs for motor wires
   - Label all connections

2. **Wire organization:**
   ```
   Battery → Switch → Voltage Regulators
                    ↓
            Motor Drivers ← Microcontroller → IMU
                    ↓
                  Motors
   ```

3. **Secure wires** with cable ties
4. **Add strain relief** at connection points
5. **Route wires** to avoid moving parts

### Step 5: Testing Electronics

**Before closing the shell, test everything:**

1. **Power test:**
   - Connect battery
   - Check all voltage rails with multimeter
   - Verify no short circuits

2. **Motor test:**
   - Run each motor individually
   - Check direction of rotation
   - Verify speed control

3. **Sensor test:**
   - Upload test firmware
   - Read IMU data
   - Verify all readings are reasonable

4. **Communication test:**
   - Test serial communication
   - Test WiFi connection (if applicable)

### Step 6: Final Assembly

1. **Cable management:**
   - Organize and secure all wiring
   - Ensure no wires interfere with moving parts
   - Check clearances

2. **Balance check:**
   - Add counterweights if necessary
   - Test center of gravity
   - Adjust component placement for balance

3. **Close the shell:**
   - Carefully place upper hemisphere
   - Align mounting holes
   - Secure with screws (don't overtighten)
   - Check for gaps (seal if needed for dust protection)

4. **External components:**
   - Install charging port (accessible from outside)
   - Ensure power switch is reachable
   - Add status LED if desired

### Step 7: Final Testing

1. **Mechanical test:**
   - Gently roll the robot
   - Check for any loose parts or rattling
   - Verify smooth motion

2. **Functional test:**
   - Power on the robot
   - Test basic movements
   - Check IMU readings during motion
   - Test communication

3. **Safety test:**
   - Emergency stop functionality
   - Battery voltage monitoring
   - Timeout behavior

## Troubleshooting

### Common Issues

**Robot doesn't power on:**
- Check battery charge level
- Verify power switch connection
- Check for short circuits
- Test voltage at each regulator

**Motors don't run:**
- Verify motor driver connections
- Check PWM signals with oscilloscope
- Test motors directly with power supply
- Check firmware motor control code

**IMU readings incorrect:**
- Verify I2C connections
- Check I2C address with scanner
- Ensure sensor is mounted rigidly
- Calibrate IMU sensor

**Unstable motion:**
- Check center of gravity
- Verify motor mounting
- Balance the robot
- Tune control parameters

**Communication issues:**
- Verify baud rate settings
- Check cable connections
- Test with serial monitor
- Try WiFi reset (if applicable)

## Calibration

### IMU Calibration
1. Place robot on level surface
2. Run calibration routine
3. Rotate through all axes
4. Save calibration values

### Motor Calibration
1. Test individual motor speeds
2. Match motor responses
3. Set speed limits
4. Configure acceleration profiles

### Balance Adjustment
1. Place robot on flat surface
2. Observe natural resting position
3. Add/move counterweights as needed
4. Test motion stability

## Maintenance

### Regular Checks
- Battery voltage and health
- Motor bearing condition
- Loose screws or connections
- Wire wear and damage
- IMU mounting stability

### Cleaning
- Wipe shell with soft cloth
- Clean sensor surfaces
- Remove debris from motor area
- Check for dust in electronics

### Battery Care
- Charge before storage
- Store at ~50% charge
- Check voltage regularly
- Replace if swollen or damaged

## Next Steps

After successful assembly:
1. Upload control firmware
2. Test motion control
3. Implement LiDAR excitation algorithms
4. Calibrate and tune parameters
5. Run experiments

## Documentation

Take photos/videos of:
- Assembly process
- Wiring layout
- Component placement
- Final robot

This helps with:
- Troubleshooting
- Replication
- Documentation
- Publication materials

## Support

For issues or questions:
- Check troubleshooting section
- Review software documentation
- Consult BOM and hardware designs
- Contact maintainers

## Safety Reminders

- Always work in a safe environment
- Keep battery safety in mind
- Test incrementally
- Document any modifications
- Have emergency shutdown ready

---

**Congratulations on completing the assembly!**

Now proceed to software setup and testing. Good luck with your PERAL robot experiments!

