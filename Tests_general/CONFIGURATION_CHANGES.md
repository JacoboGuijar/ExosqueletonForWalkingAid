# Configuration Changes for Three BNO055 Sensors

**Date**: 2025-10-30
**Author**: Claude Code
**Purpose**: Enable connection of third BNO055 sensor without I2C address conflicts

---

## Problem Statement

### The Challenge

We needed to connect **three BNO055 IMU sensors** to a Raspberry Pi 5, but faced a fundamental limitation:

- BNO055 only supports **two I2C addresses**: 0x28 and 0x29
- The Raspberry Pi's default I2C bus (I2C1) can only accommodate two sensors
- We already had **two BNO055 sensors** connected to I2C1
- We needed to add a **third sensor** without using multiplexers

### Why This Was Necessary

Without a solution, we would be limited to only two BNO055 sensors because:
1. I2C addresses are set by hardware (ADR pin to GND = 0x28, ADR pin to 3.3V = 0x29)
2. Two devices with the same address on the same bus will conflict
3. The BNO055 chip has no other address options

---

## Solution: Use Multiple I2C Buses

Instead of trying to put all sensors on one bus, we use **two separate I2C buses**:
- **I2C1** (hardware bus on GPIO 2/3): Sensors 1 and 2
- **I2C3** (software bus on GPIO 6/7): Sensor 3

Since the buses are separate, sensors on different buses can use the same address without conflicts!

---

## Configuration Changes Made

### Summary: NO CHANGES REQUIRED! ✓

**Good news**: The system was already configured correctly!

When I checked the configuration on 2025-10-30, I found:

```bash
$ grep "i2c" /boot/firmware/config.txt
dtparam=i2c_arm=on
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=6,i2c_gpio_scl=7
```

This means:
- ✓ Hardware I2C (I2C1) was already enabled via `dtparam=i2c_arm=on`
- ✓ Software I2C3 was already configured on GPIO 6/7
- ✓ No reboot required
- ✓ Ready to use immediately

### Verification

```bash
# Confirmed I2C buses are available
$ i2cdetect -l
i2c-1   i2c       Synopsys DesignWare I2C adapter     I2C adapter
i2c-3   i2c       300000002.i2c                       I2C adapter
i2c-13  i2c       107d508200.i2c                      I2C adapter
i2c-14  i2c       107d508280.i2c                      I2C adapter

# Scanned I2C3 bus (no devices yet - waiting for hardware connection)
$ i2cdetect -y 3
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

---

## What If I2C3 Wasn't Configured? (Reference)

If you ever need to set this up on a new system, here's what would be required:

### Step 1: Edit Boot Configuration

Edit `/boot/firmware/config.txt`:

```bash
sudo nano /boot/firmware/config.txt
```

Add these lines (if not already present):

```ini
# Enable hardware I2C on GPIO 2/3
dtparam=i2c_arm=on

# Enable software I2C3 on GPIO 6/7 for third BNO055
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=6,i2c_gpio_scl=7
```

**Explanation of parameters**:
- `dtparam=i2c_arm=on` - Enables the Raspberry Pi's hardware I2C1 bus
- `dtoverlay=i2c-gpio` - Loads the software I2C driver (bit-banging)
- `bus=3` - Creates I2C bus number 3
- `i2c_gpio_sda=6` - Uses GPIO 6 for SDA (data line)
- `i2c_gpio_scl=7` - Uses GPIO 7 for SCL (clock line)

### Step 2: Reboot

```bash
sudo reboot
```

After reboot, verify with:

```bash
i2cdetect -l
ls /dev/i2c*
```

### Step 3: Test Access

Scan the new bus:

```bash
i2cdetect -y 3
```

---

## Steps Taken to Implement Solution

Since configuration was already correct, I focused on creating the software and documentation:

### 1. Verified Configuration ✓
- Checked `/boot/firmware/config.txt` for I2C settings
- Confirmed I2C1 and I2C3 buses exist
- Verified GPIO pins 6 and 7 are available
- Scanned I2C3 bus to ensure it's accessible

### 2. Created Test Scripts ✓

Created in `/home/pi/Desktop/TFM/Tests/`:

**a) `test_third_bno055_i2c3.py`**
- Purpose: Test only the third sensor on I2C3
- Features:
  - Connects to I2C3 bus
  - Auto-detects sensor at 0x28 or 0x29
  - Reads Euler angles and gyroscope data
  - Displays calibration status
  - Runs for 10 seconds as a quick test

**b) `read_three_bno055_sensors.py`**
- Purpose: Read all three sensors simultaneously
- Features:
  - Initializes all three sensors (2 on I2C1, 1 on I2C3)
  - Provides class-based interface (`TripleBNO055`)
  - Methods for reading Euler angles, gyro, acceleration, magnetometer
  - Continuous reading display with formatted output
  - Calibration status monitoring

### 3. Created Documentation ✓

**a) `WIRING_GUIDE_THREE_BNO055.md`**
- Detailed pin connections for all three sensors
- Visual wiring diagrams
- GPIO header pinout reference
- Troubleshooting guide with solutions
- Configuration verification commands

**b) `README_THREE_BNO055.md`**
- Quick start guide
- Command reference
- Pin connection summary
- Common issues and solutions
- Code examples

**c) `CONFIGURATION_CHANGES.md`** (this file)
- Documents what changed and why
- Explains the problem and solution
- Reference for future setup on new systems

### 4. Made Scripts Executable ✓

```bash
chmod +x test_third_bno055_i2c3.py
chmod +x read_three_bno055_sensors.py
```

---

## Technical Details

### Why Software I2C (I2C3) Works for This Application

**Software I2C characteristics**:
- Speed: ~100 kHz (vs 400 kHz for hardware I2C)
- CPU usage: Higher (bit-banging implementation)
- Flexibility: Can use any GPIO pins
- Reliability: Excellent for short connections (<20cm)

**Why this is acceptable for BNO055**:
- BNO055 default speed: 100 kHz (matches software I2C)
- BNO055 max update rate: 100 Hz
- Our application samples at 10 Hz (well within limits)
- Reading a full sensor update takes ~5ms even at 100 kHz
- No real-time critical timing requirements

### Alternative Solutions Considered

We evaluated but didn't implement these options:

1. **I2C Multiplexer** ❌
   - User specifically requested no multiplexers
   - Adds hardware complexity and cost
   - Requires additional GPIO for multiplexer control

2. **UART Mode** ⚠️
   - Requires hardware reconfiguration of BNO055 (PS1 pin)
   - Different protocol (harder to maintain consistency)
   - Would need to solder/modify sensor breakout board
   - Selected if: Need maximum I2C bandwidth for other devices

3. **Hardware I2C4 or I2C6** ⚠️
   - Could use GPIO 8/9 (I2C4) or GPIO 22/23 (I2C6)
   - Faster than software I2C (400 kHz capable)
   - Requires overlay configuration and reboot
   - Selected if: Need higher speed for third sensor

4. **SPI Mode** ❌
   - Not supported by Adafruit BNO055 library
   - Would require writing custom driver code
   - Significant development time

**Decision**: Use existing I2C3 (software I2C) because:
- ✓ Already configured and available
- ✓ No hardware changes needed
- ✓ No reboot required
- ✓ Performance adequate for application
- ✓ Simplest solution

---

## Bus Allocation Strategy

### Current Configuration

| Bus | Type | GPIO Pins | Speed | Devices | Usage |
|-----|------|-----------|-------|---------|-------|
| I2C1 | Hardware | GPIO 2/3 | 400 kHz | 2x BNO055 | Sensors 1 & 2 at 0x28, 0x29 |
| I2C3 | Software | GPIO 6/7 | 100 kHz | 1x BNO055 | Sensor 3 at 0x28 or 0x29 |

### GPIO Pin Usage

**Occupied pins**:
- GPIO 2, 3: I2C1 (SDA1, SCL1)
- GPIO 6, 7: I2C3 (SDA, SCL)
- GPIO 14, 15: UART0 (TX, RX)

**Available for future expansion**:
- GPIO 4, 5: Available for I2C4 or other use
- GPIO 8, 9: Available for I2C4 hardware bus
- GPIO 10-13: Available
- GPIO 16-27: Mostly available
- GPIO 22, 23: Available for I2C6 hardware bus

### Future Expansion Options

If you need to add more I2C devices:

**Option 1: Add devices to existing buses**
- I2C1 can support up to 127 devices (limited by addresses)
- I2C3 can support up to 127 devices (limited by addresses)
- Use devices with different addresses than 0x28/0x29

**Option 2: Add more I2C buses**
```bash
# I2C4 on GPIO 8/9 (hardware)
dtoverlay=i2c4,pins_8_9

# I2C6 on GPIO 22/23 (hardware)
dtoverlay=i2c6,pins_22_23

# Additional software bus on GPIO 22/23
dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=22,i2c_gpio_scl=23
```

---

## Testing and Validation

### Test Plan

After connecting hardware, follow these steps:

#### 1. Hardware Verification
```bash
# Scan I2C1 for sensors 1 and 2
i2cdetect -y 1
# Expected: Devices at 0x28 and 0x29

# Scan I2C3 for sensor 3
i2cdetect -y 3
# Expected: Device at 0x28 or 0x29
```

#### 2. Individual Sensor Test
```bash
cd /home/pi/Desktop/TFM/Tests
python3 test_third_bno055_i2c3.py
```

**Expected output**:
- ✓ I2C3 bus initialized
- ✓ BNO055 found at address
- ✓ Temperature reading displayed
- ✓ Calibration status shown
- ✓ 10 seconds of Euler angle and gyro data

#### 3. All Sensors Test
```bash
python3 read_three_bno055_sensors.py
```

**Expected output**:
- ✓ All three sensors initialize
- ✓ Calibration status for each sensor
- ✓ Continuous data from all sensors
- ✓ Data updates at ~10 Hz

#### 4. Performance Check

Monitor CPU usage while reading sensors:
```bash
# In one terminal
python3 read_three_bno055_sensors.py

# In another terminal
top
```

Expected: CPU usage <10% for continuous reading at 10 Hz

---

## Summary

### What Changed
**Nothing!** The I2C3 bus was already configured in `/boot/firmware/config.txt`.

### What Was Created
- ✓ Test script for third sensor only
- ✓ Complete three-sensor example
- ✓ Comprehensive wiring documentation
- ✓ Quick start guide
- ✓ This configuration reference document

### Why This Works
- Two separate I2C buses allow the same addresses on different buses
- I2C3 (software I2C) provides sufficient performance for BNO055
- No address conflicts between sensors
- No multiplexer hardware required
- Minimal CPU overhead

### Next Steps for User

1. **Connect third BNO055 hardware**:
   - SDA → GPIO 6 (Pin 31)
   - SCL → GPIO 7 (Pin 26)
   - VIN → 3.3V, GND → GND
   - ADR → GND or 3.3V

2. **Verify connection**:
   ```bash
   i2cdetect -y 3
   ```

3. **Run tests**:
   ```bash
   cd /home/pi/Desktop/TFM/Tests
   python3 test_third_bno055_i2c3.py
   python3 read_three_bno055_sensors.py
   ```

4. **Integrate into application**:
   - Use `TripleBNO055` class from example
   - Or adapt existing code to add I2C3 sensor
   - Reference: `/home/pi/Desktop/TFM/Tests/read_three_bno055_sensors.py`

---

## References

### Configuration Files
- `/boot/firmware/config.txt` - Boot configuration (I2C overlays)
- `/dev/i2c-1` - Hardware I2C bus device
- `/dev/i2c-3` - Software I2C bus device

### Documentation Created
- `WIRING_GUIDE_THREE_BNO055.md` - Hardware connections and troubleshooting
- `README_THREE_BNO055.md` - Quick start and commands
- `CONFIGURATION_CHANGES.md` - This file

### Code Created
- `test_third_bno055_i2c3.py` - Third sensor test
- `read_three_bno055_sensors.py` - Three-sensor example

### External Resources
- [Raspberry Pi I2C Documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#i2c-software)
- [Device Tree Overlays](https://www.raspberrypi.com/documentation/computers/configuration.html#device-trees-overlays-and-parameters)
- [BNO055 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- [Adafruit BNO055 Library](https://github.com/adafruit/Adafruit_CircuitPython_BNO055)

---

**Document Status**: Complete
**Last Updated**: 2025-10-30
**Verified**: I2C3 configuration confirmed working
