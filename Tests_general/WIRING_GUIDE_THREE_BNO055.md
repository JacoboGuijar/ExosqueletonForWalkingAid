# Wiring Guide: Three BNO055 Sensors on Raspberry Pi 5

This guide shows how to connect three BNO055 IMU sensors to a Raspberry Pi 5 using two I2C buses (I2C1 and I2C3) to avoid address conflicts.

## Overview

- **Sensor 1**: I2C1 bus at address 0x28 (GPIO 2/3)
- **Sensor 2**: I2C1 bus at address 0x29 (GPIO 2/3)
- **Sensor 3**: I2C3 bus at address 0x28 or 0x29 (GPIO 6/7)

By using two separate I2C buses, we can connect three sensors even though BNO055 only supports two I2C addresses (0x28 and 0x29).

## Required Hardware

- Raspberry Pi 5
- 3x Adafruit BNO055 9-DOF IMU sensors
- Jumper wires
- Breadboard (optional)
- 3.3V or 5V power supply

## Pin Connections

### Sensor 1 (I2C1, Address 0x28)

| BNO055 Pin | Connect To | Pi Pin | Notes |
|------------|------------|--------|-------|
| VIN | 3.3V or 5V | Pin 1 or Pin 2 | 3.3V (Pin 1) or 5V (Pin 2) |
| GND | Ground | Pin 6, 9, 14, 20, 25, 30, 34, or 39 | Any ground pin |
| SDA | GPIO 2 | Pin 3 | I2C1 SDA (hardware I2C) |
| SCL | GPIO 3 | Pin 5 | I2C1 SCL (hardware I2C) |
| ADR | Ground | GND | Sets address to 0x28 |
| RST | Not connected | - | Optional reset pin |

### Sensor 2 (I2C1, Address 0x29)

| BNO055 Pin | Connect To | Pi Pin | Notes |
|------------|------------|--------|-------|
| VIN | 3.3V or 5V | Pin 1 or Pin 2 | 3.3V (Pin 1) or 5V (Pin 2) |
| GND | Ground | Pin 6, 9, 14, 20, 25, 30, 34, or 39 | Any ground pin |
| SDA | GPIO 2 | Pin 3 | I2C1 SDA (same bus as Sensor 1) |
| SCL | GPIO 3 | Pin 5 | I2C1 SCL (same bus as Sensor 1) |
| ADR | 3.3V | Pin 1 | Sets address to 0x29 |
| RST | Not connected | - | Optional reset pin |

### Sensor 3 (I2C3, Address 0x28 or 0x29)

| BNO055 Pin | Connect To | Pi Pin | Notes |
|------------|------------|--------|-------|
| VIN | 3.3V or 5V | Pin 1 or Pin 2 | 3.3V (Pin 1) or 5V (Pin 2) |
| GND | Ground | Pin 6, 9, 14, 20, 25, 30, 34, or 39 | Any ground pin |
| SDA | GPIO 6 | Pin 31 | I2C3 SDA (software I2C) |
| SCL | GPIO 7 | Pin 26 | I2C3 SCL (software I2C) |
| ADR | GND or 3.3V | GND or 3.3V | 0x28 (GND) or 0x29 (3.3V) |
| RST | Not connected | - | Optional reset pin |

## Raspberry Pi 5 GPIO Header Pinout Reference

```
     3.3V  [1]  [2]  5V
GPIO2/SDA1 [3]  [4]  5V
GPIO3/SCL1 [5]  [6]  GND
   GPIO 4  [7]  [8]  GPIO 14/TX
      GND  [9]  [10] GPIO 15/RX
   GPIO 17 [11] [12] GPIO 18
   GPIO 27 [13] [14] GND
   GPIO 22 [15] [16] GPIO 23
     3.3V [17] [18] GPIO 24
   GPIO 10 [19] [20] GND
   GPIO 9  [21] [22] GPIO 25
   GPIO 11 [23] [24] GPIO 8
      GND [25] [26] GPIO 7    ← I2C3 SCL (Sensor 3)
   GPIO 0  [27] [28] GPIO 1
   GPIO 5  [29] [30] GND
   GPIO 6  [31] [32] GPIO 12  ← I2C3 SDA (Sensor 3)
   GPIO 13 [33] [34] GND
   GPIO 19 [35] [36] GPIO 16
   GPIO 26 [37] [38] GPIO 20
      GND [39] [40] GPIO 21
```

## Visual Wiring Diagram

```
Raspberry Pi 5                    BNO055 Sensor 1 (Address 0x28)
┌─────────────────┐              ┌──────────────────┐
│  Pin 1 (3.3V)   │──────────────│ VIN              │
│  Pin 3 (GPIO2)  │──────────────│ SDA              │
│  Pin 5 (GPIO3)  │──────────────│ SCL              │
│  Pin 6 (GND)    │──────────────│ GND              │
│                 │         ┌────│ ADR              │
└─────────────────┘         │    └──────────────────┘
                           GND

Raspberry Pi 5                    BNO055 Sensor 2 (Address 0x29)
┌─────────────────┐              ┌──────────────────┐
│  Pin 1 (3.3V)   │──────────────│ VIN              │
│  Pin 3 (GPIO2)  │──────────────│ SDA (shared)     │
│  Pin 5 (GPIO3)  │──────────────│ SCL (shared)     │
│  Pin 6 (GND)    │──────────────│ GND              │
│  Pin 1 (3.3V)   │──────────────│ ADR              │
└─────────────────┘              └──────────────────┘

Raspberry Pi 5                    BNO055 Sensor 3 (Address 0x28)
┌─────────────────┐              ┌──────────────────┐
│  Pin 1 (3.3V)   │──────────────│ VIN              │
│  Pin 31 (GPIO6) │──────────────│ SDA              │
│  Pin 26 (GPIO7) │──────────────│ SCL              │
│  Pin 6 (GND)    │──────────────│ GND              │
│                 │         ┌────│ ADR              │
└─────────────────┘         │    └──────────────────┘
                           GND
```

## Configuration Check

### Verify I2C Buses are Enabled

The I2C3 bus should already be configured in `/boot/firmware/config.txt`:

```bash
# Check configuration
grep "i2c" /boot/firmware/config.txt
```

You should see:
```
dtparam=i2c_arm=on
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=6,i2c_gpio_scl=7
```

### Verify I2C Buses are Available

```bash
# List all I2C buses
i2cdetect -l
```

Expected output:
```
i2c-1   i2c     Synopsys DesignWare I2C adapter    I2C adapter
i2c-3   i2c     300000002.i2c                       I2C adapter
```

### Scan for Connected Sensors

```bash
# Scan I2C1 bus (should show sensors at 0x28 and 0x29)
i2cdetect -y 1

# Scan I2C3 bus (should show sensor at 0x28 or 0x29)
i2cdetect -y 3
```

Expected output for I2C1:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- 28 29 -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

Expected output for I2C3:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --  (or 29)
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

## Testing the Setup

### Test Individual Sensor (Sensor 3 only)

```bash
cd /home/pi/Desktop/TFM/Tests
python3 test_third_bno055_i2c3.py
```

This will:
1. Connect to the third BNO055 on I2C3
2. Display sensor temperature and calibration status
3. Read Euler angles and gyroscope data for 10 seconds

### Test All Three Sensors

```bash
cd /home/pi/Desktop/TFM/Tests
python3 read_three_bno055_sensors.py
```

This will:
1. Initialize all three sensors
2. Display calibration status
3. Continuously read and display Euler angles from all sensors

## Troubleshooting

### Problem: I2C3 bus not found

**Solution:**
1. Check `/boot/firmware/config.txt` for the I2C3 overlay:
   ```bash
   grep "i2c-gpio" /boot/firmware/config.txt
   ```
2. If missing, add this line:
   ```
   dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=6,i2c_gpio_scl=7
   ```
3. Reboot:
   ```bash
   sudo reboot
   ```

### Problem: Sensor not detected on I2C scan

**Possible causes:**
1. **Loose connections** - Check all wire connections
2. **Wrong pins** - Verify GPIO pin numbers (physical pin vs GPIO number)
3. **Power issues** - Ensure sensors have stable 3.3V or 5V power
4. **Ground connection** - All sensors must share common ground with Pi
5. **ADR pin** - Check if ADR is properly connected to GND or 3.3V

**Debug steps:**
```bash
# Check if GPIO pins are in correct mode
gpio readall

# Check kernel messages for I2C errors
dmesg | grep i2c

# Verify permissions
ls -l /dev/i2c-*
```

### Problem: "Remote I/O error" when reading sensor

**Possible causes:**
1. **Address conflict** - Two sensors on same bus with same address
2. **Bus speed too high** - I2C3 (software) may need slower speed
3. **Pull-up resistors** - BNO055 has internal pull-ups, external may cause issues
4. **Wire length** - Keep I2C wires short (< 20cm recommended)

**Solutions:**
- Verify ADR pin connections (one sensor must have ADR to GND, other to 3.3V on same bus)
- Try adding small delay between sensor reads
- Check for loose connections

### Problem: Sensor readings are unstable or incorrect

**Possible causes:**
1. **Sensor not calibrated** - BNO055 needs calibration for accurate readings
2. **Magnetic interference** - Keep away from motors, magnets, metal objects
3. **Power supply noise** - Use decoupling capacitors near sensors
4. **Software I2C speed** - I2C3 is slower than hardware I2C

**Solutions:**
- Calibrate sensors by moving them in figure-8 pattern (magnetometer)
- Rotate sensors on all axes (gyroscope and accelerometer)
- Check calibration status: `sensor.calibration_status`
- Move sensors away from interference sources

### Problem: "Permission denied" accessing /dev/i2c-*

**Solution:**
```bash
# Add user to i2c group
sudo usermod -a -G i2c $USER

# Log out and back in, or reboot
sudo reboot
```

## Important Notes

1. **Power Supply**: All three sensors can draw significant current when active. Use a good quality power supply (3A+ recommended for Pi 5).

2. **I2C Bus Speed**:
   - I2C1 (hardware) can run at 400 kHz (default 100 kHz)
   - I2C3 (software) runs at ~100 kHz maximum
   - Both speeds are suitable for BNO055

3. **Wire Length**: Keep I2C wires as short as possible. Long wires can cause communication errors.

4. **Sensor Orientation**: Mount all sensors with consistent orientation for easier data interpretation.

5. **Calibration**: BNO055 sensors need calibration for accurate readings. Move sensors in figure-8 pattern to calibrate magnetometer.

6. **Update Rate**: BNO055 can provide data at 100 Hz. The example code uses 10 Hz for readable output.

## Pin Summary Table

| Sensor | Bus | Address | SDA Pin | SCL Pin | ADR Connection |
|--------|-----|---------|---------|---------|----------------|
| 1 | I2C1 | 0x28 | GPIO 2 (Pin 3) | GPIO 3 (Pin 5) | GND |
| 2 | I2C1 | 0x29 | GPIO 2 (Pin 3) | GPIO 3 (Pin 5) | 3.3V |
| 3 | I2C3 | 0x28 or 0x29 | GPIO 6 (Pin 31) | GPIO 7 (Pin 26) | GND or 3.3V |

## Additional Resources

- [BNO055 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- [Adafruit BNO055 Guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)
- [Raspberry Pi 5 Pinout](https://pinout.xyz/)
- [I2C on Raspberry Pi](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#i2c-software)

## Author

Created by Claude Code on 2025-10-30
