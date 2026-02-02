# Three BNO055 Sensors Setup - Quick Start Guide

This directory contains everything you need to connect and test three BNO055 IMU sensors on your Raspberry Pi 5 without using multiplexers.

## Solution: Two I2C Buses

The BNO055 only supports two I2C addresses (0x28 and 0x29), so we use two separate I2C buses:
- **I2C1** (GPIO 2/3): Sensors 1 and 2 at addresses 0x28 and 0x29
- **I2C3** (GPIO 6/7): Sensor 3 at address 0x28 or 0x29

## Quick Start

### 1. Verify I2C3 is Enabled

```bash
i2cdetect -l
```

You should see both `i2c-1` and `i2c-3` listed. If not, check `/boot/firmware/config.txt`.

### 2. Connect the Hardware

See `WIRING_GUIDE_THREE_BNO055.md` for detailed wiring instructions.

**Quick reference:**
- **Sensor 1**: I2C1 (GPIO 2/3), ADR → GND (address 0x28)
- **Sensor 2**: I2C1 (GPIO 2/3), ADR → 3.3V (address 0x29)
- **Sensor 3**: I2C3 (GPIO 6/7), ADR → GND or 3.3V (address 0x28 or 0x29)

### 3. Scan for Sensors

```bash
# Check I2C1 (should show 0x28 and 0x29)
i2cdetect -y 1

# Check I2C3 (should show 0x28 or 0x29)
i2cdetect -y 3
```

### 4. Test Third Sensor Only

```bash
cd /home/pi/Desktop/TFM/Tests
python3 test_third_bno055_i2c3.py
```

This tests only the third sensor on I2C3 for 10 seconds.

### 5. Test All Three Sensors

```bash
cd /home/pi/Desktop/TFM/Tests
python3 read_three_bno055_sensors.py
```

This reads all three sensors simultaneously and displays Euler angles.

## Files in This Directory

| File | Description |
|------|-------------|
| `test_third_bno055_i2c3.py` | Test script for third sensor only |
| `read_three_bno055_sensors.py` | Complete example reading all 3 sensors |
| `WIRING_GUIDE_THREE_BNO055.md` | Detailed wiring instructions and troubleshooting |
| `README_THREE_BNO055.md` | This file - quick start guide |

## How It Works

```python
# Import required libraries
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055
import busio
import board

# Initialize I2C1 (hardware I2C on GPIO 2/3)
i2c1 = busio.I2C(board.SCL, board.SDA)
sensor1 = adafruit_bno055.BNO055_I2C(i2c1, address=0x28)
sensor2 = adafruit_bno055.BNO055_I2C(i2c1, address=0x29)

# Initialize I2C3 (software I2C on GPIO 6/7)
i2c3 = I2C(3)  # Bus number 3
sensor3 = adafruit_bno055.BNO055_I2C(i2c3, address=0x28)

# Read data from all sensors
euler1 = sensor1.euler  # Returns (yaw, pitch, roll)
euler2 = sensor2.euler
euler3 = sensor3.euler
```

## Pin Connections Summary

### Sensor 1 (I2C1, 0x28)
- VIN → 3.3V (Pin 1)
- GND → GND (Pin 6)
- SDA → GPIO 2 (Pin 3)
- SCL → GPIO 3 (Pin 5)
- ADR → GND

### Sensor 2 (I2C1, 0x29)
- VIN → 3.3V (Pin 1)
- GND → GND (Pin 6)
- SDA → GPIO 2 (Pin 3) - shared with Sensor 1
- SCL → GPIO 3 (Pin 5) - shared with Sensor 1
- ADR → 3.3V

### Sensor 3 (I2C3, 0x28 or 0x29)
- VIN → 3.3V (Pin 1)
- GND → GND (Pin 6)
- SDA → GPIO 6 (Pin 31)
- SCL → GPIO 7 (Pin 26)
- ADR → GND (for 0x28) or 3.3V (for 0x29)

## Common Issues

### "No such file or directory: '/dev/i2c-3'"
- I2C3 not enabled in `/boot/firmware/config.txt`
- Check for line: `dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=6,i2c_gpio_scl=7`
- Reboot if you added the line

### Sensor not detected in i2cdetect scan
- Check wire connections (especially SDA/SCL)
- Verify power connections (VIN and GND)
- Check ADR pin connection
- Use multimeter to verify 3.3V is present at sensor VIN

### "Remote I/O error"
- Check for address conflicts (two sensors on same bus with same ADR setting)
- Verify sensor is powered on
- Try shorter wires
- Check for loose connections

### Unstable readings
- Calibrate sensors by moving them in figure-8 pattern
- Check `sensor.calibration_status` - should be (3, 3, 3, 3) when fully calibrated
- Keep sensors away from magnetic interference (motors, magnets)

## Calibration Tips

BNO055 sensors need calibration for accurate readings:

1. **Magnetometer** (most important): Move sensor in figure-8 pattern in all three axes
2. **Gyroscope**: Rotate sensor on all axes, then leave still
3. **Accelerometer**: Orient sensor in 6 positions (each face down), hold for 2-3 seconds

Check calibration status:
```python
status = sensor.calibration_status  # Returns (sys, gyro, accel, mag)
# Each value: 0 (uncalibrated) to 3 (fully calibrated)
```

## Performance Notes

- **I2C1** (hardware): Up to 400 kHz, low CPU usage
- **I2C3** (software): ~100 kHz, higher CPU usage (but still sufficient for BNO055)
- Both buses can read at 100 Hz (BNO055 maximum update rate)
- Example code uses 10 Hz for readable output

## Next Steps

Once you verify all three sensors are working:

1. Integrate into your main application
2. Add sensor fusion for combined orientation data
3. Implement calibration save/restore for faster startup
4. Add data logging or visualization

## Alternative Solutions (Not Used)

We considered but didn't use:
- **I2C multiplexer** (user specifically asked not to use)
- **UART mode** (requires hardware reconfiguration of BNO055)
- **SPI mode** (not supported by Adafruit library)
- **Additional hardware I2C buses** (I2C4, I2C6) - available if you need hardware I2C performance for third sensor

## Support

For detailed wiring diagrams and troubleshooting, see:
- `WIRING_GUIDE_THREE_BNO055.md` - Complete wiring guide
- [Adafruit BNO055 Guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)
- [Raspberry Pi I2C Documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#i2c-software)

---

**Author**: Claude Code
**Date**: 2025-10-30
**Tested on**: Raspberry Pi 5 with Adafruit BNO055 sensors
