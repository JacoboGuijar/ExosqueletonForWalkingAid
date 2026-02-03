#!/usr/bin/env python3
"""
Test script for third BNO055 sensor on I2C3 bus (GPIO 6/7)

This script tests only the third BNO055 sensor connected to the
software I2C3 bus to verify it's working correctly.

Hardware Setup:
- BNO055 SDA → GPIO 6
- BNO055 SCL → GPIO 7
- BNO055 VIN → 3.3V (or 5V)
- BNO055 GND → GND
- BNO055 ADR → GND (for address 0x28) or 3.3V (for address 0x29)

Author: Claude Code
Date: 2025-10-30
"""

import time
import board
import busio
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055

def test_third_sensor():
    """Test the third BNO055 sensor on I2C3 bus."""

    print("=" * 60)
    print("Testing Third BNO055 Sensor on I2C3 (GPIO 6/7)")
    print("=" * 60)

    try:
        # Initialize I2C3 bus
        print("\n[1/3] Initializing I2C3 bus...")
        i2c3 = I2C(3)  # Bus number 3
        print("✓ I2C3 bus initialized successfully")

        # Try address 0x28 first (default, ADR pin to GND)
        print("\n[2/3] Connecting to BNO055 at address 0x28...")
        try:
            sensor = adafruit_bno055.BNO055_I2C(i2c3, address=0x28)
            address = "0x28"
            print(f"✓ BNO055 found at address {address}")
        except Exception as e:
            print(f"  Not found at 0x28, trying 0x29...")
            try:
                sensor = adafruit_bno055.BNO055_I2C(i2c3, address=0x29)
                address = "0x29"
                print(f"✓ BNO055 found at address {address}")
            except Exception as e2:
                print(f"✗ Error: Could not find BNO055 on I2C3")
                print(f"  Make sure the sensor is connected to GPIO 6 (SDA) and GPIO 7 (SCL)")
                return False

        # Read sensor data
        print(f"\n[3/3] Reading sensor data...")
        print("\nSensor Information:")
        print(f"  - Temperature: {sensor.temperature}°C")

        # System status
        status = sensor.calibration_status
        print(f"\nCalibration Status (sys, gyro, accel, mag): {status}")

        print("\n" + "-" * 60)
        print("Reading sensor data for 10 seconds...")
        print("(Press Ctrl+C to stop)")
        print("-" * 60)
        print(f"{'Time':>6} | {'Euler (°)':^30} | {'Gyro (°/s)':^30}")
        print(f"{'(sec)':>6} | {'Yaw':>8} {'Pitch':>9} {'Roll':>9} | {'X':>8} {'Y':>9} {'Z':>9}")
        print("-" * 60)

        start_time = time.time()
        while time.time() - start_time < 10:
            # Read Euler angles
            euler = sensor.euler
            if euler[0] is not None:
                yaw, pitch, roll = euler
            else:
                yaw = pitch = roll = 0.0

            # Read gyroscope
            gyro = sensor.gyro
            if gyro[0] is not None:
                gyro_x, gyro_y, gyro_z = gyro
            else:
                gyro_x = gyro_y = gyro_z = 0.0

            elapsed = time.time() - start_time
            print(f"{elapsed:6.1f} | {yaw:8.2f} {pitch:9.2f} {roll:9.2f} | "
                  f"{gyro_x:8.2f} {gyro_y:9.2f} {gyro_z:9.2f}")

            time.sleep(0.1)  # 10 Hz update rate

        print("-" * 60)
        print("\n✓ Test completed successfully!")
        print(f"  Third BNO055 sensor on I2C3 is working correctly at address {address}")
        return True

    except KeyboardInterrupt:
        print("\n\n✓ Test stopped by user")
        return True

    except Exception as e:
        print(f"\n✗ Error during test: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_third_sensor()
    exit(0 if success else 1)
