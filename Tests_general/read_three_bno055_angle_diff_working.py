#!/usr/bin/env python3
"""
Read quaternion data from three BNO055 sensors simultaneously

Hardware Setup:
-------------
Sensor 1 (I2C1, address 0x28):
  - SDA → GPIO 2
  - SCL → GPIO 3
  - ADR → GND
  - VIN → 3.3V (or 5V)
  - GND → GND

Sensor 2 (I2C1, address 0x29):
  - SDA → GPIO 2
  - SCL → GPIO 3
  - ADR → 3.3V
  - VIN → 3.3V (or 5V)
  - GND → GND

Sensor 3 (I2C3, address 0x28):
  - SDA → GPIO 6
  - SCL → GPIO 7
  - ADR → GND
  - VIN → 3.3V (or 5V)
  - GND → GND

Quaternion Data:
- Returns (w, x, y, z) components
- Represents orientation in 3D space
- Unit quaternion (magnitude should be close to 1.0)
"""

import time
import math
import board
import busio
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055


class TripleBNO055Quaternion:
    """Class to manage three BNO055 sensors and read quaternion data."""

    def __init__(self):
        """Initialize all three BNO055 sensors."""
        print("=" * 70)
        print("Initializing Three BNO055 Sensors for Quaternion Reading")
        print("=" * 70)

        # Initialize I2C1 bus (hardware I2C on GPIO 2/3)
        print("\n[1/4] Initializing I2C1 bus (GPIO 2/3)...")
        self.i2c1 = busio.I2C(board.SCL, board.SDA)
        print("✓ I2C1 initialized")

        # Initialize Sensor 1 on I2C1 at 0x28
        print("\n[2/4] Connecting to Sensor 1 (I2C1, address 0x28)...")
        try:
            self.sensor1 = adafruit_bno055.BNO055_I2C(self.i2c1, address=0x28)
            print(f"✓ Sensor 1 connected (Temp: {self.sensor1.temperature}°C)")
        except Exception as e:
            print(f"✗ Failed to connect to Sensor 1: {e}")
            raise

        # Initialize Sensor 2 on I2C1 at 0x29
        print("\n[3/4] Connecting to Sensor 2 (I2C1, address 0x29)...")
        try:
            self.sensor2 = adafruit_bno055.BNO055_I2C(self.i2c1, address=0x29)
            print(f"✓ Sensor 2 connected (Temp: {self.sensor2.temperature}°C)")
        except Exception as e:
            print(f"✗ Failed to connect to Sensor 2: {e}")
            raise

        # Initialize I2C3 bus (software I2C on GPIO 6/7) and Sensor 3
        print("\n[4/4] Initializing I2C3 bus (GPIO 6/7) and Sensor 3...")
        try:
            self.i2c3 = I2C(3)
            print("✓ I2C3 initialized, waiting for bus to stabilize...")
            time.sleep(0.5)  # Give I2C3 time to stabilize
            self.sensor3 = adafruit_bno055.BNO055_I2C(self.i2c3, address=0x28)
            print(f"✓ Sensor 3 connected (I2C3, address 0x28, "
                  f"Temp: {self.sensor3.temperature}°C)")
        except Exception as e:
            print(f"✗ Failed to connect to Sensor 3: {e}")
            raise

        print("\n" + "=" * 70)
        print("All three sensors initialized successfully!")
        print("=" * 70)

    def read_all_quaternions(self):
        """Read quaternion data from all three sensors.

        Returns:
            tuple: (sensor1_quat, sensor2_quat, sensor3_quat)
                   Each quaternion is (w, x, y, z)
        """
        quat1 = self.sensor1.quaternion
        quat2 = self.sensor2.quaternion
        quat3 = self.sensor3.quaternion
        return quat1, quat2, quat3

    def get_calibration_status(self):
        """Get calibration status for all three sensors.

        Returns:
            tuple: (sensor1_status, sensor2_status, sensor3_status)
                   Each status is (sys, gyro, accel, mag)
        """
        status1 = self.sensor1.calibration_status
        status2 = self.sensor2.calibration_status
        status3 = self.sensor3.calibration_status
        return status1, status2, status3


def quaternion_to_yaw(w, x, y, z):
    """
    Extract yaw angle (rotation around Z axis) from quaternion.

    Args:
        w, x, y, z: Quaternion components

    Returns:
        float: Yaw angle in degrees (-180 to 180)
    """
    # Calculate yaw from quaternion
    yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    yaw_deg = math.degrees(yaw_rad)
    return yaw_deg


def normalize_angle_difference(angle):
    """
    Normalize angle difference to range [-180, 180] degrees.

    Args:
        angle: Angle in degrees

    Returns:
        float: Normalized angle in range [-180, 180]
    """
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def main():
    """Main demonstration: Read quaternions from all three sensors continuously."""
    try:
        # Initialize sensors
        sensors = TripleBNO055Quaternion()
        time.sleep(0.5)

        # Display calibration status
        print("\nCalibration Status (sys, gyro, accel, mag):")
        cal1, cal2, cal3 = sensors.get_calibration_status()
        print(f"  Sensor 1: {cal1}")
        print(f"  Sensor 2: {cal2}")
        print(f"  Sensor 3: {cal3}")
        print("\n  Note: Move sensors in a figure-8 pattern to calibrate magnetometer")
        print("        Rotate sensors to calibrate gyroscope and accelerometer")

        # Continuous reading
        print("\n" + "=" * 70)
        print("Reading yaw angle differences (Press Ctrl+C to stop)")
        print("=" * 70)
        print(f"\n{'Time':>8} | {'Sensor3 - Sensor1 (°)':>22} | {'Sensor3 - Sensor2 (°)':>22}")
        print("-" * 60)

        start_time = time.time()
        while True:
            # Read quaternions from all sensors
            quat1, quat2, quat3 = sensors.read_all_quaternions()

            # Extract values (handle None values)
            if quat1[0] is not None:
                w1, x1, y1, z1 = quat1
            else:
                w1 = x1 = y1 = z1 = 0.0

            if quat2[0] is not None:
                w2, x2, y2, z2 = quat2
            else:
                w2 = x2 = y2 = z2 = 0.0

            if quat3[0] is not None:
                w3, x3, y3, z3 = quat3
            else:
                w3 = x3 = y3 = z3 = 0.0

            # Convert quaternions to yaw angles
            yaw1 = quaternion_to_yaw(w1, x1, y1, z1)
            yaw2 = quaternion_to_yaw(w2, x2, y2, z2)
            yaw3 = quaternion_to_yaw(w3, x3, y3, z3)

            # Calculate angle differences
            diff_3_1 = normalize_angle_difference(yaw3 - yaw1)
            diff_3_2 = normalize_angle_difference(yaw3 - yaw2)

            elapsed = time.time() - start_time

            # Display data
            print(f"{elapsed:8.2f} | {diff_3_1:22.2f} | {diff_3_2:22.2f}")

            time.sleep(0.02)  # 50 Hz update rate

    except KeyboardInterrupt:
        print("\n\n" + "=" * 70)
        print("Program stopped by user")
        print("=" * 70)

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
