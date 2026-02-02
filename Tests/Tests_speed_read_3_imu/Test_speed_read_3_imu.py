#!/usr/bin/env python3
"""
Read linear velocity data from three BNO055 sensors simultaneously

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

Linear Velocity Data:
- Returns (x, y, z) components in m/s
- Represents velocity in 3D space
- Requires sensor fusion mode (NDOF recommended)
"""

import time
import board
import busio
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055


class TripleBNO055Velocity:
    """Class to manage three BNO055 sensors and read linear velocity data."""

    def __init__(self):
        """Initialize all three BNO055 sensors."""
        print("=" * 70)
        print("Initializing Three BNO055 Sensors for Linear Velocity Reading")
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

    def read_all_velocities(self):
        """Read linear velocity data from all three sensors.

        Returns:
            tuple: (sensor1_vel, sensor2_vel, sensor3_vel)
                   Each velocity is (x, y, z) in m/s
        """
        vel1 = self.sensor1.linear_acceleration
        vel2 = self.sensor2.linear_acceleration
        vel3 = self.sensor3.linear_acceleration
        return vel1, vel2, vel3

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


def main():
    """Main demonstration: Read linear velocities from all three sensors continuously."""
    try:
        # Initialize sensors
        sensors = TripleBNO055Velocity()
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
        print("Reading linear velocity data from all three sensors (Press Ctrl+C to stop)")
        print("=" * 70)
        print(f"\n{'Time':>6} | {'Sensor 1 Velocity (x, y, z) [m/s]':^40} | "
              f"{'Sensor 2 Velocity (x, y, z) [m/s]':^40} | "
              f"{'Sensor 3 Velocity (x, y, z) [m/s]':^40}")
        print("-" * 130)

        start_time = time.time()
        while True:
            # Read velocities from all sensors
            vel1, vel2, vel3 = sensors.read_all_velocities()

            # Extract values (handle None values)
            if vel1 is not None and vel1[0] is not None:
                x1, y1, z1 = vel1
            else:
                x1 = y1 = z1 = 0.0

            if vel2 is not None and vel2[0] is not None:
                x2, y2, z2 = vel2
            else:
                x2 = y2 = z2 = 0.0

            if vel3 is not None and vel3[0] is not None:
                x3, y3, z3 = vel3
            else:
                x3 = y3 = z3 = 0.0

            elapsed = time.time() - start_time

            # Display data
            print(f"{elapsed:6.1f} | "
                  f"{x1:8.3f}, {y1:8.3f}, {z1:8.3f} | "
                  f"{x2:8.3f}, {y2:8.3f}, {z2:8.3f} | "
                  f"{x3:8.3f}, {y3:8.3f}, {z3:8.3f}")

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
