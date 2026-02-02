#!/usr/bin/env python3
"""
Read acceleration and gyroscope data from three BNO055 sensors simultaneously

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

Sensor Data:
- Acceleration: (x, y, z) components in m/s²
- Gyroscope: (x, y, z) components in rad/s (angular velocity)
- Both measurements are in the sensor's local coordinate frame
"""

import time
import board
import busio
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055


class TripleBNO055AccelGyro:
    """Class to manage three BNO055 sensors and read acceleration and gyroscope data."""

    def __init__(self):
        """Initialize all three BNO055 sensors."""
        print("=" * 70)
        print("Initializing Three BNO055 Sensors for Acceleration and Gyroscope Reading")
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

    def read_all_data(self):
        """Read acceleration and gyroscope data from all three sensors.

        Returns:
            tuple: ((accel1, gyro1), (accel2, gyro2), (accel3, gyro3))
                   Each accel is (x, y, z) in m/s²
                   Each gyro is (x, y, z) in rad/s
        """
        accel1 = self.sensor1.acceleration
        gyro1 = self.sensor1.gyro
        
        accel2 = self.sensor2.acceleration
        gyro2 = self.sensor2.gyro
        
        accel3 = self.sensor3.acceleration
        gyro3 = self.sensor3.gyro
        
        return (accel1, gyro1), (accel2, gyro2), (accel3, gyro3)

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
    """Main demonstration: Read acceleration and gyroscope data from all three sensors continuously."""
    try:
        # Initialize sensors
        sensors = TripleBNO055AccelGyro()
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
        print("Reading acceleration and gyroscope data from all three sensors")
        print("Press Ctrl+C to stop")
        print("=" * 70)
        print(f"\n{'Time':>6} | Sensor | {'Acceleration (x, y, z) [m/s²]':^35} | {'Gyroscope (x, y, z) [rad/s]':^35}")
        print("-" * 100)

        start_time = time.time()
        while True:
            # Read acceleration and gyro from all sensors
            data1, data2, data3 = sensors.read_all_data()
            accel1, gyro1 = data1
            accel2, gyro2 = data2
            accel3, gyro3 = data3

            elapsed = time.time() - start_time

            # Extract and display Sensor 1
            if accel1 is not None and accel1[0] is not None:
                ax1, ay1, az1 = accel1
            else:
                ax1 = ay1 = az1 = 0.0
            
            if gyro1 is not None and gyro1[0] is not None:
                gx1, gy1, gz1 = gyro1
            else:
                gx1 = gy1 = gz1 = 0.0
            
            print(f"{elapsed:6.1f} | S1     | "
                  f"{ax1:8.3f}, {ay1:8.3f}, {az1:8.3f} | "
                  f"{gx1:8.3f}, {gy1:8.3f}, {gz1:8.3f}")

            # Extract and display Sensor 2
            if accel2 is not None and accel2[0] is not None:
                ax2, ay2, az2 = accel2
            else:
                ax2 = ay2 = az2 = 0.0
            
            if gyro2 is not None and gyro2[0] is not None:
                gx2, gy2, gz2 = gyro2
            else:
                gx2 = gy2 = gz2 = 0.0
            
            print(f"       | S2     | "
                  f"{ax2:8.3f}, {ay2:8.3f}, {az2:8.3f} | "
                  f"{gx2:8.3f}, {gy2:8.3f}, {gz2:8.3f}")

            # Extract and display Sensor 3
            if accel3 is not None and accel3[0] is not None:
                ax3, ay3, az3 = accel3
            else:
                ax3 = ay3 = az3 = 0.0
            
            if gyro3 is not None and gyro3[0] is not None:
                gx3, gy3, gz3 = gyro3
            else:
                gx3 = gy3 = gz3 = 0.0
            
            print(f"       | S3     | "
                  f"{ax3:8.3f}, {ay3:8.3f}, {az3:8.3f} | "
                  f"{gx3:8.3f}, {gy3:8.3f}, {gz3:8.3f}")
            print("-" * 100)

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
