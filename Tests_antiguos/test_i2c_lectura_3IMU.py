#!/usr/bin/env python3
import busio
import board
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055

print("Checking all three BNO055 sensors...\n")

# Test sensors on I2C1
try:
    i2c1 = busio.I2C(board.SCL, board.SDA)
    
    # Test IMU1 at 0x28
    try:
        imu1 = adafruit_bno055.BNO055_I2C(i2c1, address=0x28)
        temp = imu1.temperature
        print(f"✓ IMU1 (I2C1, 0x28): {temp}°C")
    except:
        print("✗ IMU1 (I2C1, 0x28): Not found")
    
    # Test IMU2 at 0x29
    try:
        imu2 = adafruit_bno055.BNO055_I2C(i2c1, address=0x29)
        temp = imu2.temperature
        print(f"✓ IMU2 (I2C1, 0x29): {temp}°C")
    except:
        print("✗ IMU2 (I2C1, 0x29): Not found")
except:
    print("✗ I2C1 bus error")

# Test sensor on I2C3
try:
    i2c3 = I2C(3)
    
    try:
        imu3 = adafruit_bno055.BNO055_I2C(i2c3, address=0x28)
        temp = imu3.temperature
        print(f"✓ IMU3 (I2C3, 0x28): {temp}°C")
    except:
        print("✗ IMU3 (I2C3, 0x28): Not found")
except:
    print("✗ I2C3 bus error")