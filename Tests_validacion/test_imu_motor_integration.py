#!/usr/bin/env python3
"""
Integration test: Three BNO055 IMU sensors with motor control

Test Objective:
--------------
Monitor angular velocity from three BNO055 sensors and activate motor when
any sensor's angular velocity magnitude exceeds 3 rad/s threshold.

Hardware Setup:
--------------
IMU Sensors:
  Sensor 1 (I2C1, address 0x28): SDA→GPIO2, SCL→GPIO3, ADR→GND
  Sensor 2 (I2C1, address 0x29): SDA→GPIO2, SCL→GPIO3, ADR→3.3V
  Sensor 3 (I2C3, address 0x28): SDA→GPIO6, SCL→GPIO7, ADR→GND

Motor:
  Maxon EPOS4 controller via USB

Test Logic:
----------
1. Read gyroscope data from all three sensors at 50 Hz
2. Calculate angular velocity magnitude: |ω| = √(ωx² + ωy² + ωz²)
3. If any sensor exceeds 3 rad/s threshold:
   - Start motor movement
   - Keep motor running for 1 second from last detection
   - Continue monitoring sensors during motor movement
4. Test duration: 20 seconds total
"""

import time
import math
import board
import busio
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055
from ctypes import *


# ============================================================================
# MOTOR CONFIGURATION
# ============================================================================

# EPOS Command Library path
EPOS_LIB_PATH = '/home/pi/Desktop/Repo/ExosqueletonForWalkingAid/lib/EPOS_Linux_Library/lib/arm/v8/libEposCmd.so.6.8.1.0'

# Load library
cdll.LoadLibrary(EPOS_LIB_PATH)
epos = CDLL(EPOS_LIB_PATH)

# Motor configuration parameters
NODE_ID = 1
BAUDRATE = 1000000
TIMEOUT = 500
ACCELERATION = 30000  # rpm/s
DECELERATION = 30000  # rpm/s
TARGET_VELOCITY = 3000  # rpm

# EPOS return variables
pErrorCode = c_uint()


# ============================================================================
# IMU SENSOR CLASS
# ============================================================================

class TripleBNO055Gyro:
    """Manage three BNO055 sensors for gyroscope reading."""
    
    def __init__(self):
        """Initialize all three BNO055 sensors."""
        print("=" * 70)
        print("Initializing Three BNO055 Sensors")
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
            time.sleep(0.5)
            self.sensor3 = adafruit_bno055.BNO055_I2C(self.i2c3, address=0x28)
            print(f"✓ Sensor 3 connected (I2C3, address 0x28, "
                  f"Temp: {self.sensor3.temperature}°C)")
        except Exception as e:
            print(f"✗ Failed to connect to Sensor 3: {e}")
            raise
        
        print("\n" + "=" * 70)
        print("All three sensors initialized successfully!")
        print("=" * 70)
    
    def read_all_gyro(self):
        """Read gyroscope data from all three sensors.
        
        Returns:
            tuple: (gyro1, gyro2, gyro3)
                   Each gyro is (x, y, z) in rad/s
        """
        gyro1 = self.sensor1.gyro
        gyro2 = self.sensor2.gyro
        gyro3 = self.sensor3.gyro
        return gyro1, gyro2, gyro3


# ============================================================================
# MOTOR CONTROL FUNCTIONS
# ============================================================================

def motor_set_enable_state(keyHandle):
    """Enable motor."""
    epos.VCS_SetEnableState(keyHandle, NODE_ID, byref(pErrorCode))


def motor_set_disable_state(keyHandle):
    """Disable motor."""
    epos.VCS_SetDisableState(keyHandle, NODE_ID, byref(pErrorCode))


def motor_activate_velocity_mode(keyHandle):
    """Activate profile velocity mode."""
    epos.VCS_ActivateProfileVelocityMode(keyHandle, NODE_ID, byref(pErrorCode))


def motor_move_at_velocity(keyHandle, velocity, acceleration, deceleration):
    """Start motor movement at specified velocity."""
    epos.VCS_SetVelocityProfile(keyHandle, NODE_ID, acceleration, deceleration, byref(pErrorCode))
    epos.VCS_MoveWithVelocity(keyHandle, NODE_ID, velocity, byref(pErrorCode))


def motor_halt(keyHandle):
    """Stop motor movement."""
    epos.VCS_HaltVelocityMovement(keyHandle, NODE_ID, byref(pErrorCode))


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def calculate_angular_velocity_magnitude(gyro_data):
    """Calculate magnitude of angular velocity vector.
    
    Args:
        gyro_data: tuple (x, y, z) in rad/s
        
    Returns:
        float: magnitude |ω| = √(ωx² + ωy² + ωz²) in rad/s
    """
    if gyro_data is None or gyro_data[0] is None:
        return 0.0
    
    gx, gy, gz = gyro_data
    magnitude = math.sqrt(gx**2 + gy**2 + gz**2)
    return magnitude


def check_threshold_exceeded(gyro1, gyro2, gyro3, threshold=3.0):
    """Check if any sensor exceeds angular velocity threshold.
    
    Args:
        gyro1, gyro2, gyro3: gyroscope data tuples (x, y, z)
        threshold: threshold in rad/s (default: 3.0)
        
    Returns:
        tuple: (exceeded, max_magnitude, sensor_id)
    """
    mag1 = calculate_angular_velocity_magnitude(gyro1)
    mag2 = calculate_angular_velocity_magnitude(gyro2)
    mag3 = calculate_angular_velocity_magnitude(gyro3)
    
    max_mag = max(mag1, mag2, mag3)
    
    if max_mag >= threshold:
        if max_mag == mag1:
            sensor_id = 1
        elif max_mag == mag2:
            sensor_id = 2
        else:
            sensor_id = 3
        return True, max_mag, sensor_id
    
    return False, max_mag, 0


# ============================================================================
# MAIN TEST
# ============================================================================

def main():
    """Main test: Monitor IMUs and control motor based on angular velocity."""
    
    # Initialize motor connection
    print("\n" + "=" * 70)
    print("Initializing Motor Controller")
    print("=" * 70)
    
    keyHandle = epos.VCS_OpenDevice(
        b'EPOS4', b'MAXON SERIAL V2', b'USB', b'USB0', byref(pErrorCode)
    )
    epos.VCS_SetProtocolStackSettings(keyHandle, BAUDRATE, TIMEOUT, byref(pErrorCode))
    epos.VCS_ClearFault(keyHandle, NODE_ID, byref(pErrorCode))
    motor_activate_velocity_mode(keyHandle)
    motor_set_enable_state(keyHandle)
    print("✓ Motor controller initialized and enabled")
    
    # Initialize IMU sensors
    sensors = TripleBNO055Gyro()
    time.sleep(0.5)
    
    # Test parameters
    THRESHOLD = 3.0  # rad/s
    TEST_DURATION = 20  # seconds
    MOTOR_ACTIVE_DURATION = 1.0  # seconds
    LOOP_RATE = 50  # Hz
    
    print("\n" + "=" * 70)
    print(f"Starting Test (Duration: {TEST_DURATION}s)")
    print(f"Angular Velocity Threshold: {THRESHOLD} rad/s")
    print(f"Motor Active Duration: {MOTOR_ACTIVE_DURATION}s")
    print("=" * 70)
    print(f"\n{'Time':>6} | {'Motor':^8} | {'Max |ω|':>10} | {'Sensor':^8} | Status")
    print("-" * 70)
    
    start_time = time.time()
    motor_active = False
    motor_stop_time = 0
    
    try:
        while time.time() - start_time < TEST_DURATION:
            loop_start = time.time()
            
            # Read gyroscope data from all sensors
            gyro1, gyro2, gyro3 = sensors.read_all_gyro()
            
            # Check if threshold is exceeded
            exceeded, max_magnitude, sensor_id = check_threshold_exceeded(
                gyro1, gyro2, gyro3, THRESHOLD
            )
            
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Motor control logic
            if exceeded:
                # Threshold exceeded - start/extend motor movement
                if not motor_active:
                    motor_move_at_velocity(keyHandle, TARGET_VELOCITY, ACCELERATION, DECELERATION)
                    motor_active = True
                    status = "MOTOR STARTED"
                else:
                    status = "MOTOR EXTENDED"
                
                # Update motor stop time (1 second from now)
                motor_stop_time = current_time + MOTOR_ACTIVE_DURATION
                
                motor_state = "RUNNING"
            else:
                # Threshold not exceeded - check if motor should stop
                if motor_active and current_time >= motor_stop_time:
                    motor_halt(keyHandle)
                    motor_active = False
                    status = "MOTOR STOPPED"
                    motor_state = "IDLE"
                elif motor_active:
                    status = "MONITORING"
                    motor_state = "RUNNING"
                else:
                    status = "MONITORING"
                    motor_state = "IDLE"
            
            # Display status
            sensor_str = f"S{sensor_id}" if exceeded else "---"
            print(f"{elapsed:6.2f} | {motor_state:^8} | {max_magnitude:10.3f} | "
                  f"{sensor_str:^8} | {status}")
            
            # Maintain loop rate
            loop_duration = time.time() - loop_start
            sleep_time = max(0, (1.0 / LOOP_RATE) - loop_duration)
            time.sleep(sleep_time)
        
        # Test complete - stop motor if still running
        if motor_active:
            motor_halt(keyHandle)
            print("\n✓ Motor stopped (test complete)")
        
        print("\n" + "=" * 70)
        print("Test Complete")
        print("=" * 70)
    
    except KeyboardInterrupt:
        print("\n\n" + "=" * 70)
        print("Test interrupted by user")
        print("=" * 70)
        if motor_active:
            motor_halt(keyHandle)
            print("✓ Motor stopped")
    
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        motor_halt(keyHandle)
        motor_set_disable_state(keyHandle)
    
    finally:
        # Cleanup
        motor_set_disable_state(keyHandle)
        epos.VCS_CloseDevice(keyHandle, byref(pErrorCode))
        print("✓ Motor controller disabled and connection closed")


if __name__ == "__main__":
    main()
