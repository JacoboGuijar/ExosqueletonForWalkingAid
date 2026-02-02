import time
import board
import busio
import adafruit_bno055
import sys
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
 
# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)
 
# Try to initialize the sensors with different addresses
try:
    # Sensor 1 at default address 0x28
    imu1 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    # Sensor 2 at alternate address 0x29 (requires ADR pin to be connected to 3.3V)
    imu2 = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
    print("Both IMUs initialized successfully")
except Exception as e:
    print(f"Error initializing IMUs: {e}")
    sys.exit(1)
 
# Previously defined calibration and data reading functions here...
# [Include all the previously defined functions from the earlier code]
# Try to initialize the sensors with different addresses
try:
    # Sensor 1 at default address 0x28
    imu1 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    # Sensor 2 at alternate address 0x29 (requires ADR pin to be connected to 3.3V)
    imu2 = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
    print("Both IMUs initialized successfully")
except Exception as e:
    print(f"Error initializing IMUs: {e}")
    sys.exit(1)
 
def calibrate_imu(imu, sensor_name="IMU"):
    """
    Guide the user through calibrating the BNO055 sensor.
    Returns True when fully calibrated or False if aborted.
    """
    print(f"\n=== Starting {sensor_name} Calibration ===")
    print("Calibration status: 0 = uncalibrated, 3 = fully calibrated")
    print("\nCalibration Instructions:")
    print("  System: Will calibrate as other components are calibrated")
    print("  Gyroscope: Keep the sensor stationary on a flat surface")
    print("  Accelerometer: Move the sensor in different orientations:")
    print("    - Place on all six sides (top, bottom, left, right, front, back)")
    print("    - Hold in each position for a few seconds")
    print("  Magnetometer: Move in a figure-8 pattern in the air")
    print("\nPress Ctrl+C at any time to abort calibration")
 
    # Start the calibration process
    try:
        calibrated = False
        start_time = time.time()
        timeout = 120  # 2 minutes timeout
 
        while not calibrated and (time.time() - start_time) < timeout:
            # Get current calibration status
            sys_cal, gyro_cal, accel_cal, mag_cal = imu.calibration_status
 
            # Display current calibration status
            print(f"\r  System: {sys_cal}/3 | Gyro: {gyro_cal}/3 | Accel: {accel_cal}/3 | Mag: {mag_cal}/3", end="")
 
            # Check if all components are fully calibrated
            if sys_cal == 3 and gyro_cal == 3 and accel_cal == 3 and mag_cal == 3:
                calibrated = True
                print("\n\nCalibration complete!")
 
                # Get and display calibration offsets and radius
                # Note: These values could be saved for future use
                try:
                    # The following registers store calibration data
                    # These specific register addresses and operations are based on
                    # the BNO055 datasheet and Adafruit's implementation
                    offsets = {
                        "accel": imu._read_offset_registers(0x55, 0x56, 0x57),  # Accel offset registers
                        "mag": imu._read_offset_registers(0x5B, 0x5C, 0x5D),    # Mag offset registers
                        "gyro": imu._read_offset_registers(0x61, 0x62, 0x63),   # Gyro offset registers
                        "radius": {
                            "accel": imu._read_radius_registers(0x67, 0x68),    # Accel radius registers
                            "mag": imu._read_radius_registers(0x69, 0x6A)       # Mag radius registers
                        }
                    }
                    print("\nCalibration values (for reference):")
                    print(f"  Accel offsets: {offsets['accel']}")
                    print(f"  Mag offsets: {offsets['mag']}")
                    print(f"  Gyro offsets: {offsets['gyro']}")
                    print(f"  Accel radius: {offsets['radius']['accel']}")
                    print(f"  Mag radius: {offsets['radius']['mag']}")
                except Exception as e:
                    print(f"\nCouldn't read calibration values: {e}")
 
                return True
 
            time.sleep(0.5)
 
        if not calibrated:
            print("\n\nCalibration timed out after 2 minutes.")
            return False
 
    except KeyboardInterrupt:
        print("\n\nCalibration aborted.")
        return False
 
def read_imu_data(imu, sensor_name="IMU"):
    """Read and return all relevant data from a BNO055 sensor."""
    try:
        # Temperature in degrees Celsius
        temp_c = imu.temperature
 
        # Acceleration in m/s^2
        accel_x, accel_y, accel_z = imu.acceleration
 
        # Gyroscope data in rad/s
        gyro_x, gyro_y, gyro_z = imu.gyro
 
        # Magnetometer data in microteslas
        mag_x, mag_y, mag_z = imu.magnetic
 
        # Euler angles in degrees
        euler_x, euler_y, euler_z = imu.euler
 
        # Quaternion
        quat_w, quat_x, quat_y, quat_z = imu.quaternion
 
        # Linear acceleration (acceleration without gravity) in m/s^2
        lin_accel_x, lin_accel_y, lin_accel_z = imu.linear_acceleration
 
        # Gravity vector in m/s^2
        grav_x, grav_y, grav_z = imu.gravity
 
        # Calibration status as tuple with (system, gyro, accel, mag) 
        # where 0 = uncalibrated and 3 = fully calibrated
        sys_cal, gyro_cal, accel_cal, mag_cal = imu.calibration_status
 
        data = {
            "sensor": sensor_name,
            "temperature": temp_c,
            "acceleration": (accel_x, accel_y, accel_z),
            "gyroscope": (gyro_x, gyro_y, gyro_z),
            "magnetometer": (mag_x, mag_y, mag_z),
            "euler_angles": (euler_x, euler_y, euler_z),
            "quaternion": (quat_w, quat_x, quat_y, quat_z),
            "linear_acceleration": (lin_accel_x, lin_accel_y, lin_accel_z),
            "gravity": (grav_x, grav_y, grav_z),
            "calibration": {
                "system": sys_cal,
                "gyro": gyro_cal,
                "accel": accel_cal,
                "mag": mag_cal
            }
        }
        return data
 
    except Exception as e:
        print(f"Error reading {sensor_name} data: {e}")
        return None
 
# Add method to read offset registers (needed for calibration info)
def _read_offset_registers(imu, reg_x, reg_y, reg_z):
    """Read offset registers for a sensor axis (x, y, z)"""
    try:
        x = imu._read_signed_byte_pair(reg_x)
        y = imu._read_signed_byte_pair(reg_y)
        z = imu._read_signed_byte_pair(reg_z)
        return (x, y, z)
    except:
        return (None, None, None)
 
def _read_radius_registers(imu, reg_1, reg_2):
    """Read radius registers for a sensor"""
    try:
        radius = imu._read_signed_byte_pair(reg_1, reg_2)
        return radius
    except:
        return None
 
# Add these methods to the BNO055 class
adafruit_bno055.BNO055_I2C._read_offset_registers = _read_offset_registers
adafruit_bno055.BNO055_I2C._read_radius_registers = _read_radius_registers
 
# New functions for knee angle calculation
 
def calculate_knee_angle_euler(euler_angles_upper, euler_angles_lower):
    """
    Calculate knee angle using Euler angles.
    This is a simplified approach that works best when the leg movement is primarily in one plane.
 
    Args:
        euler_angles_upper: Tuple of (roll, pitch, yaw) in degrees for upper leg IMU
        euler_angles_lower: Tuple of (roll, pitch, yaw) in degrees for lower leg IMU
 
    Returns:
        Knee angle in degrees
    """
    # For simplicity, we'll focus on the pitch angle (rotation around X-axis)
    # This assumes the IMUs are oriented so that pitch corresponds to knee flexion/extension
    upper_pitch = euler_angles_upper[1]  # Y-axis rotation in euler angles
    lower_pitch = euler_angles_lower[1]
 
    # Calculate the relative angle
    knee_angle = abs(upper_pitch - lower_pitch)
 
    # Normalize to 0-180 range (assuming knee doesn't hyperextend beyond 180)
    if knee_angle > 180:
        knee_angle = 360 - knee_angle
 
    return knee_angle
 
def quaternion_to_rotation_matrix(q):
    """
    Convert quaternion to rotation matrix.
 
    Args:
        q: Quaternion as (w, x, y, z)
 
    Returns:
        3x3 rotation matrix as numpy array
    """
    # Using scipy's Rotation class for robust quaternion handling
    return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
 
def calculate_knee_angle_quaternion(quat_upper, quat_lower):
    """
    Calculate knee angle using quaternions (more accurate approach).
    This works in full 3D space regardless of orientation.
 
    Args:
        quat_upper: Quaternion (w, x, y, z) for upper leg IMU
        quat_lower: Quaternion (w, x, y, z) for lower leg IMU
 
    Returns:
        Knee angle in degrees
    """
    # Convert quaternions to rotation matrices
    R_upper = quaternion_to_rotation_matrix(quat_upper)
    R_lower = quaternion_to_rotation_matrix(quat_lower)
 
    # Calculate relative rotation matrix
    # This represents the rotation from upper leg to lower leg
    R_relative = np.dot(np.linalg.inv(R_upper), R_lower)
 
    # Convert to rotation object for easier angle extraction
    rot_relative = R.from_matrix(R_relative)
 
    # Extract the angle of rotation (in radians)
    angle_rad = np.linalg.norm(rot_relative.as_rotvec())
 
    # Convert to degrees
    angle_deg = math.degrees(angle_rad)
 
    return angle_deg
 
def calculate_simple_knee_angle(grav_upper, grav_lower):
    """
    Calculate knee angle using gravity vectors.
    This is often more reliable for knee flexion/extension angle.
 
    Args:
        grav_upper: Gravity vector from upper leg IMU
        grav_lower: Gravity vector from lower leg IMU
 
    Returns:
        Knee angle in degrees
    """
    # Normalize gravity vectors
    v1 = np.array(grav_upper)
    v2 = np.array(grav_lower)
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
 
    # Calculate the angle between the two vectors
    dot_product = np.clip(np.dot(v1, v2), -1.0, 1.0)
    angle_rad = np.arccos(dot_product)
    angle_deg = math.degrees(angle_rad)
 
    return angle_deg
 
def record_reference_position(imu1, imu2):
    """
    Record a reference "zero" position for the knee.
    This can be used to calibrate the system for individual differences.
 
    Returns:
        Dictionary with reference orientations
    """
    print("\nRecord reference position")
    print("=========================")
    print("Position the leg in the reference position (typically full extension)")
    input("Press Enter when ready...")
 
    # Collect data from IMUs
    imu1_data = read_imu_data(imu1, "Upper Leg IMU")
    imu2_data = read_imu_data(imu2, "Lower Leg IMU")
 
    if not imu1_data or not imu2_data:
        print("Error reading IMU data for reference position")
        return None
 
    print("Reference position recorded!")
 
    # Return the reference orientation information
    return {
        "upper_quaternion": imu1_data["quaternion"],
        "lower_quaternion": imu2_data["quaternion"],
        "upper_euler": imu1_data["euler_angles"],
        "lower_euler": imu2_data["euler_angles"],
        "upper_gravity": imu1_data["gravity"],
        "lower_gravity": imu2_data["gravity"]
    }
 
def calculate_corrected_knee_angle(imu1_data, imu2_data, reference=None, method="quaternion"):
    """
    Calculate knee angle with optional correction based on reference position.
 
    Args:
        imu1_data: Data dictionary from upper leg IMU
        imu2_data: Data dictionary from lower leg IMU
        reference: Reference position data (optional)
        method: Calculation method - "euler", "quaternion", or "gravity"
 
    Returns:
        Knee angle in degrees
    """
    if not imu1_data or not imu2_data:
        return None
 
    if method == "euler":
        angle = calculate_knee_angle_euler(
            imu1_data["euler_angles"], 
            imu2_data["euler_angles"]
        )
        # Apply reference correction if available
        if reference:
            ref_angle = calculate_knee_angle_euler(
                reference["upper_euler"],
                reference["lower_euler"]
            )
            angle = abs(angle - ref_angle)
 
    elif method == "gravity":
        angle = calculate_simple_knee_angle(
            imu1_data["gravity"],
            imu2_data["gravity"]
        )
        # Apply reference correction if available
        if reference:
            ref_angle = calculate_simple_knee_angle(
                reference["upper_gravity"],
                reference["lower_gravity"]
            )
            angle = abs(angle - ref_angle)
 
    else:  # Default to quaternion method
        angle = calculate_knee_angle_quaternion(
            imu1_data["quaternion"],
            imu2_data["quaternion"]
        )
        # Apply reference correction if available
        if reference:
            ref_angle = calculate_knee_angle_quaternion(
                reference["upper_quaternion"],
                reference["lower_quaternion"]
            )
            angle = abs(angle - ref_angle)
 
    return angle
 
# Main program
print("BNO055 IMU Knee Angle Measurement")
print("=================================")
 
# Calibrate both IMUs
print("\nStarting calibration process...")
imu1_calibrated = calibrate_imu(imu1, "Upper Leg IMU")
imu2_calibrated = calibrate_imu(imu2, "Lower Leg IMU")
 
if not (imu1_calibrated and imu2_calibrated):
    print("\nWarning: One or both IMUs were not fully calibrated.")
    print("Data accuracy may be compromised.")
    proceed = input("Do you want to proceed anyway? (y/n): ")
    if proceed.lower() != 'y':
        print("Exiting program.")
        sys.exit(0)
 
# Set up reference position
use_reference = input("\nDo you want to set a reference position? (y/n): ")
reference_data = None
if use_reference.lower() == 'y':
    reference_data = record_reference_position(imu1, imu2)
 
# Select calculation method
print("\nSelect knee angle calculation method:")
print("1. Euler angles (simple but subject to gimbal lock)")
print("2. Quaternions (robust but more complex)")
print("3. Gravity vectors (often most reliable for knee angle)")
method_choice = input("Enter choice (1-3) [default=3]: ") or "3"
 
if method_choice == "1":
    angle_method = "euler"
elif method_choice == "2":
    angle_method = "quaternion"
else:
    angle_method = "gravity"
 
print(f"\nUsing {angle_method} method for angle calculation")
 
# Main data collection and angle calculation loop
print("\nStarting knee angle measurement. Press Ctrl+C to exit.")
try:
    while True:
        # Read data from each IMU
        imu1_data = read_imu_data(imu1, "Upper Leg IMU")
        imu2_data = read_imu_data(imu2, "Lower Leg IMU")
 
        # Calculate knee angle
        knee_angle = calculate_corrected_knee_angle(
            imu1_data, 
            imu2_data, 
            reference_data, 
            angle_method
        )
 
        # Display results
        if knee_angle is not None:
            print(f"\rKnee Angle: {knee_angle:.1f}° ", end="")
        else:
            print("\rError calculating knee angle", end="")
 
        # Brief delay between readings
        time.sleep(0.1)
 
except KeyboardInterrupt:
    print("\n\nExiting program")
