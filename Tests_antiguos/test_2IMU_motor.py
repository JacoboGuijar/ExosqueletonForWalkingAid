import time
import board
import busio
import adafruit_bno055
import numpy as np
from scipy.spatial.transform import Rotation as R

import serial
import time
from ctypes import *
"""
CONFIGURACIÓN DEL MOTOR
"""
# EPOS Command Library path
path='/home/raspberry/Desktop/TFMExoesqueleto/ExosqueletonForWalkingAid-main/lib/EPOS_Linux_Library/lib/arm/v8/libEposCmd.so.6.8.1.0'

# Load library
cdll.LoadLibrary(path)
epos = CDLL(path)

# Defining return variables from Library Functions
ret = 0
pErrorCode = c_uint()
pDeviceErrorCode = c_uint()
plsIsEnabled = c_bool()

# Defining a variable NodeID and configuring connection
nodeID = 1
baudrate = 1000000
timeout = 500

# Configure desired motion profile
acceleration = 30000 # rpm/s, up to 1e7 would be possible
deceleration = 30000 # rpm/s

targetVelocity = c_int64(3000)

"""
FIN CONFIGURACIÓN DEL MOTOR
"""


#	MOTOR

# Query motor position
def GetPositionIs():
    pPositionIs=c_long()
    pErrorCode=c_uint()
    ret=epos.VCS_GetPositionIs(keyHandle, nodeID, byref(pPositionIs), byref(pErrorCode))
    return pPositionIs.value # motor steps

# Move at speed during X seconds
def moveAtVelocity_X_Seconds(v, profileAcceleration, profileDeceleration):
    if targetVelocity != 0 and profileAcceleration != 0:
        ret = epos.VCS_SetVelocityProfile(keyHandle, nodeID, profileAcceleration, profileDeceleration, byref(pErrorCode))
        ret = epos.VCS_MoveWithVelocity(keyHandle, nodeID, v, byref(pErrorCode))
        print(ret)
        now = time.time()
        while now + 3 > time.time():
            pass
        epos.VCS_HaltVelocityMovement(keyHandle, nodeID, byref(pErrorCode))
    else:
        epos.VCS_HaltVelocityMovement(keyHandle, nodeID, byref(pErrorCode))

# Move at speed
def moveAtVelocity(v, profileAcceleration, profileDeceleration):
	if targetVelocity != 0 and profileAcceleration != 0:
		ret = epos.VCS_SetVelocityProfile(keyHandle, nodeID, profileAcceleration, profileDeceleration, byref(pErrorCode))
		ret = epos.VCS_MoveWithVelocity(keyHandle, nodeID, v, byref(pErrorCode))
		#print(ret)
	else:
		epos.VCS_HaltVelocityMovement(keyHandle, nodeID, byref(pErrorCode))

# Funciones que siempre reciben los mismos parámetros y pueden ser "simplificadas" para facilitar la lectura del código
def setEnableState():
    epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode)) # enable device

def setDisableState():
	epos.VCS_SetDisableState(keyHandle, nodeID, byref(pErrorCode))

def activateProfilePositionMode(): 
    epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode)) # activate profile position mode

def activateProfileVelocityMode():
    # Activate profile velocity mode
    epos.VCS_ActivateProfileVelocityMode(keyHandle, nodeID, byref(pErrorCode))

def haltVelocityMovement():
	epos.VCS_HaltVelocityMovement(keyHandle, nodeID, byref(pErrorCode))



def setup_bno055_sensors():
    """
    Setup two BNO055 IMU sensors on different I2C addresses.
    
    Returns:
    --------
    tuple
        Tuple containing the thigh IMU and shank IMU objects
    """
    # Create I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # BNO055 has default address 0x28, but can be changed to 0x29 by connecting 
    # the ADR pin to 3.3V on the second sensor
    thigh_imu = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    shank_imu = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
    
    # Wait for sensors to initialize
    time.sleep(1)
    """
    # Ensure both sensors are calibrated
    print("Please calibrate the IMU sensors...")
    while not (is_calibrated(thigh_imu) and is_calibrated(shank_imu)):
        print("Calibration status:")
        print(f"Thigh IMU: {thigh_imu.calibration_status}")
        print(f"Shank IMU: {shank_imu.calibration_status}")
        time.sleep(1)
    
    print("Both IMUs calibrated successfully!")
    """
    return thigh_imu, shank_imu

def is_calibrated(imu):
    """
    Check if an IMU is fully calibrated.
    
    Parameters:
    -----------
    imu : adafruit_bno055.BNO055_I2C
        The IMU object to check
        
    Returns:
    --------
    bool
        True if fully calibrated, False otherwise
    """
    # Calibration status returns tuple (system, gyro, accel, mag)
    # Each value ranges from 0 (not calibrated) to 3 (fully calibrated)
    sys, gyro, accel, mag = imu.calibration_status
    
    # Require system, gyro and accelerometer to be fully calibrated
    # Magnetometer can be less calibrated for our purposes
    return sys == 3 and gyro == 3 and accel >= 2

def read_quaternion(imu):
    """
    Read quaternion data from BNO055 sensor.
    
    Parameters:
    -----------
    imu : adafruit_bno055.BNO055_I2C
        The IMU object to read from
        
    Returns:
    --------
    numpy.ndarray
        Quaternion as [w, x, y, z]
    """
    # BNO055 returns quaternion as (w, x, y, z)
    quat = imu.quaternion
    
    # Convert to numpy array and handle None case (if sensor is not ready)
    if quat is None:
        return np.array([1.0, 0.0, 0.0, 0.0])  # Default to identity quaternion
    
    return np.array(quat)

def calculate_knee_angle(thigh_quat, shank_quat):
    """
    Calculate the knee angle between thigh and shank segments using quaternion data from IMUs.
    
    Parameters:
    -----------
    thigh_quat : numpy.ndarray
        Quaternion representing the orientation of the thigh IMU [w, x, y, z]
    shank_quat : numpy.ndarray
        Quaternion representing the orientation of the shank IMU [w, x, y, z]
    
    Returns:
    --------
    float
        Knee flexion/extension angle in degrees (positive indicates flexion)
        Measured as rotation around the Z-axis
    """
    # Step 1: Convert quaternions to rotation objects
    thigh_rotation = R.from_quat([thigh_quat[1], thigh_quat[2], thigh_quat[3], thigh_quat[0]])  # scipy uses [x,y,z,w]
    shank_rotation = R.from_quat([shank_quat[1], shank_quat[2], shank_quat[3], shank_quat[0]])
    
    # Step 2: Calculate the relative rotation between segments
    # This gives us the rotation from thigh to shank coordinate frame
    relative_rotation = thigh_rotation.inv() * shank_rotation
    
    # Step 3: Convert to Euler angles (using XYZ sequence to get Z rotation)
    # The rotation around Z-axis will correspond to flexion/extension in this case
    euler_angles = relative_rotation.as_euler('XYZ', degrees=True)
    
    # Step 4: Extract the flexion/extension angle (Z-axis rotation)
    knee_angle = euler_angles[2]
    
    # Step 5: Apply sign convention (positive for flexion, negative for extension)
    # Depending on your IMU placement, you might need to flip the sign
    # Here we assume standard placement where positive angle means flexion
    
    return knee_angle

def process_imu_data(thigh_quaternions, shank_quaternions):
    """
    Process a series of quaternion measurements from thigh and shank IMUs.
    
    Parameters:
    -----------
    thigh_quaternions : list of arrays
        List of quaternions from thigh IMU, each quaternion as [w, x, y, z]
    shank_quaternions : list of arrays
        List of quaternions from shank IMU, each quaternion as [w, x, y, z]
    
    Returns:
    --------
    list
        List of calculated knee angles in degrees
    """
    if len(thigh_quaternions) != len(shank_quaternions):
        raise ValueError("The number of quaternions must be the same for both segments")
    
    knee_angles = []
    
    for i in range(len(thigh_quaternions)):
        angle = calculate_knee_angle(thigh_quaternions[i], shank_quaternions[i])
        knee_angles.append(angle)
    
    return knee_angles

def calibrate_neutral_position(thigh_imu, shank_imu, samples=50, delay=0.02):
    """
    Calibrate the neutral position (0 degrees) for knee angle calculation.
    This records the relative orientation when the knee is fully extended.
    
    Parameters:
    -----------
    thigh_imu : adafruit_bno055.BNO055_I2C
        The thigh IMU object
    shank_imu : adafruit_bno055.BNO055_I2C
        The shank IMU object
    samples : int
        Number of samples to average for calibration
    delay : float
        Delay between samples in seconds
        
    Returns:
    --------
    tuple
        (neutral_thigh_quat, neutral_shank_quat) - quaternions representing the neutral position
    """
    print("Calibrating neutral position...")
    print("Keep knee fully extended and still...")
    
    # Wait a moment for user to get into position
    time.sleep(2)
    
    # Collect samples
    thigh_quats = []
    shank_quats = []
    
    for i in range(samples):
        thigh_quats.append(read_quaternion(thigh_imu))
        shank_quats.append(read_quaternion(shank_imu))
        time.sleep(delay)
        if i % 10 == 0:
            print(f"Collected {i+1}/{samples} samples...")
    
    # Average the quaternions (simple approach - could use more sophisticated quaternion averaging)
    thigh_quat_avg = np.mean(thigh_quats, axis=0)
    shank_quat_avg = np.mean(shank_quats, axis=0)
    
    # Normalize the averaged quaternions
    thigh_quat_avg = thigh_quat_avg / np.linalg.norm(thigh_quat_avg)
    shank_quat_avg = shank_quat_avg / np.linalg.norm(shank_quat_avg)
    
    print("Neutral position calibrated")
    return thigh_quat_avg, shank_quat_avg

def main():
    """
    Main function to continuously read from IMUs and calculate knee angle.
    """
    # Setup IMU sensors
    try:
        thigh_imu, shank_imu = setup_bno055_sensors()
    except Exception as e:
        print(f"Error setting up IMUs: {e}")
        return
    
    # Calibrate neutral position
    try:
        neutral_thigh_quat, neutral_shank_quat = calibrate_neutral_position(thigh_imu, shank_imu)
    except Exception as e:
        print(f"Error during calibration: {e}")
        return
    
    # Create rotation objects for the neutral position
    neutral_thigh_rotation = R.from_quat([neutral_thigh_quat[1], neutral_thigh_quat[2], 
                                        neutral_thigh_quat[3], neutral_thigh_quat[0]])
    neutral_shank_rotation = R.from_quat([neutral_shank_quat[1], neutral_shank_quat[2],
                                        neutral_shank_quat[3], neutral_shank_quat[0]])
    
    #print("Starting knee angle measurement...")
    #print("Press Ctrl+C to exit")


    try:
        while True:
            
            # Read current quaternions
            thigh_quat = read_quaternion(thigh_imu)
            shank_quat = read_quaternion(shank_imu)
            
            # Convert to rotation objects
            thigh_rotation = R.from_quat([thigh_quat[1], thigh_quat[2], thigh_quat[3], thigh_quat[0]])
            shank_rotation = R.from_quat([shank_quat[1], shank_quat[2], shank_quat[3], shank_quat[0]])
            
            # Apply the neutral position calibration
            # Calculate the relative rotations with respect to the neutral position
            rel_thigh_rotation = neutral_thigh_rotation.inv() * thigh_rotation
            rel_shank_rotation = neutral_shank_rotation.inv() * shank_rotation
            
            # Calculate relative rotation between segments
            relative_rotation = rel_thigh_rotation.inv() * rel_shank_rotation
            
            # Convert to Euler angles (using XYZ sequence)
            euler_angles = relative_rotation.as_euler('XYZ', degrees=True)
            
            # Extract the knee angle (Z-axis rotation)
            knee_angle = euler_angles[2]
            
            # Display the knee angle
            knee_angle_text = f"{knee_angle:.2f}º"
            print(f"Knee angle: {knee_angle_text:10}", end="\r")
            
            if knee_angle > 45:
                moveAtVelocity(1800, acceleration, deceleration)
            else:
                haltVelocityMovement()
            
            # Short delay
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"\nError during measurement: {e}")

if __name__ == "__main__":
# Initiating connection and setting motion profile
    print("Entramos al main")
    print("USB")
    #print(epos.VCS_OpenDeviceDlg(byref(pErrorCode)))
    epos.VCS_GetDeviceNameSelection
    keyHandle = epos.VCS_OpenDevice(b'EPOS4', b'MAXON SERIAL V2', b'USB', b'USB0', byref(pErrorCode)) # specify EPOS version and interface
    epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode)) # set baudrate
    epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode)) # clear all faults
    # activateProfilePositionMode()
    activateProfileVelocityMode()

    main()
