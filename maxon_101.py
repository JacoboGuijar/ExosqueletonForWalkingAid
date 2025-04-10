import serial
import time
from ctypes import *

# EPOS Command Library path
path="C:\\Users\\jacob\\OneDrive\\Escritorio\\Escritorio\\Universidad\\MUII\\TFM\\Python\\EposCmd64.dll"

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

# Query motor position
def GetPositionIs():
    pPositionIs=c_long()
    pErrorCode=c_uint()
    ret=epos.VCS_GetPositionIs(keyHandle, nodeID, byref(pPositionIs), byref(pErrorCode))
    return pPositionIs.value # motor steps

# Move to position at speed
def MoveToPositionSpeed(target_position,target_speed):
    while True:
        if target_speed != 0:
            epos.VCS_SetPositionProfile(keyHandle, nodeID, target_speed, acceleration, deceleration, byref(pErrorCode)) # set profile parameters

            epos.VCS_MoveToPosition(keyHandle, nodeID, target_position, True, True, byref(pErrorCode)) # move to position
            
        elif target_speed == 0:
            epos.VCS_HaltPositionMovement(keyHandle, nodeID, byref(pErrorCode)) # halt motor

        true_position = GetPositionIs()
        if true_position == target_position:
            break

# Move at speed
def moveAtVelocity(v, profileAcceleration, profileDeceleration):
    if targetVelocity != 0 and profileAcceleration != 0:
        ret = epos.VCS_SetVelocityProfile(keyHandle, nodeID, profileAcceleration, profileDeceleration, byref(pErrorCode))
        ret = epos.VCS_MoveWithVelocity(keyHandle, nodeID, 2000, byref(pErrorCode))
        print(ret)
        now = time.time()
        while now + 5 > time.time():
            pass
        epos.VCS_HaltVelocityMovement(keyHandle, nodeID, byref(pErrorCode))
    else:
        epos.VCS_HaltVelocityMovement(keyHandle, nodeID, byref(pErrorCode))

def setEnableState():
    epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode)) # enable device

def activateProfilePositionMode(): 
    epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode)) # activate profile position mode

def activateProfileVelocityMode():
    # Activate profile velocity mode
    epos.VCS_ActivateProfileVelocityMode(keyHandle, nodeID, byref(pErrorCode))

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
    setEnableState()
    
    moveAtVelocity(1200, acceleration, deceleration)

    print('Motor position: %s' % (GetPositionIs()))
    time.sleep(1)