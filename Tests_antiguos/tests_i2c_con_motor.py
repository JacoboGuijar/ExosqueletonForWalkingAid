import time
import board
import adafruit_bno055
import serial
from ctypes import *
"""
CONFIGURACIÓN DEL MOTOR
"""
# EPOS Command Library path
path='/home/pi/Desktop/Repo/ExosqueletonForWalkingAid/Maxon_libraries_linux/EPOS_Linux_Library/lib/arm/v8/libEposCmd.so.6.8.1.0'

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

"""
CONFIGURACIÓN DEL SENSOR BNO055 E I2C
"""
i2c = board.I2C()  # uses board.SCL and board.SDA --Also sets tge frequency
sensor = adafruit_bno055.BNO055_I2C(i2c)
# Change mode to IMUPLUS_MODE to get relative orientation via acc and gyroscope data (fastest possible orientation)
sensor.mode = adafruit_bno055.IMUPLUS_MODE
"""
CONFIGURACIÓN DEL SENSOR BNO055 E I2C
"""

"""
FUNCIONES
"""

#	SENSOR

"""
Detecta la rotación en sentido positivo o negativo, con un margen de +-10 grados en el que no detecta giro
Devuelve 1 si la rotación en sentido positivo, -1 en sentido negativo, 0 si está dentro del margen asignado.
"""
def detect_rotation(roll: float) -> int:
	is_rotating = 0
	if roll > 10:
		is_rotating = 1
	elif roll < -10:
		is_rotating = -1
	else:
		is_rotating = 0
		
	return is_rotating

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

#	MOTOR Y SENSOR

"""
Hace girar al motor en función de la inclinación (roll) del sensor
Devuelve: nada
"""
def rotate_with_direction()-> None:
	rotation = detect_rotation(sensor.euler[2])
	#Giro antihorario
	if rotation == 1:
		print("Rotacion: {:02} - Giro horario".format(rotation))
		moveAtVelocity(1800, acceleration, deceleration)
	#Detiene el movimiento
	elif rotation == 0:
		haltVelocityMovement()
	#Giro horario
	elif rotation == -1:
		print("Rotacion: {:02} - Giro antihorario".format(rotation))
		moveAtVelocity(-1800, acceleration, deceleration)
	#Si por lo que sea rotation no es 1, 0, -1 detiene completamente el motor
	else:
		setDisableState()
"""
FIN FUNCIONES
"""



"""
MAIN
"""
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
	
	#moveAtVelocity(1200, acceleration, deceleration)
	now = time.time()
	duration = 20
	while now + duration > time.time():
		rotate_with_direction()
		time.sleep(0.01)
		
	print('Motor position: %s' % (GetPositionIs()))
	time.sleep(1)

"""
FIN MAIN
"""

"""
TODO:
	DONE: CONSEGUIR FORMATEAR LA SALIDA DE LOS ANGULOS DE EULER PARA TRES DIGITOS SOLO
	DONE: CONSEGUIR MOVER EL MOTOR EN FUNCIÓN DE LOS ÁNGULOS DE EULER
	
"""

"""
NOTAS
#Para utilizar un segundo sensor
#sensor_2 = adafruit_bno055.BNO055_I2C(i2c, address = 0x29)

Formateo para X decimales: print("Roll: {:07.2f}".format(roll))
	
"""
