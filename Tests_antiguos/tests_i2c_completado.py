# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Source https://github.com/adafruit/Adafruit_CircuitPython_BNO055/blob/main/examples/bno055_simpletest.py
import time
import board
import adafruit_bno055


i2c = board.I2C()  # uses board.SCL and board.SDA --Also sets tge frequency
sensor = adafruit_bno055.BNO055_I2C(i2c)
# Change mode to IMUPLUS_MODE to get relative orientation via acc and gyroscope data (fastest possible orientation)
sensor.mode = adafruit_bno055.IMUPLUS_MODE

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

sleep_time = 0.1

while True:
	euler_angles = sensor.euler # Yaw, Roll, pitch
	roll = euler_angles[2]
	rotation = detect_rotation(roll)
	
	#print("Roll: {:07.2f}. Rotacion: {}".format(roll, rotation))
	print("{}, {}, {}".format(euler_angles[0], euler_angles[1], euler_angles[2]))
	if roll > 180 or roll < -180:
		sleep_time = 0.1
	time.sleep(sleep_time)



"""
TODO:
	CONSEGUIR FORMATEAR LA SALIDA DE LOS ANGULOS DE EULER PARA TRES DIGITOS SOLO
	CONSEGUIR MOVER EL MOTOR EN FUNCIÓN DE LOS ÁNGULOS DE EULER
	
"""

"""
NOTAS
#Para utilizar un segundo sensor
#sensor_2 = adafruit_bno055.BNO055_I2C(i2c, address = 0x29)

Formateo para X decimales: print("Roll: {:07.2f}".format(roll))
	
"""
