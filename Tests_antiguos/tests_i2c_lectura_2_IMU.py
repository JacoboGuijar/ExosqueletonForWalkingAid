# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Source https://github.com/adafruit/Adafruit_CircuitPython_BNO055/blob/main/examples/bno055_simpletest.py
import time
import board
import adafruit_bno055


i2c = board.I2C()  # uses board.SCL and board.SDA --Also sets tge frequency
sensor_1 = adafruit_bno055.BNO055_I2C(i2c, address = 0x28)
sensor_2 = adafruit_bno055.BNO055_I2C(i2c, address = 0x29)

# Change mode to IMUPLUS_MODE to get relative orientation via acc and gyroscope data (fastest possible orientation)
sensor_1.mode = adafruit_bno055.IMUPLUS_MODE
sensor_2.mode = adafruit_bno055.IMUPLUS_MODE

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


while True:
	rotation_1 = detect_rotation(sensor_1.euler[2])
	rotation_2 = detect_rotation(sensor_2.euler[2])
	#print("Roll: {:07.2f}. Rotacion: {}".format(roll, rotation))
	print("Giro 1: {:02} - Giro 2: {:02}".format(rotation_1, rotation_2))
	time.sleep(0.1)



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
