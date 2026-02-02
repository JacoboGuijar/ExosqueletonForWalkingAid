# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Source https://github.com/adafruit/Adafruit_CircuitPython_BNO055/blob/main/examples/bno055_simpletest.py
import time
import board
import adafruit_bno055
import math

class KalmanFilter:
	def __init__(self, Q = 0.1, R = 0.1):
		self.Q = Q # Ruido del proceso
		self.R = R # Ruido de la medición
		self.P = 1 # Covarianza estimada del error
		self.V = 0 # Valor estimado inicial
		
	# Predicción
	def update(self, measurement):
		self.P = self.P + self.Q # Actualiza la incertidumbre de la estimación
		K = self.P / (self.P + self.R) # Ganancia de Kalman
		self.V = self.V + K * (measurement - self.V) # Actualización con la nueva medida
		self.P = (1 - K) * self.P # Actualización de la covarianza del error
		
		return self.V

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

"""
CONFIGURACIÓN DEL SENSOR BNO055 E I2C
"""
i2c = board.I2C()  # uses board.SCL and board.SDA --Also sets tge frequency
sensor_1 = adafruit_bno055.BNO055_I2C(i2c, address = 0x28)
sensor_2 = adafruit_bno055.BNO055_I2C(i2c, address = 0x29)

# Change mode to IMUPLUS_MODE to get relative orientation via acc and gyroscope data (fastest possible orientation)
sensor_1.mode = adafruit_bno055.IMUPLUS_MODE
sensor_2.mode = adafruit_bno055.IMUPLUS_MODE
#Calibracion sensor 2 0x28
#  Offsets_Magnetometer:  (466, 170, -204)
#  Offsets_Gyroscope:     (-1, -1, -1)
#  Offsets_Accelerometer: (-18, 35, 4)
sensor_1.offsets_accelerometer = (-18, 35, 4)
sensor_1.offsets_magnetometer = (466, 170, -204)
sensor_1.offsets_gyroscope = (-1, -1, -1)
"""
CONFIGURACIÓN DEL SENSOR BNO055 E I2C
"""

kfz1 = KalmanFilter(Q = 0.1, R = 0.3)
kfy1 = KalmanFilter(Q = 0.28, R = 0.4)
kfz2 = KalmanFilter(Q = 0.1, R = 0.3)
kfy2 = KalmanFilter(Q = 0.28, R = 0.4)

while True:
	euler_1 = sensor_1.euler
	euler_2 = sensor_2.euler
	z_1_filtrado = kfz1.update(euler_1[2])
	y_1_filtrado = kfy1.update(euler_1[1])
	
	anglefill = math.atan2(y_1_filtrado, z_1_filtrado)*180/math.pi
	#print("Roll: {:07.2f}. Rotacion: {}".format(roll, rotation))
	print("Orientacion: {:7.2f} - Kalman: {:7.2f} - Angle: {}".format(euler_1[2], z_1_filtrado, anglefill))
	time.sleep(0.1)



"""
TODO:
	CONSEGUIR FORMATEAR LA SALIDA DE LOS ANGULOS DE EULER PARA TRES DIGITOS SOLO
	CONSEGUIR MOVER EL MOTOR EN FUNCIÓN DE LOS ÁNGULOS DE EULER
	Corregir el error de lectura de los imus que devuelve valores por encima de 1000
"""

"""
NOTAS
#Para utilizar un segundo sensor
#sensor_2 = adafruit_bno055.BNO055_I2C(i2c, address = 0x29)

Formateo para X decimales: print("Roll: {:07.2f}".format(roll))
	
"""
