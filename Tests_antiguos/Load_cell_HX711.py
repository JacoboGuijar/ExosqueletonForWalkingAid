#Youtube.com/watch?v=j3QSFnRhyFE 06 installing HX711 library with python for the load cell
#Code for 100 lbs model SN: 876513
from hx711 import HX711
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)


dout_pin = 5 #data out pint GPIO 5
pd_sck_pin = 6 #Serial clock pin GPIO 6
hx = HX711(dout_pin, pd_sck_pin) 

while True:
	reading = hx.get_raw_data_mean()
	print(reading)
	
	time.sleep(0.1)
	

#TODO: codigo para la calibración, uso de la función hx.zero() y 
