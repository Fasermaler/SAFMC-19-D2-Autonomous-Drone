#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 21 19:41:51 2019

@author: fasermaler
"""

import VL53L1X
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False) 

XSHUT_N = 14
XSHUT_S = 15

GPIO.setup(XSHUT_N, GPIO.OUT)
GPIO.setup(XSHUT_S, GPIO.OUT)

GPIO.output(XSHUT_N, GPIO.LOW)
GPIO.output(XSHUT_S, GPIO.LOW)
time.sleep(0.50)

tofN = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x2B)
tofS = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x2D)

GPIO.output(XSHUT_N, GPIO.HIGH)
tofN.open()
tofN.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range




GPIO.output(XSHUT_S, GPIO.HIGH)
#tofS.change_address(0x36
tofS.open() # Initialise the i2c bus and configure the sensor
tofS.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range


for i in range(100):
	distance_in_mm_N = tofN.get_distance() # Grab the range in mm

	print("Sensor N distance: " + str(distance_in_mm_N))

	distance_in_mm_S = tof2.get_distance() # Grab the range in mm
	

	print("Sensor S distance: " + str(distance_in_mm_S))
	time.sleep(0.1)

tofN.stop_ranging()
tofS.stop_ranging()
tofN.close()
tofS.close()

GPIO.cleanup()