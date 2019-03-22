#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 21 19:41:51 2019

@author: fasermaler
"""

import VL53L1X

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
print("Sensor 1 has address: 0x29")

tof.open() # Initialise the i2c bus and configure the sensor

tof.start_ranging(1) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

distance_in_mm_1 = tof.get_distance() # Grab the range in mm

tof.stop_ranging() # Stop ranging


print("Sensor 1 distance: " + distance_in_mm_1)


