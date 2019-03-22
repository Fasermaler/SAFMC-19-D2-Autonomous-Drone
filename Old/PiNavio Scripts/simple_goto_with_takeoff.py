#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

takeOffAlt = 0.5
takeOffGain = 1
homeAlt = None #home altitude is set by take-off command
hoverThrust = 0.5

if hoverThrust > 0.5:
    hoverThrust = 0.5

armed = False
stage = 0
takenOff = False

vehicle = connect('udp:127.0.0.1:14650', wait_ready=True)


def takeoff():
    
    global homeAlt  

    if homeAlt is None:
        homeAlt = vehicle.location.global_relative_frame.alt
        print("Altitude zeroed at: " + str(homeAlt))
    currentAlt = vehicle.location.global_relative_frame.alt
    userThrust = (currentAlt - (homeAlt + takeOffAlt)) * takeOffGain

    if userThrust > 1.0:
        userThrust = 1.0
    elif userThrust < 0.0:
        userThrust = 0.0
    #thrust = hoverThrust - (hoverThrust * thrust)   

    print("takeoff thrust " + str(userThrust) + " at alt " + str(currentAlt))

    #send_attitude_target(0,0,0,thrust,1)
    send_attitude_target(thrust = userThrust)

    print("takeoff check " + str(abs(currentAlt - (homeAlt + takeOffAlt))))

    if (abs(currentAlt - (homeAlt + takeOffAlt)) < 0.1):
        print("reached")
        return True
    else:
        return False

def arm_and_takeoff(aTargetAltitude):
    armed = False
    
    if not armed:
        print("arming...")
        vehicle = connect('udp:127.0.0.1:14650', wait_ready=True)
        vehicle.mode = VehicleMode("GUIDED_NOGPS")
        #while True:
        vehicle.armed = True

        while not vehicle.mode.name=='GUIDED_NOGPS' or not vehicle.armed:
            print("Getting ready...")
            time.sleep(1)
            
        armed = True
        print("Armed")


    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True
    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    time.sleep(5)
    print("Taking off!")
    takeoff()  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    #hile True:
        #print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        #if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            #print("Reached target altitude")
            #break
        #time.sleep(1)


arm_and_takeoff(10)

# print("Set default/target airspeed to 3")
# vehicle.airspeed = 3

# print("Going towards first point for 30 seconds ...")
# point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
# vehicle.simple_goto(point1)

# # sleep so we can see the change in map
# time.sleep(30)

# print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
# point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
# vehicle.simple_goto(point2, groundspeed=10)

# # sleep so we can see the change in map
# time.sleep(30)

# print("Returning to Launch")
# vehicle.mode = VehicleMode("RTL")

# # Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()


