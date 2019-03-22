from __future__ import print_function

import readchar
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

disarm = False
# Connect to the Vehicle
vehicle = connect('udp:127.0.0.1:14650', wait_ready=True)

def arm_vehicle():
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    

def disarm_vehicle():
	vehicle.armed = False
	print("[INFO]: Vehicle Disarmed")

def set_stablized_mode():
	vehicle.mode = VehicleMode("STABILIZED")
	print("[INFO]: Vehicle Mode set to STABILIZED")


def takeoff_vehicle(aTargetAltitude):
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
    time.sleep(1)

def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)    

print("[INFO] Program Initialized")
print("[INFO] BINDINGS ARE AS FOLLOWS:")
print("[INFO] Y - Arm Vehicle")
print("[INFO] U - Disarm Vehicle")
print("[INFO] T - Takeoff Vehicle")
print("[INFO] G - Land Vehicle")
print("[INFO] M - Engage Manual RC Control (STABILIZED)")
print("[INFO] W, A, S, D - Forward, Left, Backwards, Right in the X, Y plane")
print("[INFO] Q, E - Ascend, Descend in the Z plane")
print("[WARN] Combining vectors has NOT been implemented. DO NOT TRY IT.")



while not disarm:
	print("[INFO] Altitude: " + vehicle.altitude)
	getted_char = repr(readchar.readchar())
	if getted_char == "'y'":
		arm_vehicle() # Arm 
	elif getted_char == "'u'":
		disarm_vehicle()
		disarm = True
	elif getted_char == "'t'":
		takeoff_vehicle(1) # Takeoff vehicle to 1m
	elif getted_char == 'g':
		vehicle.mode = VehicleMode("LAND")
	elif getted_char == "'m'":
		set_stablized_mode()
	elif getted_char == "'w'":
		send_global_velocity(1,0,0,1)
	elif getted_char == "'s'":
		send_global_velocity(-1,0,0,1)
	elif getted_char == "'d'":
		send_global_velocity(0,1,0,1)
	elif getted_char == "'a'":
		send_global_velocity(0,-1,0,1)
	elif getted_char == "'q'":
		send_global_velocity(0,0,1,1)
	elif getted_char == "'e'":
		send_global_velocity(0,0,1,1)








#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")
