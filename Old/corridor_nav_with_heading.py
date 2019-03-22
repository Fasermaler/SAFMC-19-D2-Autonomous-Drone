from __future__ import print_function
import time
import math
import thread

# Dk imports
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

# Mux and TOF imports
import I2CMultiplexer
import VL53L1X

# CV imports
import cv2
import numpy as np

# Connect to Vehicle
connection_string = '/dev/ttyUSB0'
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)

# Global variables for distance:
distance_in_mm_N = 0 # North Sensor
distance_in_mm_S = 0 # South Sensor
distance_in_mm_E = 0 # East Sensor
distance_in_mm_W = 0 # West Sensor
distance_in_mm_45 = 0 # 45 degree south east sensor

#Create an I2C Multiplexer object, the address of I2C Multiplexer is 0X70
I2CMulti = I2CMultiplexer.I2CMultiplexer(0x70)  
# Init TOF obj
tof = VL53L1X.VL53L1X()
 # STarts the TOFs on their respective ports
try:
    # for i in [0,2,4,6]:
    for i in [0,1,2,7,3]:
    	I2CMulti.selectPort(i)
    	tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
    	tof.open() # Initialise the i2c bus and configure the sensor
    	tof.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
except:
    print("port init failed")

# Arm and rakeoff to specific altitude
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    #Don't try to arm until autopilot is ready
    # while not vehicle.is_armable:
    #     print(" Waiting for vehicle to initialise...")
    #     time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # while not vehicle.armed == True:
    #     print("Not Armed")
    #     time.sleep(0.4)

    # while not vehicle.armed == True:
    #     vehicle.armed = True
    #     print("Not Armed 2")
    #     time.sleep(0.4)


    #Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude


    # Wait until the vehicle reaches a safe height before processing the goto 
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.rangefinder.distance)
        current_alt = vehicle.rangefinder.distance
        if current_alt > 20:
            current_alt = 0
        print(" Arm state: ", vehicle.armed)
        # Break and return from function just below target altitude.
        if current_alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance
    print("Initiating GOTO")

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: " + str(remainingDistance))
        if remainingDistance < 0.11: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)

# Sends a velocity to the drone at a rate of 2 Hx
def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
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
        time.sleep(0.5)



# Sets the Yaw - vehicle will yaw according to the yaw slew rate set in params
# give the vehicle more time (give a 0 velocity vector for x amount of seconds - enough for
# the drone to complete the yaw)
def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

# The following 2 methods allow for the drone attitude to be directly controlled
# the movement is not OF corrected - avoid usage where possible
def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """
    
    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """
    
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000, # Type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

    start = time.time()
    while time.time() - start < duration:
        vehicle.send_mavlink(msg)
        #time.sleep(0.1)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

# Gets the readings from the TOF sensors and updates the distance vars
def get_I2C_readings():
    global distance_in_mm_N
    global distance_in_mm_S
    global distance_in_mm_E
    global distance_in_mm_W
    global distance_in_mm_45
    while(True):

        I2CMulti.selectPort(0)
        distance_in_mm_N = tof.get_distance() # Grab the range in mm

        I2CMulti.selectPort(3)
        distance_in_mm_S = tof.get_distance() # Grab the range in mm

        I2CMulti.selectPort(7)
        distance_in_mm_E = tof.get_distance() # Grab the range in mm

        I2CMulti.selectPort(2)
        distance_in_mm_W = tof.get_distance() # Grab the range in mm
        I2CMulti.selectPort(1)
        distance_in_mm_45 = tof.get_distance() # Grab the range in mm


		#print("Sensor N distance: " + str(distance_in_mm_N) + " \nSensor S distance: " + str(distance_in_mm_S) + "\nSensor E distance: " + str(distance_in_mm_E) + "\nSensor W distance: " + str(distance_in_mm_W))
        time.sleep(0.05)


# Starts TOF readings before takeoff
thread.start_new_thread(get_I2C_readings, ())

# Gets vehicle heading on thr ground (this is assumed to be the forward heading)
heading1 = vehicle.heading

# Takeoff to 1.5m
arm_and_takeoff(1.5)

# Corridor Variables
INCREMENT_DISTANCE = 0.1
CORRIDOR_WIDTH_HALVED = 1300 # in mm
THRESHOLD_DISTANCE = 100
lower_bound = CORRIDOR_WIDTH_HALVED - THRESHOLD_DISTANCE
upper_bound = CORRIDOR_WIDTH_HALVED + THRESHOLD_DISTANCE

# Calculates the forward heading 
heading_rad = math.radians(heading1)
fwd_X = (np.cos(heading_rad) / 5) #NORTH
fwd_Y = (np.sin(heading_rad) / 5) #EAST
print(str(fwd_X) + str(fwd_Y))


#Calculates the Right heading
heading_rad -= 1.5708
if heading_rad < 0:
    heading_rad += 2 * (6.283)
right_X = (np.cos(heading_rad) / 5) #NORTH
right_Y = (np.sin(heading_rad) / 5) #EAST

print(str(right_X) + str(right_Y))

# This first part is to go forward down the corridor
# The logic here is to check for YAW first, then check for left right alignment, then finally check to move forward
while distance_in_mm_S < 2000 or distance_in_mm_N < lower_bound or distance_in_mm_N > upper_bound or distance_in_mm_E < lower_bound or distance_in_mm_E > upper_bound:
    print("N distance: " + str(distance_in_mm_N))
    print("E distance: " + str(distance_in_mm_E))
    print("S distance: " + str(distance_in_mm_S))
	if (distance_in_mm_E + distance_in_mm_W) > (CORRIDOR_WIDTH_HALVED*2 + 300): # Checks for a case of extreme Yaw
	    #print("DETECTED SERIOUS YAW")
	    # There is a 10 degree margin on both sides of the axis
	    if np.arctan(distance_in_mm_45 / distance_in_mm_E) > 1.74533: # degrees of leeway
	        print("detected right yaw")
	        condition_yaw(350, True) # Yaw left 10 degrees
	        send_global_velocity(0,0,0,6) # Give it 3 seconds to complete
	    elif np.arctan(distance_in_mm_45 / distance_in_mm_E) < 1.39626:
	        print("detected left yaw")
	        condition_yaw(10, True) # Yaw right 10 degrees
	        send_global_velocity(0,0,0,6) # Give it 3 seconds to complete

    if distance_in_mm_E < lower_bound:
        send_global_velocity(-right_X,-right_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going left")
    elif distance_in_mm_E > upper_bound:
        send_global_velocity(right_X,right_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going right")
    elif distance_in_mm_N < lower_bound and distance_in_mm_S < 3000: # checks if there is wall behind
        send_global_velocity(fwd_X,fwd_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going forward")
    elif distance_in_mm_N < lower_bound and distance_in_mm_S > 3000:
        send_global_velocity(-fwd_X,-fwd_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going backward")
    elif distance_in_mm_N > upper_bound:
        send_global_velocity(fwd_X,fwd_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going forward")

print("Reached Target first waypoint")

send_global_velocity(-right_X,-right_Y,0,10)
send_global_velocity(0,0,0,1) # Send an empty velocity 
# This second part is to go down the left corridor

while distance_in_mm_E < 2000 or distance_in_mm_N < lower_bound or distance_in_mm_N > upper_bound or distance_in_mm_W < lower_bound or distance_in_mm_W > upper_bound:
    print("N distance: " + str(distance_in_mm_N))
    print("E distance: " + str(distance_in_mm_E))
    print("S distance: " + str(distance_in_mm_S))
	if (distance_in_mm_E + distance_in_mm_W) > (CORRIDOR_WIDTH_HALVED*2 + 300): # Checks for a case of extreme Yaw
	    #print("DETECTED SERIOUS YAW")
	    # There is a 10 degree margin on both sides of the axis
	    if np.arctan(distance_in_mm_45 / distance_in_mm_E) > 1.74533: # degrees of leeway
	        print("detected right yaw")
	        condition_yaw(350, True) # Yaw left 10 degrees
	        send_global_velocity(0,0,0,6) # Give it 3 seconds to complete
	    elif np.arctan(distance_in_mm_45 / distance_in_mm_E) < 1.39626:
	        print("detected left yaw")
	        condition_yaw(10, True) # Yaw right 10 degrees
	
    if distance_in_mm_N < lower_bound:
        send_global_velocity(-fwd_X,-fwd_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going back")
    elif distance_in_mm_N > upper_bound:
        send_global_velocity(fwd_X,fwd_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going forward")
    elif distance_in_mm_W < lower_bound and distance_in_mm_E < 3000:
        send_global_velocity(-right_X,-right_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going left")
    elif distance_in_mm_W < lower_bound and distance_in_mm_E > 3000:
        send_global_velocity(right_X,right_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going right")
    elif distance_in_mm_W > upper_bound:
        send_global_velocity(-right_X,-right_Y,0,2)
        send_global_velocity(0,0,0,1) # Send an empty velocity 
        print("Going left")

print("Reached destination")
# condition_yaw(90, True)
# condition_yaw(-90, True)
        






print("Landing")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()






I2CMulti.i2c.write_byte(0x70,0) # how it closes?
tof.stop_ranging() # Stop ranging











