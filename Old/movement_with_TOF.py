import I2CMultiplexer
import VL53L1X
import cv2
import time

from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative


connection_string = '/dev/ttyUSB0'
sitl = None

# Global variables for distance:
distance_in_mm_N = 0
distance_in_mm_S = 0
distance_in_mm_E = 0
distance_in_mm_W = 0

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)

'''
I2CMultiAddr = 0x70     #I2C Multiplexer addr
port = 0                #The I2C device is connected to  port 0

i2cDevAddr = 41         #I2C device addr
reg = 0                 #I2C register addr

buf = [111,107]         #The information will be sent 
nbytes = 3              #The length of received data
'''
#Create an I2C Multiplexer object, the address of I2C Multiplexer is 0X70
I2CMulti = I2CMultiplexer.I2CMultiplexer(0x70)  


#I2CMulti.selectPort(2)
tof = VL53L1X.VL53L1X()
#print("Sensor N has address: 0x29")

# for i in [0,2,4,6]:
for i in [0,1,2,7]:
	I2CMulti.selectPort(i)
	tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
	tof.open() # Initialise the i2c bus and configure the sensor
	tof.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
'''
#I2CMulti.selectPort(0)
tofS = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
print("Sensor S has address: 0x29")
tofS.open() # Initialise the i2c bus and configure the sensor
tofS.start_ranging(1) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

#I2CMulti.selectPort(4)
tofE = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
print("Sensor E has address: 0x29")
tofE.open() # Initialise the i2c bus and configure the sensor
tofE.start_ranging(1) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

#I2CMulti.selectPort(6)
tofW = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
print("Sensor W has address: 0x29")
tofW.open() # Initialise the i2c bus and configure the sensor
tofW.start_ranging(1) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

#Write buf to the Reg register of the I2C device on port 0
#I2CMulti.writeto_mem(port,i2cDevAddr,reg,buf)

#Read message of 3 bytes length from the Reg register of the I2C device on Port 0.
#data = I2CMulti.readfrom_mem(port,i2cDevAddr,reg,nbytes)
#print(data)
'''
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
        print(" Arm state: ", vehicle.armed)
        # Break and return from function just below target altitude.
        if vehicle.rangefinder.distance >= aTargetAltitude * 0.95:
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
        
    return targetlocation;

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
        time.sleep(1)


def get_I2C_readings:
	global distance_in_mm_N
	global distance_in_mm_S
	global distance_in_mm_E
	global distance_in_mm_W
	while(True):

		I2CMulti.selectPort(1)
		distance_in_mm_N = tof.get_distance() # Grab the range in mm

		I2CMulti.selectPort(2)
		distance_in_mm_S = tof.get_distance() # Grab the range in mm

		I2CMulti.selectPort(7)
		distance_in_mm_E = tof.get_distance() # Grab the range in mm

		I2CMulti.selectPort(0)
		distance_in_mm_W = tof.get_distance() # Grab the range in mm


		print("Sensor N distance: " + str(distance_in_mm_N) + " \nSensor S distance: " + str(distance_in_mm_S) + "\nSensor E distance: " + str(distance_in_mm_E) + "\nSensor W distance: " + str(distance_in_mm_W))
		time.sleep(0.05)



arm_and_takeoff(1)
send_global_velocity(0.1,0,0,5)
send_global_velocity(-0.1,0,0,5)
send_global_velocity(0,0.1,0,5)
send_global_velocity(0,-0.1,0,5)

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





