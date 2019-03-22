# This is the SAFMC_drone class that deals with all dronekit and mavlink subroutines
# It is the cleaned up and refractored version of the original script

import math
import time

# Dronekit Imports
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative


# This class does not support SITL mode

class SAFMC_drone:

    def __init__(self, connection_string, baud_rate):

        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, wait_ready=True, baud=baud_rate)


    # Arm and takeoff to specific altitude
    def arm_and_takeoff(self, aTargetAltitude):

        print("Arming motors")
        
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        #Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:

            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")

        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait for vehicle to reach target altitude before actually exiting the function
        while True:

            # Uses rangefinder distance because EKF altitude is inaccurate
            print(" Altitude: ", self.vehicle.rangefinder.distance)
            current_alt = self.vehicle.rangefinder.distance

            # Any altitude values above the height restriction are filtered out
            if current_alt > 20:

                current_alt = 0

            print(" Arm state: ", self.vehicle.armed)
            
            # Break and return from function just below target altitude.
            
            if current_alt >= aTargetAltitude * 0.95:
                
                print("Reached target altitude")
                
                break
            
            time.sleep(1)

    # Give velocity based on the North and East frames
    def send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration):

        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
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
            
            self.vehicle.send_mavlink(msg)
            time.sleep(0.5) # Controls the mavlink message send rate, do not set lower than 0.3



    # Sets the Yaw - vehicle will yaw according to the yaw slew rate set in params
    # give the vehicle more time (give a 0 velocity vector for x amount of seconds - enough for
    # the drone to complete the yaw)
    def condition_yaw(self, heading, relative=False):

        if relative:
            
            is_relative = 1 #yaw relative to direction of travel
        
        else:
            
            is_relative = 0 #yaw is an absolute angle
        
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        
        self.vehicle.send_mavlink(msg)

    # The following 2 methods allow for the drone attitude to be directly controlled
    # the movement is not OF corrected - avoid usage where possible
    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):

        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
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
        self.vehicle.send_mavlink(msg)

        start = time.time()
        
        while time.time() - start < duration:
            
            self.vehicle.send_mavlink(msg)
            #time.sleep(0.1)

    # This function is used by the set_attitude function because it only accepts quarternion
    def to_quaternion(self, roll = 0.0, pitch = 0.0, yaw = 0.0):

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

    # set land mode
    def set_land(self):
        
        print("Landing")
        self.vehicle.mode = VehicleMode("LAND") 

    # Close vehicle object before exiting script
    def close_vehicle(self):
        
        print("Close vehicle object")
        self.vehicle.close()













