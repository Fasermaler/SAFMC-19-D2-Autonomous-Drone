
# import thread for multithreading
import thread


# import custom classes
from drone_class import SAFMC_drone
from corridor_navigation import corridor
from TOF import TOF_sensor
from velocity_calculator import vel_calc



# Vehicle's connection string and baud rate
CONNECTION_STRING = '/dev/ttyUSB0'
BAUD_RATE = 57600

# initialize the TOF object with ports 0,1,2,3,7
tofs = TOF_sensor(0,1,2,3,7)

# initialize tofs with distance param 3
tofs.initialize_sensors(3)

# starts the multithread to read the sensors at 0.05s rate
thread.start_new_thread(get_sensor_readings, (0.05))

# initialize the drone object
drone = SAFMC_drone(CONNECTION_STRING, BAUD_RATE)

# starts the velocity calculator object and get the list of velocities
velocity = vel_calc(drone.get_heading())
vel_list = velocity.calc_vel_FBLR()

#initialize the corridor object
corridor_nav  = corridor(drone, tofs, vel_list)






# Land the drone and close the vehicle object
drone.set_land()
drone.close_vehicle()
# set the termination condition of the TOFs
tofs.terminate = True



