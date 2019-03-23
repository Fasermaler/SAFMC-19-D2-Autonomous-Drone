'''These functions exist because dronekit's send_global_velocity uses North East Down as inputs
thus for a drone to move in any direction, it has to be a combination of the North and East vectors given
These functions are for easy calculations of the drone's velocity from it's initial heading
'''


import math


class vel_calc:

	# Upon initalization of the velocity class, it takes the immediate bearing 
	# and assumes that this is the 'forward' bearing for the drone
	# takes the angle in degrees (as given by dronekit directly

	# Allows the user to set the mavlink update rate as well (in seconds)
	# this is important because the update rate will affect the distance travelled
	def __init__(self, initial_bearing, update_rate=0.5):

		self.i_bearing = math.radians(initial_bearing)
		self.poll_rate = update_rate

	# allows user to set a new bearing in degrees
	def set_bearing(self, bearing):

		self.i_bearing = math.radians(bearing)

	# allows user to set update rate
	def set_update_rate(self, update_rate)

		self.poll_rate = update_rate

	# This function allows the velocity to be calculated given an angle (0 is forward)
	# takes the angle in degrees
	# Takes velocity in m/s
	# The function factors in update rate so as to reduce the calculated velocity accordingly
	# Returns a tuple
	def calc_vel_from_angle(self, angle, velocity=1):

		# get the actual angle with initial bearing factored in
		full_angle = math.radians(angle) + self.i_bearing 

		# Ensures the angle stays within 0-360 degrees
		if full_angle > math.radians(360):
        	rads -= math.radians(360)
    	elif full_angle < -math.radians(360):
        	full_angle += math.radians(360)

        # The divisions is the amount of times the calculated velocity has to be
        # divided by so as to achieve the actual user set velocity
        divisions = 1 / self.poll_rate
        velocity_N = velocity*(np.cos(full_angle)) / divisions
        velocity_E = velocity*(np.sin(full_angle)) / divisions

        return (velocity_N, velocity_E)

    # this subroutine calculates the velocities for forward, backward, left and right
    # returns a list (FBLR)
    def calc_vel_FBLR(self):

    	# velocity list
    	vel_list = []
    	angles = [0, 90, 180, 270]
    	for i in range(4):
    		vel_list.append(calc_vel_from_angle(angles[i]))

    	return vel_list





