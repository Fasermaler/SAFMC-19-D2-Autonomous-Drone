
'''
this class deals with drone movement
'''

class movement:

	# gets the relevant objects needed upon initialization
	def __init__(self, drone_obj, tof_obj, velocity_obj):
		self.drone = drone_obj
		self.tof = tof_obj
		self.velocity = velocity_obj
		self.velocities = self.velocity.calc_vel_FBLR()

	'''the following functions command the drone to move in their respective
	directions. times is just the number of times the send_global_velocity will be called
	Check the drone class' send rate to determine the number of seconds (default 0.5)
	1'''
	def move_forward(self, times=1):
		self.drone.send_global_velocity(*velocities[0],0,times)
	def move_backward(self, times=1)
		self.drone.send_global_velocity(*velocities[1],0,times)
	def move_left(self, times=1)
		self.drone.send_global_velocity(*velocities[2],0,times)
	def move_right(self, times=1)
		self.drone.send_global_velocity(*velocities[3],0,times)

	# this is a subroutine to call the drone to move at an angle relative to itself (0 is forward)
	# the default velocity is 0.1m/s
	# down corresponds to the z axis, postive means the drone moves down
	def move_angle(self, angle, velocity=0.1, down=0, times=1):
		velocity_tuple = self.velocity.calc_vel_from_angle(angle, velocity)
		self.drone.send_global_velocity(*velocity_tuple, down, times)


	# this is the raw movement call, do not use it unless you are sure of what you are doing
	# the drone movement is in accordance to the North East Down convention
	# combine the vectors to obtain the correct moevemtn in correct direction
	def move_vels(self, North, East, Down=0, times=1)
		self.drone.send_global_velocity(North, East, Down, times)




