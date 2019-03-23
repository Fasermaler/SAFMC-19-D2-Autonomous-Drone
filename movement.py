
'''
this class deals with drone movement
'''

class movement:

	# gets the relevant objects needed upon initialization
	def __init__(self, drone_obj, tof_obj, velocity_obj, threshold_distance=1500):
		self.drone = drone_obj
		self.tof = tof_obj
		self.velocity = velocity_obj
		self.velocities = self.velocity.calc_vel_FBLR()

		# this sets the closest the drone can get to an obstacle before velocity commands are terminated
		self.threshold = threshold_distance

	'''the following functions command the drone to move in their respective
	directions. times is just the number of times the send_global_velocity will be called
	Check the drone class' send rate to determine the number of seconds (default 0.5)
	
	The drone will always check if it is about to crash using the corresponding tof sensor
	If the TOF detects the distance is less than 1500cm (Default), it will attempt to abort the command
	

	By default, the TOF sensors get weird readings of less than 800mm once the distance exceeds 4m (the limit)
	thus there is a hardcoded minimum below which the TOF will ignore readings. This is only possible because
	our drone was never meant to operate closer than 1.5m any wall. 
	This is just a warning to those who may want to change the threshold to be lower than 800mm
	'''
	def move_forward(self, times=1):
		for i in range(times):
			if self.tof.dist_list[0] > threshold:
				self.drone.send_global_velocity(*velocities[0],0,1)
			else:
				self.drone.send_global_velocity(0,0,0,1)
	def move_backward(self, times=1)
		for i in range(times):
			if self.tof.dist_list[1] > threshold:
				self.drone.send_global_velocity(*velocities[1],0,times)
			else:
				self.drone.send_global_velocity(0,0,0,1)
	def move_left(self, times=1)
		for i in range(times):
			if self.tof.dist_list[2] > threshold:
				self.drone.send_global_velocity(*velocities[2],0,times)
			else:
				self.drone.send_global_velocity(0,0,0,1)
	def move_right(self, times=1)
		for i in range(times):
			if self.tof.dist_list[3] > threshold:
				self.drone.send_global_velocity(*velocities[3],0,times)
			else:
				self.drone.send_global_velocity(0,0,0,1)

	# this is a subroutine to call the drone to move at an angle relative to itself (0 is forward)
	# the default velocity is 0.1m/s
	# down corresponds to the z axis, postive means the drone moves down
	'''
	do note that there is no way to check for obstacles when the drone isnt flying in
	the four main directions, thus there is no TOF check for these movements
	'''

	def move_angle(self, angle, velocity=0.1, down=0, times=1):
		velocity_tuple = self.velocity.calc_vel_from_angle(angle, velocity)
		for i in times:
			self.drone.send_global_velocity(*velocity_tuple, down, 1)


	# this is the raw movement call, do not use it unless you are sure of what you are doing
	# the drone movement is in accordance to the North East Down convention
	# combine the vectors to obtain the correct moevemtn in correct direction
	def move_vels(self, North, East, Down=0, times=1)
		self.drone.send_global_velocity(North, East, Down, times)

	'''the following are raw directional movement calls without TOF checks'''

	def move_forward_raw(self, times=1):
		for i in range(times):
			self.drone.send_global_velocity(*velocities[0],0,1)

	def move_backward(self, times=1)
		for i in range(times):
			self.drone.send_global_velocity(*velocities[1],0,times)

	def move_left(self, times=1)
		for i in range(times):
			self.drone.send_global_velocity(*velocities[2],0,times)

	def move_right(self, times=1)
		for i in range(times):
			self.drone.send_global_velocity(*velocities[3],0,times)




