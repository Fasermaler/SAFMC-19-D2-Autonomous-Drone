




class corridor:

	# gets the objects to call and list of velocities from velocity_calculator
	def __init__(self, tof_obj, movement_obj):
		
		# gets objects from the script
		self.tof = tof_obj
		self.movement_obj = movement_obj

	# checks if the drone is centred relative to tthe respective walls
	'''
	do note that if the VL53L1X can't see a wall within 4m, it will begin
	to output values less than 800mm, thus these checks aren't for use when 
	the axis is along the long corridor.
	only use these checks to align the drone along the short axis of the corridor
	'''
	def check_N_position(self):

		if self.tof.dist_list[0] > 1550:
			self.movement_obj.move_forward(4)
		elif self.tof.dist_list[0] < 1450:
			elf.movement_obj.move_backward(4)
		self.movement_obj.stop(2)



	def check_S_position(self):

		if self.tof.dist_list[1] > 1550:
			self.movement_obj.move_backward(4)
		elif self.tof.dist_list[1] < 1450:
			self.movement_obj.move_forward(4)
		self.movement_obj.stop(2)

	def check_E_position(self):

		if self.tof.dist_list[2] > 1500:
			self.movement_obj.move_right(4)
		self.movement_obj.stop(2)

	def check_W_position(self):

		if self.tof.dist_list[3] > 1500:
			self.movement_obj.move_left(4)
		self.movement_obj.stop(2)


	# goes down the first right corridor
	def go_first_right_corridor(self):

		# checks if the corridor is too far away
		# or if the distance is less than 800, it is also considered to be an erroneous value 
		while self.tof.dist_list[2] > 1500 or self.tof.dist_list[2] < 800:
			# checks and attempts to centre the drone on the short axis first
			
			# if the sensor is so far that it is getting an erroneous reading, use the north wall 
			# to centre the drone
			if self.tof.dist_list[2] < 800 
				self.check_N_position()
			else:
				self.check_S_position()

			self.movement_obj.move_right(4)



	# navigates the corridor based on the direction given
	def navigate_corridor(self, direction):

		if 




