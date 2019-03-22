'''This is the class that governs the time of flight sensors
the sensors in question are the vl53l1x TOF sensors and the drone uses 5 of them'''

# Mux and TOF imports
import I2CMultiplexer
import VL53L1X

import time

class TOF_sensor:

	'''
	asks for the ports on the I2C multiplexer that the sensors are connected to
	NORTH - Sensor pointing north of Drone
	SOUTH - Sensor pointing south of Drone
	EAST - Sensor pointing east of Drone
	WEST - Sensor pointing west of drone
	FIFTH - fifth sensor pointed 45 degrees north west of drone
	NORTH_OFFSET - offset (in mm) from centre of drone
	SOUTH_OFFSET - offset (in mm) from centre of drone
	EAST_OFFSET - offset (in mm) from centre of drone
	WEST_OFFSET - offset (in mm) from centre of drone
	FIFTH_OFFSET - offset (in mm) from centre of drone
	'''

	def __init__(self, NORTH, SOUTH, EAST, WEST, FIFTH, NORTH_OFFSET=0, 
		         SOUTH_OFFSET=0, EAST_OFFSET=0, WEST_OFFSET=0, FIFTH_OFFSET=0):
		
		self.ports = [NORTH, SOUTH, EAST, WEST, FIFTH]
		self.offsets = [NORTH_OFFSET, SOUTH_OFFSET, EAST_OFFSET, WEST_OFFSET, FIFTH_OFFSET]

		# intialize the distance list
		# distance list is in N, S, E, W, 5
		self.dist_list = [0,0,0,0,0]


		# variable to check if TOFs are initialized
		self.init = False

		# variable to terminate sensor readings in get_sensor_readings
		self.terminate = False

		# initialize the mux object:
		# the mux used is a TCA9548A with default address 0x70
		self.mux_obj = I2CMultiplexer.I2CMultiplexer(0x70)  

		# initialize the tof object:
		# VL53L1X has default address 0x29
		self.tof_obj = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)


	# subroutine to initialize sensors
	# the reason why this is callable is to avoid the sensors being initialized when they are not ready
	# be careful about intialization because if any of the ports fail, there will be an I/O error that will clog up the script
	'''
	optional TOF param, dist takes 3 values:
	1 = short range (up to 1m)
	2 = medium range
	3 = long range (up to 4m)
	'''
	def initialize_sensors(self, dist=3):

		try:

			for port in self.ports:

				# selects the right port to read/writeon 
				self.mux_obj.selectPort(port)
				# initialize the tof object
				self.tof_obj.open() 
				# starts ranging
				self.start_ranging(dist)

			# set initialization to True
			print("Sensors successfully initialized")
			self.init = True
		except:
			print("Failed to initialize sensors")
			self.init = False

	# subroutine to get the sensor readings
	# user can set the rate in seconds
	def get_sensor_readings(self, rate=0.05):

		while not self.init:

			print("Sensors have not been initialized, initializing..")
			self.initialize_sensors()

		print("Getting sensor readings..")
		
		# checks if the loop is to terminate
		while not self.terminate:

			# gets reading from each tof and updates the dist_list
			for port in self.ports:
				self.mux_obj.selectPort(port)
				self.dist_list[i] = self.tof_obj.get_distance() - self.offsets[i]

			time.sleep(rate)

