import I2CMultiplexer
import VL53L1X
import time


def multi_i2c_test():
	#Create an I2C Multiplexer object, the address of I2C Multiplexer is 0X70
	I2CMulti = I2CMultiplexer.I2CMultiplexer(0x70)  
	#I2CMulti.selectPort(2)
	tof = VL53L1X.VL53L1X()
	#print("Sensor N has address: 0x29")

	for i in [0,1,2,3,7]:
		I2CMulti.selectPort(i)
		tof.open() # Initialise the i2c bus and configure the sensor
		tof.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
	while(True):

		I2CMulti.selectPort(0)
		distance_in_mm_N = tof.get_distance() # Grab the range in mm
		I2CMulti.selectPort(1)
		distance_in_mm_S = tof.get_distance() # Grab the range in mm
		I2CMulti.selectPort(2)
		distance_in_mm_E = tof.get_distance() # Grab the range in mm
		I2CMulti.selectPort(3)
		distance_in_mm_W = tof.get_distance() # Grab the range in mm
		I2CMulti.selectPort(7)
		distance_in_mm_45 = tof.get_distance() # Grab the range in mm

		print("Sensor N distance: " + str(distance_in_mm_N) + 
			"\nSensor S distance: " + str(distance_in_mm_S) + 
			"\nSensor E distance: " + str(distance_in_mm_E) + 
			"\nSensor W distance: " + str(distance_in_mm_W) + 
			"\nSensor 45 distance: " + str(distance_in_mm_45))
		time.sleep(1)


	I2CMulti.i2c.write_byte(0x70,0) # how it closes?
	tof.stop_ranging() # Stop ranging

def single_i2c_test(port):
	I2CMulti = I2CMultiplexer.I2CMultiplexer(0x70)  
	#I2CMulti.selectPort(2)
	tof = VL53L1X.VL53L1X()
	#print("Sensor N has address: 0x29")
	I2CMulti.selectPort(port)
	tof.open() # Initialise the i2c bus and configure the sensor
	tof.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

	while(True):
		I2CMulti.selectPort(port)
		distance_in_mm = tof.get_distance() # Grab the range in mm
		print("Sensor Distance: " + str(distance_in_mm))

	I2CMulti.i2c.write_byte(0x70,0)
	tof.stop_ranging


multi_i2c_test()








