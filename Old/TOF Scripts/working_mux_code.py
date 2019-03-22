import I2CMultiplexer
import VL53L1X
import cv2
import time
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


I2CMulti.i2c.write_byte(0x70,0) # how it closes?
tof.stop_ranging() # Stop ranging





