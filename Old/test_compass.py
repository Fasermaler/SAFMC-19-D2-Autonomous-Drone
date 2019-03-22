import smbus2
import time

bus = smbus2.SMBus(0)
address = 0x3b

def get_heading(address):
	byte1 = bus.read_byte_data(address, 1)
	byte2 = bus.read_byte_data(address, 2)

	heading = (byte1 * 255 + byte2)
	return heading

while True:
	print(get_heading(address))
	time.sleep(0.5)