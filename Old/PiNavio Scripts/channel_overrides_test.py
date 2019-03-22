# Test Script for Vehicle Channels

from __future__ import print_function
from dronekit import connect


# Connect to the Vehicle
vehicle = connect('udp:127.0.0.1:14650', wait_ready=True)

# Get all original channel values (before override)
print("Channel values from RC Tx:", vehicle.channels)

while True:
	# Access channels individually
	print("Read channels individually:")
	print(" Ch1: %s" % vehicle.channels['1'])
	print(" Ch2: %s" % vehicle.channels['2'])
	print(" Ch3: %s" % vehicle.channels['3'])
	print(" Ch4: %s" % vehicle.channels['4'])
	print(" Ch5: %s" % vehicle.channels['5'])
	print(" Ch6: %s" % vehicle.channels['6'])
	print(" Ch7: %s" % vehicle.channels['7'])
	print(" Ch8: %s" % vehicle.channels['8'])
	print("Number of channels: %s" % len(vehicle.channels))




vehicle.close()

# Shut down simulator if it was started.


print("Completed")
