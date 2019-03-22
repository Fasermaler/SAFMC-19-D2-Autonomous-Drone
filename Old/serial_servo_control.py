import serial
import RPi.GPIO as GPIO
import time

print('test')

ser = serial.Serial('/dev/ttyACM0', 115200)
#ser.flushInput()

print('okok')
for i in range(1000,1800):
	#print(i)
	print ('ok')
	try:
		print ('ok')

		int1 = 1000	
		int1_encode = b'%d' %int1
		ser.write(int1_encode)
		print("I sent " + str(int1_encode))

		while ser.in_waiting:
			incoming = ser.read()
			print(incoming)

	except KeyboardInterrupt:
		GPIO.cleanup()
		break
	time.sleep(0.1)