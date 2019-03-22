import serial
DEVICE_NAME = '/dev/ttyUSB0' # Replace with actual device name
ser = serial.Serial(DEVICE_NAME, 9600) 
while 1: 
    if(ser.in_waiting >0):
        line = ser.readline()
        print(line)