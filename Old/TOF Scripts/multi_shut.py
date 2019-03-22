import VL53L1X
from time import sleep
import time
import RPi.GPIO as GPIO

SHUTX_PIN_N = 14
SHUTX_PIN_S = 15
SHUTX_PIN_W = 16
SHUTX_PIN_E = 17

def get_and_print_measurement():
    # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
    start = time.time()
    tof.start_ranging(3)
    distance_in_mm = tof.get_distance()
    tof.stop_ranging()
    print("sensor on pin: %d\tvalue: %d\ttime: %f" % (pin, distance_in_mm,time.time()-start) )

def toggle_pin(pin):
    if pin == SHUTX_PIN_N:
        new_pin = SHUTX_PIN_E
    elif pin == SHUTX_PIN_E:
        new_pin = SHUTX_PIN_S
    elif pin == SHUTX_PIN_S:
        new_pin = SHUTX_PIN_W:
    elif pin == SHUTX_PIN_W:
        pin == SHUTX_PIN_N
    return new_pin

GPIO.setwarnings(False)

# Setup GPIO for shutdown pins on each VL53L0X
GPIO.setmode(GPIO.BCM)
GPIO.setup(SHUTX_PIN_N, GPIO.OUT)
GPIO.setup(SHUTX_PIN_S, GPIO.OUT)
GPIO.setup(SHUTX_PIN_E, GPIO.OUT)
GPIO.setup(SHUTX_PIN_W, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
GPIO.output(SHUTX_PIN_N, GPIO.LOW)
GPIO.output(SHUTX_PIN_S, GPIO.LOW)
GPIO.output(SHUTX_PIN_E, GPIO.LOW)
GPIO.output(SHUTX_PIN_W, GPIO.LOW)
sleep(1)

# Start with first sensor
pin = SHUTX_PIN_1
GPIO.output(pin, GPIO.HIGH)

# Initialise the i2c bus and configure the sensor
tof = VL53L1X.VL53L1X()
tof.open()
GPIO.output(pin, GPIO.LOW)

while True:
    pin = toggle_pin(pin)
    GPIO.output(pin, GPIO.HIGH)
    get_and_print_measurement()
    GPIO.output(pin, GPIO.LOW)
    sleep(0.01)