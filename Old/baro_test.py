import time

import navio.ms5611
import navio.util

navio.util.check_apm()

baro = navio.ms5611.MS5611()
baro.initialize()

while(True):
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()

	baro.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready 10ms
	baro.readTemperature()

	baro.calculatePressureAndTemperature()

	print "Temperature(C): %.6f" % (baro.TEMP), "Pressure(millibar): %.6f" % (baro.PRES)

time.sleep(1)
