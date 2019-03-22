
#include "DFRobot_I2CMultiplexer.h"
#include <Wire.h>
#include <SPI.h>
#include <VL53L1X.h>

VL53L1X sensorN;
VL53L1X sensorS;
VL53L1X sensorE;
VL53L1X sensorW;

DFRobot_I2CMultiplexer I2CMulti(0x70);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  sensorN.setTimeout(500);
  sensorS.setTimeout(500);
  sensorE.setTimeout(500);
  sensorW.setTimeout(500);
  Wire.setClock(400000);
  
  I2CMulti.selectPort(0);
  if (!sensorE.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensorE.setDistanceMode(VL53L1X::Long);
  sensorE.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensorE.startContinuous(50);
  }

  I2CMulti.selectPort(2)
  if (!sensorN.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensorN.setDistanceMode(VL53L1X::Long);
  sensorN.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensorN.startContinuous(50);
  }
}

  I2CMulti.selectPort(4)
  if (!sensorS.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensorS.setDistanceMode(VL53L1X::Long);
  sensorS.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensorS.startContinuous(50);
  }
}
  I2CMulti.selectPort(6)
  if (!sensorW.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensorW.setDistanceMode(VL53L1X::Long);
  sensorW.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensorW.startContinuous(50);
  }
}



void loop() {
  // put your main code here, to run repeatedly:
  I2CMulti.selectPort(0)
  int sE_read = sensorE.read();
  String sE_string = "SensorE: " + sE_read;
  Serial.println(sE_string);

  I2CMulti.selectPort(2)
  int sN_read = sensorN.read();
  String sN_string = "SensorN: " + sN_read;
  Serial.println(sN_string);

  I2CMulti.selectPort(4)
  int sS_read = sensorS.read();
  String sS_string = "SensorS: " + sS_read;
  Serial.println(sS_string);
  
  I2CMulti.selectPort(6)
  int sW_read = sensorW.read();
  String sW_string = "SensorW: " + sW_read;
  Serial.println(sW_string);
}
