 /*!
  * file scanI2C.ino
  * 
  * Connect I2C devices to I2CMultiplexer, and connect I2CMultiplexer to Arduino, then download this example
  * @n open serial monitor to check the I2C addr.
  *
  * Copyright   [DFRobot](http://www.dfrobot.com), 2016
  * Copyright   GNU Lesser General Public License
  *
  * version  V0.1
  * date  2018-5-10
  */
#include <DFRobot_I2CMultiplexer.h>
#include <Wire.h>
/*Create an I2CMultiplexer object, the address of I2CMultiplexer is 0x70*/
DFRobot_I2CMultiplexer I2CMultiplexer(0x70);

void setup(){
  //while(!Serial);
  Serial.begin(9600);
  delay(1000);
  Serial.println("Scan ready!");
  
  /*Print I2C device of each port*/
  for (uint8_t port=0; port<8; port++) {
    Serial.print("Port:"); Serial.print(port);
    uint8_t* dev = I2CMultiplexer.scan(port);
    while(*dev){
      Serial.print("  i2c addr ");Serial.print(*dev);
      dev++;
    }
    Serial.println();
  }
}
void loop(){
  
}
