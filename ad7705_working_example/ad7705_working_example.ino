// liman324@yandex.ru
// Alexander Liman
// rcl-radio.ru

#include <SPI.h>
#include "AD7705.h"
AD7705 ad(26,25);// DRDY,RESET
//AD7705 ad(9,10);// DRDY,RESET

//esp32c3
/*
 #define SS    7 // CS
 #define MOSI  6 // DIN
 #define MISO  5 // DOUT
 #define SCK   4 // SCLK

*/
//esp32
// ss    2  // CS
// MOSI  23 // DIN
// MISO  19 // DOUT
// SCK   18 // SCLK

long data;

void setup() {
  Serial.begin(115200);
  ad.conf();
  delay(500);
}

void loop() {
  // CH1 = 1/CH2 = 2
  // GAIN = 1,2,4,8,16,32,64,128
  // RATE = 20,25,100,200 Hz
  // UNIPOLAR = 0/BIPOLAR = 1 

  ad.setSetup(1,1,200,0);
  data = ad.read_unipolar();
  Serial.print("CH1 ");Serial.print(data);

  ad.setSetup(2,1,200,1);
  data = ad.read_bipolar();
  Serial.print("  CH2 ");Serial.println(data);

  delay(10);
}

/****CH1,GAIN1,25Hz,bipolar*****
 ad.setSetup(1,1,25,1);
 data = ad.read_bipolar();
 // long data = -32768...32767 = -2.5...+2.5 V
 */

/****CH1,GAIN1,25Hz,unipolar*****
 ad.setSetup(1,1,25,0);
 data = ad.read_unipolar();
 // long data = 0...65535  = 0...+5 V
 */
