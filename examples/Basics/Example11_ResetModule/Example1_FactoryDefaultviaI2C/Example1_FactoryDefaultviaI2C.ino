/*
  Send command to reset module over I2C
  By: Nathan Seidle
  Date: January 29rd, 2019
  License: MIT. See license file for more information.

  This example shows how to reset the U-Blox module to factory defaults over I2C.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  while (Serial.available()) Serial.read(); //Trash any incoming chars
  Serial.println("Press a key to reset module to factory defaults");
  while (Serial.available() == false) ; //Wait for user to send character

  myGNSS.factoryDefault(); //Reset everything: baud rate, I2C address, update rate, everything.

  delay(5000); // Wait while the module restarts

  while (myGNSS.begin() == false) //Attempt to re-connect
  {
    delay(1000);
    Serial.println(F("Attempting to re-connect to u-blox GNSS..."));
  }

  Serial.println("Unit has now been factory reset. Freezing...");
  while(1); // Do nothing more
}

void loop()
{

}
