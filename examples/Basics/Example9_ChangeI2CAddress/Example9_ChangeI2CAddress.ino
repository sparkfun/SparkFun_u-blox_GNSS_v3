/*
  Change the I2C address of a u-blox module using I2C
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information.

  This example shows how to change the I2C address of a u-blox module

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

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

long lastTime = 0; //Tracks the passing of 2000ms (2 seconds)

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  uint8_t oldAddress = 0x42; //The default address for u-blox modules is 0x42
  uint8_t newAddress = 0x3F; //Address you want to change to. Valid is 0x08 to 0x77.

  //oldAddress = 0x3F; //Uncomment these lines to change the address back again
  //newAddress = 0x42;

  while (Serial.available()) Serial.read(); //Trash any incoming chars
  Serial.print("Press a key to change address to 0x");
  Serial.println(newAddress, HEX);
  while (Serial.available() == false) ; //Wait for user to send character

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  
  if (myGNSS.begin(Wire, oldAddress) == true) //Connect to the u-blox module using Wire port and the old address
  {
    Serial.print("GNSS found at address 0x");
    Serial.println(oldAddress, HEX);

    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX messages only. Turn off the NMEA noise

    myGNSS.setI2CAddress(newAddress, VAL_LAYER_RAM_BBR); //Change I2C address of this device in RAM and BBR
    //Device's I2C address is stored to memory and loaded on each power-on

    delay(2000); // Allow time for the change to take

    if (myGNSS.begin(Wire, newAddress) == true)
    {
      Serial.print("Address successfully changed to 0x");
      Serial.println(newAddress, HEX);
      Serial.print("Now load another example sketch using .begin(Wire, 0x");
      Serial.print(newAddress, HEX);
      Serial.println(") to use this GPS module");
      Serial.println("Freezing...");
      while (1);
    }
  }

  //Something went wrong, begin looking for the I2C device
  Serial.println("Address change probably failed. Beginning an I2C scan.");

  Wire.begin();
}

void loop() {

  byte address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
//    else if (error == 4)
//    {
//      Serial.print("Unknown error at address 0x");
//      if (address < 16)
//        Serial.print("0");
//      Serial.println(address, HEX);
//    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
