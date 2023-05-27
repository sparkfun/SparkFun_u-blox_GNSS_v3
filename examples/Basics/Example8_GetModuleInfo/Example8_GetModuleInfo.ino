/*
  Reading the protocol and firmware versions of a u-blox module
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information.

  This example shows how to query a u-blox module for its protocol version.

  Various modules have various protocol version. We've seen v18 up to v27. Depending
  on the protocol version there are different commands available. This is a handy
  way to predict which commands will or won't work.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

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

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(Serial); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  if (myGNSS.getModuleInfo())
  {
    Serial.print(F("FWVER: "));
    Serial.print(myGNSS.getFirmwareVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getFirmwareVersionLow()); // Returns uint8_t
    
    Serial.print(F("Firmware: "));
    Serial.println(myGNSS.getFirmwareType()); // Returns HPG, SPG etc. as (const char *)

    Serial.print(F("PROTVER: "));
    Serial.print(myGNSS.getProtocolVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getProtocolVersionLow()); // Returns uint8_t
    
    Serial.print(F("MOD: "));
    Serial.println(myGNSS.getModuleName()); // Returns ZED-F9P, MAX-M10S etc. as (const char *)
  }
  else
    Serial.println(F("Error: could not read module info!"));

  // Use the helper method to read the unique chip ID as a string
  // Returns "000000000000" if the read fails
  Serial.print(F("Unique chip ID: 0x"));
  Serial.println(myGNSS.getUniqueChipIdStr());

  // Or we can read the ID and use the helper method to convert it to string
  UBX_SEC_UNIQID_data_t chipID;
  if (myGNSS.getUniqueChipId(&chipID))
  {
    Serial.print(F("Unique chip ID: 0x"));
    Serial.println(myGNSS.getUniqueChipIdStr(&chipID));
  }
  else
    Serial.println(F("Error: could not read chip ID!"));

  // Or we can read and print the unique chip ID manually
  if (myGNSS.getUniqueChipId(&chipID))
  {
    Serial.print(F("Unique chip ID: 0x"));
    // The ID is five bytes on the F9 and M9 (version 1) but six bytes on the M10 (version 2)
    for (uint8_t i = 0; i < (chipID.version + 4); i++)
    {
      if (chipID.uniqueId[i] < 0x10)
        Serial.print(F("0"));
      Serial.print(chipID.uniqueId[i], HEX);
    }
    Serial.println();
  }
  else
    Serial.println(F("Error: could not read chip ID!"));

}

void loop()
{
  //Do nothing
}