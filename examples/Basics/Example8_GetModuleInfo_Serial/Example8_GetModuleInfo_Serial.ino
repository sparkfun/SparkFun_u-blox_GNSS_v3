/*
  Reading the protocol and firmware versions of a u-blox module
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information.

  This example shows how to query a u-blox module for its protocol version.

  Note: this may fail on boards like the UNO (ATmega328P) with modules like the ZED-F9P
        because getProtocolVersion returns a lot of data - more than the UNO's serial buffer can hold

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

#include <SoftwareSerial.h>

#define mySerial Serial1 // Uncomment this line to connect via Serial1
// - or -
//SoftwareSerial mySerial(10, 11); // Uncomment this line to connect via SoftwareSerial(RX, TX). Connect pin 10 to GNSS TX pin.
// - or -
//#define mySerial Serial1 // Uncomment this line if you just want to keep using Serial

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SERIAL myGNSS;

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Serial.println("Trying 38400 baud");
  mySerial.begin(38400);
  if (myGNSS.begin(mySerial))
  {
    Serial.println("GNSS connected at 38400 baud");
  }
  else
  {
    Serial.println("Trying 9600 baud");
    mySerial.begin(9600);
    if (myGNSS.begin(mySerial))
    {
      Serial.println("GNSS connected at 9600 baud");
    }
    else
    {
      Serial.println("Could not connect to GNSS. Freezing...");
      while(1); // Do nothing more
    }
  }

  if (myGNSS.getModuleInfo())
  {
      Serial.print(F("FWVER: "));
      Serial.print(myGNSS.getFirmwareVersionHigh());
      Serial.print(F("."));
      Serial.println(myGNSS.getFirmwareVersionLow());
      
      Serial.print(F("Firmware: "));
      Serial.println(myGNSS.getFirmwareType());    

      Serial.print(F("PROTVER: "));
      Serial.print(myGNSS.getProtocolVersionHigh());
      Serial.print(F("."));
      Serial.println(myGNSS.getProtocolVersionLow());
      
      Serial.print(F("MOD: "));
      Serial.println(myGNSS.getModuleName());    
  }
  else
    Serial.println(F("Error: could not read module info!"));
}

void loop()
{
  //Do nothing
}
