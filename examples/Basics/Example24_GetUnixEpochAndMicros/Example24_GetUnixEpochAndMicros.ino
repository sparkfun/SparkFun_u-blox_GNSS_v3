/*
  Getting Unix Epoch Time and micros using u-blox commands
  By: UT2UH
  Date: March 31th, 2021
  License: MIT. See license file for more information.

  This example shows how to query a u-blox module for the current time and date as Unix Epoch uint32_t type to avoid time.h dependency.
  We also turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic dramatically.

  Note: this example works best on modules like the ZED_F9P.

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
  
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // Uncomment the next line if you need to completely reset your module
  //myGNSS.factoryDefault(); delay(5000); // Reset everything and wait while the module restarts

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGNSS.saveConfiguration();        //Optional: Save the current settings to flash and BBR

  Serial.println(F("Compare Unix Epoch given with reference one from https://www.epochconverter.com/"));

}

void loop()
{
  //Query module.
  //The module only responds when a new position is available
  if (myGNSS.getPVT())
  {
    // getUnixEpoch marks the PVT data as stale so you will get Unix time and PVT time on alternate seconds

    uint32_t us;  //microseconds returned by getUnixEpoch()
    uint32_t epoch = myGNSS.getUnixEpoch();
    Serial.print(F("Unix Epoch rounded: "));
    Serial.print(epoch, DEC);    
    epoch = myGNSS.getUnixEpoch(us);
    Serial.print(F("  Exact Unix Epoch: "));
    Serial.print(epoch, DEC);
    Serial.print(F("  micros: "));
    Serial.println(us, DEC);

    Serial.print(myGNSS.getYear());
    Serial.print(F("-"));
    Serial.print(myGNSS.getMonth());
    Serial.print(F("-"));
    Serial.print(myGNSS.getDay());
    Serial.print(F(" "));
    Serial.print(myGNSS.getHour());
    Serial.print(F(":"));
    Serial.print(myGNSS.getMinute());
    Serial.print(F(":"));
    Serial.print(myGNSS.getSecond());
    
    Serial.print(F("  Time is "));
    if (myGNSS.getTimeFullyResolved() == false)
    {
      Serial.print(F("not fully resolved but "));
    } else {
      Serial.print(F("fully resolved and "));
    }
    if (myGNSS.getTimeValid() == false)
    {
      Serial.print(F("not "));
    }
    Serial.print(F("valid "));
    if (myGNSS.getConfirmedTime() == false)
    {
      Serial.print(F("but not "));
    } else {
      Serial.print(F("and "));
    }
    Serial.print(F("confirmed"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F("  SIV: "));
    Serial.println(SIV);
  }
}
