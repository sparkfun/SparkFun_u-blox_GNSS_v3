/*
  Set Dynamic Model
  By: Paul Clark (PaulZC)
  Date: April 22nd, 2020

  Based extensively on Example3_GetPosition
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information.

  This example shows how to change the u-blox module's dynamic platform model and then
  query its lat/long/altitude. We also turn off the NMEA output on the I2C port.
  This decreases the amount of I2C traffic dramatically.

  Possible values for the dynamic model are: PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE,
  SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE, MOWER, ESCOOTER

  Note: Long/lat are large numbers because they are * 10^7. To convert lat/long
  to something google maps understands simply divide the numbers by 10,000,000. We
  do this so that we don't have to use floating point numbers.

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
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // If we are going to change the dynamic platform model, let's do it here.
  // Possible values are:
  // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE, MOWER, ESCOOTER

  if (myGNSS.setDynamicModel(DYN_MODEL_PORTABLE, VAL_LAYER_RAM_BBR) == false) // Set the dynamic model to PORTABLE in both RAM and BBR
  {
    Serial.println(F("*** Warning: setDynamicModel failed ***"));
  }
  else
  {
    Serial.println(F("Dynamic platform model changed successfully!"));
  }

  // Let's read the new dynamic model to see if it worked
  uint8_t newDynamicModel = myGNSS.getDynamicModel(VAL_LAYER_RAM); // Read the value from the RAM layer. (We could choose DEFAULT instead.)
  if (newDynamicModel == DYN_MODEL_UNKNOWN)
  {
    Serial.println(F("*** Warning: getDynamicModel failed ***"));
  }
  else
  {
    Serial.print(F("The new dynamic model is: "));
    Serial.println(newDynamicModel);
  }

  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF); //Uncomment this line to save only the NAV settings to flash and BBR
}

void loop()
{
  //The module only responds when a new position is available
  if (myGNSS.getPVT())
  {
    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.println();
  }
}
