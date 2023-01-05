/*
  By: Nathan Seidle
  SparkFun Electronics
  Date: August, 2021
  License: MIT. See license file for more information.

  This example configures the AutoAlignment option for the IMU.
  The ZED-F9R Integration guide recommends enabling Auto Alignment once
  the device has been attached to the vehicle's frame.
  Enabling auto-alignment will cause the the sensor fusion status
  to begin initialization. After driving around a few turns, the sensors
  should enter 'Calibrated' state. See example 1 for fusion state or
  monitor UBX-ESF-STATUS.

  As of writing the ZED-F9R is using HPS v1.30 firmware. Please update using u-center if necessary.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9R: https://www.sparkfun.com/products/16344
  ZED-F9R pHat: https://www.sparkfun.com/products/16475

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a Redboard Qwiic
  If you don't have a platform with a Qwiic connection use the
  SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/17912)
  Open the serial monitor at 115200 baud to see the output

*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("Warning! u-blox GNSS did not begin correctly."));
  }

  bool esfAutoAlignment = myGNSS.getESFAutoAlignment();
  Serial.print(F("esfAutoAlignment: "));
  if (esfAutoAlignment == true)
    Serial.println(F("True"));
  else
    Serial.println(F("False"));

  myGNSS.setESFAutoAlignment(true); //Enable Automatic IMU-mount Alignment

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

void loop()
{
  // ESF data is produced at the navigation rate, so by default we'll get fresh data once per second
  if (myGNSS.getEsfInfo()) // Poll new ESF STATUS data
  {
    Serial.print(F("Fusion Mode: "));
    Serial.print(myGNSS.packetUBXESFSTATUS->data.fusionMode);
    if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 0)
      Serial.println(F("  Sensor is initializing..."));
    else if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 1)
      Serial.println(F("  Sensor is calibrated!"));
    else if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 2)
      Serial.println(F("  Sensor fusion is suspended!"));
    else if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 3)
      Serial.println(F("  Sensor fusion is disabled!"));
  }
}
