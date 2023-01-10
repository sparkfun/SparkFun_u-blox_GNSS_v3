/*
  Send UBX binary commands to enable RTCM sentences on u-blox ZED-F9P module
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 9th, 2019
  License: MIT. See license file for more information.

  u-blox changed how to configure their modules in 2019. As of version 23 of the UBX protocol the
  UBX-CFG commands are deprecated; they still work, they just recommend using VALSET, VALGET, and VALDEL
  commands instead. This example shows how to use this new command structure.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard Qwiic or BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("u-blox getVal example");

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  //myGNSS.enableDebugging(); //Enable debug messages over Serial (default)
  //myGNSS.enableDebugging(SerialUSB); //Enable debug messages over Serial USB

  bool setValueSuccess = true;
  // If we AND (&=) the result of each setVal into setValueSuccess, setValueSuccess will be false if any one setVal fails

  //These key values are hard coded and defined in u-blox_config_keys.h.
  //You can obtain them from the ZED-F9P interface description doc
  //or from u-center's Messages->CFG->VALSET window. Keys must be 32-bit.
  
  setValueSuccess &= myGNSS.setVal8(UBLOX_CFG_NMEA_HIGHPREC, 1); //Enable high precision NMEA. UBLOX_CFG_NMEA_HIGHPREC is L (bool) but we use 8-bits (uint8_t)
  
  //setValueSuccess &= myGNSS.setVal16(UBLOX_CFG_RATE_MEAS, 100); //Set measurement rate to 100ms (10Hz update rate). UBLOX_CFG_RATE_MEAS is 16-bit U2 (uint16_t)
  setValueSuccess &= myGNSS.setVal16(UBLOX_CFG_RATE_MEAS, 1000); //Set measurement rate to 1000ms (1Hz update rate). UBLOX_CFG_RATE_MEAS is 16-bit U2 (uint16_t)

  //setValueSuccess &= myGNSS.setVal8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1); //Set output rate of msg 1005 over the I2C port to once per second

  if (setValueSuccess == true)
  {
    Serial.println("VALSET was successful");
  }
  else
    Serial.println("VALSET failed");
}

void loop()
{
}
