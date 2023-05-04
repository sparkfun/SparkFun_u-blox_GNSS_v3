/*
  Get the RTCM 1005 sentence using getLatestRTCM1005
  By: Paul Clark
  SparkFun Electronics
  Date: May 4th, 2023
  License: MIT. See license file for more information.

  This example shows how to turn on/off the RTCM sentences being output over I2C.
  It then demonstrates how to use the new getLatestRTCM1005 function to retrieve the latest RTCM 1005 message.
  getLatestRTCM1005 returns immediately - it is not blocking.
  It returns:
    0 if no data is available
    1 if the data is valid but is stale (you have read it before)
    2 if the data is valid and fresh

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

void setup()
{

  Serial.begin(115200);
  Serial.println(F("SparkFun u-blox GNSS Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  while (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
    delay(1000);
  }

  //Disable or enable various RTCM sentences over the I2C interface
  myGNSS.setI2COutput(COM_TYPE_NMEA | COM_TYPE_UBX | COM_TYPE_RTCM3); // Turn on UBX, NMEA and RTCM sentences on I2C
  myGNSS.newCfgValset(VAL_LAYER_RAM_BBR); // Use cfgValset to disable / enable individual RTCM messages
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1); // Enable RTCM 1005 at current navigation rate
  if (myGNSS.sendCfgValset()) // Send the configuration VALSET
    Serial.println(F("RTCM messages were configured successfully"));
  else
    Serial.println(F("RTCM message configuration failed!"));

  //myGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save only the ioPort and message settings to NVM

  //myGNSS.setRTCMOutputPort(Serial); // Uncomment this line to echo all RTCM data to Serial for debugging

  if (sizeof(double) != 8) // Check double is 64-bit
    Serial.println(F("double is not 64-bit. ECEF resolution may be limited!"));
}

void loop()
{
  // getLatestRTCM1005 calls checkUblox for us. We don't need to do it here

  RTCM_1005_data_t data; // Storage for the RTCM 1005 data
  uint8_t result = myGNSS.getLatestRTCM1005(&data); // Get the latest RTCM 1005 data (if any)
  
  if (result == 0)
  {
    Serial.println(F("No RTCM 1005 data available"));
  }
  else if (result == 1)
  {
    Serial.println(F("RTCM 1005 data is available but is stale"));
  }
  else // if (result == 2)
  {
    double x = data.AntennaReferencePointECEFX;
    x /= 10000.0; // Convert to m
    double y = data.AntennaReferencePointECEFY;
    y /= 10000.0; // Convert to m
    double z = data.AntennaReferencePointECEFZ;
    z /= 10000.0; // Convert to m

    Serial.print(F("Latest RTCM 1005: ARP ECEF-X: "));
    Serial.print(x, 4); // 4 decimal places
    Serial.print(F("  Y: "));
    Serial.print(y, 4); // 4 decimal places
    Serial.print(F("  Z: "));
    Serial.println(z, 4); // 4 decimal places
  }

  delay(250);
}
