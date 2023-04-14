/*
  Turn on/off various NMEA sentences.
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information.

  This example shows how to turn on/off the NMEA sentences being output
  over UART1. We use the I2C interface on the u-blox module for configuration
  but you won't see any output from this sketch. You'll need to hook up
  a Serial Basic or other USB to Serial device to UART1 on your u-blox module
  to see the output.

  This example turns off all sentences except for the GPGGA and GPVTG sentences.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
  Hookup a Serial Basic (https://www.sparkfun.com/products/15096) to UART1 on the u-blox module. Open a terminal at 57600bps
  and see GPGGA and GPVTG sentences.
*/
#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

unsigned long lastGNSSsend = 0;

void setup()
{
  Serial.begin(115200); // Serial debug output over USB visible from Arduino IDE
  Serial.println("Example showing how to enable/disable certain NMEA sentences");

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  //Disable or enable various NMEA sentences over the UART1 interface
  myGNSS.newCfgValset(VAL_LAYER_RAM); // Use cfgValset to disable / enable individual NMEA messages. Change the configuration in the RAM layer only (don't save to BBR)
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1, 0); //Several of these are on by default so let's disable them
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_UART1, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1, 1); //Only leaving GGA & VTG enabled at current navigation rate
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1, 1);
  if (myGNSS.sendCfgValset()) // Send the configuration VALSET
    Serial.println(F("NMEA messages were configured successfully"));
  else
    Serial.println(F("NMEA message configuration failed!"));

  myGNSS.setUART1Output(COM_TYPE_NMEA); //Turn off UBX and RTCM sentences on the UART1 interface

  myGNSS.setSerialRate(57600, COM_PORT_UART1, VAL_LAYER_RAM); //Set UART1 to 57600bps. Change the configuration in the RAM layer only (don't save to BBR)

  //myGNSS.saveConfiguration(); //Optional: Save these settings to NVM

  Serial.println(F("Messages configured. NMEA now being output over the UART1 port on the u-blox module at 57600bps."));
}

void loop()
{
  if (millis() - lastGNSSsend > 200)
  {
    myGNSS.checkUblox(); //See if new data is available, but we don't want to get NMEA here. Go check UART1.
    lastGNSSsend = millis();
  }
}
