/*
  Reading Position, Velocity and Time (PVT) via UBX binary commands
  By: Paul Clark
  SparkFun Electronics
  Date: December 21st, 2022
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to query a u-blox module for its position, velocity and time (PVT) data using UART2.

  UART2 is only available on modules like the ZED-F9P/R/T/K etc.. It is not available on the MAX-M10.

  Important note:
  By default, the UBX protocol is enabled for INPUT on UART2, but not for OUTPUT.
  The code needs to enable UBX output for the begin to succeed.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Hook up the TX, RX and GND pins, plus 3V3 or 5V depending on your needs
  Connect: GNSS TX to microcontroller RX; GNSS RX to microcontroller TX
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

SFE_UBLOX_GNSS_SERIAL myGNSS;

#define mySerial Serial1 // Use Serial1 to connect to the GNSS module. Change this if required

void setup()
{
  Serial.begin(115200);
  delay(1000); 
  Serial.println("SparkFun u-blox Example");

  mySerial.begin(38400); // u-blox F9 and M10 modules default to 38400 baud. Change this if required

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  myGNSS.connectedToUART2(); // This tells the library we are connecting to UART2 so it uses the correct configuration keys

  while (myGNSS.begin(mySerial) == false) //Connect to the u-blox module using mySerial (defined above)
  {
    Serial.println(F("u-blox GNSS not detected"));
    
    Serial.println(F("Attempting to enable the UBX protocol for output"));
    
    myGNSS.setUART2Output(COM_TYPE_UBX); // Enable UBX output. Disable NMEA output
    
    Serial.println(F("Retrying..."));
    delay (1000);
  }

  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR
}

void loop()
{
  // Request (poll) the position, velocity and time (PVT) information.
  // The module only responds when a new position is available. Default is once per second.
  // getPVT() returns true when new data is received.
  if (myGNSS.getPVT() == true)
  {
    int32_t latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    int32_t longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.println();
  }
}
