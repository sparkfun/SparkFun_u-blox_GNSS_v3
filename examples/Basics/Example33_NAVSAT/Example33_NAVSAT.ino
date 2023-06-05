/*
  Polling NAV SAT
  By: Paul Clark
  SparkFun Electronics
  Date: June 5th, 2023
  License: MIT. See license file for more information.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  myGNSS.setPacketCfgPayloadSize(UBX_NAV_SAT_MAX_LEN); // Allocate extra RAM to store the full NAV SAT data

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(1); //Produce one solution per second
}

void loop()
{
  if (myGNSS.getNAVSAT()) // Poll the latest NAV SAT data
  {
    Serial.println();

    // See u-blox_structs.h for the full definition of UBX_NAV_SAT_data_t
    Serial.print(F("New NAV SAT data received. It contains data for "));
    Serial.print(myGNSS.packetUBXNAVSAT->data.header.numSvs);
    if (myGNSS.packetUBXNAVSAT->data.header.numSvs == 1)
        Serial.println(F(" SV."));
    else
        Serial.println(F(" SVs."));

    // Just for giggles, print the signal strength for each SV as a barchart
    for (uint16_t block = 0; block < myGNSS.packetUBXNAVSAT->data.header.numSvs; block++) // For each SV
    {
        switch (myGNSS.packetUBXNAVSAT->data.blocks[block].gnssId) // Print the GNSS ID
        {
        case 0:
            Serial.print(F("GPS     "));
        break;
        case 1:
            Serial.print(F("SBAS    "));
        break;
        case 2:
            Serial.print(F("Galileo "));
        break;
        case 3:
            Serial.print(F("BeiDou  "));
        break;
        case 4:
            Serial.print(F("IMES    "));
        break;
        case 5:
            Serial.print(F("QZSS    "));
        break;
        case 6:
            Serial.print(F("GLONASS "));
        break;
        default:
            Serial.print(F("UNKNOWN "));
        break;      
        }
        
        Serial.print(myGNSS.packetUBXNAVSAT->data.blocks[block].svId); // Print the SV ID
        
        if (myGNSS.packetUBXNAVSAT->data.blocks[block].svId < 10) Serial.print(F("   "));
        else if (myGNSS.packetUBXNAVSAT->data.blocks[block].svId < 100) Serial.print(F("  "));
        else Serial.print(F(" "));

        // Print the signal strength as a bar chart
        for (uint8_t cno = 0; cno < myGNSS.packetUBXNAVSAT->data.blocks[block].cno; cno++)
        Serial.print(F("="));

        Serial.println();
    }
  }
}
