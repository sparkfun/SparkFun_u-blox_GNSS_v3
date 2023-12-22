/*
  Configuring the NEO-F10N GNSS to send RXM RAWX reports over I2C and display them using a callback
  By: Paul Clark
  SparkFun Electronics
  Date: November 24th, 2023
  License: MIT. See license file for more information.

  This example shows how to configure the u-blox NEO-F10N GNSS to send RXM RAWX reports automatically
  and access the data via a callback. It also demonstrates how to mark the L5 signals as healthy.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  NEO-F10N: https://www.sparkfun.com/products/24114

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

// Callback: newRAWX will be called when new RXM RAWX data arrives
// See u-blox_structs.h for the full definition of UBX_RXMRAWX_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRXMRAWXcallback
//        /             _____  This _must_ be UBX_RXM_RAWX_data_t
//        |            /                _____ You can use any name you like for the struct
//        |            |               /
//        |            |               |
void newRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("New RAWX data received. It contains "));
  Serial.print(ubxDataStruct->header.numMeas); // Print numMeas (Number of measurements / blocks)
  Serial.println(F(" data blocks:"));

  for (uint8_t block = 0; block < ubxDataStruct->header.numMeas; block++) // For each block
  {
    char SV[14 + 4 + 4 + 256 + 1]; // Allocate space for sigId plus svId plus cno plus bars plus NULL
    switch (ubxDataStruct->blocks[block].gnssId)
    {
      case 0:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "GPS L1 C/A    ");
              break;
            case 3:
              sprintf(SV, "GPS L2 CL     ");
              break;
            case 4:
              sprintf(SV, "GPS L2 CM     ");
              break;
            case 6:
              sprintf(SV, "GPS L5 I      ");
              break;
            case 7:
              sprintf(SV, "GPS L5 Q      ");
              break;
            default:
              sprintf(SV, "GPS Unknown   ");
              break;
          }
        }
        break;
      case 1:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "SBAS L1 C/A   ");
              break;
            default:
              sprintf(SV, "SBAS Unknown  ");
              break;
          }
        }
        break;
      case 2:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "Galileo E1 C  ");
              break;
            case 1:
              sprintf(SV, "Galileo E1 B  ");
              break;
            case 3:
              sprintf(SV, "Galileo E5 aI ");
              break;
            case 4:
              sprintf(SV, "Galileo E5 aQ ");
              break;
            case 5:
              sprintf(SV, "Galileo E5 bI ");
              break;
            case 6:
              sprintf(SV, "Galileo E5 bQ ");
              break;
            default:
              sprintf(SV, "GAL Unknown   ");
              break;
          }
        }
        break;
      case 3:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "BeiDou B1I D1 ");
              break;
            case 1:
              sprintf(SV, "BeiDou B1I D2 ");
              break;
            case 2:
              sprintf(SV, "BeiDou B2I D1 ");
              break;
            case 3:
              sprintf(SV, "BeiDou B2I D2 ");
              break;
            case 5:
              sprintf(SV, "BeiDou B1 Cp  ");
              break;
            case 6:
              sprintf(SV, "BeiDou B2 Cd  ");
              break;
            case 7:
              sprintf(SV, "BeiDou B2 ap  ");
              break;
            case 8:
              sprintf(SV, "BeiDou B2 ad  ");
              break;
            default:
              sprintf(SV, "BDS Unknown   ");
              break;
          }
        }
        break;
      case 5:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "QZSS L1 C/A   ");
              break;
            case 1:
              sprintf(SV, "QZSS L1S      ");
              break;
            case 4:
              sprintf(SV, "QZSS L2 CM    ");
              break;
            case 5:
              sprintf(SV, "QZSS L2 CL    ");
              break;
            case 8:
              sprintf(SV, "QZSS L5 I     ");
              break;
            case 9:
              sprintf(SV, "QZSS L5 Q     ");
              break;
            default:
              sprintf(SV, "QZSS Unknown  ");
              break;
          }
        }
        break;
      case 6:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "GLONASS L1 OF ");
              break;
            case 2:
              sprintf(SV, "GLONASS L2 OF ");
              break;
            default:
              sprintf(SV, "GLO Unknown   ");
              break;
          }
        }
        break;
      case 7:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "NavIC L5 A    ");
              break;
            default:
              sprintf(SV, "NavIC Unknown ");
              break;
          }
        }
        break;
      default:
        sprintf(SV, "Unknown       ");
        break;
    }

  if (ubxDataStruct->blocks[block].svId < 10) sprintf(&SV[14], "%d   ", ubxDataStruct->blocks[block].svId); // Align the svId
  else if (ubxDataStruct->blocks[block].svId < 100) sprintf(&SV[14], "%d  ", ubxDataStruct->blocks[block].svId); // Align the svId
  else sprintf(&SV[14], "%d ", ubxDataStruct->blocks[block].svId); // Align the svId

  if (ubxDataStruct->blocks[block].cno < 10) sprintf(&SV[18], "  %d ", ubxDataStruct->blocks[block].cno); // Align the cno
  else if (ubxDataStruct->blocks[block].cno < 100) sprintf(&SV[18], " %d ", ubxDataStruct->blocks[block].cno); // Align the cno
  else sprintf(&SV[18], "%d ", ubxDataStruct->blocks[block].cno); // Align the cno

  // Print cno as a bar chart
  uint8_t cno;
  for (cno = 0; cno <= ubxDataStruct->blocks[block].cno; cno++)
    SV[22 + cno] = '=';
  SV[22 + cno] = 0; // NULL

  Serial.println(SV);
  }
}

void setup()
{
  delay(1000);

  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setMeasurementRate(5000); //Produce one solution every five seconds (RAWX produces a _lot_ of data!)

  myGNSS.setVal8(UBLOX_CFG_SIGNAL_GPS_L5_ENA, 1); // Make sure the GPS L5 band is enabled (needed on the NEO-F9P)

  myGNSS.setGPSL5HealthOverride(true); // Mark L5 signals as healthy - store in RAM and BBR

  myGNSS.setLNAMode(SFE_UBLOX_LNA_MODE_NORMAL); // Set the LNA gain to normal (full). Other options: LOWGAIN, BYPASS

  myGNSS.softwareResetGNSSOnly(); // Restart the GNSS to apply the L5 health override

  myGNSS.setAutoRXMRAWXcallbackPtr(&newRAWX); // Enable automatic RXM RAWX messages with callback to newRAWX
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
}
