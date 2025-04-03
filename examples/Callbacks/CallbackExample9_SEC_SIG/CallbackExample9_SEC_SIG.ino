/*
  Configuring the GNSS to automatically send SEC SIG reports over I2C and display them using a callback
  By: Paul Clark
  SparkFun Electronics
  Date: April 3rd, 2025
  License: MIT. See license file for more information.

  This example shows how to configure the u-blox GNSS to send SEC SIG reports automatically
  and access the data via a callback. No more polling!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

// Callback: newSECSIG will be called when new SEC SIG data arrives
// See u-blox_structs.h for the full definition of UBX_SEC_SIG_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoSECSIGcallback
//        /             _____  This _must_ be UBX_SEC_SIG_data_t
//        |            /                _____ You can use any name you like for the struct
//        |            |               /
//        |            |               |
void newSECSIG(UBX_SEC_SIG_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("New SEC SIG data received. It contains version "));
  Serial.print(ubxDataStruct->version);
  Serial.println(F(" data."));

  if (ubxDataStruct->version == 1)
  {
    Serial.print(F("Jamming detection is "));
    bool jamDetEnabled = (bool)ubxDataStruct->versions.version1.jamFlags.bits.jamDetEnabled;
    Serial.println(jamDetEnabled ? "enabled" : "disabled");
    if (jamDetEnabled)
    {
      Serial.print(F("Jamming state: "));
      switch (ubxDataStruct->versions.version1.jamFlags.bits.jammingState)
      {
        case 1:
          Serial.println(F("no jamming indicated"));
          break;
        case 2:
          Serial.println(F("warning (jamming indicated but fix OK)"));
          break;
        case 3:
          Serial.println(F("critical (jamming indicated and no fix)"));
          break;
        case 0:
        default:
          Serial.println(F("unknown"));
          break;
      }
    }
    Serial.print(F("Spoofing detection is "));
    bool spfDetEnabled = (bool)ubxDataStruct->versions.version1.spfFlags.bits.spfDetEnabled;
    Serial.println(spfDetEnabled ? "enabled" : "disabled");
    if (spfDetEnabled)
    {
      Serial.print(F("Spoofing state: "));
      switch (ubxDataStruct->versions.version1.spfFlags.bits.spoofingState)
      {
        case 1:
          Serial.println(F("no spoofing indicated"));
          break;
        case 2:
          Serial.println(F("spoofing indicated"));
          break;
        case 3:
          Serial.println(F("spoofing affirmed"));
          break;
        case 0:
        default:
          Serial.println(F("unknown"));
          break;
      }
    }
  }

  else if (ubxDataStruct->version == 2)
  {
    Serial.print(F("Jamming detection is "));
    bool jamDetEnabled = (bool)ubxDataStruct->versions.version2.sigSecFlags.bits.jamDetEnabled;
    Serial.println(jamDetEnabled ? "enabled" : "disabled");
    if (jamDetEnabled)
    {
      Serial.print(F("Jamming state: "));
      switch (ubxDataStruct->versions.version2.sigSecFlags.bits.jamState)
      {
        case 1:
          Serial.println(F("no jamming indicated"));
          break;
        case 2:
          Serial.println(F("warning (jamming indicated)"));
          break;
        case 0:
        default:
          Serial.println(F("unknown"));
          break;
      }
    }
    Serial.print(F("Spoofing detection is "));
    bool spfDetEnabled = (bool)ubxDataStruct->versions.version2.sigSecFlags.bits.spfDetEnabled;
    Serial.println(spfDetEnabled ? "enabled" : "disabled");
    if (spfDetEnabled)
    {
      Serial.print(F("Spoofing state: "));
      switch (ubxDataStruct->versions.version2.sigSecFlags.bits.spfState)
      {
        case 1:
          Serial.println(F("no spoofing indicated"));
          break;
        case 2:
          Serial.println(F("spoofing indicated"));
          break;
        case 3:
          Serial.println(F("spoofing affirmed"));
          break;
        case 0:
        default:
          Serial.println(F("unknown"));
          break;
      }
    }
    Serial.print(F("Number of jamming center frequencies: "));
    uint8_t jamNumCentFreqs = ubxDataStruct->versions.version2.jamNumCentFreqs;
    Serial.println(jamNumCentFreqs);
    if (jamNumCentFreqs > 0)
    {
      for (uint8_t i = 0; i < jamNumCentFreqs; i++)
      {
        Serial.print(F("Center frequency: "));
        Serial.print(ubxDataStruct->versions.version2.jamStateCentFreq[i].bits.centFreq);
        Serial.print(F(" kHz "));
        if (ubxDataStruct->versions.version2.jamStateCentFreq[i].bits.jammed)
          Serial.print("- jammed");
        Serial.println();
      }
    }
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  //myGNSS.enableDebugging(Serial, true); // Uncomment this line to enable only the major debug messages on Serial

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only. Stop the NMEA messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  // Just to prove we can, poll the SEC SIG data manually
  Serial.println("Polling SEC SIG data:");
  UBX_SEC_SIG_data_t secSig;
  if (myGNSS.getSECSIG(&secSig))
    newSECSIG(&secSig); // Call the callback manually to print the data
  else
    Serial.println("getSECSIG failed!");

  // Now enable automatic (periodic) SEC SIG messages with callback to newSECSIG
  myGNSS.setAutoSECSIGcallbackPtr(&newSECSIG);
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
}
