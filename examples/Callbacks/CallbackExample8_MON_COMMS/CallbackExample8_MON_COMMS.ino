/*
  Configuring the GNSS to automatically send MON COMMS reports over I2C and display them using a callback
  By: Paul Clark
  SparkFun Electronics
  Date: Novenber 4th, 2024
  License: MIT. See license file for more information.

  This example shows how to configure the u-blox GNSS to send MON COMMS reports automatically
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

// Callback: newMONCOMMS will be called when new MON COMMS data arrives
// See u-blox_structs.h for the full definition of UBX_MON_COMMS_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoMONCOMMScallback
//        /             _____  This _must_ be UBX_MON_COMMS_data_t
//        |            /                _____ You can use any name you like for the struct
//        |            |               /
//        |            |               |
void newMONCOMMS(UBX_MON_COMMS_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("New MON COMMS data received. It contains data for "));
  Serial.print(ubxDataStruct->header.nPorts);
  if (ubxDataStruct->header.nPorts == 1)
    Serial.println(F(" port."));
  else
    Serial.println(F(" ports."));

  // Mimic the data shown in u-center
  for (uint8_t port = 0; port < ubxDataStruct->header.nPorts; port++) // For each port
  {
    // Check the port ID is valid (skip 0x0101 and 0x0200)
    bool validPort = false;
    switch (ubxDataStruct->port[port].portId)
    {
      case COM_PORT_ID_I2C:
      case COM_PORT_ID_UART1:
      case COM_PORT_ID_UART2:
      case COM_PORT_ID_USB:
      case COM_PORT_ID_SPI:
        validPort = true;
      break;      
      default:
        //Serial.printf("Unknown / reserved portId 0x%04X\r\n", ubxDataStruct->port[port].portId);
      break;      
    }

    if (validPort)
    {
      switch (ubxDataStruct->port[port].portId) // Print the port ID
      {
        case COM_PORT_ID_I2C:
          Serial.print(F("I2C     "));
        break;
        case COM_PORT_ID_UART1:
          Serial.print(F("UART1   "));
        break;
        case COM_PORT_ID_UART2:
          Serial.print(F("UART2   "));
        break;
        case COM_PORT_ID_USB:
          Serial.print(F("USB     "));
        break;
        case COM_PORT_ID_SPI:
          Serial.print(F("SPI     "));
        break;
      }

      Serial.print(": txBytes ");
      String txBytes = String(ubxDataStruct->port[port].txBytes);
      Serial.print(txBytes);
      for (int i = 0; i < 10 - txBytes.length(); i++)
        Serial.print(" ");
      
      Serial.print(" : rxBytes ");
      String rxBytes = String(ubxDataStruct->port[port].rxBytes);
      Serial.print(rxBytes);
      for (int i = 0; i < 10 - rxBytes.length(); i++)
        Serial.print(" ");

      for (int i = 0; i < 4; i++)
      {
        if (ubxDataStruct->header.protIds[i] < 0xFF)
        {
          switch (ubxDataStruct->header.protIds[i])
          {
            case 0:
              Serial.print(F(" : UBX     "));
            break;
            case 1:
              Serial.print(F(" : NMEA    "));
            break;
            case 2:
              Serial.print(F(" : RTCM2   "));
            break;
            case 5:
              Serial.print(F(" : RTCM3   "));
            break;
            case 6:
              Serial.print(F(" : SPARTN  "));
            break;
            default:
              Serial.print(F(" : UNKNOWN "));
            break;
          }
          String msgs = String(ubxDataStruct->port[port].msgs[i]);
          Serial.print(msgs);
          for (int i = 0; i < 5 - msgs.length(); i++)
            Serial.print(" ");
        }
      }
      
      Serial.print(" : skipped ");
      Serial.print(ubxDataStruct->port[port].skipped);
      
      Serial.println();
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

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_NMEA | COM_TYPE_UBX); //Set the I2C port to output NMEA and UBX
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(1); //Produce one solution per second

  // Just to prove we can, poll the MON COMMS data manually
  Serial.println("Polling MON COMMS data:");
  UBX_MON_COMMS_data_t portInfo;
  if (myGNSS.getCommsPortInfo(&portInfo))
    newMONCOMMS(&portInfo); // Call the callback manually to print the data
  else
    Serial.println("getCommsPortInfo failed!");

  // Now enable automatic (periodic) MON COMMS messages with callback to newMONCOMMS
  myGNSS.setAutoMONCOMMScallbackPtr(&newMONCOMMS);
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
}
