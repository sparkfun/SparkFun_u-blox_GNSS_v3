/*
  Use the NEO-D9S L-Band receiver to provide corrections as SPARTN data over Serial
  By: SparkFun Electronics / Paul Clark
  Based on original code by: u-blox AG / Michael Ammann
  v3 updates: Decembe 22nd, 2022
  License: MIT. See license file for more information.

  This example shows how to obtain SPARTN correction data from a NEO-D9S L-Band receiver and push it over Serial (UART).
  This example parses (extracts) SPARTN packets from the L-Band data stream and validates them before pushing.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  NEO-D9S:      https://www.sparkfun.com/products/19390
  Combo Board:  https://www.sparkfun.com/products/20167

  Hardware Connections:
  Use a Qwiic cable to connect the NEO-D9S to your board
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <driver/uart.h>      //Required for uart_set_rx_full_threshold() on cores <v2.0.5
HardwareSerial serialGNSS(2); // TX on 17, RX on 16

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myLBand; // NEO-D9S

const uint32_t myLBandFreq = 1556290000; // Uncomment this line to use the US SPARTN 1.8 service
//const uint32_t myLBandFreq = 1545260000; // Uncomment this line to use the EU SPARTN 1.8 service

#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!")) // Convert uint8_t into OK/ERROR

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: pushRXMPMP will be called when new PMP data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_PMP_message_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setRXMPMPmessageCallbackPtr
//        /               _____  This _must_ be UBX_RXM_PMP_message_data_t
//        |              /              _____ You can use any name you like for the struct
//        |              |             /
//        |              |             |
void pushRXMPMP(UBX_RXM_PMP_message_data_t *pmpData)
{
  //Extract the raw message payload length
  uint16_t payloadLen = ((uint16_t)pmpData->lengthMSB << 8) | (uint16_t)pmpData->lengthLSB;

  uint16_t numBytesUserData = pmpData->payload[2] | ((uint16_t)pmpData->payload[3] << 8);
  uint16_t fecBits = pmpData->payload[20] | ((uint16_t)pmpData->payload[21] << 8);
  float ebno = (float)pmpData->payload[22] / 8;
  
  Serial.print(F("New RXM-PMP data received. userData: "));
  Serial.print(numBytesUserData);
  Serial.print(F(" Bytes. fecBits: "));
  Serial.print(fecBits);
  Serial.print(F(". ebno (dB): "));
  Serial.print(ebno);
  Serial.println(F("."));

  //Parse the SPARTN data stream contained in the userData
  for (uint16_t i = 0; i < numBytesUserData; i++)
  {
    bool valid = false;
    uint16_t len;
    uint8_t *spartn = myLBand.parseSPARTN(pmpData->payload[24 + i], valid, len);

    if (valid)
    {
      Serial.print(F("Valid SPARTN data parsed. Pushing "));
      Serial.print(len);
      Serial.println(F(" bytes to Serial..."));
      
      serialGNSS.write(spartn, len); // Push the SPARTN data to the module using Serial
    }
  }

  (void)payloadLen;
  //serialGNSS.write(&pmpData->sync1, (size_t)payloadLen + 6); // Push the sync chars, class, ID, length and payload
  //serialGNSS.write(&pmpData->checksumA, (size_t)2); // Push the checksum bytes

}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println(F("NEO-D9S SPARTN Corrections"));

  serialGNSS.begin(38400); // UART2 on pins 16/17.
                                           
  Wire.begin(); //Start I2C

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin and configure the NEO-D9S L-Band receiver

  //myLBand.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myLBand.begin(Wire, 0x43) == false) //Connect to the u-blox NEO-D9S using Wire port. The D9S default I2C address is 0x43 (not 0x42)
  {
    Serial.println(F("u-blox NEO-D9S not detected at default I2C address. Please check wiring."));
    delay(2000);
  }
  Serial.println(F("u-blox NEO-D9S connected"));

  myLBand.newCfgValset(); // Create a new Configuration Interface message - this defaults to VAL_LAYER_RAM_BBR (change in RAM and BBR)
  myLBand.addCfgValset(UBLOX_CFG_PMP_CENTER_FREQUENCY,     myLBandFreq); // Default 1539812500 Hz
  myLBand.addCfgValset(UBLOX_CFG_PMP_SEARCH_WINDOW,        2200);        // Default 2200 Hz
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_SERVICE_ID,       0);           // Default 1 
  myLBand.addCfgValset(UBLOX_CFG_PMP_SERVICE_ID,           21845);       // Default 50821
  myLBand.addCfgValset(UBLOX_CFG_PMP_DATA_RATE,            2400);        // Default 2400 bps
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_DESCRAMBLER,      1);           // Default 1
  myLBand.addCfgValset(UBLOX_CFG_PMP_DESCRAMBLER_INIT,     26969);       // Default 23560
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_PRESCRAMBLING,    0);           // Default 0
  myLBand.addCfgValset(UBLOX_CFG_PMP_UNIQUE_WORD,          16238547128276412563ull); 
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C,   1);           // Ensure UBX-RXM-PMP is enabled on the I2C port 
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART1, 1);           // Output UBX-RXM-PMP on UART1
  myLBand.addCfgValset(UBLOX_CFG_UART2OUTPROT_UBX,         1);           // Enable UBX output on UART2
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2, 1);           // Output UBX-RXM-PMP on UART2
  myLBand.addCfgValset(UBLOX_CFG_UART1_BAUDRATE,           38400);       // match baudrate with ZED default
  myLBand.addCfgValset(UBLOX_CFG_UART2_BAUDRATE,           38400);       // match baudrate with ZED default
  bool ok = myLBand.sendCfgValset(); // Apply the settings
  
  Serial.print(F("L-Band: configuration "));
  Serial.println(OK(ok));

  myLBand.softwareResetGNSSOnly(); // Do a restart

  myLBand.setRXMPMPmessageCallbackPtr(&pushRXMPMP); // Call pushRXMPMP when new PMP data arrives. Push it to the GNSS

}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  myLBand.checkUblox(); // Check for the arrival of new PMP data and process it.
  myLBand.checkCallbacks(); // Check if any LBand callbacks are waiting to be processed.
}
