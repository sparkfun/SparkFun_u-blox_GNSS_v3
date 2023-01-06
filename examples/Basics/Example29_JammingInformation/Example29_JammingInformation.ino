/*
  Get the jamming state and indication
  By: Paul Clark
  SparkFun Electronics
  Date: March 2nd, 2022
  License: MIT. See license file for more information.

  This example shows how to enable the jamming / interference monitor and read the
  jamming state and information.

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
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println(F("SparkFun u-blox GNSS Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // Enable the jamming / interference monitor

  uint8_t itfmEnabled;

  if (myGNSS.getVal8(UBLOX_CFG_ITFM_ENABLE, &itfmEnabled)) // Check if interference detection is enabled
  {
    Serial.print(F("The jamming / interference monitor is "));
    if (itfmEnabled == 0) // Check if the monitor is already enabled
      Serial.print(F("not "));
    Serial.println(F("enabled"));

    if (itfmEnabled == 0) // Check if the monitor is already enabled
    {
      Serial.print(F("Enabling the jamming / interference monitor: "));
      if (myGNSS.setVal8(UBLOX_CFG_ITFM_ENABLE, 1, VAL_LAYER_RAM)) // Enable interference detection - in RAM only (not BBR)
        Serial.println(F("success"));
      else
        Serial.println(F("failed!"));
    }
  }
  else
    Serial.println(F("Error: could not read UBLOX_CFM_ITFM_ENABLE (interference detection enable)!"));

}

void loop()
{
  // Create storage to hold the hardware status
  // See the definition of UBX_MON_HW_data_t in u-blox_structs.h for more details
  UBX_MON_HW_data_t hwStatus;

  // Note: the MAX-M10S SPG 5.00 supports UBX-MON-HW but it returns 56 bytes of reserved data
  // The ZED-F9P will return 60 bytes of correctly-formatted data
  if (myGNSS.getHWstatus(&hwStatus)) // Read the hardware status
  {
    Serial.println(F("Hardware status (UBX_MON_HW):"));

    // Note: This flag is deprecated in protocol versions that support UBX-SEC-SIG (version 0x02);
    //       instead jammingState in UBX-SEC-SIG should be monitored.
    Serial.print(F("Jamming state: "));
    Serial.print(hwStatus.flags.bits.jammingState);
    if (hwStatus.flags.bits.jammingState == 0)
      Serial.println(F(" = unknown / disabled"));
    else if (hwStatus.flags.bits.jammingState == 1)
      Serial.println(F(" = ok"));
    else if (hwStatus.flags.bits.jammingState == 2)
      Serial.println(F(" = warning"));
    else // if (hwStatus.flags.bits.jammingState == 3)
      Serial.println(F(" = critical!"));

    Serial.print(F("Noise level: "));
    Serial.println(hwStatus.noisePerMS);
    
    Serial.print(F("AGC monitor: "));
    Serial.println(hwStatus.agcCnt);
    
    Serial.print(F("CW jamming indicator: "));
    Serial.println(hwStatus.jamInd);

    Serial.println();
  }

  // Create storage to hold the RF information from a ZED-F9
  // See the definition of UBX_MON_RF_data_t in u-blox_structs.h for more details
  UBX_MON_RF_data_t rfInformation;

  // Read the RF information from the ZED-F9n. Allow 2 seconds for the data to be returned. Will time out on M8 modules
  if (myGNSS.getRFinformation(&rfInformation, 2000))
  {
    Serial.print(F("The UBX_MON_RF message contains "));
    Serial.print(rfInformation.header.nBlocks); // Print how many information blocks were returned. Should be 0, 1 or 2
    Serial.println(F(" information blocks"));

    for (uint8_t block = 0; block < rfInformation.header.nBlocks; block++)
    {
      Serial.print(F("Block ID: "));
      Serial.print(rfInformation.blocks[block].blockId);
      if (rfInformation.blocks[block].blockId == 0)
        Serial.println(F(" = L1"));
      else // if (rfInformation.blocks[block].blockId == 1)
        Serial.println(F(" = L2 / L5"));
        
      Serial.print(F("Jamming state: "));
      Serial.print(rfInformation.blocks[block].flags.bits.jammingState);
      if (rfInformation.blocks[block].flags.bits.jammingState == 0)
        Serial.println(F(" = unknown / disabled"));
      else if (rfInformation.blocks[block].flags.bits.jammingState == 1)
        Serial.println(F(" = ok"));
      else if (rfInformation.blocks[block].flags.bits.jammingState == 2)
        Serial.println(F(" = warning"));
      else // if (rfInformation.blocks[block].flags.bits.jammingState == 3)
        Serial.println(F(" = critical!"));

      Serial.print(F("Noise level: "));
      Serial.println(rfInformation.blocks[block].noisePerMS);
      
      Serial.print(F("AGC monitor: "));
      Serial.println(rfInformation.blocks[block].agcCnt);
      
      Serial.print(F("CW jamming indicator: "));
      Serial.println(rfInformation.blocks[block].jamInd);      
    }

    Serial.println();
  }
}
