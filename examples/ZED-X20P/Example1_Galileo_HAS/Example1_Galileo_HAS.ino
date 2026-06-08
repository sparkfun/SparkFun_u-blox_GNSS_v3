/*
  Enabling Galileo HAS on the ZED-X20P
  By: Paul Clark
  SparkFun Electronics
  Date: June 8th, 2026
  License: MIT. See license file for more information.

  This example shows how to enable Galileo HAS on the ZED-X20P.
  Note: the ZED-X20P must be running u-blox HPG 2.10 firmware - released 20-May-2026.

  https://content.u-blox.com/sites/default/files/2026-05/UBX_20_HPG_210_ZED_X20P-01B.512369040097ce18fd3475e71e7c627f.bin
  https://www.u-blox.com/sites/default/files/documents/HPG210_RN_UBXDOC-304424225-21261.pdf

  Feel like supporting open source hardware?
  Buy a board from SparkFun!

  SparkFun Allband GNSS RTK Breakout - ZED-X20P (Qwiic) https://www.sparkfun.com/sparkfun-allband-gnss-rtk-breakout-zed-x20p-qwiic.html

  SparkPNT FPX                                          https://www.sparkfun.com/sparkpnt-fpx.html
  SparkPNT FPX-T                                        https://www.sparkfun.com/sparkpnt-fpx-t.html

  SparkPNT GNSS Flex Module - ZED-X20P                  https://www.sparkfun.com/sparkpnt-gnss-flex-module-zed-x20p.html
  SparkPNT GNSS Flex Module - ZED-X20P & IM19 IMU       https://www.sparkfun.com/sparkpnt-gnss-flex-module-zed-x20p-im19-imu.html
  SparkFun GNSS Flex pHAT - ZED-X20P                    https://www.sparkfun.com/sparkfun-gnss-flex-phat-zed-x20p.html
  SparkFun GNSS Flex pHAT - ZED-X20P & IM19 IMU         https://www.sparkfun.com/sparkfun-gnss-flex-phat-zed-x20p-im19-imu.html
  SparkFun GNSS Flex Breakout                           https://www.sparkfun.com/sparkfun-gnss-flex-breakout.html

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallbackPtr
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  Serial.println();
  
  double latitude = ubxDataStruct->lat; // Print the latitude
  Serial.print(F("Lat: "));
  Serial.print(latitude / 10000000.0, 7);

  double longitude = ubxDataStruct->lon; // Print the longitude
  Serial.print(F("  Long: "));
  Serial.print(longitude / 10000000.0, 7);

  double altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
  Serial.print(F("  Height: "));
  Serial.print(altitude / 1000.0, 3);

  uint8_t fixType = ubxDataStruct->fixType; // Print the fix type
  Serial.print(F("  Fix: "));
  Serial.print(fixType);
  if (fixType == 0)
    Serial.print(F(" (None)"));
  else if (fixType == 1)
    Serial.print(F(" (Dead Reckoning)"));
  else if (fixType == 2)
    Serial.print(F(" (2D)"));
  else if (fixType == 3)
    Serial.print(F(" (3D)"));
  else if (fixType == 4)
    Serial.print(F(" (GNSS + Dead Reckoning)"));
  else if (fixType == 5)
    Serial.print(F(" (Time Only)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln; // Print the carrier solution
  Serial.print(F("  Carrier Solution: "));
  Serial.print(carrSoln);
  if (carrSoln == 0)
    Serial.print(F(" (None)"));
  else if (carrSoln == 1)
    Serial.print(F(" (Floating)"));
  else if (carrSoln == 2)
    Serial.print(F(" (Fixed)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint32_t hAcc = ubxDataStruct->hAcc; // Print the horizontal accuracy estimate
  Serial.print(F("  Horizontal Accuracy Estimate: "));
  Serial.print(hAcc);
  Serial.print(F(" (mm)"));

  Serial.println();    
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(Serial); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  if (myGNSS.getModuleInfo())
  {
    Serial.print(F("FWVER: "));
    Serial.print(myGNSS.getFirmwareVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getFirmwareVersionLow()); // Returns uint8_t
    
    Serial.print(F("Firmware: "));
    Serial.println(myGNSS.getFirmwareType()); // Returns HPG, SPG etc. as (const char *)

    Serial.print(F("PROTVER: "));
    Serial.print(myGNSS.getProtocolVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getProtocolVersionLow()); // Returns uint8_t
    
    Serial.print(F("MOD: "));
    Serial.println(myGNSS.getModuleName()); // Returns ZED-F9P, MAX-M10S etc. as (const char *)
  }
  else
    Serial.println(F("Error: could not read module info!"));

  // Check for HPG firmware >= 2.10
  if ((myGNSS.getFirmwareVersionHigh() < 2)
      || ((myGNSS.getFirmwareVersionHigh() == 2) && (myGNSS.getFirmwareVersionLow() < 10)))
  {
    Serial.println(F("Please update your ZED-X20P firmware to HPG >= 2.10. Freezing."));
    while (1);
  }

  // Enable Galileo HAS
  bool setValueSuccess = true;
  // Begin with newCfgValset
  //setValueSuccess &= myGNSS.newCfgValset(); // This defaults to configuring the setting in RAM and BBR
  setValueSuccess &= myGNSS.newCfgValset(VAL_LAYER_RAM); // Set this and the following settings in RAM only
  // Add KeyIDs and Values
  setValueSuccess &= myGNSS.addCfgValset(UBLOX_CFG_NAVCOR_ENABLE_HOST, 0);    // Disable HOST corrections
  setValueSuccess &= myGNSS.addCfgValset(UBLOX_CFG_NAVCOR_ENABLE_GAL_HAS, 1); // Enable Galileo HAS corrections
  // Send the packet using sendCfgValset
  setValueSuccess &= myGNSS.sendCfgValset();
  if (setValueSuccess == true)
    Serial.println("GAL HAS successfully enabled");
  else
    Serial.println("GAL HAS enable failed");

  // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to float
  //myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // This defaults to configuring the setting in RAM and BBR
  myGNSS.setAutoPVTcallbackPtr(&printPVTdata, VAL_LAYER_RAM); // Set this setting in RAM only
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
}