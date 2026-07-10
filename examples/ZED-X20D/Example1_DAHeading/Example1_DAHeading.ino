/*
  Configure ZED-X20D to output DAHEADING relative positioning information
  By: Paul Clark
  SparkFun Electronics
  Date: June 9th, 2026
  License: MIT. See license file for more information.

  This example shows how to query the module for DAHEADING information in the NED frame.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun ZED-X20D Breakout                 coming soon!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard Qwiic or BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

// Callback: printDAHEADINGdata will be called when new NAV DAHEADING data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_DAHEADING_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoDAHEADINGcallback
//        /                  _____  This _must_ be UBX_NAV_DAHEADING_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printDAHEADINGdata(UBX_NAV_DAHEADING_data_t *ubxDataStruct)
{
  Serial.println();
  Serial.println("New DAHEADING data received:");

  Serial.print("version: ");
  Serial.println(ubxDataStruct->version);

  // double won't work well on AVR platforms...
  Serial.print("relPosN (m): ");
  Serial.println(((double)ubxDataStruct->relPosN / 1000), 3); // Convert mm to m
  Serial.print("relPosE (m): ");
  Serial.println(((double)ubxDataStruct->relPosE / 1000), 3);
  Serial.print("relPosD (m): ");
  Serial.println(((double)ubxDataStruct->relPosD / 1000), 3);

  Serial.print("relPosLength (m): ");
  Serial.println(((double)ubxDataStruct->relPosLength / 1000), 3); // Convert mm to m
  Serial.print("relPosHeading (Deg): ");
  Serial.println((double)ubxDataStruct->relPosHeading / 100000); // Convert deg * 1e-5 to degrees

  Serial.print("accN (m): ");
  Serial.println((double)ubxDataStruct->accN / 1000, 3); // Convert mm to m
  Serial.print("accE (m): ");
  Serial.println((double)ubxDataStruct->accE / 1000, 3);
  Serial.print("accD (m): ");
  Serial.println((double)ubxDataStruct->accD / 1000, 3);

  Serial.print("gnssFixOk: ");
  if (ubxDataStruct->flags.bits.gnssFixOK == true)
    Serial.println("x");
  else
    Serial.println("");

  Serial.print("diffSolution: ");
  if (ubxDataStruct->flags.bits.diffSoln == true)
    Serial.println("x");
  else
    Serial.println("");

  Serial.print("relPosValid: ");
  if (ubxDataStruct->flags.bits.relPosValid == true)
    Serial.println("x");
  else
    Serial.println("");

  Serial.print("carrier Solution Type: ");
  if (ubxDataStruct->flags.bits.carrSoln == 0)
    Serial.println("None");
  else if (ubxDataStruct->flags.bits.carrSoln == 1)
    Serial.println("Float");
  else if (ubxDataStruct->flags.bits.carrSoln == 2)
    Serial.println("Fixed");

  Serial.print("relPosHeadingValid: ");
  if (ubxDataStruct->flags.bits.relPosHeadingValid == true)
    Serial.println("x");
  else
    Serial.println("");
}

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("u-blox DAHEADING example");

  Wire.begin();

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Retrying..."));
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  //myGNSS.factoryDefault(); delay(5000);

  // Change the module configuration
  bool setValueSuccess = true;
  // Begin with newCfgValset
  //setValueSuccess &= myGNSS.newCfgValset(); // This defaults to configuring the setting in RAM and BBR
  setValueSuccess &= myGNSS.newCfgValset(VAL_LAYER_RAM); // Set this and the following settings in RAM only

  // Enable Galileo HAS
  setValueSuccess &= myGNSS.addCfgValset(UBLOX_CFG_NAVCOR_ENABLE_HOST, 0);    // Disable HOST corrections
  setValueSuccess &= myGNSS.addCfgValset(UBLOX_CFG_NAVCOR_ENABLE_GAL_HAS, 1); // Enable Galileo HAS corrections

  // Configure the user-defined offset between the dual-antenna baseline heading and the vehicle forward axis.
  // User-defined offset between the dual-antenna baseline heading and the vehicle forward axis.
  // Only applicable in moving baseline mode. Accepted range is -180.00 to 180.00 degrees.
  setValueSuccess &= myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_DAHEADING_OFFSET, -4500); // Offset of -45 / 1e-2 degrees

  // Send the packet using sendCfgValset
  setValueSuccess &= myGNSS.sendCfgValset();
  if (setValueSuccess == true)
    Serial.println("X20D configuration : success");
  else
    Serial.println("X20D configuration : failed");

  myGNSS.setAutoDAHEADINGcallbackPtr(&printDAHEADINGdata); // Enable automatic NAV DAHEADING messages with callback to printDAHEADINGdata
}

void loop()
{
  myGNSS.checkUblox(); // Check for new RELPOSNED data
  myGNSS.checkCallbacks();
  
  Serial.print(".");
  delay(50);
}
