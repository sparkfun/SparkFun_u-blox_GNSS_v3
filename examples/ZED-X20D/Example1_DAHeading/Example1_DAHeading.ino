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

  // double won't work well on AVR platforms...
  Serial.print("relPosN (m): ");
  Serial.println(((double)ubxDataStruct->relPosN / 100) + ((double)ubxDataStruct->relPosHPN / 10000), 4); // Convert cm and 0.1mm to m
  Serial.print("relPosE (m): ");
  Serial.println(((double)ubxDataStruct->relPosE / 100) + ((double)ubxDataStruct->relPosHPE / 10000), 4);
  Serial.print("relPosD (m): ");
  Serial.println(((double)ubxDataStruct->relPosD / 100) + ((double)ubxDataStruct->relPosHPD / 10000), 4);

  Serial.print("relPosLength (m): ");
  Serial.println(((double)ubxDataStruct->relPosLength / 100) + ((double)ubxDataStruct->relPosHPLength / 10000), 4); // Convert cm to m
  Serial.print("relPosHeading (Deg): ");
  Serial.println((double)ubxDataStruct->relPosHeading / 100000); // Convert deg * 1e-5 to degrees

  Serial.print("accN (m): ");
  Serial.println((double)ubxDataStruct->accN / 10000, 4); // Convert 0.1mm to m
  Serial.print("accE (m): ");
  Serial.println((double)ubxDataStruct->accE / 10000, 4);
  Serial.print("accD (m): ");
  Serial.println((double)ubxDataStruct->accD / 10000, 4);

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

  Serial.print("isMoving: ");
  if (ubxDataStruct->flags.bits.isMoving == true)
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

  myGNSS.setAutoDAHEADINGcallbackPtr(&printDAHEADINGdata); // Enable automatic NAV DAHEADING messages with callback to printDAHEADINGdata
}

void loop()
{
  myGNSS.checkUblox(); // Check for new RELPOSNED data
  myGNSS.checkCallbacks();
  
  Serial.print(".");
  delay(50);
}
