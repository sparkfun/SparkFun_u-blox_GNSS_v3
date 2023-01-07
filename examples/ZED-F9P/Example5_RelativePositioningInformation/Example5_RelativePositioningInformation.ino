/*
  Send UBX binary commands to enable RTCM sentences on u-blox ZED-F9P module
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 9th, 2019
  License: MIT. See license file for more information.

  This example shows how to query the module for RELPOS information in the NED frame.
  It assumes you already have RTCM correction data being fed to the receiver.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
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

//#define USE_RTCM_SERIAL // Uncomment this line to push the RTCM data from Serial to the module via I2C

#ifdef USE_RTCM_SERIAL
  // If our board supports it, we can receive the RTCM data on Serial and push to I2C
  #define rtcmSerial Serial1
#endif

// Callback: printRELPOSNEDdata will be called when new NAV RELPOSNED data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_RELPOSNED_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRELPOSNEDcallback
//        /                  _____  This _must_ be UBX_NAV_RELPOSNED_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printRELPOSNEDdata(UBX_NAV_RELPOSNED_data_t *ubxDataStruct)
{
  Serial.println();
  Serial.println("New RELPOSNED data received:");

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
  Serial.println("u-blox Base station example");

#ifdef USE_RTCM_SERIAL
  // If our board supports it, we can receive the RTCM data on Serial
  rtcmSerial.begin(115200);
#endif

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Retrying..."));
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  //myGNSS.factoryDefault(); delay(5000);

#ifdef USE_RTCM_SERIAL
  Serial.print(F("Enabling UBX and RTCM input on I2C. Result: "));
  Serial.print(myGNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_RTCM3, VAL_LAYER_RAM_BBR)); //Enable UBX and RTCM input on I2C. Save in RAM and BBR
#endif

  myGNSS.setAutoRELPOSNEDcallbackPtr(&printRELPOSNEDdata); // Enable automatic NAV RELPOSNED messages with callback to printRELPOSNEDdata
}

void loop()
{
  myGNSS.checkUblox(); // Check for new RELPOSNED data
  myGNSS.checkCallbacks();
  
#ifdef USE_RTCM_SERIAL
  // Buffer and push the RTCM data to the module
  
  static uint8_t store[256];
  static uint16_t numBytes = 0; // Record the number of bytes received from rtcmSerial
  
  while ((rtcmSerial.available()) && (numBytes < 256)) // Check if data has been received
  {
    store[numBytes++] = rtcmSerial.read(); // Read a byte from rtcmSerial and store it
  }
  
  if (numBytes > 0) // Check if data was received
  {
    //Serial.print("Pushing ");
    //Serial.print(numBytes);
    //Serial.println(" bytes via I2C");

    //On processors which have large I2C buffers, like the ESP32, we can make the push more efficient by
    //calling setI2CTransactionSize first to increase the maximum I2C transmission size
    //(setI2CTransactionSize only needs to be called once, so it should be in setup, not loop)
    //myGNSS.setI2CTransactionSize(128); // Send up to 128 bytes in one I2C transmission

    myGNSS.pushRawData(((uint8_t *)&store), numBytes); // Push the RTCM data via I2C
    numBytes = 0; // Reset numBytes
  }
#endif
}
