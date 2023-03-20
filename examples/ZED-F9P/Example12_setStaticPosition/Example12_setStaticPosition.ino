/*
  Set the static position of the receiver.
  By: SparkFun Electronics / Nathan Seidle
  Date: September 26th, 2020
  License: MIT. See license file for more information.

  This example shows how to set the static position of a receiver
  using an Earth-Centered, Earth-Fixed (ECEF) location. This is the
  output from a long (24 hour+) survey-in. Setting the static position
  immediately causes the receiver to begin outputting RTCM data (if
  enabled), perfect for setting up your own RTCM NTRIP caster or CORS.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  delay(1000);
  
  Serial.begin(115200); // You may need to increase this for high navigation rates!
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  bool success = true;
  // If we AND (&=) the result of each command into success, success will be false if any one fails

  //-1280208.308,-4716803.847,4086665.811 is SparkFun HQ in ECEF coordinates so...

  //Units are cm so 1234 = 12.34m
  //
  //If you were setting up a full GNSS station, you would want to save these settings to Battery-Backed RAM.
  //Because setting an incorrect static position will disable the ability to get a lock, we will save to RAM layer only in this example - not RAM_BBR.  
  //success &= myGNSS.setStaticPosition(-128020831, -471680385, 408666581, false, VAL_LAYER_RAM); // False at end enables ECEF input

  // Units are cm with a high precision extension so -1234.5678 should be called: (-123456, -78)
  //
  //If you were setting up a full GNSS station, you would want to save these settings to Battery-Backed RAM.
  //Because setting an incorrect static position will disable the ability to get a lock, we will save to RAM layer only in this example - not RAM_BBR.  
  success &= myGNSS.setStaticPosition(-128020830, -80, -471680384, -70, 408666581, 10, false, VAL_LAYER_RAM); // ECEF with high precision 0.1mm parts

  // We can also set via lat/long
  // 40.09029751,-105.18507900,1560.238
  //
  //If you were setting up a full GNSS station, you would want to save these settings to Battery-Backed RAM.
  //Because setting an incorrect static position will disable the ability to get a lock, we will save to RAM layer only in this example - not RAM_BBR.  
  //success &= myGNSS.setStaticPosition(400902975, -1051850790, 156024, true, VAL_LAYER_RAM); // True at end enables lat/long input

  // For High Precision Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
  //
  //If you were setting up a full GNSS station, you would want to save these settings to Battery-Backed RAM.
  //Because setting an incorrect static position will disable the ability to get a lock, we will save to RAM layer only in this example - not RAM_BBR.  
  //success &= myGNSS.setStaticPosition(400902975, 10, -1051850790, 0, 156023, 80, true, VAL_LAYER_RAM);

  if (!success) Serial.println(F("At least one call to setStaticPosition failed!"));



  //Now let's use getVals to read back the data

  // New in v3: we can request both vals at the same time using a custom packet and newCfgValget, addCfgValget and sendCfgValget

  // Let's create our custom packet
  uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes
  
  // The next line creates and initialises the packet information which wraps around the payload
  ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  // Create a new VALGET construct. Read data from the RAM layer (not DEFAULT)
  myGNSS.newCfgValget(&customCfg, MAX_PAYLOAD_SIZE, VAL_LAYER_RAM);
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TMODE_POS_TYPE); // Get the position type 
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TMODE_LAT);
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TMODE_LON);
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TMODE_HEIGHT);
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TMODE_ECEF_X);
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TMODE_ECEF_Y);
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TMODE_ECEF_Z);
  if (myGNSS.sendCfgValget(&customCfg)) // Send the VALGET
  {
    // We can also use the "extract" helper functions to read the data - useful for 16, 32 and 64-bit values
    // The full list is:
    //   uint64_t extractLongLong(ubxPacket *msg, uint16_t spotToStart);       // Combine eight bytes from payload into uint64_t
    //   uint64_t extractSignedLongLong(ubxPacket *msg, uint16_t spotToStart); // Combine eight bytes from payload into int64_t
    //   uint32_t extractLong(ubxPacket *msg, uint16_t spotToStart);           // Combine four bytes from payload into long
    //   int32_t extractSignedLong(ubxPacket *msg, uint16_t spotToStart);      // Combine four bytes from payload into signed long (avoiding any ambiguity caused by casting)
    //   uint16_t extractInt(ubxPacket *msg, uint16_t spotToStart);            // Combine two bytes from payload into int
    //   int16_t extractSignedInt(ubxPacket *msg, uint16_t spotToStart);
    //   uint8_t extractByte(ubxPacket *msg, uint16_t spotToStart);            // Get byte from payload
    //   int8_t extractSignedChar(ubxPacket *msg, uint16_t spotToStart);       // Get signed 8-bit value from payload
    //   float extractFloat(ubxPacket *msg, uint16_t spotToStart);             // Get 32-bit float from payload
    //   double extractDouble(ubxPacket *msg, uint16_t spotToStart);           // Get 64-bit double from payload

    Serial.print(F("Position type: "));
    uint8_t posType = myGNSS.extractByte(&customCfg, 8); // Position type is in Byte 8
    Serial.print(posType);
    if (posType == 0)
      Serial.println(F(" (ECEF)"));
    else
      Serial.println(F(" (LLH)"));

    if (posType == 0)
    {
      Serial.print(F("X (m): "));
      double xyz = (double)myGNSS.extractSignedLong(&customCfg, 37) / 100; // ECEF X is in Byte 37. Convert from cm to m
      Serial.print(xyz);

      Serial.print(F("   Y (m): "));
      xyz = (double)myGNSS.extractSignedLong(&customCfg, 45) / 100; // ECEF Y is in Byte 45. Convert from cm to m
      Serial.print(xyz);
      
      Serial.print(F("   Z (m): "));
      xyz = (double)myGNSS.extractSignedLong(&customCfg, 53) / 100; // ECEF Z is in Byte 53. Convert from cm to m
      Serial.println(xyz);
    }
    else
    {
      Serial.print(F("LAT (Deg): "));
      double llh = (double)myGNSS.extractSignedLong(&customCfg, 13) / 10000000; // LAT is in Byte 13. Convert to Degrees
      Serial.print(llh, 7);

      Serial.print(F("   LON (Deg): "));
      llh = (double)myGNSS.extractSignedLong(&customCfg, 21) / 10000000; // LON is in Byte 21. Convert to Degrees
      Serial.print(llh, 7);

      Serial.print(F("   HEIGHT (m): "));
      llh = (double)myGNSS.extractSignedLong(&customCfg, 29) / 100; // HEIGHT is in Byte 29. Convert to m
      Serial.println(llh);
    }

    // New in v3 : we can also use a template method to extract the data automatically
    
    Serial.print(F("Position type: "));
    // We still need to know the type / size of each key value...
    myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TMODE_POS_TYPE, &posType, sizeof(posType));
    Serial.print(posType);
    if (posType == 0)
      Serial.println(F(" (ECEF)"));
    else
      Serial.println(F(" (LLH)"));

    if (posType == 0)
    {
      Serial.print(F("X (m): "));
      int32_t xyz; // We still need to know the type / size of each key value...
      myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TMODE_ECEF_X, &xyz, sizeof(xyz));
      double xyz_d = (double)xyz / 100; // Convert from cm to m
      Serial.print(xyz_d);

      Serial.print(F("   Y (m): "));
      myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TMODE_ECEF_Y, &xyz, sizeof(xyz));
      xyz_d = (double)xyz / 100; // Convert from cm to m
      Serial.print(xyz_d);
      
      Serial.print(F("   Z (m): "));
      myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TMODE_ECEF_Z, &xyz, sizeof(xyz));
      xyz_d = (double)xyz / 100; // Convert from cm to m
      Serial.println(xyz_d);
    }
    else
    {
      Serial.print(F("LAT (Deg): "));
      int32_t llh; // We still need to know the type / size of each key value...
      myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TMODE_LAT, &llh, sizeof(llh));
      double llh_d = (double)llh / 10000000; // Convert to Degrees
      Serial.print(llh_d, 7);

      Serial.print(F("   LON (Deg): "));
      myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TMODE_LON, &llh, sizeof(llh));
      llh_d = (double)llh / 10000000; // Convert to Degrees
      Serial.print(llh_d, 7);

      Serial.print(F("   HEIGHT (m): "));
      myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TMODE_HEIGHT, &llh, sizeof(llh));
      llh_d = (double)llh / 100; // Convert from cm to m
      Serial.println(llh_d);
    }
  }
  else
  {
    Serial.println(F("VALGET failed!"));
  }    

  Serial.println(F("Done!"));
}

void loop()
{
}
