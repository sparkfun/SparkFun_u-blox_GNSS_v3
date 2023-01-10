/*
  Configuring u-blox Module using new VALGET / VALSET / VALDEL methods

  Please see u-blox_config_keys.h for the definitions of _all_ of the configuration keys
  
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information.

  u-blox deprecated many -CFG messages and replaced them with new
  VALGET, VALSET, VALDEL methods. This shows the basics of how to use
  these methods.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

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
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  byte response;
  if (myGNSS.getVal8(UBLOX_CFG_I2C_ADDRESS, &response, VAL_LAYER_RAM)) // Get the I2C address (see u-blox_config_keys.h for details)
  {
    Serial.print(F("1 : I2C Address: 0x"));
    Serial.println(response >> 1, HEX); //We have to shift by 1 to get the common '7-bit' I2C address format
  }
  else
  {
    Serial.println(F("VALGET failed!"));
  }

  if (myGNSS.getVal8(UBLOX_CFG_I2COUTPROT_NMEA, &response, VAL_LAYER_RAM)) // Get the flag indicating is NMEA should be output on I2C
  {
    Serial.print(F("1 : Output NMEA over I2C port: 0x"));
    Serial.println(response, HEX);
  }
  else
  {
    Serial.println(F("VALGET failed!"));
  }

  // New in v3: we can request both vals at the same time using a custom packet and newCfgValget, addCfgValget and sendCfgValget

  // Let's create our custom packet
  uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes
  
  // The next line creates and initialises the packet information which wraps around the payload
  ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  // The structure of ubxPacket is:
  // uint8_t cls           : The message Class
  // uint8_t id            : The message ID
  // uint16_t len          : Length of the payload. Does not include cls, id, or checksum bytes
  // uint16_t counter      : Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  // uint16_t startingSpot : The counter value needed to go past before we begin recording into payload array
  // uint8_t *payload      : The payload
  // uint8_t checksumA     : Given to us by the module. Checked against the rolling calculated A/B checksums.
  // uint8_t checksumB
  // sfe_ublox_packet_validity_e valid            : Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  // sfe_ublox_packet_validity_e classAndIDmatch  : Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID

  myGNSS.newCfgValget(&customCfg, MAX_PAYLOAD_SIZE, VAL_LAYER_RAM); // Create a new VALGET construct
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_I2C_ADDRESS); // Get the I2C address (see u-blox_config_keys.h for details)
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_I2COUTPROT_NMEA); // Get the flag indicating is NMEA should be output on I2C
  if (myGNSS.sendCfgValget(&customCfg)) // Send the VALGET
  {
    Serial.print(F("2 : I2C Address: 0x"));
    // Extract the 8-bit value manually:
    //   Payload bytes 0-3 will be the message version, layer and skip key bytes
    //   Bytes 4-7 wil be the Config Key (UBLOX_CFG_I2C_ADDRESS)
    //   Byte 8 contains the address.
    Serial.println(customPayload[8] >> 1, HEX); //We have to shift by 1 to get the common '7-bit' I2C address format

    Serial.print(F("2 : Output NMEA over I2C port: 0x"));
    // Extract the 8-bit value manually:
    //   Payload bytes 9-12 will be the Config Key (UBLOX_CFG_I2COUTPROT_NMEA)
    //   Byte 13 contains the flag.
    Serial.println(customPayload[13], HEX);

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

    Serial.print(F("3 : I2C Address: 0x"));
    Serial.println(myGNSS.extractByte(&customCfg, 8) >> 1, HEX); //We have to shift by 1 to get the common '7-bit' I2C address format
    Serial.print(F("3 : Output NMEA over I2C port: 0x"));
    Serial.println(myGNSS.extractByte(&customCfg, 13), HEX);

    // New in v3: we can use a template method to extract the value for us
    
    uint8_t i2cAddress; // We still need to know the type...
    if (myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_I2C_ADDRESS, &i2cAddress, sizeof(i2cAddress))) // Get the I2C address - using the key
    {
      Serial.print(F("4 : I2C Address: 0x"));
      Serial.println(i2cAddress >> 1, HEX); //We have to shift by 1 to get the common '7-bit' I2C address format
    }
    else
    {
      Serial.println(F("extractConfigValueByKey failed!"));      
    }

    bool nmeaI2c; // We still need to know the type...
    if (myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_I2COUTPROT_NMEA, &nmeaI2c, sizeof(nmeaI2c))) // Get the I2COUTPROT_NMEA flag - using the key
    {
      Serial.print(F("4 : Output NMEA over I2C port: 0x"));
      Serial.println(nmeaI2c, HEX);
    }
    else
    {
      Serial.println(F("extractConfigValueByKey failed!"));      
    }
  }
  else
  {
    Serial.println(F("VALGET failed!"));
  }    
}

void loop()
{
}
