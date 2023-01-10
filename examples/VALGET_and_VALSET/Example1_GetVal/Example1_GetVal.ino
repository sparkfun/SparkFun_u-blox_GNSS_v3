/*
  Get a device's I2C address using advanced getVal method
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 9th, 2019
  License: MIT. See license file for more information.

  u-blox changed how to configure their modules in 2019. As of version 23 of the UBX protocol the
  UBX-CFG commands are deprecated; they still work, they just recommend using VALSET, VALGET, and VALDEL
  commands instead. This example shows how to use this new command structure.

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

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("u-blox getVal example");

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  //myGNSS.enableDebugging(); //Enable debug messages over Serial (default)
  //myGNSS.enableDebugging(SerialUSB); //Enable debug messages over Serial USB

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Retrying..."));
    delay(1000);
  }

  // These key values are hard coded and defined in u-blox_config_keys.h.
  // You can obtain them from the ZED-F9P interface description doc
  // or from u-center's Messages->CFG->VALSET window. Keys must be 32-bit.

  // UBLOX_CFG_I2C_ADDRESS is 8-bit U1 (uint8_t)

  uint8_t currentI2Caddress; // This will hold the module's I2C address

  // 1 : The 'unsafe' version - we don't know if a bus error / timeout occurred. currentI2Caddress could be invalid.
  
  currentI2Caddress = myGNSS.getVal8(UBLOX_CFG_I2C_ADDRESS);
  Serial.print("1: Current I2C address (should be 0x42): 0x");
  Serial.println(currentI2Caddress >> 1, HEX); //u-blox module returns a shifted 8-bit address. Make it 7-bit unshifted.

  // 2 : The safe version - getVal returns true if the VALGET was successful. The data is returned safely via a pointer.
  
  if (myGNSS.getVal8(UBLOX_CFG_I2C_ADDRESS, &currentI2Caddress) == true) // Use the address of currentI2Caddress as a pointer
  {
    Serial.print("2: Current I2C address (should be 0x42): 0x");
    Serial.println(currentI2Caddress >> 1, HEX); //u-blox module returns a shifted 8-bit address. Make it 7-bit unshifted.
  }
  else
  {
    Serial.print("VALGET failed!");
  }
  
  // 3 : Read the I2C address from the RAM layer - this is the default.
  
  if (myGNSS.getVal8(UBLOX_CFG_I2C_ADDRESS, &currentI2Caddress, VAL_LAYER_RAM) == true) // Use the address of currentI2Caddress as a pointer
  {
    Serial.print("3: Current I2C address (should be 0x42): 0x");
    Serial.println(currentI2Caddress >> 1, HEX); //u-blox module returns a shifted 8-bit address. Make it 7-bit unshifted.
  }
  else
  {
    Serial.print("VALGET failed!");
  }
  
  // 4 : Read the I2C address from the DEFAULT layer.
  
  if (myGNSS.getVal8(UBLOX_CFG_I2C_ADDRESS, &currentI2Caddress, VAL_LAYER_DEFAULT) == true) // Use the address of currentI2Caddress as a pointer
  {
    Serial.print("4: Current I2C address (should be 0x42): 0x");
    Serial.println(currentI2Caddress >> 1, HEX); //u-blox module returns a shifted 8-bit address. Make it 7-bit unshifted.
  }
  else
  {
    Serial.print("VALGET failed!");
  }
  
  // 5 & 6 : Read the I2C address using a custom packet and multi-getVal.

  // Let's create our custom packet
  uint8_t customPayload[9]; // This array holds the payload data bytes. We only need 9 bytes: 4 + 4 (Key ID) + 1 (Val8)
  
  // The next line creates and initialises the packet information which wraps around the payload
  ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  myGNSS.newCfgValget(&customCfg, 9, VAL_LAYER_RAM); // Create a new VALGET construct. Read the data from the RAM layer.
  
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_I2C_ADDRESS); // Get the I2C address (see u-blox_config_keys.h for details)
  
  if (myGNSS.sendCfgValget(&customCfg) == true) // Send the VALGET
  {
    // Extract the 8-bit value manually:
    //   Payload bytes 0-3 will be the message version, layer and skip key bytes
    //   Bytes 4-7 wil be the Config Key (UBLOX_CFG_I2C_ADDRESS)
    //   Byte 8 contains the address.
    Serial.print("5: Current I2C address (should be 0x42): 0x");
    currentI2Caddress = customPayload[8];
    Serial.println(currentI2Caddress >> 1, HEX); //u-blox module returns a shifted 8-bit address. Make it 7-bit unshifted.

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

    Serial.print("6: Current I2C address (should be 0x42): 0x");
    currentI2Caddress = myGNSS.extractByte(&customCfg, 8);
    Serial.println(currentI2Caddress >> 1, HEX); //u-blox module returns a shifted 8-bit address. Make it 7-bit unshifted.

    // New in v3: we can use a template method to extract the value for us
    
    uint8_t i2cAddress; // We still need to know the type...
    if (myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_I2C_ADDRESS, &i2cAddress, sizeof(i2cAddress))) // Get the I2C address - using the key
    {
      Serial.print(F("7: Current I2C address (should be 0x42): 0x"));
      Serial.println(i2cAddress >> 1, HEX); //We have to shift by 1 to get the common '7-bit' I2C address format
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
  // Nothing to do here
}
