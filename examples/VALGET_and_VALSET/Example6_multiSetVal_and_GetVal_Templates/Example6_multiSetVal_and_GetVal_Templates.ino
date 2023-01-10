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

  // New in v3: we can set multiple configuration items at the same time without necessarily knowing the data width
  
  myGNSS.newCfgValset(VAL_LAYER_RAM); // Create a new VALSET construct
  if (!myGNSS.addCfgValset(UBLOX_CFG_I2C_ADDRESS, 0x42 << 1)) // The module stores the address in shifted format. We need to shift the address left by 1 bit.
    Serial.println(F("addCfgValset(UBLOX_CFG_I2C_ADDRESS, 0x42) failed!"));
  if (!myGNSS.addCfgValset(UBLOX_CFG_I2COUTPROT_NMEA, 1))
    Serial.println(F("addCfgValset(UBLOX_CFG_I2COUTPROT_NMEA, 1) failed!"));
  if (!myGNSS.addCfgValset(UBLOX_CFG_UART1_BAUDRATE, 38400))
    Serial.println(F("addCfgValset(UBLOX_CFG_UART1_BAUDRATE, 38400) failed!"));
  if (!myGNSS.sendCfgValset())
    Serial.println(F("VALSET failed!"));

  // New in v3: we can request multiple vals at the same time using a custom packet and newCfgValget, addCfgValget and sendCfgValget

  // Let's create our custom packet
  uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes
  
  // The next line creates and initialises the packet information which wraps around the payload
  ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  myGNSS.newCfgValget(&customCfg, MAX_PAYLOAD_SIZE, VAL_LAYER_RAM); // Create a new VALGET construct
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_I2C_ADDRESS); // Get the I2C address (see u-blox_config_keys.h for details)
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_I2COUTPROT_NMEA); // Get the flag indicating is NMEA should be output on I2C
  myGNSS.addCfgValget(&customCfg, UBLOX_CFG_UART1_BAUDRATE); // Get the UART1 baud rate 
  if (myGNSS.sendCfgValget(&customCfg)) // Send the VALGET
  {
    // New in v3: we can use a template method to extract the value for us
    
    uint8_t i2cAddress; // We still need to know the type...
    if (myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_I2C_ADDRESS, &i2cAddress, sizeof(i2cAddress))) // Get the I2C address - using the key
    {
      Serial.print(F("I2C Address: 0x"));
      Serial.println(i2cAddress >> 1, HEX); //We have to shift by 1 to get the common '7-bit' I2C address format
    }
    else
    {
      Serial.println(F("extractConfigValueByKey failed!"));      
    }

    bool nmeaI2c; // We still need to know the type...
    if (myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_I2COUTPROT_NMEA, &nmeaI2c, sizeof(nmeaI2c))) // Get the I2COUTPROT_NMEA flag - using the key
    {
      Serial.print(F("Output NMEA over I2C port: 0x"));
      Serial.println(nmeaI2c, HEX);
    }
    else
    {
      Serial.println(F("extractConfigValueByKey failed!"));      
    }

    uint32_t baud; // We still need to know the type...
    if (myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART1_BAUDRATE, &baud, sizeof(baud))) // Get the baud rate - using the key
    {
      Serial.print(F("UART1 Baud Rate: "));
      Serial.println(baud);
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
