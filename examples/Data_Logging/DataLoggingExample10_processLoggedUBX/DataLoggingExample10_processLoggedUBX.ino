/*
  Demonstrate how to 'process' any UBX message - without needing the Auto methods
  By: Paul Clark
  SparkFun Electronics
  Date: July 14th, 2025
  License: MIT. See license file for more information.

  This example shows how to configure the u-blox GNSS to output (e.g.) UBX_MON_SYS
  and process those messages within your own code. This is cool because UBX_MON_SYS
  does not have full "Auto" support. There are no setAutoMONSYS or setAutoMONSYScallbackPtr methods.
  But we can process it anyway...!

  ** Please note: processLoggedUBX is called by the library's internal processUBXpacket method. **
  ** The library will stall while processLoggedUBX is executing. **
  ** For best results, avoid Serial prints and delays etc. within processLoggedUBX. **

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a Redboard Qwiic
  If you don't have a platform with a Qwiic connection use the
  SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  
  To minimise I2C bus errors, it is a good idea to open the I2C pull-up split pad links on
  the u-blox module breakout.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3

// Define a custom class so we can override processLoggedUBX
class my_SFE_UBLOX_GNSS : public SFE_UBLOX_GNSS
{
  public:
    void processLoggedUBX(ubxPacket *incomingUBX)
    {
      // processLoggedUBX will stall the library while it is executing
      // For best results, quickly copy the data elsewhere. Avoid Serial prints, delays etc..
      // incomingUBX->len contains the payload length (uint16_t)
      // incomingUBX->payload points to the payload (uint8_t *)
      // The ubxPacket struct is defined in u-blox_external_typedefs.h

      Serial.printf("UBX message received: Class 0x%02x ID 0x%02x", incomingUBX->cls, incomingUBX->id);
      if ((incomingUBX->cls == UBX_CLASS_MON) && (incomingUBX->id == UBX_MON_SYS))
      {
        // We can use the "extract" helper functions to read the data - useful for 16, 32 and 64-bit values
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
        
        uint8_t cpuLoad = extractByte(incomingUBX, 2);
        uint32_t runTime = extractLong(incomingUBX, 8);
        int8_t tempValue = extractSignedChar(incomingUBX, 18);

        Serial.printf(" : UBX-MON-SYS : cpuLoad %d%% runTime %ds tempValue %dC", (int)cpuLoad, (int)runTime, (int)tempValue);
      }
      Serial.println();
    }
};
my_SFE_UBLOX_GNSS myGNSS;

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin(); // Start I2C communication

  //myGNSS.enableDebugging(); // Uncomment this line to enable lots of helpful GNSS debug messages on Serial
  //myGNSS.enableDebugging(Serial, true); // Or, uncomment this line to enable only the important GNSS debug messages on Serial

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring..."));
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  // This will (re)enable the standard NMEA messages too
  // This will also disable any "auto" UBX messages that were enabled and saved by other examples and reduce the load on the I2C bus
  //myGNSS.factoryDefault(); delay(5000);

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Set the I2C port to output UBX, NMEA and RTCM messages

  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(1); //Produce one navigation solution per second

  myGNSS.newCfgValset(VAL_LAYER_RAM_BBR); // Use cfgValset to disable / enable individual messages
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_UBX_MON_SYS_I2C, 1); // Enable the UBX-MON-SYS message on I2C
  myGNSS.sendCfgValset(); // Send the configuration VALSET

  myGNSS.enableUBXlogging(UBX_CLASS_MON, UBX_MON_SYS, false, true); // Disable logging of MON-SYS. Enable processing
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data. MON-SYS data will be processed by processLoggedUBX
}
