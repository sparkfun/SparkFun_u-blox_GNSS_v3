/*
  Time Pulse Parameters - Frequency
  By: Paul Clark (PaulZC)
  Date: January 13th, 2021

  License: MIT. See license file for more information.

  This example shows how to change the time pulse parameters and configure the TIMEPULSE (PPS)
  pin to produce a 1kHz squarewave

  The SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic) (https://www.sparkfun.com/products/16481)
  has solder pads which will let you connect an SMA connector to the TIMEPULSE signal. Need an
  accurate frequency or clock source for your latest project? This is the product for you!

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

#include "SparkFun_u-blox_GNSS_v3.h" //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
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

  // The Configuration Interface supports two Time Pulse pins TP1 and TP2.
  // Here we are configuring TP1, but identical keys exist for TP2 (if your module supports it). See CFG-TP in u-blox_config_keys.h for more details.
  
  // We can configure the time pulse pin to produce a defined frequency or period
  // Here is how to set the frequency:

  Serial.println(F("Setting the time pulse configuration:"));

  myGNSS.newCfgValset(VAL_LAYER_RAM); // Create a new Configuration Interface VALSET message. Apply the changes in RAM only (not BBR).

  // New in v3 : we can use the addCfgValset template method to deduce the data size automatically

  // While the module is _locking_ to GNSS time, make it generate 2kHz
  myGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_TP1, 2000); // Set the frequency to 2000Hz
  myGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_TP1, 100.0 / 3.0); // Set the pulse ratio / duty to 33.333% to produce 33.333:66.666 mark:space

  // When the module is _locked_ to GNSS time, make it generate 1kHz
  myGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_LOCK_TP1, 1000); // Set the frequency to 1000Hz
  myGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_LOCK_TP1, 50.0); // Set the pulse ratio / duty to 50% to produce 50:50 mark:space

  myGNSS.addCfgValset(UBLOX_CFG_TP_TP1_ENA, 1); // Make sure the enable flag is set to enable the time pulse. (Set to 0 to disable.)
  myGNSS.addCfgValset(UBLOX_CFG_TP_USE_LOCKED_TP1, 1); // Tell the module to use FREQ while locking and FREQ_LOCK when locked to GNSS time
  myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_DEF, 1); // Tell the module that we want to set the frequency (not the period). PERIOD = 0. FREQ = 1.
  myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_LENGTH_DEF, 0); // Tell the module to set the pulse ratio / duty (not the pulse length). RATIO = 0. LENGTH = 1.
  myGNSS.addCfgValset(UBLOX_CFG_TP_POL_TP1, 1); // Tell the module that we want the rising edge at the top of second. Falling Edge = 0. Rising Edge = 1.

  // Check how many keys have been added - should be 9
  Serial.print(F("VALSET contains this many keys: "));
  Serial.println(myGNSS.getCfgValsetLen());

  // Check how much space is remaining in packetCfg for more keys
  Serial.print(F("There are this many bytes available in payloadCfg: "));
  Serial.println(myGNSS.getCfgValsetSpaceRemaining());;

  // Now set the time pulse parameters
  if (myGNSS.sendCfgValset() == false)
  {
    Serial.println(F("VALSET failed!"));
  }
  else
  {
    Serial.println(F("Success! Reading back the configuration:"));

    // New in v3 : addCfgValget can deduce the data size automatically

    // Let's create our custom packet
    uint16_t payloadSize = 40;
    //payloadSize = 20; // Just for giggles, you can make customPayload too small for the DUTY by uncommenting this line
    uint8_t customPayload[payloadSize]; // This array holds the payload data bytes
    
    // The next line creates and initialises the packet information which wraps around the payload
    ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

    myGNSS.newCfgValget(&customCfg, payloadSize, VAL_LAYER_RAM);// Create a new Configuration Interface VALGET message
    
    if (myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TP_FREQ_TP1)) // Request the frequency (U4) - the size is deduced automatically
      Serial.println(F("UBLOX_CFG_TP_FREQ_TP1 added"));
      
    if (myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TP_DUTY_TP1)) // Request the duty (R8) - the size is deduced automatically
      Serial.println(F("UBLOX_CFG_TP_DUTY_TP1 added"));
      
    if (myGNSS.addCfgValget(&customCfg, UBLOX_CFG_TP_TP1_ENA)) // Request the ENA (L) - the size is deduced automatically
      Serial.println(F("UBLOX_CFG_TP_TP1_ENA added"));

    // Check how many keys have been added - should be 3 unless you made payloadSize = 20
    Serial.print(F("VALGET contains this many keys: "));
    Serial.println(myGNSS.getNumGetCfgKeys());

    // Check what the expected response size will be
    Serial.print(F("Expected VALGET response is this many bytes: "));
    Serial.println(myGNSS.getLenCfgValGetResponse());;
    
    // Now get the time pulse parameters
    if (myGNSS.sendCfgValget(&customCfg) == false)
    {
      Serial.println(F("VALGET failed!"));
    }
    else
    {
      uint32_t freq = 0;
      if (myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TP_FREQ_TP1, &freq, sizeof(freq)))
      {
        Serial.print(F("Frequency: "));
        Serial.println(freq);
      }
      
      double duty = 0.0;
      if (myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TP_DUTY_TP1, &duty, sizeof(duty)))
      {
        Serial.print(F("Duty: "));
        Serial.println(duty, 5);
      }

      bool enabled = 0;
      if (myGNSS.extractConfigValueByKey(&customCfg, UBLOX_CFG_TP_TP1_ENA, &enabled, sizeof(enabled)))
      {
        Serial.print(F("Enabled: "));
        Serial.println(enabled);
      }
    }
  }
}

void loop()
{
  // Nothing to do here
}
