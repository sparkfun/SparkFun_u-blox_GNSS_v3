/*
  Time Pulse Parameters - Bullet Time (https://en.wikipedia.org/wiki/Bullet_time)
  By: Paul Clark (PaulZC)
  Date: January 13th, 2021

  License: MIT. See license file for more information.

  This example shows how to change the time pulse parameters and configure the TIMEPULSE (PPS)
  pin to produce a pulse once per second but with an adjustable delay. You could use this to
  trigger multiple cameras and replicate the "bullet time" effect.

  The SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic) (https://www.sparkfun.com/products/16481)
  has solder pads which will let you connect an SMA connector to the TIMEPULSE signal. Need an
  accurate timelapse camera shutter signal? This is the product for you!

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
  // Here is how to set the period:

  myGNSS.newCfgValset(VAL_LAYER_RAM); // Create a new Configuration Interface VALSET message. Apply the changes in RAM only (not BBR).

  // Let's say that we want our pulse-per-second to be as accurate as possible. So, let's tell the module
  // to generate no signal while it is _locking_ to GNSS time. We want the signal to start only when the module is
  // _locked_ to GNSS time.
  myGNSS.addCfgValset(UBLOX_CFG_TP_PERIOD_TP1, 0); // Set the period to zero
  myGNSS.addCfgValset(UBLOX_CFG_TP_LEN_TP1, 0); // Set the pulse length to zero

  // When the module is _locked_ to GNSS time, make it generate a 0.1 second pulse once per second
  myGNSS.addCfgValset(UBLOX_CFG_TP_PERIOD_LOCK_TP1, 1000000); // Set the period to 1,000,000 us
  myGNSS.addCfgValset(UBLOX_CFG_TP_LEN_LOCK_TP1, 100000); // Set the pulse length to 0.1s (100,000 us)
  myGNSS.addCfgValset(UBLOX_CFG_TP_POL_TP1, 1); // Set the polarity to "1" (high for 0.1s, low for 0.9s, rising edge at top of second)

  // We can use CFG_TP_USER_DELAY to delay the pulse for each camera. The delay needs to be negative for this example.
  // We can delay the pulse by +/- 2^31 nanoseconds (+/- 2.147 seconds).
  // addCfgValset32 expects an unsigned value. We can avoid any ambiguity by using converter32 to convert from signed (int32_t) to unsigned (uint32_t).
  DevUBLOXGNSS::unsignedSigned32 converter32;
  //converter32.signed32 = 0; // Camera 1: delay the pulse by 0ns
  //converter32.signed32 = -100000000; // Camera 2: delay the pulse by 0.1s (100,000,000 ns)
  //converter32.signed32 = -200000000; // Camera 3: delay the pulse by 0.2s (200,000,000 ns)
  converter32.signed32 = -300000000; // Camera 4: delay the pulse by 0.3s (300,000,000 ns)
  //converter32.signed32 = -400000000; // Camera 5: delay the pulse by 0.4s (400,000,000 ns)
  //converter32.signed32 = -500000000; // Camera 6: delay the pulse by 0.5s (500,000,000 ns)
  //converter32.signed32 = -600000000; // Camera 7: delay the pulse by 0.6s (600,000,000 ns)
  //converter32.signed32 = -700000000; // Camera 8: delay the pulse by 0.7s (700,000,000 ns)
  //converter32.signed32 = -800000000; // Camera 9: delay the pulse by 0.8s (800,000,000 ns)
  //converter32.signed32 = -900000000; // Camera 10: delay the pulse by 0.9s (900,000,000 ns)
  myGNSS.addCfgValset(UBLOX_CFG_TP_USER_DELAY_TP1, converter32.unsigned32); // Avoid any ambiguity by using converter32 to convert from signed to unsigned

  myGNSS.addCfgValset(UBLOX_CFG_TP_TP1_ENA, 1); // Make sure the enable flag is set to enable the time pulse. (Set to 0 to disable.)
  myGNSS.addCfgValset(UBLOX_CFG_TP_USE_LOCKED_TP1, 1); // Tell the module to use PERIOD while locking and PERIOD_LOCK when locked to GNSS time
  myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_DEF, 0); // Tell the module that we want to set the period (not the frequency). PERIOD = 0. FREQ = 1.
  myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_LENGTH_DEF, 1); // Tell the module to set the pulse length (not the pulse ratio / duty). RATIO = 0. LENGTH = 1.

  // Now set the time pulse parameters
  if (myGNSS.sendCfgValset() == false)
  {
    Serial.println(F("VALSET failed!"));
  }
  else
  {
    Serial.println(F("Success!"));
  }
}

void loop()
{
  // Nothing to do here
}
