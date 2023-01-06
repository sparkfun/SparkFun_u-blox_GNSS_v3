/*
  Getting leap second event info as SNTP Leap Indicator, time to a leap second event and the number of leap seconds since GPS epoch
  By: UT2UH
  Date: April 14th, 2021
  License: MIT. See license file for more information.

  This example shows how to query a u-blox module for the leap second event info to cast to SNTP Leap Indicator enumeration.
  We also turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic dramatically.

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

typedef enum { 
  LI_NO_WARNING,      //Time leaping not scheduled
  LI_LAST_MINUTE_61_SEC,  //Last minute has 61 seconds
  LI_LAST_MINUTE_59_SEC,  //Last minute has 59 seconds
  LI_ALARM_CONDITION    //The NTP server's clock not synchronized
} ntp_LI_e;


void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();
  
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // Uncomment the next line if you need to completely reset your module
  //myGNSS.factoryDefault(); delay(5000); // Reset everything and wait while the module restarts

  myGNSS.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM); //Set the I2C port to output UBX only (turn off NMEA noise) - change it in the RAM layer only (not BBR)
  //myGNSS.saveConfiguration();        //Optional: Save the current settings to flash and BBR

  Serial.println(F("Compare Unix Epoch given with reference one from https://www.epochconverter.com/"));
  
}

void loop()
{
  //Query module. The module only responds when a new position is available
  if (myGNSS.getPVT())
  {
    // getUnixEpoch marks the PVT data as stale so you will get Unix time and PVT time on alternate seconds

    uint32_t us;  //microseconds returned by getUnixEpoch()
    uint32_t epoch = myGNSS.getUnixEpoch();
    Serial.print(F("Unix Epoch rounded: "));
    Serial.print(epoch, DEC);    
    epoch = myGNSS.getUnixEpoch(us);
    Serial.print(F("  Exact Unix Epoch: "));
    Serial.print(epoch, DEC);
    Serial.print(F("  micros: "));
    Serial.println(us, DEC);
    int32_t timeToLeapSecEvent;
    ntp_LI_e leapIndicator = (ntp_LI_e)myGNSS.getLeapIndicator(timeToLeapSecEvent);
    Serial.print(F("NTP LI: "));
    Serial.print(leapIndicator, DEC);
    switch (leapIndicator){
      case LI_NO_WARNING:
        Serial.print(F(" - No event scheduled"));
        break;
      case LI_LAST_MINUTE_61_SEC:
        Serial.print(F(" - last minute will end at 23:60"));
        break;
      case LI_LAST_MINUTE_59_SEC:
        Serial.print(F(" - last minute will end at 23:58"));
        break; 
      case LI_ALARM_CONDITION:
      default:
        Serial.print(F(" - Unknown (clock not synchronized)"));
        break; 
    }
    if (timeToLeapSecEvent < 0)
    {
      Serial.print(F(". Time since the last leap second event: "));
      Serial.println(timeToLeapSecEvent * -1, DEC);
    }
    else
    {
      Serial.print(F(". Time to the next leap second event: "));
      Serial.println(timeToLeapSecEvent, DEC);
    }

    sfe_ublox_ls_src_e leapSecSource;
    Serial.print(F("Leap seconds since GPS Epoch (Jan 6th, 1980): "));
    Serial.print(myGNSS.getCurrentLeapSeconds(leapSecSource), DEC);
    switch (leapSecSource){
      case SFE_UBLOX_LS_SRC_DEFAULT:
        Serial.print(F(" - hardcoded"));
        break;
      case SFE_UBLOX_LS_SRC_GLONASS:
        Serial.print(F(" - derived from GPS and GLONASS time difference"));
        break;
      case SFE_UBLOX_LS_SRC_GPS:
        Serial.print(F(" - according to GPS"));
        break; 
      case SFE_UBLOX_LS_SRC_SBAS:
        Serial.print(F(" - according to SBAS"));
        break;
      case SFE_UBLOX_LS_SRC_BEIDOU:
        Serial.print(F(" - according to BeiDou"));
        break;
      case SFE_UBLOX_LS_SRC_GALILEO:
        Serial.print(F(" - according to Galileo"));
        break;
      case SFE_UBLOX_LS_SRC_AIDED:
        Serial.print(F(" - last minute will end at 23:58"));
        break; 
      case SFE_UBLOX_LS_SRC_CONFIGURED:
        Serial.print(F(" - as configured)"));
        break;
      case SFE_UBLOX_LS_SRC_UNKNOWN:
      default:
        Serial.print(F(" - source unknown"));
        break;
    }
    Serial.println();
  }
  Serial.println();
}
