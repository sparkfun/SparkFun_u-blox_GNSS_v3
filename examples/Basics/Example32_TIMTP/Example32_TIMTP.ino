/*
  Displaying the TIM TP timing information for the _next_ TP time pulse
  By: Paul Clark
  SparkFun Electronics
  Date: March 24th, 2023
  License: MIT. See license file for more information.

  This example shows how to query a u-blox module for the TIM TP time-pulse-of-week.
  This contains the timing information for the _next_ time pulse. I.e. it is output
  _ahead_ of time. PVT provides the time of the navigation solution which is usually
  aligned with the _previous_ pulse.

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

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
}

void loop()
{
/*
  // Read the time pulse of week and construct the microseconds
  if (myGNSS.getTIMTP()) //getTIMTP will return true if new fresh data is received
  {
    uint16_t week = myGNSS.getTIMTPweek(); //Get the time base week number
    uint32_t towMS = myGNSS.getTIMTPtowMS(); //Get the time base pulse-of-week (ms)
    uint32_t towSubMS = myGNSS.getTIMTPtowSubMS(); //Read the sub-millisecond data (ms * 2^-32)

    double tow = towMS;
    tow /= 1000.0; //Convert to seconds

    double subMS = towSubMS;
    subMS *= pow(2.0, -32.0); //Convert to ms
    subMS /= 1000.0; //Convert to seconds

    tow += subMS; //Construct the time of week
    
    Serial.print("TIM TP: week: ");
    Serial.print(week); //Print the time base week number
    Serial.print(" tow: ");
    Serial.println(tow, 6); //Print the time of week with six deimal places (i.e. show the microseconds)
  }
*/

  //getTIMTP and getPVT will return true if fresh data is available.
  //Note: both methods poll data from the GNSS:
  //      getTIMTP will poll TIM TP, wait and return true when the TIM TP data arrives.
  //      Then getPVT will poll NAV PVT, wait and return true when the NAV PVT data arrives.
  //      The TIM TP data will then be one second old.
  //      Because TIM TP provides the time of the _next_ time pulse and NAV PVT provides
  //      the time of the navigation solution close to the _previous_ time pulse,
  //      the two Epochs should be the same!
  if (myGNSS.getTIMTP() && myGNSS.getPVT())
  {
    // Read the time pulse of week as Unix Epoch
    // CAUTION! Assumes the time base is UTC and the week number is GPS
    uint32_t microsTP;
    uint32_t epochTP = myGNSS.getTIMTPAsEpoch(microsTP); //Read the next time pulse of week as Unix Epoch

    Serial.print("TIM TP  as Epoch: ");
    Serial.print(epochTP); //Print the time of the next pulse
    Serial.print(".");
    if (microsTP < 100000) Serial.print("0"); //Pad the zeros if needed
    if (microsTP < 10000) Serial.print("0");
    if (microsTP < 1000) Serial.print("0");
    if (microsTP < 100) Serial.print("0");
    if (microsTP < 10) Serial.print("0");
    Serial.println(microsTP);

    // Read the PVT time of week as Unix Epoch
    uint32_t microsPVT;
    uint32_t epochPVT = myGNSS.getUnixEpoch(microsPVT); //Read the time of week as Unix Epoch

    Serial.print("NAV PVT as Epoch: ");
    Serial.print(epochPVT); //Print the time of the previous navigation solution
    Serial.print(".");
    if (microsPVT < 100000) Serial.print("0"); //Pad the zeros if needed
    if (microsPVT < 10000) Serial.print("0");
    if (microsPVT < 1000) Serial.print("0");
    if (microsPVT < 100) Serial.print("0");
    if (microsPVT < 10) Serial.print("0");
    Serial.println(microsPVT);

    Serial.println();
  }
}
