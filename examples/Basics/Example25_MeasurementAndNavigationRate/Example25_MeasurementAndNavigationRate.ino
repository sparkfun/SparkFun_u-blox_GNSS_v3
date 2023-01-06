/*
  Demonstrate get/setMeasurementRate and get/setNavigationRate
  By: Paul Clark
  SparkFun Electronics
  Date: March 30th, 2021
  License: MIT. See license file for more information.

  This example shows how to slow down the measurement and navigation rates.
  This should run on any GNSS module but has only been tested on the ZED_F9P and ZOE_M8Q.

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

unsigned long lastTime = 0; //Simple local timer. Used to calc the message interval.

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  // Uncomment the next line if you need to completely reset your module
  //myGNSS.factoryDefault(); delay(5000); // Reset everything and wait while the module restarts

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // Begin by printing the current measurement rate and navigation rate

  uint16_t measRate = myGNSS.getMeasurementRate(); //Get the measurement rate of this module
  Serial.print("Current measurement interval (ms): ");
  Serial.println(measRate);

  uint16_t navRate = myGNSS.getNavigationRate(); //Get the navigation rate of this module
  Serial.print("Current navigation ratio (cycles): ");
  Serial.println(navRate);

  // The measurement rate is the elapsed time between GNSS measurements, which defines the rate
  // e.g. 100 ms => 10 Hz, 1000 ms => 1 Hz, 10000 ms => 0.1 Hz.
  // Let's set the measurement rate (interval) to 5 seconds = 5000 milliseconds
  if (myGNSS.setMeasurementRate(5000, VAL_LAYER_RAM) == false) // Change the rate in RAM only - don't save to BBR
  {
    Serial.println(F("Could not set the measurement rate. Freezing."));
    while (1);
  }

  // setMeasurementRate will set i2cPollingWait to a quarter of the interval
  // Let's override that so we can poll the module more frequently and avoid timeouts
  myGNSS.setI2CpollingWait(25); // Set i2cPollingWait to 25ms

  // The navigation rate is the ratio between the number of measurements and the number of navigation solutions
  // e.g. 5 means five measurements for every navigation solution. Maximum value is 127
  // Let's set the navigation rate (ratio) to 12 to produce a solution every minute
  if (myGNSS.setNavigationRate(12, VAL_LAYER_RAM) == false) // Change the rate in RAM only - don't save to BBR
  {
    Serial.println(F("Could not set the navigation rate. Freezing."));
    while (1);
  }

  // Read and print the updated measurement rate and navigation rate

  measRate = myGNSS.getMeasurementRate(); //Get the measurement rate of this module
  Serial.print("New measurement interval (ms): ");
  Serial.println(measRate);

  navRate = myGNSS.getNavigationRate(); //Get the navigation rate of this module
  Serial.print("New navigation ratio (cycles): ");
  Serial.println(navRate);

  Serial.print("PVT data will be sent every ");
  Serial.print(measRate * navRate / 1000);
  Serial.println(" seconds");

  if ((measRate * navRate / 1000) == 60)
  {
    Serial.println(F("Fun fact: GPS time does not include the 18 leap seconds since 1980."));
    Serial.println(F("PVT data will be sent at the 42 second mark."));
  }

  lastTime = millis();
}

void loop()
{
  // i2cPollingWait will prevent us from thrashing the I2C bus

  if (myGNSS.getPVT()) //Check for new Position, Velocity, Time data. getPVT returns true if new data is available.
  {    
      long latitude = myGNSS.getLatitude();
      Serial.print(F("Lat: "));
      Serial.print(latitude);

      long longitude = myGNSS.getLongitude();
      Serial.print(F(" Long: "));
      Serial.print(longitude);

      //Calculate the interval since the last message
      Serial.print(F(" Interval: "));
      Serial.print(((float)(millis() - lastTime)) / 1000.0, 2);
      Serial.print(F("s"));

      Serial.println();

      lastTime = millis(); //Update lastTime
  }
}
