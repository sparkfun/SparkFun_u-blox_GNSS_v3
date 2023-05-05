/*
  Get the RTCM 1005 sentence using getLatestRTCM1005
  By: Paul Clark
  SparkFun Electronics
  Date: May 4th, 2023
  License: MIT. See license file for more information.

  This example shows how to perform a survey-in and then enable RTCM sentences over I2C.
  It then demonstrates how to use the new getLatestRTCM1005 function to retrieve the latest RTCM 1005 message.
  getLatestRTCM1005 returns immediately - it is not blocking.
  It returns:
    0 if no data is available
    1 if the data is valid but is stale (you have read it before)
    2 if the data is valid and fresh

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

void setup()
{

  delay(1000);

  Serial.begin(115200);
  Serial.println(F("SparkFun u-blox GNSS Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  while (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
    delay(1000);
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //We need to do a survey-in before the ZED will generate RTCM 1005

  //Check if Survey is in Progress before initiating one
  bool response = myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
  
  if (response == false) // Check if fresh data was received
  {
    Serial.println(F("Failed to get Survey In status. Freezing..."));
    while (1); //Freeze
  }

  if (myGNSS.getSurveyInActive() == true) // Use the helper function
  {
    Serial.print(F("Survey already in progress."));
  }
  else
  {
    //Start survey - define the minimum observationTime and requiredAccuracy
    uint32_t observationTime = 60; float requiredAccuracy = 5.0; //  60 seconds, 5.0m
    
    response = myGNSS.enableSurveyModeFull(observationTime, requiredAccuracy, VAL_LAYER_RAM); //Enable Survey in. Save setting in RAM layer only (not BBR)

    if (response == false)
    {
      Serial.println(F("Survey start failed. Freezing..."));
      while (1); //Freeze
    }

    Serial.println(F("Survey started."));
    Serial.print(F("This will run until "));
    Serial.print(observationTime);
    Serial.print(F("s have passed _and_ better than "));
    Serial.print(requiredAccuracy, 2);
    Serial.println(F("m accuracy is achieved."));
    Serial.println();
  }

  while(Serial.available()) Serial.read(); //Clear buffer
  
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Wait for survey to complete

  while (myGNSS.getSurveyInValid() == false) // Call the helper function
  {
    if(Serial.available())
    {
      byte incoming = Serial.read();
      if (incoming == 'x')
      {
        //Stop survey mode
        response = myGNSS.disableSurveyMode(); //Disable survey
        Serial.println(F("Survey stopped"));
        break;
      }
    }

    response = myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
    
    if (response == true) // Check if fresh data was received
    {
      Serial.print(F("\r\nPress x to end survey - "));
      Serial.print(F("Time elapsed: "));
      Serial.print((String)myGNSS.getSurveyInObservationTimeFull()); // Call the helper function
      Serial.print(F(" ("));
      Serial.print((String)myGNSS.packetUBXNAVSVIN->data.dur); // Read the survey-in duration directly from packetUBXNAVSVIN

      Serial.print(F(") Accuracy: "));
      Serial.print((String)myGNSS.getSurveyInMeanAccuracy()); // Call the helper function
      Serial.print(F(" ("));
      // Read the mean accuracy directly from packetUBXNAVSVIN and manually convert from mm*0.1 to m
      float meanAcc = ((float)myGNSS.packetUBXNAVSVIN->data.meanAcc) / 10000.0;
      Serial.print((String)meanAcc); 
      Serial.println(F(")"));
    }
    else
    {
      Serial.println(F("\r\nSVIN request failed"));
    }
  }

  Serial.println(F("Base survey complete. Configuring RTCM."));

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Disable or enable various RTCM sentences over the I2C interface

  myGNSS.setI2COutput(COM_TYPE_NMEA | COM_TYPE_UBX | COM_TYPE_RTCM3); // Turn on UBX, NMEA and RTCM sentences on I2C

  myGNSS.newCfgValset(VAL_LAYER_RAM); // Use cfgValset to disable / enable individual RTCM messages. Save setting in RAM layer only (not BBR)
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1); // Enable RTCM 1005 (Stationary Antenna Reference Point, No Height Information) at current navigation rate
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_I2C, 1); // Enable RTCM 1074 (GPS MSM4). RTCM 1077 (GPS MSM7) is higher resolution but is almost twice as many bytes
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_I2C, 1); // Enable RTCM 1084 (GLONASS MSM4). RTCM 1087 (GLONASS MSM7) is higher resolution
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_I2C, 1); // Enable RTCM 1094 (Galileo MSM4). RTCM 1097 (Galileo MSM7) is higher resolution
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_I2C, 1); // Enable RTCM 1124 (BeiDou MSM4). RTCM 1127 (BeiDou MSM7) is higher resolution
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C, 10); // Enable RTCM message 1230 (GLONASS L1 and L2 Code-Phase Biases) every 10 seconds
  if (myGNSS.sendCfgValset()) // Send the configuration VALSET
    Serial.println(F("RTCM messages were configured successfully"));
  else
    Serial.println(F("RTCM message configuration failed!"));

  //myGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save only the ioPort and message settings to NVM

  //myGNSS.setRTCMOutputPort(Serial); // Uncomment this line to echo all RTCM data to Serial for debugging

  if (sizeof(double) != 8) // Check double is 64-bit
    Serial.println(F("double is not 64-bit. ECEF resolution may be limited!"));
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void loop()
{
  // getLatestRTCM1005 calls checkUblox for us. We don't need to do it here

  RTCM_1005_data_t data; // Storage for the RTCM 1005 data
  uint8_t result = myGNSS.getLatestRTCM1005(&data); // Get the latest RTCM 1005 data (if any)
  
  if (result == 0)
  {
    Serial.println(F("No RTCM 1005 data available"));
  }
  else if (result == 1)
  {
    Serial.println(F("RTCM 1005 data is available but is stale"));
  }
  else // if (result == 2)
  {
    double x = data.AntennaReferencePointECEFX;
    x /= 10000.0; // Convert to m
    double y = data.AntennaReferencePointECEFY;
    y /= 10000.0; // Convert to m
    double z = data.AntennaReferencePointECEFZ;
    z /= 10000.0; // Convert to m

    Serial.print(F("Latest RTCM "));
    Serial.print(data.MessageNumber);
    Serial.print(F(":  ARP ECEF-X: "));
    Serial.print(x, 4); // 4 decimal places
    Serial.print(F("  Y: "));
    Serial.print(y, 4); // 4 decimal places
    Serial.print(F("  Z: "));
    Serial.println(z, 4); // 4 decimal places
  }

  delay(250);
}
