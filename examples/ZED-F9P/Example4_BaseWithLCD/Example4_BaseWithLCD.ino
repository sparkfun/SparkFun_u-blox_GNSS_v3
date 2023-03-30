/*
  Note: compiles OK with v2.0 but is untested. The previous example works fine though.
  
  Send UBX binary commands to enable RTCM sentences on u-blox ZED-F9P module
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 9th, 2019
  License: MIT. See license file for more information.

  This example does all steps to configure and enable a ZED-F9P as a base station:
    Begin Survey-In
    Once we've achieved 2m accuracy and 300s have passed, survey is complete
    Enable six RTCM messages
    Begin outputting RTCM bytes

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  Plug a SerLCD onto the Qwiic bus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Watch the output on the LCD or open the serial monitor at 115200 baud to see the output
*/

#define STAT_LED LED_BUILTIN // Change this if required

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

//#define SERIAL_OUTPUT // Uncomment this line to push the RTCM data to a Serial port

#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
SerLCD lcd;         // Initialize the library with default I2C address 0x72

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println(F("u-blox GNSS I2C Test"));

#ifdef SERIAL_OUTPUT
  // If our board supports it, we can output the RTCM data automatically on (e.g.) Serial1
  Serial1.begin(115200);
  myGNSS.setRTCMOutputPort(Serial1);
#endif

  Wire.begin();

  pinMode(STAT_LED, OUTPUT);
  digitalWrite(STAT_LED, LOW);

  lcd.begin(Wire);            //Set up the LCD for Serial communication at 9600bps
  lcd.setBacklight(0x4B0082); //indigo, a kind of dark purplish blue
  lcd.clear();
  lcd.print(F("LCD Ready"));

  myGNSS.begin(Wire);
  if (myGNSS.isConnected() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    lcd.setCursor(0, 1);
    lcd.print(F("No GNSS detected"));
    while (1)
      ;
  }

  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  lcd.setCursor(0, 1);
  lcd.print("GNSS Detected");

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Ensure RTCM3 is enabled
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR

  bool response = myGNSS.newCfgValset(); // Create a new Configuration Item VALSET message
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1); //Enable message 1005 to output through I2C port, message every second
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C, 10); // Enable message 1230 every 10 seconds
  response &= myGNSS.sendCfgValset(); // Send the VALSET

  // Use _UART1 for the above six messages to direct RTCM messages out UART1
  // _UART2, _USB and _SPI are also available
  // For example: response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1, 1);

  if (response == true)
  {
    Serial.println(F("RTCM messages enabled"));
  }
  else
  {
    Serial.println(F("RTCM failed to enable. Are you sure you have an ZED-F9P?"));
    while (1); //Freeze
  }

  //Check if Survey is in Progress before initiating one
  // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
  // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
  // You can either read the data from packetUBXNAVSVIN directly
  // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; and getSurveyInMeanAccuracy
  response = myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
  if (response == false)
  {
    Serial.println(F("Failed to get Survey In status. Freezing."));
    while (1)
      ; //Freeze
  }

  if (myGNSS.getSurveyInActive() == true) // Use the helper function
  {
    Serial.print(F("Survey already in progress."));
    lcd.setCursor(0, 2);
    lcd.print(F("Survey already going"));
  }
  else
  {
    //Start survey
    response = myGNSS.enableSurveyMode(60, 5.000, VAL_LAYER_RAM); //Enable Survey in, 60 seconds, 5.0m. Save setting in RAM layer only (not BBR)
    if (response == false)
    {
      Serial.println(F("Survey start failed"));
      lcd.setCursor(0, 3);
      lcd.print(F("Survey start failed. Freezing."));
      while (1)
        ;
    }
    Serial.println(F("Survey started. This will run until 60s has passed and less than 5m accuracy is achieved."));
  }

  while (Serial.available())
    Serial.read(); //Clear buffer

  lcd.clear();
  lcd.print(F("Survey in progress"));

  //Begin waiting for survey to complete
  while (myGNSS.getSurveyInValid() == false) // Call the helper function
  {
    if (Serial.available())
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

    // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
    // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
    // You can either read the data from packetUBXNAVSVIN directly
    // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; and getSurveyInMeanAccuracy
    response = myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
    if (response == true)
    {
      Serial.print(F("Press x to end survey - "));
      Serial.print(F("Time elapsed: "));
      Serial.print((String)myGNSS.getSurveyInObservationTime()); // Call the helper function

      lcd.setCursor(0, 1);
      lcd.print(F("Elapsed: "));
      lcd.print((String)myGNSS.getSurveyInObservationTime()); // Call the helper function

      Serial.print(F(" Accuracy: "));
      Serial.print((String)myGNSS.getSurveyInMeanAccuracy()); // Call the helper function
      Serial.println();

      lcd.setCursor(0, 2);
      lcd.print(F("Accuracy: "));
      lcd.print((String)myGNSS.getSurveyInMeanAccuracy()); // Call the helper function
    }
    else
    {
      Serial.println(F("SVIN request failed"));
    }

    delay(1000);
  }
  Serial.println(F("Survey valid!"));

  Serial.println(F("Base survey complete! RTCM now broadcasting."));
  lcd.clear();
  lcd.print(F("Transmitting RTCM"));

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)
  
}

void loop()
{
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  //Do anything you want. Call checkUblox() every second. ZED-F9P has TX buffer of 4k bytes.

  delay(250); //Don't pound too hard on the I2C bus
}

//This function gets called from the SparkFun u-blox Arduino Library.
//As each RTCM byte comes in you can specify what to do with it
//Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void DevUBLOXGNSS::processRTCM(uint8_t incoming)
{
  static uint16_t byteCounter = 0;

  //Pretty-print the HEX values to Serial
  if (myGNSS.rtcmFrameCounter == 1)
  {
    byteCounter = 0;
    Serial.println();
  }
  if (byteCounter % 16 == 0)
    Serial.println();
  
  if (incoming < 0x10) Serial.print(F("0"));
  Serial.print(incoming, HEX);
  Serial.print(F(" "));
  
  byteCounter++;
}
