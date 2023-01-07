/*
  Configuring port settings using the newer getVal/setVal methods
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 23rd, 2020
  License: MIT. See license file for more information.

  This example shows how to query a u-blox module for its UART1 settings and
  then change them if the settings aren't what we want.

  Note: getVal/setVal/delVal are only support in u-blox protocol versions 27 and higher.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard
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

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // These key values are hard coded and defined in u-blox_config_keys.h.
  // You can obtain them from the ZED-F9P interface description doc
  // or from u-center's Messages->CFG->VALSET window. Keys must be 32-bit.

  // We can read the settings using 'unsafe' methods - as shown below.
  // They are 'unsafe' because we don't know the the I2C transaction was successful.

  Serial.println("Using the 'unsafe' getVal methods:");
  
  // Read the settings from RAM (what the module is running right now, not BBR, Flash, or default)
  // The INPROT values are all single-bit L (bool) but are transferred as 8-bit uint8_t
  uint8_t currentUART1Setting_ubx = myGNSS.getVal8(UBLOX_CFG_UART1INPROT_UBX);
  uint8_t currentUART1Setting_nmea = myGNSS.getVal8(UBLOX_CFG_UART1INPROT_NMEA);
  uint8_t currentUART1Setting_rtcm3 = myGNSS.getVal8(UBLOX_CFG_UART1INPROT_RTCM3X);

  Serial.print("currentUART1Setting_ubx: ");
  Serial.println(currentUART1Setting_ubx);
  Serial.print("currentUART1Setting_nmea: ");
  Serial.println(currentUART1Setting_nmea);
  Serial.print("currentUART1Setting_rtcm3: ");
  Serial.println(currentUART1Setting_rtcm3);

  // Here are the safe versions:
  Serial.println("Using the safe getVal methods:");
  
  bool response = true;
  // If we AND (&=) the result of each getVal into response, response will be false if any one getVal fails

  response &= myGNSS.getVal8(UBLOX_CFG_UART1INPROT_UBX, &currentUART1Setting_ubx, VAL_LAYER_RAM);
  response &= myGNSS.getVal8(UBLOX_CFG_UART1INPROT_NMEA, &currentUART1Setting_nmea, VAL_LAYER_RAM);
  response &= myGNSS.getVal8(UBLOX_CFG_UART1INPROT_RTCM3X, &currentUART1Setting_rtcm3, VAL_LAYER_RAM);

  if (response == false)
    Serial.println("VALGET failed");
  else
    Serial.println("VALGET succeeded");

  Serial.print("currentUART1Setting_ubx: ");
  Serial.println(currentUART1Setting_ubx);
  Serial.print("currentUART1Setting_nmea: ");
  Serial.println(currentUART1Setting_nmea);
  Serial.print("currentUART1Setting_rtcm3: ");
  Serial.println(currentUART1Setting_rtcm3);
  
  //Check if NMEA and RTCM are enabled for UART1
  if (currentUART1Setting_ubx == 0 || currentUART1Setting_nmea == 0)
  {
    Serial.println("Updating UART1 configuration");

    //setVal sets the values for RAM, BBR, and Flash automatically so no .saveConfiguration() is needed
    response = true; // Reset response just in case it is already false
    response &= myGNSS.setVal8(UBLOX_CFG_UART1INPROT_UBX, 1);    //Enable UBX on UART1 Input
    response &= myGNSS.setVal8(UBLOX_CFG_UART1INPROT_NMEA, 1);   //Enable NMEA on UART1 Input
    response &= myGNSS.setVal8(UBLOX_CFG_UART1INPROT_RTCM3X, 0); //Disable RTCM on UART1 Input

    if (response == false)
      Serial.println("VALSET failed");
    else
      Serial.println("VALSET succeeded");
  }
  else
    Serial.println("No port change needed");

  // Change speed of UART2
  // UBLOX_CFG_UART2_BAUDRATE is 32-bit U4 (uint32_t)
  uint32_t currentUART2Baud;
  
  response = myGNSS.getVal32(UBLOX_CFG_UART2_BAUDRATE, &currentUART2Baud, VAL_LAYER_RAM);

  if (response)
  {
    Serial.print("currentUART2Baud: ");
    Serial.println(currentUART2Baud);
  }
  else
  {
    Serial.print("VALGET failed!");
  }

  if (currentUART2Baud != 57600)
  {
    response = myGNSS.setVal32(UBLOX_CFG_UART2_BAUDRATE, 57600);
    if (response == false)
      Serial.println("VALSET failed");
    else
      Serial.println("VALSET succeeded");
  }
  else
    Serial.println("No baud change needed");

  Serial.println("Done");
}

void loop()
{
}
