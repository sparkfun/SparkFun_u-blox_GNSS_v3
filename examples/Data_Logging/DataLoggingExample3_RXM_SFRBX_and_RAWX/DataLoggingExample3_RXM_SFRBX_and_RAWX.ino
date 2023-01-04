/*
  Configuring the GNSS to automatically send RXM SFRBX and RAWX reports over I2C and log them to file on SD card
  By: Paul Clark
  SparkFun Electronics
  Date: October 18th, 2021
  License: MIT. See license file for more information.

  This example shows how to configure the u-blox GNSS to send RXM SFRBX and RAWX reports automatically
  and log the data to SD card in UBX format.

  ** Please note: this example will only work on u-blox ADR or High Precision GNSS or Time Sync products **

  ** Please note: this example will only work on processors like the Artemis which have plenty of RAM available **

  Data is logged in u-blox UBX format. Please see the u-blox protocol specification for more details.
  You can replay and analyze the data using u-center:
  https://www.u-blox.com/en/product/u-center
  Or you can use (e.g.) RTKLIB to analyze the data and extract your precise location or produce
  Post-Processed Kinematic data:
  https://rtklibexplorer.wordpress.com/
  http://rtkexplorer.com/downloads/rtklib-code/

  This code is intended to be run on the ESP32 Thing Plus USB-C
  but can be adapted by changing the chip select pin and SPI definitions:
  https://www.sparkfun.com/products/20168
  
  Hardware Connections:
  Please see: https://learn.sparkfun.com/tutorials/esp32-thing-plus-usb-c-hookup-guide
  Connect your GNSS breakout to the Thing Plus C using a Qwiic cable.
  Insert a formatted micro-SD card into the socket on the Thing Plus.
  Connect the Thing Plus to your computer using a USB-C cable.
  This code has been tested using version 2.0.5 of the Espressif Systems ESP32 board package on Arduino IDE 1.8.19.
  Select "SparkFun ESP32 Thing Plus C" as the board type.
  Press upload to upload the code onto the ESP32.
  Open the Serial Monitor at 115200 baud to see the output.

  To minimise I2C bus errors, it is a good idea to open the I2C pull-up split pad links on
  the u-blox module breakout.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481

*/

#include "FS.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

File myFile; //File that all GNSS data is written to

//Define the microSD (SPI) Chip Select pin. Adjust for your processor if necessary.
const int sd_cs = 5; //Thing Plus C

#define sdWriteSize 512 // Write data to the SD card in blocks of 512 bytes
#define fileBufferSize 16384 // Allocate 16KBytes of RAM for UBX message storage
uint8_t *myBuffer; // Use myBuffer to hold the data while we write it to SD card

unsigned long lastPrint; // Record when the last Serial print took place

// Note: we'll keep a count of how many SFRBX and RAWX messages arrive - but the count will not be completely accurate.
// If two or more SFRBX messages arrive together as a group and are processed by one call to checkUblox, the count will
// only increase by one.

int numSFRBX = 0; // Keep count of how many SFRBX message groups have been received (see note above)
int numRAWX = 0; // Keep count of how many RAWX message groups have been received (see note above)

// Callback: newSFRBX will be called when new RXM SFRBX data arrives
// See u-blox_structs.h for the full definition of UBX_RXMSFRBX_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRXMSFRBXcallback
//        /                  _____  This _must_ be UBX_RXM_SFRBX_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void newSFRBX(UBX_RXM_SFRBX_data_t *ubxDataStruct)
{
  numSFRBX++; // Increment the count
}

// Callback: newRAWX will be called when new RXM RAWX data arrives
// See u-blox_structs.h for the full definition of UBX_RXMRAWX_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRXMRAWXcallback
//        /             _____  This _must_ be UBX_RXM_RAWX_data_t
//        |            /                _____ You can use any name you like for the struct
//        |            |               /
//        |            |               |
void newRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct)
{
  numRAWX++; // Increment the count
}

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  pinMode(LED_BUILTIN, OUTPUT); // Flash LED_BUILTIN each time we write to the SD card
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin(); // Start I2C communication

  while (Serial.available()) // Make sure the Serial buffer is empty
  {
    Serial.read();
  }

  Serial.println(F("Press any key to start logging."));

  while (!Serial.available()) // Wait for the user to press a key
  {
    ; // Do nothing
  }

  delay(100); // Wait, just in case multiple characters were sent

  while (Serial.available()) // Empty the Serial buffer
  {
    Serial.read();
  }

  Serial.println("Initializing SD card...");

  // See if the card is present and can be initialized:
  if (!SD.begin(sd_cs))
  {
    Serial.println("Card failed, or not present. Freezing...");
    // don't do anything more:
    while (1);
  }
  Serial.println("SD card initialized.");

  // Create or open a file called "RXM_RAWX.ubx" on the SD card.
  // If the file already exists, the new data is appended to the end of the file.
  myFile = SD.open("/RXM_RAWX.ubx", FILE_WRITE);
  if(!myFile)
  {
    Serial.println(F("Failed to create UBX data file! Freezing..."));
    while (1);
  }

  //myGNSS.enableDebugging(); // Uncomment this line to enable lots of helpful GNSS debug messages on Serial
  //myGNSS.enableDebugging(Serial, true); // Or, uncomment this line to enable only the important GNSS debug messages on Serial

  // RAWX messages can be over 2KBytes in size, so we need to make sure we allocate enough RAM to hold all the data.
  // SD cards can occasionally 'hiccup' and a write takes much longer than usual. The buffer needs to be big enough
  // to hold the backlog of data if/when this happens.
  // getMaxFileBufferAvail will tell us the maximum number of bytes which the file buffer has contained.
  myGNSS.setFileBufferSize(fileBufferSize); // setFileBufferSize must be called _before_ .begin

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing..."));
    while (1);
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  // (This will also disable any "auto" messages that were enabled and saved by other examples and reduce the load on the I2C bus)
  //myGNSS.factoryDefault(); delay(5000);

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(1); //Produce one navigation solution per second (that's plenty for Precise Point Positioning)

  myGNSS.setAutoRXMSFRBXcallbackPtr(&newSFRBX); // Enable automatic RXM SFRBX messages with callback to newSFRBX

  myGNSS.logRXMSFRBX(); // Enable RXM SFRBX data logging

  myGNSS.setAutoRXMRAWXcallbackPtr(&newRAWX); // Enable automatic RXM RAWX messages with callback to newRAWX

  myGNSS.logRXMRAWX(); // Enable RXM RAWX data logging

  myBuffer = new uint8_t[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card  

  Serial.println(F("Press any key to stop logging."));

  lastPrint = millis(); // Initialize lastPrint
}

void loop()
{
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  while (myGNSS.fileBufferAvailable() >= sdWriteSize) // Check to see if we have at least sdWriteSize waiting in the buffer
  {
    digitalWrite(LED_BUILTIN, HIGH); // Flash LED_BUILTIN each time we write to the SD card

    myGNSS.extractFileBufferData(myBuffer, sdWriteSize); // Extract exactly sdWriteSize bytes from the UBX file buffer and put them into myBuffer

    myFile.write(myBuffer, sdWriteSize); // Write exactly sdWriteSize bytes from myBuffer to the ubxDataFile on the SD card

    // In case the SD writing is slow or there is a lot of data to write, keep checking for the arrival of new data
    myGNSS.checkUblox(); // Check for the arrival of new data and process it.
    myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

    digitalWrite(LED_BUILTIN, LOW); // Turn LED_BUILTIN off again
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  if (millis() > (lastPrint + 1000)) // Print the message count once per second
  {
    Serial.print(F("Number of message groups received: SFRBX: ")); // Print how many message groups have been received (see note above)
    Serial.print(numSFRBX);
    Serial.print(F(" RAWX: "));
    Serial.println(numRAWX);

    uint16_t maxBufferBytes = myGNSS.getMaxFileBufferAvail(); // Get how full the file buffer has been (not how full it is now)

    //Serial.print(F("The maximum number of bytes which the file buffer has contained is: ")); // It is a fun thing to watch how full the buffer gets
    //Serial.println(maxBufferBytes);

    if (maxBufferBytes > ((fileBufferSize / 5) * 4)) // Warn the user if fileBufferSize was more than 80% full
    {
      Serial.println(F("Warning: the file buffer has been over 80% full. Some data may have been lost."));
    }

    lastPrint = millis(); // Update lastPrint
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  if (Serial.available()) // Check if the user wants to stop logging
  {
    uint16_t remainingBytes = myGNSS.fileBufferAvailable(); // Check if there are any bytes remaining in the file buffer

    while (remainingBytes > 0) // While there is still data in the file buffer
    {
      digitalWrite(LED_BUILTIN, HIGH); // Flash LED_BUILTIN while we write to the SD card

      uint16_t bytesToWrite = remainingBytes; // Write the remaining bytes to SD card sdWriteSize bytes at a time
      if (bytesToWrite > sdWriteSize)
      {
        bytesToWrite = sdWriteSize;
      }

      myGNSS.extractFileBufferData(myBuffer, bytesToWrite); // Extract bytesToWrite bytes from the UBX file buffer and put them into myBuffer

      myFile.write(myBuffer, bytesToWrite); // Write bytesToWrite bytes from myBuffer to the ubxDataFile on the SD card

      remainingBytes -= bytesToWrite; // Decrement remainingBytes
    }

    digitalWrite(LED_BUILTIN, LOW); // Turn LED_BUILTIN off

    myFile.close(); // Close the data file

    Serial.println(F("Logging stopped. Freezing..."));
    while(1); // Do nothing more
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
}
