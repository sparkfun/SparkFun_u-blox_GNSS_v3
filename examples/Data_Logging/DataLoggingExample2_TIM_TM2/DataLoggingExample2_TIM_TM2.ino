/*
  Configuring the GNSS to automatically send TIM TM2 reports over I2C and log them to file on SD card
  By: Paul Clark
  SparkFun Electronics
  Date: October 18th, 2021
  License: MIT. See license file for more information.

  This example shows how to configure the u-blox GNSS to send TIM TM2 reports automatically
  and log the data to SD card in UBX format.

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

  Connecting the PPS (Pulse Per Second) breakout pin to the INT (Interrupt) pin with a jumper wire
  will cause a TIM TM2 message to be produced once per second. You can then study the timing of the
  pulse edges with nanosecond resolution!

  Note: TIM TM2 can only capture the timing of one rising edge and one falling edge per
  navigation solution. So with setNavigationFrequency set to 1Hz, we can only see the timing
  of one rising and one falling edge per second. If the frequency of the signal on the INT pin
  is higher than 1Hz, we will only be able to see the timing of the most recent edges.
  However, the module can count the number of rising edges too, at rates faster than the navigation rate.

  TIM TM2 messages are only produced when a rising or falling edge is detected on the INT pin.
  If you disconnect your PPS to INT jumper wire, the messages will stop.

  Data is logged in u-blox UBX format. Please see the u-blox protocol specification for more details.
  You can replay and analyze the data using u-center:
  https://www.u-blox.com/en/product/u-center

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

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

#define packetLength 36 // TIM TM2 is 28 + 8 bytes in length (including the sync chars, class, id, length and checksum bytes)
uint8_t *myBuffer; // Use myBuffer to hold the data while we write it to SD card

int dotsPrinted = 0; // Print dots in rows of 50 while waiting for a TIM TM2 message

// Callback: printTIMTM2data will be called when new TIM TM2 data arrives
// See u-blox_structs.h for the full definition of UBX_TIM_TM2_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoTIMTM2callback
//        /                  _____  This _must_ be UBX_TIM_TM2_data_t
//        |                 /                   _____ You can use any name you like for the struct
//        |                 |                  /
//        |                 |                  |
void printTIMTM2data(UBX_TIM_TM2_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("newFallingEdge: ")); // 1 if a new falling edge was detected
  Serial.print(ubxDataStruct->flags.bits.newFallingEdge);

  Serial.print(F(" newRisingEdge: ")); // 1 if a new rising edge was detected
  Serial.print(ubxDataStruct->flags.bits.newRisingEdge);

  Serial.print(F(" Rising Edge Counter: ")); // Rising edge counter
  Serial.print(ubxDataStruct->count);

  Serial.print(F(" towMsR: ")); // Time Of Week of rising edge (ms)
  Serial.print(ubxDataStruct->towMsR);

  Serial.print(F(" towSubMsR: ")); // Millisecond fraction of Time Of Week of rising edge in nanoseconds
  Serial.print(ubxDataStruct->towSubMsR);

  Serial.print(F(" towMsF: ")); // Time Of Week of falling edge (ms)
  Serial.print(ubxDataStruct->towMsF);

  Serial.print(F(" towSubMsF: ")); // Millisecond fraction of Time Of Week of falling edge in nanoseconds
  Serial.println(ubxDataStruct->towSubMsF);

  dotsPrinted = 0; // Reset dotsPrinted
}

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin(); // Start I2C communication with the GNSS

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

  // Create or open a file called "TIM_TM2.ubx" on the SD card.
  // If the file already exists, the new data is appended to the end of the file.
  myFile = SD.open("/TIM_TM2.ubx", FILE_WRITE);
  if(!myFile)
  {
    Serial.println(F("Failed to create UBX data file! Freezing..."));
    while (1);
  }

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful GNSS debug messages on Serial

  // TIM TM2 messages are 36 bytes long.
  // In this example, the data will arrive no faster than one message per second.
  // So, setting the file buffer size to 109 bytes should be more than adequate.
  // I.e. room for three messages plus an empty tail byte.
  myGNSS.setFileBufferSize(109); // setFileBufferSize must be called _before_ .begin

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

  myGNSS.setNavigationFrequency(1); //Produce one navigation solution per second

  myGNSS.setAutoTIMTM2callbackPtr(&printTIMTM2data); // Enable automatic TIM TM2 messages with callback to printTIMTM2data

  myGNSS.logTIMTM2(); // Enable TIM TM2 data logging

  myBuffer = new uint8_t[packetLength]; // Create our own buffer to hold the data while we write it to SD card  

  Serial.println(F("Press any key to stop logging."));
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  if (myGNSS.fileBufferAvailable() >= packetLength) // Check to see if a new packetLength-byte TIM TM2 message has been stored
  {
    myGNSS.extractFileBufferData(myBuffer, packetLength); // Extract exactly packetLength bytes from the UBX file buffer and put them into myBuffer

    myFile.write(myBuffer, packetLength); // Write exactly packetLength bytes from myBuffer to the ubxDataFile on the SD card

    //printBuffer(myBuffer); // Uncomment this line to print the data
  }

  if (Serial.available()) // Check if the user wants to stop logging
  {
    myFile.close(); // Close the data file
    Serial.println(F("\r\nLogging stopped. Freezing..."));
    while(1); // Do nothing more
  }

  Serial.print("."); // Print dots in rows of 50
  delay(50);
  if (++dotsPrinted > 50)
  {
    Serial.println();
    dotsPrinted = 0;
  }
}

// Print the buffer contents as Hexadecimal
// You should see:
// SYNC CHAR 1: 0xB5
// SYNC CHAR 2: 0x62
// CLASS: 0x0D for TIM
// ID: 0x03 for TM2
// LENGTH: 2-bytes Little Endian (0x1C00 = 28 bytes for TIM TM2)
// PAYLOAD: LENGTH bytes
// CHECKSUM_A
// CHECKSUM_B
// Please see the u-blox protocol specification for more details
void printBuffer(uint8_t *ptr)
{
  for (int i = 0; i < packetLength; i++)
  {
    if (ptr[i] < 16) Serial.print("0"); // Print a leading zero if required
    Serial.print(ptr[i], HEX); // Print the byte as Hexadecimal
    Serial.print(" ");
  }
  Serial.println();
}
