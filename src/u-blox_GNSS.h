/*
  An Arduino Library which allows you to communicate seamlessly with u-blox GNSS modules using the Configuration Interface

  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15136
  https://www.sparkfun.com/products/16481
  https://www.sparkfun.com/products/16344
  https://www.sparkfun.com/products/18037
  https://www.sparkfun.com/products/18719
  https://www.sparkfun.com/products/18774
  https://www.sparkfun.com/products/19663
  https://www.sparkfun.com/products/17722

  Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
  v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020
  v3.0 rework by Paul Clark @ SparkFun Electronics, December 8th, 2022

  https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3

  This library is an updated version of the popular SparkFun u-blox GNSS Arduino Library.
  v3 uses the u-blox Configuration Interface (VALSET and VALGET) to:
  detect the module (during begin); configure message intervals; configure the base location; etc..

  This version of the library will not work with older GNSS modules.
  It is specifically written for newer modules like the ZED-F9P, ZED-F9R and MAX-M10S.
  For older modules, please use v2 of the library: https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.19

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  The MIT License (MIT)
  Copyright (c) 2018 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

// Uncomment the next line (or add SFE_UBLOX_REDUCED_PROG_MEM as a compiler directive) to reduce the amount of program memory used by the library
// #define SFE_UBLOX_REDUCED_PROG_MEM // Uncommenting this line will delete the minor debug messages to save memory

// Uncomment the next line (or add SFE_UBLOX_DISABLE_AUTO_NMEA as a compiler directive) to reduce the amount of program memory used by the library
// #define SFE_UBLOX_DISABLE_AUTO_NMEA // Uncommenting this line will disable auto-NMEA support to save memory

// Uncomment the next line (or add SFE_UBLOX_DISABLE_RTCM_LOGGING as a compiler directive) to reduce the amount of program memory used by the library
// #define SFE_UBLOX_DISABLE_RTCM_LOGGING // Uncommenting this line will disable RTCM logging support to save memory

// Uncomment the next line (or add SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT as a compiler directive) to reduce the amount of program memory used by the library
// #define SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT // Uncommenting this line will disable the RAM-heavy RXM and NAV-SAT support to save memory

// Uncomment the next line (or add SFE_UBLOX_DISABLE_ESF as a compiler directive) to reduce the amount of program memory used by the library
// #define SFE_UBLOX_DISABLE_ESF // Uncommenting this line will disable the ESF support to save memory

// Uncomment the next line (or add SFE_UBLOX_DISABLE_HNR as a compiler directive) to reduce the amount of program memory used by the library
// #define SFE_UBLOX_DISABLE_HNR // Uncommenting this line will disable the HNR support to save memory

// The code exceeds the program memory on the ATmega328P (Arduino Uno), so let's delete the minor debug messages and disable auto-NMEA and RAM-heavy support anyway
// However, the ATmega2560 and ATmega1280 _do_ have enough memory, so let's exclude those
#if !defined(SFE_UBLOX_REDUCED_PROG_MEM) && defined(ARDUINO_ARCH_AVR) && !defined(ARDUINO_AVR_MEGA2560) && !defined(ARDUINO_AVR_MEGA) && !defined(ARDUINO_AVR_ADK)
#define SFE_UBLOX_REDUCED_PROG_MEM
#endif
#if !defined(SFE_UBLOX_DISABLE_AUTO_NMEA) && defined(ARDUINO_ARCH_AVR) && !defined(ARDUINO_AVR_MEGA2560) && !defined(ARDUINO_AVR_MEGA) && !defined(ARDUINO_AVR_ADK)
#define SFE_UBLOX_DISABLE_AUTO_NMEA
#endif
#if !defined(SFE_UBLOX_DISABLE_RTCM_LOGGING) && defined(ARDUINO_ARCH_AVR) && !defined(ARDUINO_AVR_MEGA2560) && !defined(ARDUINO_AVR_MEGA) && !defined(ARDUINO_AVR_ADK)
#define SFE_UBLOX_DISABLE_RTCM_LOGGING
#endif
#if !defined(SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT) && defined(ARDUINO_ARCH_AVR) && !defined(ARDUINO_AVR_MEGA2560) && !defined(ARDUINO_AVR_MEGA) && !defined(ARDUINO_AVR_ADK)
#define SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
#endif

#include <Arduino.h>
#include "u-blox_config_keys.h"
#include "u-blox_structs.h"
#include "u-blox_external_typedefs.h"
#include "u-blox_Class_and_ID.h"
#include "sfe_bus.h"

// Define a digital pin to aid debugging
// Leave set to -1 if not needed
const int debugPin = -1;

class DevUBLOXGNSS
{
public:
  DevUBLOXGNSS(void);
  ~DevUBLOXGNSS(void);

  // New in v3.0: hardware interface is abstracted
  bool isConnected(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

protected:
  enum commTypes
  {
    COMM_TYPE_I2C = 0,
    COMM_TYPE_SERIAL,
    COMM_TYPE_SPI
  } _commType = COMM_TYPE_I2C; // Controls which port we look to for incoming bytes
  bool init(uint16_t maxWait, bool assumeSuccess);
  void setCommunicationBus(SparkFun_UBLOX_GNSS::GNSSDeviceBus &theBus);
  // For I2C, ping the _address
  // Not Applicable for SPI and Serial
  bool ping();
  // For Serial, return Serial.available()
  // For I2C, read registers 0xFD and 0xFE. Return bytes available as uint16_t
  // Not Applicable for SPI
  uint16_t available();
  // For Serial, do Serial.write
  // For I2C, push data to register 0xFF. Chunkify if necessary. Prevent single byte writes as these are illegal
  // For SPI, writing bytes will also read bytes simultaneously. Read data is _ignored_ here. Use writeReadBytes
  uint8_t writeBytes(uint8_t *data, uint8_t length);
  // For SPI, writing bytes will also read bytes simultaneously. Read data is returned in readData
  uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length);
  void startWriteReadByte();
  void writeReadByte(const uint8_t *data, uint8_t *readData);
  void writeReadByte(const uint8_t data, uint8_t *readData);
  void endWriteReadByte();
  // For Serial, attempt Serial.read
  // For I2C, read from register 0xFF
  // For SPI, read the byte while writing 0xFF
  uint8_t readBytes(uint8_t *data, uint8_t length);
  // Flag to indicate if we are connected to UART1 or UART2
  // Needed to select the correct config items when enabling a periodic message
  bool _UART2 = false; // Default to UART1

  // These lock / unlock functions can be used if you have multiple tasks writing to the bus.
  // The idea is that in a RTOS you override this class and the functions in which you take and give a mutex.
  virtual bool createLock(void) { return true; }
  virtual bool lock(void) { return true; }
  virtual void unlock(void) {}
  virtual void deleteLock(void) {}

public:
  void connectedToUART2(bool connected = true) { _UART2 = connected; }

  // Depending on the sentence type the processor will load characters into different arrays
  enum sfe_ublox_sentence_types_e
  {
    SFE_UBLOX_SENTENCE_TYPE_NONE = 0,
    SFE_UBLOX_SENTENCE_TYPE_NMEA,
    SFE_UBLOX_SENTENCE_TYPE_UBX,
    SFE_UBLOX_SENTENCE_TYPE_RTCM
  } currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;

  // New in v2.0: allow the payload size for packetCfg to be changed
  bool setPacketCfgPayloadSize(size_t payloadSize); // Set packetCfgPayloadSize
  size_t getPacketCfgSpaceRemaining();              // Returns the number of free bytes remaining in packetCfgPayload

  void end(void); // Stop all automatic message processing. Free all used RAM

  void setI2CpollingWait(uint8_t newPollingWait_ms); // Allow the user to change the I2C polling wait if required
  void setSPIpollingWait(uint8_t newPollingWait_ms); // Allow the user to change the SPI polling wait if required

  // Set the max number of bytes set in a given I2C transaction
  uint8_t i2cTransactionSize = 32; // Default to ATmega328 limit

  // Control the size of the internal I2C transaction amount
  void setI2CTransactionSize(uint8_t transactionSize);
  uint8_t getI2CTransactionSize(void);

  // Control the size of the internal I2C transaction amount
  void setSpiTransactionSize(uint8_t transactionSize);
  uint8_t getSpiTransactionSize(void);

  // Control the size of the SPI transfer buffer. If the buffer isn't big enough, we'll start to lose bytes
  void setSpiBufferSize(size_t bufferSize);
  size_t getSpiBufferSize(void);

  // A dedicated buffer for RTCM data - separate to the logging buffer
  // RTCM data can be stored here and then extracted - avoiding processRTCM
  // This is useful on SPI systems, avoiding bus collisions between checkUblox/processRTCM
  // and pushing the RTCM data to (e.g.) Ethernet
  void setRTCMBufferSize(uint16_t bufferSize);                             // Set the size of the RTCM buffer. This must be called _before_ .begin.
  uint16_t getRTCMBufferSize(void);                                        // Return the size of the RTCM buffer
  uint16_t extractRTCMBufferData(uint8_t *destination, uint16_t numBytes); // Extract numBytes of data from the RTCM buffer. Copy it to destination. It is the user's responsibility to ensure destination is large enough.
  uint16_t rtcmBufferAvailable(void);                                      // Returns the number of bytes available in the RTCM buffer which are waiting to be read
  void clearRTCMBuffer(void);                                              // Empty the RTCM buffer - discard all contents

  // Control the size of maxNMEAByteCount
  void setMaxNMEAByteCount(int8_t newMax);
  int8_t getMaxNMEAByteCount(void);

// Enable debug messages using the chosen Serial port (Stream)
// Boards like the RedBoard Turbo use SerialUSB (not Serial).
// But other boards like the SAMD51 Thing Plus use Serial (not SerialUSB).
// These lines let the code compile cleanly on as many SAMD boards as possible.
#if defined(ARDUINO_ARCH_SAMD)                                                        // Is this a SAMD board?
#if defined(USB_VID)                                                                  // Is the USB Vendor ID defined?
#if (USB_VID == 0x1B4F)                                                               // Is this a SparkFun board?
#if !defined(ARDUINO_SAMD51_THING_PLUS) & !defined(ARDUINO_SAMD51_MICROMOD)           // If it is not a SAMD51 Thing Plus or SAMD51 MicroMod
  void enableDebugging(Print &debugPort = SerialUSB, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#else
  void enableDebugging(Print &debugPort = Serial, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#endif
#else
  void enableDebugging(Print &debugPort = Serial, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#endif
#else
  void enableDebugging(Print &debugPort = Serial, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#endif
#else
  void enableDebugging(Print &debugPort = Serial, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#endif

  void disableDebugging(void);                       // Turn off debug statements
  void debugPrint(char *message);                    // Safely print debug statements
  void debugPrintln(char *message);                  // Safely print debug statements
  const char *statusString(sfe_ublox_status_e stat); // Pretty print the return value

  // Check for the arrival of new I2C/Serial data
  // Changed in V1.8.1: provides backward compatibility for the examples that call checkUblox directly
  // Will default to using packetCfg to look for explicit autoPVT packets so they get processed correctly by processUBX
  bool checkUblox(uint8_t requestedClass = 0, uint8_t requestedID = 0); // Checks module with user selected commType

  bool checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);    // Method for I2C polling of data, passing any new bytes to process()
  bool checkUbloxSerial(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID); // Method for serial polling of data, passing any new bytes to process()
  bool checkUbloxSpi(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);    // Method for spi polling of data, passing any new bytes to process()
  bool processSpiBuffer(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID); // Called by checkUbloxSpi to process any backlog data in the spiBuffer

  // Process the incoming data

  void process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);             // Processes NMEA and UBX binary sentences one byte at a time
  void processNMEA(char incoming) __attribute__((weak));                                                           // Given a NMEA character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries
  sfe_ublox_sentence_types_e processRTCMframe(uint8_t incoming, uint16_t *rtcmFrameCounter) __attribute__((weak)); // Monitor the incoming bytes for start and length bytes
  void processRTCM(uint8_t incoming) __attribute__((weak));                                                        // Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.
  void processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);          // Given a character, file it away into the uxb packet structure
  void processUBXpacket(ubxPacket *msg);                                                                           // Once a packet has been received and validated, identify this packet's class/id and update internal flags

  // Send I2C/Serial/SPI commands to the module

  void calcChecksum(ubxPacket *msg);                                                                                               // Sets the checksumA and checksumB of a given messages
  sfe_ublox_status_e sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool expectACKonly = false); // Given a packet and payload, send everything including CRC bytes, return true if we got a response
  sfe_ublox_status_e sendI2cCommand(ubxPacket *outgoingUBX);
  void sendSerialCommand(ubxPacket *outgoingUBX);
  sfe_ublox_status_e sendSpiCommand(ubxPacket *outgoingUBX);
  void spiTransfer(const uint8_t byteToTransfer);

  void printPacket(ubxPacket *packet, bool alwaysPrintPayload = false); // Useful for debugging

  // After sending a message to the module, wait for the expected response (data+ACK or just data)

  sfe_ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = kUBLOXGNSSDefaultMaxWait);   // Poll the module until a config packet and an ACK is received, or just an ACK
  sfe_ublox_status_e waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = kUBLOXGNSSDefaultMaxWait); // Poll the module until a config packet is received

  // Check if any callbacks need to be called
  void checkCallbacks(void);

  // Push (e.g.) RTCM or Assist Now data directly to the module
  // Warning: this function does not check that the data is valid. It is the user's responsibility to ensure the data is valid before pushing.
  //
  // For SPI: callProcessBuffer defaults to true and forces pushRawData to call processSpiBuffer in between push transactions.
  // This is to try and prevent incoming data being 'lost' during large (bi-directional) pushes.
  // If you are only pushing limited amounts of data and/or will be calling checkUblox manually, it might be advantageous to set callProcessBuffer to false.
  //
  // Likewise for Serial: callProcessBuffer defaults to true and forces pushRawData to call checkUbloxSerial in between pushing data.
  // This is to try and prevent incoming data being 'lost' (overflowing the serial RX buffer) during a large push.
  // If you are only pushing limited amounts of data and/or will be calling checkUblox manually, it might be advantageous to set callProcessBuffer to false.
  bool pushRawData(uint8_t *dataBytes, size_t numDataBytes, bool callProcessBuffer = true);
#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  // RTCM parsing - used inside pushRawData
protected:
  void parseRTCM1005(uint8_t *dataBytes, size_t numDataBytes);
  void parseRTCM1006(uint8_t *dataBytes, size_t numDataBytes);

public:
#endif

// Push MGA AssistNow data to the module.
// Check for UBX-MGA-ACK responses if required (if mgaAck is YES or ENQUIRE).
// Wait for maxWait millis after sending each packet (if mgaAck is NO).
// Return how many bytes were pushed successfully.
// If skipTime is true, any UBX-MGA-INI-TIME_UTC or UBX-MGA-INI-TIME_GNSS packets found in the data will be skipped,
// allowing the user to override with their own time data with setUTCTimeAssistance.
// offset allows a sub-set of the data to be sent - starting from offset.
#define defaultMGAdelay 7 // Default to waiting for 7ms between each MGA message
  size_t pushAssistNowData(const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(size_t offset, bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(size_t offset, bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);

// Provide initial time assistance
#define defaultMGAINITIMEtAccS 2  // Default to setting the seconds time accuracy to 2 seconds
#define defaultMGAINITIMEtAccNs 0 // Default to setting the nanoseconds time accuracy to zero
#define defaultMGAINITIMEsource 0 // Set default source to none, i.e. on receipt of message (will be inaccurate!)
  bool setUTCTimeAssistance(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint32_t nanos = 0,
                            uint16_t tAccS = defaultMGAINITIMEtAccS, uint32_t tAccNs = defaultMGAINITIMEtAccNs, uint8_t source = defaultMGAINITIMEsource,
                            sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);

  // Provide initial position assistance
  // The units for ecefX/Y/Z and posAcc (stddev) are cm.
  bool setPositionAssistanceXYZ(int32_t ecefX, int32_t ecefY, int32_t ecefZ, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  // The units for lat and lon are degrees * 1e-7 (WGS84)
  // The units for alt (WGS84) and posAcc (stddev) are cm.
  bool setPositionAssistanceLLH(int32_t lat, int32_t lon, int32_t alt, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);

  // Find the start of the AssistNow Offline (UBX_MGA_ANO) data for the chosen day
  // The daysIntoFture parameter makes it easy to get the data for (e.g.) tomorrow based on today's date
  // Returns numDataBytes if unsuccessful
  // TO DO: enhance this so it will find the nearest data for the chosen day - instead of an exact match
  size_t findMGAANOForDate(const String &dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture = 0);
  size_t findMGAANOForDate(const uint8_t *dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture = 0);

// Read the whole navigation data base. The receiver will send all available data from its internal database.
// Data is written to dataBytes. Set maxNumDataBytes to the (maximum) size of dataBytes.
// If the database exceeds maxNumDataBytes, the excess bytes will be lost.
// The function returns the number of database bytes written to dataBytes.
// The return value will be equal to maxNumDataBytes if excess data was received.
// The function will timeout after maxWait milliseconds - in case the final UBX-MGA-ACK was missed.
#define defaultNavDBDMaxWait 3100
  size_t readNavigationDatabase(uint8_t *dataBytes, size_t maxNumDataBytes, uint16_t maxWait = defaultNavDBDMaxWait);

  // Support for data logging
  void setFileBufferSize(uint16_t bufferSize);                             // Set the size of the file buffer. This must be called _before_ .begin.
  uint16_t getFileBufferSize(void);                                        // Return the size of the file buffer
  uint16_t extractFileBufferData(uint8_t *destination, uint16_t numBytes); // Extract numBytes of data from the file buffer. Copy it to destination. It is the user's responsibility to ensure destination is large enough.
  uint16_t fileBufferAvailable(void);                                      // Returns the number of bytes available in file buffer which are waiting to be read
  uint16_t getMaxFileBufferAvail(void);                                    // Returns the maximum number of bytes which the file buffer has contained. Handy for checking the buffer is large enough to handle all the incoming data.
  void clearFileBuffer(void);                                              // Empty the file buffer - discard all contents
  void clearMaxFileBufferAvail(void);                                      // Reset fileBufferMaxAvail

  // Specific commands

  // Port configurations
  bool setI2CAddress(uint8_t deviceAddress, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxTime = kUBLOXGNSSDefaultMaxWait);                                // Changes the I2C address of the u-blox module
  bool setSerialRate(uint32_t baudrate, uint8_t uartPort = COM_PORT_UART1, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxTime = kUBLOXGNSSDefaultMaxWait); // Changes the serial baud rate of the u-blox module, uartPort should be COM_PORT_UART1/2

  bool setI2COutput(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Configure I2C port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUART1Output(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Configure UART1 port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUART2Output(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Configure UART2 port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUSBOutput(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Configure USB port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setSPIOutput(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Configure SPI port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof

  bool setI2CInput(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Configure I2C port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUART1Input(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Configure UART1 port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUART2Input(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Configure UART2 port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUSBInput(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Configure USB port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setSPIInput(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Configure SPI port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof

  void setNMEAOutputPort(Print &outputPort); // Sets the internal variable for the port to direct only NMEA characters to
  void setRTCMOutputPort(Print &outputPort); // Sets the internal variable for the port to direct only RTCM characters to
  void setUBXOutputPort(Print &outputPort);  // Sets the internal variable for the port to direct only UBX characters to
  void setOutputPort(Print &outputPort);     // Sets the internal variable for the port to direct ALL characters to

  // Reset to defaults

  void factoryReset();                                              // Send factory reset sequence (i.e. load "default" configuration and perform hardReset)
  bool factoryDefault(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Reset module to factory defaults
  void hardReset();                                                 // Perform a reset leading to a cold start (zero info start-up)
  void softwareResetGNSSOnly();                                     // Controlled Software Reset (GNSS only) only restarts the GNSS tasks, without reinitializing the full system or reloading any stored configuration.
  void softwareEnableGNSS(bool enable);                             // Controlled Software Start / Stop (GNSS only)
  void cfgRst(uint8_t *data, uint8_t len);                          // Common method for CFG RST

  // Save configuration to BBR / Flash

  bool saveConfiguration(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                        // Save current configuration to flash and BBR (battery backed RAM)
  bool saveConfigSelective(uint32_t configMask, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Save the selected configuration sub-sections to flash and BBR (battery backed RAM)
  bool cfgCfg(uint8_t *data, uint8_t len, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);       // Common method for CFG CFG

  // Functions used for RTK and base station setup
  bool setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);     // Control survey in mode
  bool setSurveyModeFull(uint8_t mode, uint32_t observationTime, float requiredAccuracy, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Control survey in mode
  bool enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Begin Survey-In for NEO-M8P / ZED-F9x
  bool enableSurveyModeFull(uint32_t observationTime, float requiredAccuracy, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);            // Begin Survey-In for NEO-M8P / ZED-F9x
  bool disableSurveyMode(uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                 // Stop Survey-In mode
  // Given coordinates, put receiver into static position. Set latlong to true to pass in lat/long values instead of ecef.
  // For ECEF the units are: cm, 0.1mm, cm, 0.1mm, cm, 0.1mm
  // For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
  bool setStaticPosition(int32_t ecefXOrLat, int8_t ecefXOrLatHP, int32_t ecefYOrLon, int8_t ecefYOrLonHP, int32_t ecefZOrAlt, int8_t ecefZOrAltHP, bool latLong, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool setStaticPosition(int32_t ecefXOrLat, int32_t ecefYOrLon, int32_t ecefZOrAlt, bool latLong, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool setDGNSSConfiguration(sfe_ublox_dgnss_mode_e dgnssMode = SFE_UBLOX_DGNSS_MODE_FIXED, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Set the DGNSS differential mode

  // Read the module's protocol version
  // For safety, call getProtocolVersion etc. inside an if(getModuleInfo())
  uint8_t getProtocolVersionHigh(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the PROTVER XX.00 from UBX-MON-VER register
  uint8_t getProtocolVersionLow(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);  // Returns the PROTVER 00.XX from UBX-MON-VER register
  uint8_t getFirmwareVersionHigh(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the FWVER XX.00 from UBX-MON-VER register
  uint8_t getFirmwareVersionLow(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);  // Returns the FWVER 00.XX from UBX-MON-VER register
  const char *getFirmwareType(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returns the firmware type (SPG, HPG, ADR, etc.) from UBX-MON-VER register
  const char *getModuleName(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Returns the module name (ZED-F9P, ZED-F9R, etc.) from UBX-MON-VER register
  bool getProtocolVersion(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);        // Deprecated. Use getModuleInfo.
  bool getModuleInfo(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);             // Queries module, extracts info. Returns true if MON-VER was read successfully
protected:
  bool prepareModuleInfo(uint16_t maxWait);

public:
  moduleSWVersion_t *moduleSWVersion = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary

  // Support for geofences
  bool addGeofence(int32_t latitude, int32_t longitude, uint32_t radius, uint8_t confidence = 0, bool pinPolarity = 0, uint8_t pin = 0, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Add a new geofence
  bool clearGeofences(uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                                   // Clears all geofences
  bool getGeofenceState(geofenceState &currentGeofenceState, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                               // Returns the combined geofence state
  // Storage for the geofence parameters. RAM is allocated for this if/when required.
  geofenceParams_t *currentGeofenceParams = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary

  // Power save / off
  bool powerOff(uint32_t durationInMs, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources = VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0, bool forceWhileUsb = true, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  // Change the dynamic platform model using UBX-CFG-NAV5
  bool setDynamicModel(dynModel newDynamicModel = DYN_MODEL_PORTABLE, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getDynamicModel(uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Get the dynamic model - returns 255 if the sendCommand fails

  // Reset / enable / configure the odometer
  bool resetOdometer(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                                                                          // Reset the odometer
  bool enableOdometer(bool enable = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                  // Enable / disable the odometer
  bool getOdometerConfig(uint8_t *flags, uint8_t *odoCfg, uint8_t *cogMaxSpeed, uint8_t *cogMaxPosAcc, uint8_t *velLpGain, uint8_t *cogLpGain, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Read the odometer configuration
  bool setOdometerConfig(uint8_t flags, uint8_t odoCfg, uint8_t cogMaxSpeed, uint8_t cogMaxPosAcc, uint8_t velLpGain, uint8_t cogLpGain, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Configure the odometer

  // Enable/Disable individual GNSS systems using UBX-CFG-GNSS
  // Note: you must leave at least one major GNSS enabled! If in doubt, enable GPS before disabling the others
  bool enableGNSS(bool enable, sfe_ublox_gnss_ids_e id, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool isGNSSenabled(sfe_ublox_gnss_ids_e id, bool *enabled, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool isGNSSenabled(sfe_ublox_gnss_ids_e id, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Unsafe overload
  uint32_t getEnableGNSSConfigKey(sfe_ublox_gnss_ids_e id);

  // Reset ESF automatic IMU-mount alignment
  bool resetIMUalignment(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  // Enable/disable esfAutoAlignment
  bool getESFAutoAlignment(bool *enabled, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool getESFAutoAlignment(uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Unsafe overload
  bool setESFAutoAlignment(bool enable, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  // RF Information (including jamming) - ZED-F9 only
  bool getRFinformation(UBX_MON_RF_data_t *data = nullptr, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Get the RF information using UBX_MON_RF

  // Extended hardware status
  bool getHW2status(UBX_MON_HW2_data_t *data = nullptr, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Get the extended hardware status using UBX_MON_HW2

  // UBX-CFG-NAVX5 - get/set the ackAiding byte. If ackAiding is 1, UBX-MGA-ACK messages will be sent by the module to acknowledge the MGA data
  uint8_t getAckAiding(uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                     // Get the ackAiding byte - returns 255 if the sendCommand fails
  bool setAckAiding(uint8_t ackAiding, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Set the ackAiding byte

  // AssistNow Autonomous support
  // UBX-CFG-NAVX5 - get/set the aopCfg byte and set the aopOrdMaxErr word. If aopOrbMaxErr is 0 (default), the max orbit error is reset to the firmware default.
  uint8_t getAopCfg(uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                             // Get the AssistNow Autonomous configuration (aopCfg) - returns 255 if the sendCommand fails
  bool setAopCfg(uint8_t aopCfg, uint16_t aopOrbMaxErr = 0, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Set the aopCfg byte and the aopOrdMaxErr word

  // SPARTN dynamic keys
  //"When the receiver boots, the host should send 'current' and 'next' keys in one message." - Use setDynamicSPARTNKeys for this.
  //"Every time the 'current' key is expired, 'next' takes its place."
  //"Therefore the host should then retrieve the new 'next' key and send only that." - Use setDynamicSPARTNKey for this.
  // The key can be provided in binary (uint8_t) format or in ASCII Hex (char) format, but in both cases keyLengthBytes _must_ represent the binary key length in bytes.
  bool setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const char *key);
  bool setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const uint8_t *key);
  bool setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const char *key1,
                            uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const char *key2);
  bool setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const uint8_t *key1,
                            uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const uint8_t *key2);

  // Support for SPARTN parsing
  uint8_t uSpartnCrc4(const uint8_t *pU8Msg, size_t size);
  uint8_t uSpartnCrc8(const uint8_t *pU8Msg, size_t size);
  uint16_t uSpartnCrc16(const uint8_t *pU8Msg, size_t size);
  uint32_t uSpartnCrc24(const uint8_t *pU8Msg, size_t size);
  uint32_t uSpartnCrc32(const uint8_t *pU8Msg, size_t size);
  uint8_t * parseSPARTN(uint8_t incoming, bool &valid, uint16_t &len, sfe_ublox_spartn_header_t *header = nullptr);

  // Get unique chip ID - UBX-SEC-UNIQID
  bool getUniqueChipId(UBX_SEC_UNIQID_data_t *data = nullptr, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Get the unique chip ID using UBX_SEC_UNIQID
  const char *getUniqueChipIdStr(UBX_SEC_UNIQID_data_t *data = nullptr, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Get the unique chip ID using UBX_SEC_UNIQID

  // General configuration (used only on protocol v27 and higher - ie, ZED-F9P)

  // VALGET

protected:                                                         // These use packetCfg - which is protected from the user
  bool newCfgValget(uint8_t layer = VAL_LAYER_RAM);                // Create a new, empty UBX-CFG-VALGET. Add entries with addCfgValget
  bool addCfgValget(uint32_t key);                                 // Add a new key to an existing UBX-CFG-VALGET ubxPacket - deduce the value size automatically
  bool sendCfgValget(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Send the CfgValget (UBX-CFG-VALGET) construct

public:
  bool newCfgValget(ubxPacket *pkt, uint16_t maxPayload, uint8_t layer = VAL_LAYER_RAM); // Create a new, empty UBX-CFG-VALGET. Add entries with addCfgValget8/16/32/64
  bool addCfgValget(ubxPacket *pkt, uint32_t key);                                       // Add a new key to an existing UBX-CFG-VALGET ubxPacket - deduce the value size automatically
  bool sendCfgValget(ubxPacket *pkt, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);       // Send the CfgValget (UBX-CFG-VALGET) construct
  uint8_t getNumGetCfgKeys() { return _numGetCfgKeys; }                                  // Return the number of keys in the VALGET packet
  uint16_t getLenCfgValGetResponse() { return _lenCfgValGetResponse; }                   // Return the expected length of the VALGET response
  uint8_t getCfgValueSizeBytes(const uint32_t key);                                      // Returns the value size in bytes for the given key

  template <typename T>
  bool extractConfigValueByKey(ubxPacket *pkt, const uint32_t key, T value, size_t maxWidth) // Extract the config value by its key. maxWidth prevents writing beyond the end of value
  {
    if (cfgValgetValueSizes == nullptr) // Check the size list exists
      return false;
    uint8_t sizePtr = 0;

    if (pkt->len < 4)
      return false;

    uint32_t k1 = key & ~UBX_CFG_SIZE_MASK; // Convert key back into an actual key

    uint16_t ptr = 4;
    while (ptr < pkt->len)
    {
      uint32_t k2 = extractLong(pkt, ptr);
      if (k1 == k2)
      {
        ptr += 4; // Point to the value
        switch (key & UBX_CFG_SIZE_MASK)
        {
        case UBX_CFG_L:
          if (maxWidth < sizeof(bool))
            return false;
          *value = (bool)extractByte(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_U1:
        case UBX_CFG_E1:
        case UBX_CFG_X1:
          if (maxWidth < sizeof(uint8_t))
            return false;
          *value = (uint8_t)extractByte(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_I1:
          if (maxWidth < sizeof(int8_t))
            return false;
          *value = (int8_t)extractSignedChar(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_U2:
        case UBX_CFG_E2:
        case UBX_CFG_X2:
          if (maxWidth < sizeof(uint16_t))
            return false;
          *value = (uint16_t)extractInt(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_I2:
          if (maxWidth < sizeof(int16_t))
            return false;
          *value = (int16_t)extractSignedInt(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_U4:
        case UBX_CFG_E4:
        case UBX_CFG_X4:
          if (maxWidth < sizeof(uint32_t))
            return false;
          *value = (uint32_t)extractLong(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_I4:
          if (maxWidth < sizeof(int32_t))
            return false;
          *value = (int32_t)extractSignedLong(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_R4:
          if (maxWidth < sizeof(float))
            return false;
          *value = (float)extractFloat(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_U8:
        case UBX_CFG_X8:
          if (maxWidth < sizeof(uint64_t))
            return false;
          *value = (uint64_t)extractLongLong(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_I8:
          if (maxWidth < sizeof(int64_t))
            return false;
          *value = (int64_t)extractSignedLongLong(pkt, ptr);
          return (true);
          break;
        case UBX_CFG_R8:
          if (maxWidth < sizeof(double))
            return false;
          *value = (double)extractDouble(pkt, ptr);
          return (true);
          break;
        default:
          return false;
          break;
        }
      }
      ptr += 4; // Update ptr
      ptr += cfgValgetValueSizes[sizePtr++];
    }
    return false;
  }

protected:
  sfe_ublox_status_e getVal(uint32_t key, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Load payload with response
public:
  bool getVal8(uint32_t key, uint8_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);        // Returns the value at a given key location
  uint8_t getVal8(uint32_t key, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                   // Unsafe overload - for backward compatibility only
  bool getVal16(uint32_t key, uint16_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Returns the value at a given key location
  uint16_t getVal16(uint32_t key, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                 // Unsafe overload - for backward compatibility only
  bool getVal32(uint32_t key, uint32_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Returns the value at a given key location
  uint32_t getVal32(uint32_t key, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                 // Unsafe overload - for backward compatibility only
  bool getVal64(uint32_t key, uint64_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Returns the value at a given key location
  uint64_t getVal64(uint32_t key, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                 // Unsafe overload - for backward compatibility only
  bool getValSigned8(uint32_t key, int8_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Returns the value at a given key location
  bool getValSigned16(uint32_t key, int16_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the value at a given key location
  bool getValSigned32(uint32_t key, int32_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the value at a given key location
  bool getValSigned64(uint32_t key, int64_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the value at a given key location
  bool getValFloat(uint32_t key, float *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Returns the value at a given key location
  bool getValDouble(uint32_t key, double *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returns the value at a given key location

  // VALSET

  bool setValN(uint32_t key, uint8_t *value, uint8_t N, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Sets the N-byte value at a given group/id/size location
  bool setVal8(uint32_t key, uint8_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);             // Sets the 8-bit value at a given group/id/size location
  bool setVal16(uint32_t key, uint16_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 16-bit value at a given group/id/size location
  bool setVal32(uint32_t key, uint32_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 32-bit value at a given group/id/size location
  bool setVal64(uint32_t key, uint64_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 64-bit value at a given group/id/size location
  bool setValSigned8(uint32_t key, int8_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);        // Sets the 8-bit value at a given group/id/size location
  bool setValSigned16(uint32_t key, int16_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Sets the 16-bit value at a given group/id/size location
  bool setValSigned32(uint32_t key, int32_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Sets the 32-bit value at a given group/id/size location
  bool setValSigned64(uint32_t key, int64_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Sets the 64-bit value at a given group/id/size location
  bool setValFloat(uint32_t key, float value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 32-bit value at a given group/id/size location
  bool setValDouble(uint32_t key, double value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);         // Sets the 64-bit value at a given group/id/size location

  bool newCfgValset(uint8_t layer = VAL_LAYER_RAM_BBR);                                                         // Create a new, empty UBX-CFG-VALSET. Add entries with addCfgValset8/16/32/64
  bool addCfgValsetN(uint32_t key, uint8_t *value, uint8_t N);                                                  // Add a new key and N-byte value to an existing UBX-CFG-VALSET ubxPacket
  bool sendCfgValset(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                              // Send the CfgValset (UBX-CFG-VALSET) construct
  uint8_t getCfgValsetLen();                                                                                    // Returns the length of the current CfgValset construct as number-of-keys
  size_t getCfgValsetSpaceRemaining();                                                                          // Returns the number of free bytes remaining in packetCfg
  void autoSendCfgValsetAtSpaceRemaining(size_t spaceRemaining) { _autoSendAtSpaceRemaining = spaceRemaining; } // Cause CFG_VALSET packets to be sent automatically when packetCfg has less than this many bytes available

  template <typename T>
  bool addCfgValset(uint32_t key, T value) // Add a new key and value to an existing UBX-CFG-VALSET ubxPacket
  {
    uint8_t val[8];

    uint32_t k1 = key & ~UBX_CFG_SIZE_MASK; // Convert key back into an actual key

    switch (key & UBX_CFG_SIZE_MASK)
    {
    case UBX_CFG_L:
      val[0] = (bool)value;
      return (addCfgValsetN(k1, val, 1));
      break;
    case UBX_CFG_U1:
    case UBX_CFG_E1:
    case UBX_CFG_X1:
      val[0] = (uint8_t)value;
      return (addCfgValsetN(k1, val, 1));
      break;
    case UBX_CFG_I1:
      unsignedSigned8 usVal8;
      usVal8.signed8 = (int8_t)value;
      return (addCfgValsetN(k1, &usVal8.unsigned8, 1));
      break;
    case UBX_CFG_U2:
    case UBX_CFG_E2:
    case UBX_CFG_X2:
      for (uint8_t i = 0; i < 2; i++)
        val[i] = (uint8_t)(((uint16_t)value) >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 2));
      break;
    case UBX_CFG_I2:
      unsignedSigned16 usVal16;
      usVal16.signed16 = (int16_t)value;
      for (uint8_t i = 0; i < 2; i++)
        val[i] = (uint8_t)(usVal16.unsigned16 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 2));
      break;
    case UBX_CFG_U4:
    case UBX_CFG_E4:
    case UBX_CFG_X4:
      for (uint8_t i = 0; i < 4; i++)
        val[i] = (uint8_t)(((uint32_t)value) >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 4));
      break;
    case UBX_CFG_I4:
      unsignedSigned32 usVal32;
      usVal32.signed32 = (int32_t)value;
      for (uint8_t i = 0; i < 4; i++)
        val[i] = (uint8_t)(usVal32.unsigned32 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 4));
      break;
    case UBX_CFG_R4:
      unsigned32float us32flt;
      us32flt.flt = (float)value;
      for (uint8_t i = 0; i < 4; i++)
        val[i] = (uint8_t)(us32flt.unsigned32 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 4));
      break;
    case UBX_CFG_U8:
    case UBX_CFG_X8:
      for (uint8_t i = 0; i < 8; i++)
        val[i] = (uint8_t)(((uint64_t)value) >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 8));
      break;
    case UBX_CFG_I8:
      unsignedSigned64 usVal64;
      usVal64.signed64 = (int64_t)value;
      for (uint8_t i = 0; i < 8; i++)
        val[i] = (uint8_t)(usVal64.unsigned64 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 8));
      break;
    case UBX_CFG_R8:
      unsigned64double us64dbl;
      us64dbl.dbl = (float)value;
      for (uint8_t i = 0; i < 8; i++)
        val[i] = (uint8_t)(us64dbl.unsigned64 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 8));
      break;
    default:
      return false;
      break;
    }
    return false;
  }

  // Deprecated - use the template method addCfgValset to auto-deduce the data size
  bool addCfgValset8(uint32_t key, uint8_t value);     // Add a new key and 8-bit value to an existing UBX-CFG-VALSET ubxPacket
  bool addCfgValset16(uint32_t key, uint16_t value);   // Add a new key and 16-bit value to an existing UBX-CFG-VALSET ubxPacket
  bool addCfgValset32(uint32_t key, uint32_t value);   // Add a new key and 32-bit value to an existing UBX-CFG-VALSET ubxPacket
  bool addCfgValset64(uint32_t key, uint64_t value);   // Add a new key and 64-bit value to an existing UBX-CFG-VALSET ubxPacket
  bool addCfgValsetFloat(uint32_t key, float value);   // Add a new key and 32-bit float (R4) to an existing UBX-CFG-VALSET ubxPacket
  bool addCfgValsetDouble(uint32_t key, double value); // Add a new key and 64-bit double (R8) to an existing UBX-CFG-VALSET ubxPacket

  // Deprecated - only included for backward-compatibility. Use newCfgValset and sendCfgValset
  bool newCfgValset8(uint32_t key, uint8_t value, uint8_t layer = VAL_LAYER_RAM_BBR);              // Define a new UBX-CFG-VALSET with the given key and 8-bit value
  bool newCfgValset16(uint32_t key, uint16_t value, uint8_t layer = VAL_LAYER_RAM_BBR);            // Define a new UBX-CFG-VALSET with the given key and 16-bit value
  bool newCfgValset32(uint32_t key, uint32_t value, uint8_t layer = VAL_LAYER_RAM_BBR);            // Define a new UBX-CFG-VALSET with the given key and 32-bit value
  bool newCfgValset64(uint32_t key, uint64_t value, uint8_t layer = VAL_LAYER_RAM_BBR);            // Define a new UBX-CFG-VALSET with the given key and 64-bit value
  bool sendCfgValset8(uint32_t key, uint8_t value, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Add the final key and 8-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
  bool sendCfgValset16(uint32_t key, uint16_t value, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Add the final key and 16-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
  bool sendCfgValset32(uint32_t key, uint32_t value, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Add the final key and 32-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
  bool sendCfgValset64(uint32_t key, uint64_t value, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Add the final key and 64-bit value to an existing UBX-CFG-VALSET ubxPacket and send it

  // get and set functions for all of the "automatic" message processing

  // Navigation (NAV)

  // getPVT will only return data once in each navigation cycle. By default, that is once per second.
  // Therefore we should set kUBLOXGNSSDefaultMaxWait to slightly longer than that.
  // If you change the navigation frequency to (e.g.) 4Hz using setNavigationFrequency(4)
  // then you should use a shorter maxWait. 300msec would be about right: getPVT(300)

  bool getNAVPOSECEF(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                         // NAV POSECEF
  bool setAutoNAVPOSECEF(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                    // Enable/disable automatic POSECEF reports at the navigation frequency
  bool setAutoNAVPOSECEF(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                               // Enable/disable automatic POSECEF reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVPOSECEFrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                    // Set the rate for automatic POSECEF reports
  bool setAutoNAVPOSECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_POSECEF_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic POSECEF reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVPOSECEF(bool enabled, bool implicitUpdate = true);                                                                                                     // In case no config access to the GPS is possible and POSECEF is send cyclically already
  void flushNAVPOSECEF();                                                                                                                                                  // Mark all the data as read/stale
  void logNAVPOSECEF(bool enabled = true);                                                                                                                                 // Log data to file buffer

  bool getNAVSTATUS(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                        // NAV STATUS
  bool setAutoNAVSTATUS(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                   // Enable/disable automatic STATUS reports at the navigation frequency
  bool setAutoNAVSTATUS(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                              // Enable/disable automatic STATUS reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVSTATUSrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                   // Set the rate for automatic STATUS reports
  bool setAutoNAVSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_NAV_STATUS_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic STATUS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVSTATUS(bool enabled, bool implicitUpdate = true);                                                                                                    // In case no config access to the GPS is possible and STATUS is send cyclically already
  void flushNAVSTATUS();                                                                                                                                                 // Mark all the data as read/stale
  void logNAVSTATUS(bool enabled = true);                                                                                                                                // Log data to file buffer

  bool getDOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // Query module for latest dilution of precision values and load global vars:. If autoDOP is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new DOP is available.
  bool setAutoDOP(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic DOP reports at the navigation frequency
  bool setAutoDOP(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic DOP reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoDOPrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic DOP reports
  bool setAutoDOPcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_DOP_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic DOP reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoDOP(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and DOP is send cyclically already
  void flushDOP();                                                                                                                                              // Mark all the DOP data as read/stale
  void logNAVDOP(bool enabled = true);                                                                                                                          // Log data to file buffer

  bool getVehAtt(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // NAV ATT Helper
  bool getNAVATT(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // NAV ATT
  bool setAutoNAVATT(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic vehicle attitude reports at the navigation frequency
  bool setAutoNAVATT(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic vehicle attitude reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVATTrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic ATT reports
  bool setAutoNAVATTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_ATT_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic ATT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVATT(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and vehicle attitude is send cyclically already
  void flushNAVATT();                                                                                                                                              // Mark all the data as read/stale
  void logNAVATT(bool enabled = true);                                                                                                                             // Log data to file buffer

  bool getPVT(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new PVT is available.
  bool setAutoPVT(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic PVT reports at the navigation frequency
  bool setAutoPVT(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic PVT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoPVTrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic PVT reports
  bool setAutoPVTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVT_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoPVT(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and PVT is send cyclically already
  void flushPVT();                                                                                                                                              // Mark all the PVT data as read/stale
  void logNAVPVT(bool enabled = true);                                                                                                                          // Log data to file buffer

  bool getNAVODO(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // NAV ODO
  bool setAutoNAVODO(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic ODO reports at the navigation frequency
  bool setAutoNAVODO(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic ODO reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVODOrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic ODO reports
  bool setAutoNAVODOcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_ODO_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic ODO reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVODO(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and ODO is send cyclically already
  void flushNAVODO();                                                                                                                                              // Mark all the data as read/stale
  void logNAVODO(bool enabled = true);                                                                                                                             // Log data to file buffer

  bool getNAVVELECEF(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                         // NAV VELECEF
  bool setAutoNAVVELECEF(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                    // Enable/disable automatic VELECEF reports at the navigation frequency
  bool setAutoNAVVELECEF(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                               // Enable/disable automatic VELECEF reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVVELECEFrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                    // Set the rate for automatic VELECEF reports
  bool setAutoNAVVELECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_VELECEF_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic VELECEF reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVVELECEF(bool enabled, bool implicitUpdate = true);                                                                                                     // In case no config access to the GPS is possible and VELECEF is send cyclically already
  void flushNAVVELECEF();                                                                                                                                                  // Mark all the data as read/stale
  void logNAVVELECEF(bool enabled = true);                                                                                                                                 // Log data to file buffer

  bool getNAVVELNED(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                        // NAV VELNED
  bool setAutoNAVVELNED(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                   // Enable/disable automatic VELNED reports at the navigation frequency
  bool setAutoNAVVELNED(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                              // Enable/disable automatic VELNED reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVVELNEDrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                   // Set the rate for automatic VELNED reports
  bool setAutoNAVVELNEDcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_VELNED_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic VELNED reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVVELNED(bool enabled, bool implicitUpdate = true);                                                                                                    // In case no config access to the GPS is possible and VELNED is send cyclically already
  void flushNAVVELNED();                                                                                                                                                 // Mark all the data as read/stale
  void logNAVVELNED(bool enabled = true);                                                                                                                                // Log data to file buffer

  bool getNAVHPPOSECEF(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                           // NAV HPPOSECEF
  bool setAutoNAVHPPOSECEF(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                      // Enable/disable automatic HPPOSECEF reports at the navigation frequency
  bool setAutoNAVHPPOSECEF(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                 // Enable/disable automatic HPPOSECEF reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVHPPOSECEFrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                      // Set the rate for automatic HPPOSECEF reports
  bool setAutoNAVHPPOSECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_HPPOSECEF_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic HPPOSECEF reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVHPPOSECEF(bool enabled, bool implicitUpdate = true);                                                                                                       // In case no config access to the GPS is possible and HPPOSECEF is send cyclically already
  void flushNAVHPPOSECEF();                                                                                                                                                    // Mark all the data as read/stale
  void logNAVHPPOSECEF(bool enabled = true);                                                                                                                                   // Log data to file buffer

  bool getHPPOSLLH(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                          // NAV HPPOSLLH
  bool setAutoHPPOSLLH(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                     // Enable/disable automatic HPPOSLLH reports at the navigation frequency
  bool setAutoHPPOSLLH(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                // Enable/disable automatic HPPOSLLH reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoHPPOSLLHrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                     // Set the rate for automatic HPPOSLLH reports
  bool setAutoHPPOSLLHcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_HPPOSLLH_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic HPPOSLLH reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoHPPOSLLH(bool enabled, bool implicitUpdate = true);                                                                                                      // In case no config access to the GPS is possible and HPPOSLLH is send cyclically already
  void flushHPPOSLLH();                                                                                                                                                   // Mark all the HPPPOSLLH data as read/stale. This is handy to get data alignment after CRC failure
  void logNAVHPPOSLLH(bool enabled = true);                                                                                                                               // Log data to file buffer

  bool getNAVPVAT(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                      // NAV PVAT
  bool setAutoNAVPVAT(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                 // Enable/disable automatic PVAT reports at the navigation frequency
  bool setAutoNAVPVAT(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                            // Enable/disable automatic PVAT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVPVATrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                 // Set the rate for automatic PVAT reports
  bool setAutoNAVPVATcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVAT_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic PVAT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVPVAT(bool enabled, bool implicitUpdate = true);                                                                                                  // In case no config access to the GPS is possible and PVAT is send cyclically already
  void flushNAVPVAT();                                                                                                                                               // Mark all the PVAT data as read/stale
  void logNAVPVAT(bool enabled = true);                                                                                                                              // Log data to file buffer

  bool getNAVTIMEUTC(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                         // NAV TIMEUTC
  bool setAutoNAVTIMEUTC(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                    // Enable/disable automatic TIMEUTC reports at the navigation frequency
  bool setAutoNAVTIMEUTC(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                               // Enable/disable automatic TIMEUTC reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVTIMEUTCrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                    // Set the rate for automatic TIMEUTC reports
  bool setAutoNAVTIMEUTCcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_TIMEUTC_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic TIMEUTC reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVTIMEUTC(bool enabled, bool implicitUpdate = true);                                                                                                     // In case no config access to the GPS is possible and TIMEUTC is send cyclically already
  void flushNAVTIMEUTC();                                                                                                                                                  // Mark all the data as read/stale
  void logNAVTIMEUTC(bool enabled = true);                                                                                                                                 // Log data to file buffer

  bool getNAVCLOCK(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                       // NAV CLOCK
  bool setAutoNAVCLOCK(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                  // Enable/disable automatic clock reports at the navigation frequency
  bool setAutoNAVCLOCK(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                             // Enable/disable automatic clock reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVCLOCKrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                  // Set the rate for automatic CLOCK reports
  bool setAutoNAVCLOCKcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_CLOCK_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic CLOCK reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVCLOCK(bool enabled, bool implicitUpdate = true);                                                                                                   // In case no config access to the GPS is possible and clock is send cyclically already
  void flushNAVCLOCK();                                                                                                                                                // Mark all the data as read/stale
  void logNAVCLOCK(bool enabled = true);                                                                                                                               // Log data to file buffer

  bool getSurveyStatus(uint16_t maxWait = 2100);                                                                                                                     // NAV SVIN - Reads survey in status
  bool setAutoNAVSVIN(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                 // Enable/disable automatic survey in reports at the navigation frequency
  bool setAutoNAVSVIN(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                            // Enable/disable automatic survey in reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVSVINrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                 // Set the rate for automatic SVIN reports
  bool setAutoNAVSVINcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SVIN_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic SVIN reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVSVIN(bool enabled, bool implicitUpdate = true);                                                                                                  // In case no config access to the GPS is possible and survey in is send cyclically already
  void flushNAVSVIN();                                                                                                                                               // Mark all the data as read/stale
  void logNAVSVIN(bool enabled = true);                                                                                                                              // Log data to file buffer

  bool getNAVEOE(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // Query module for latest dilution of precision values and load global vars:. If autoEOE is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new EOE is available.
  bool setAutoNAVEOE(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic EOE reports at the navigation frequency
  bool setAutoNAVEOE(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic EOE reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVEOErate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic EOE reports
  bool setAutoNAVEOEcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_EOE_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic EOE reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVEOE(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and EOE is send cyclically already
  void flushNAVEOE();                                                                                                                                              // Mark all the EOE data as read/stale
  void logNAVEOE(bool enabled = true);                                                                                                                             // Log data to file buffer

  // Add "auto" support for NAV TIMELS - to avoid needing 'global' storage
  bool getLeapSecondEvent(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Reads leap second event info

#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  bool getNAVSAT(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // Query module for latest AssistNow Autonomous status and load global vars:. If autoNAVSAT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new NAVSAT is available.
  bool setAutoNAVSAT(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic NAVSAT reports at the navigation frequency
  bool setAutoNAVSAT(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic NAVSAT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVSATrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic NAVSAT reports
  bool setAutoNAVSATcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SAT_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic NAVSAT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVSAT(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and NAVSAT is send cyclically already
  void flushNAVSAT();                                                                                                                                              // Mark all the NAVSAT data as read/stale
  void logNAVSAT(bool enabled = true);                                                                                                                             // Log data to file buffer

  bool getNAVSIG(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // Query module for latest AssistNow Autonomous status and load global vars:. If autoNAVSIG is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new NAVSIG is available.
  bool setAutoNAVSIG(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic NAVSIG reports at the navigation frequency
  bool setAutoNAVSIG(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic NAVSIG reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVSIGrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic NAVSIG reports
  bool setAutoNAVSIGcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SIG_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic NAVSIG reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVSIG(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and NAVSIG is send cyclically already
  void flushNAVSIG();                                                                                                                                              // Mark all the NAVSIG data as read/stale
  void logNAVSIG(bool enabled = true);                                                                                                                             // Log data to file buffer
#endif

  bool getRELPOSNED(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                           // Get Relative Positioning Information of the NED frame
  bool setAutoRELPOSNED(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                      // Enable/disable automatic RELPOSNED reports
  bool setAutoRELPOSNED(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                 // Enable/disable automatic RELPOSNED, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoRELPOSNEDrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                      // Set the rate for automatic RELPOSNEDreports
  bool setAutoRELPOSNEDcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_RELPOSNED_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic RELPOSNED reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoRELPOSNED(bool enabled, bool implicitUpdate = true);                                                                                                       // In case no config access to the GPS is possible and RELPOSNED is send cyclically already
  void flushNAVRELPOSNED();                                                                                                                                                 // Mark all the data as read/stale
  void logNAVRELPOSNED(bool enabled = true);                                                                                                                                // Log data to file buffer

  bool getAOPSTATUS(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                           // Query module for latest AssistNow Autonomous status and load global vars:. If autoAOPSTATUS is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new AOPSTATUS is available.
  bool setAutoAOPSTATUS(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                      // Enable/disable automatic AOPSTATUS reports at the navigation frequency
  bool setAutoAOPSTATUS(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                 // Enable/disable automatic AOPSTATUS reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoAOPSTATUSrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                      // Set the rate for automatic AOPSTATUS reports
  bool setAutoAOPSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_NAV_AOPSTATUS_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic AOPSTATUS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoAOPSTATUS(bool enabled, bool implicitUpdate = true);                                                                                                       // In case no config access to the GPS is possible and AOPSTATUS is send cyclically already
  void flushAOPSTATUS();                                                                                                                                                    // Mark all the AOPSTATUS data as read/stale
  void logAOPSTATUS(bool enabled = true);                                                                                                                                   // Log data to file buffer

#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  // Receiver Manager Messages (RXM)

  // Configure a callback for the UBX-RXM-PMP messages produced by the NEO-D9S
  // Note: on the NEO-D9S, the UBX-RXM-PMP messages are enabled by default on all ports.
  //       You can disable them by calling (e.g.) setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C, 0)
  //       The NEO-D9S does not support UBX-CFG-MSG
  bool setRXMPMPcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_PMP_data_t *));                // Callback receives a pointer to the data, instead of _all_ the data. Much kinder on the stack!
  bool setRXMPMPmessageCallbackPtr(void (*callbackPointerPtr)(UBX_RXM_PMP_message_data_t *)); // Use this if you want all of the PMP message (including sync chars, checksum, etc.) to push to a GNSS

  // Configure a callback for the UBX-RXM-QZSSL6 messages produced by the NEO-D9C
  // Note: on the NEO-D9C, the UBX-RXM-QZSSL6 messages are enabled by default on all ports.
  //       You can disable them by calling (e.g.) setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_I2C, 0)
  //       The NEO-D9C does not support UBX-CFG-MSG
  bool setRXMQZSSL6messageCallbackPtr(void (*callbackPointerPtr)(UBX_RXM_QZSSL6_message_data_t *)); // Use this if you want all of the QZSSL6 message (including sync chars, checksum, etc.) to push to a GNSS

  bool setRXMCORcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_COR_data_t *)); // RXM COR

  // Note: RXM-SFRBX is output-only. It cannot be polled. Strictly getRXMSFRBX should be deprecated
  bool getRXMSFRBX(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                       // RXM SFRBX
  bool setAutoRXMSFRBX(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                  // Enable/disable automatic RXM SFRBX reports at the navigation frequency
  bool setAutoRXMSFRBX(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                             // Enable/disable automatic RXM SFRBX reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoRXMSFRBXrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                  // Set the rate for automatic SFRBX reports
  bool setAutoRXMSFRBXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_SFRBX_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic SFRBX reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoRXMSFRBXmessageCallbackPtr(void (*callbackMessagePointerPtr)(UBX_RXM_SFRBX_message_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Use this if you want all of the SFRBX message (including sync chars, checksum, etc.) to push to the PointPerfect Library
  bool assumeAutoRXMSFRBX(bool enabled, bool implicitUpdate = true);                                                                                                   // In case no config access to the GPS is possible and RXM SFRBX is send cyclically already
  void flushRXMSFRBX();                                                                                                                                                // Mark all the data as read/stale
  void logRXMSFRBX(bool enabled = true);                                                                                                                               // Log data to file buffer

  bool getRXMRAWX(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                      // RXM RAWX
  bool setAutoRXMRAWX(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                 // Enable/disable automatic RXM RAWX reports at the navigation frequency
  bool setAutoRXMRAWX(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                            // Enable/disable automatic RXM RAWX reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoRXMRAWXrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                 // Set the rate for automatic RAWX reports
  bool setAutoRXMRAWXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_RAWX_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic RAWX reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoRXMRAWX(bool enabled, bool implicitUpdate = true);                                                                                                  // In case no config access to the GPS is possible and RXM RAWX is send cyclically already
  void flushRXMRAWX();                                                                                                                                               // Mark all the data as read/stale
  void logRXMRAWX(bool enabled = true);                                                                                                                              // Log data to file buffer

  bool getRXMMEASX(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                       // RXM MEASX
  bool setAutoRXMMEASX(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                  // Enable/disable automatic RXM MEASX reports at the navigation frequency
  bool setAutoRXMMEASX(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                             // Enable/disable automatic RXM MEASX reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoRXMMEASXrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                  // Set the rate for automatic MEASX reports
  bool setAutoRXMMEASXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_MEASX_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic MEASX reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoRXMMEASX(bool enabled, bool implicitUpdate = true);                                                                                                   // In case no config access to the GPS is possible and RXM MEASX is send cyclically already
  void flushRXMMEASX();                                                                                                                                                // Mark all the data as read/stale
  void logRXMMEASX(bool enabled = true);                                                                                                                               // Log data to file buffer
#endif

  // Timing messages (TIM)

  bool getTIMTM2(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // TIM TM2
  bool setAutoTIMTM2(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic TIM TM2 reports at the navigation frequency
  bool setAutoTIMTM2(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic TIM TM2 reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoTIMTM2rate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic TIM TM2 reports
  bool setAutoTIMTM2callbackPtr(void (*callbackPointerPtr)(UBX_TIM_TM2_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic TM2 reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoTIMTM2(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and TIM TM2 is send cyclically already
  void flushTIMTM2();                                                                                                                                              // Mark all the data as read/stale
  void logTIMTM2(bool enabled = true);                                                                                                                             // Log data to file buffer

  bool getTIMTP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                    // TIM TP
  bool setAutoTIMTP(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                               // Enable/disable automatic TIM TP reports at the navigation frequency
  bool setAutoTIMTP(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                          // Enable/disable automatic TIM TP reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoTIMTPrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);               // Set the rate for automatic TIM TP reports
  bool setAutoTIMTPcallbackPtr(void (*callbackPointerPtr)(UBX_TIM_TP_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic TP reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoTIMTP(bool enabled, bool implicitUpdate = true);                                                                                                // In case no config access to the GPS is possible and TIM TP is send cyclically already
  void flushTIMTP();                                                                                                                                             // Mark all the data as read/stale
  void logTIMTP(bool enabled = true);                                                                                                                            // Log data to file buffer

  bool getMONHW(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                    // MON HW
  bool setAutoMONHW(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                               // Enable/disable automatic MON HW reports at the navigation frequency
  bool setAutoMONHW(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                          // Enable/disable automatic MON HW reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoMONHWrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);               // Set the rate for automatic MON HW reports
  bool setAutoMONHWcallbackPtr(void (*callbackPointerPtr)(UBX_MON_HW_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic MON HW reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoMONHW(bool enabled, bool implicitUpdate = true);                                                                                                // In case no config access to the GPS is possible and MON HW is send cyclically already
  void flushMONHW();                                                                                                                                             // Mark all the data as read/stale
  void logMONHW(bool enabled = true);                                                                                                                            // Log data to file buffer

#ifndef SFE_UBLOX_DISABLE_ESF
  // Sensor fusion (dead reckoning) (ESF)

  bool getEsfAlignment(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                               // ESF ALG Helper
  bool getESFALG(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // ESF ALG
  bool setAutoESFALG(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic ESF ALG reports
  bool setAutoESFALG(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic ESF ALG reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFALGrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic ALG reports
  bool setAutoESFALGcallbackPtr(void (*callbackPointerPtr)(UBX_ESF_ALG_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic ALG reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFALG(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and ESF ALG is send cyclically already
  void flushESFALG();                                                                                                                                              // Mark all the data as read/stale
  void logESFALG(bool enabled = true);                                                                                                                             // Log data to file buffer

  bool getEsfInfo(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                          // ESF STATUS Helper
  bool getESFSTATUS(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                        // ESF STATUS
  bool setAutoESFSTATUS(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                   // Enable/disable automatic ESF STATUS reports
  bool setAutoESFSTATUS(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                              // Enable/disable automatic ESF STATUS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFSTATUSrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                   // Set the rate for automatic STATUS reports
  bool setAutoESFSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_STATUS_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic STATUS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFSTATUS(bool enabled, bool implicitUpdate = true);                                                                                                    // In case no config access to the GPS is possible and ESF STATUS is send cyclically already
  void flushESFSTATUS();                                                                                                                                                 // Mark all the data as read/stale
  void logESFSTATUS(bool enabled = true);                                                                                                                                // Log data to file buffer

  bool getEsfIns(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // ESF INS Helper
  bool getESFINS(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // ESF INS
  bool setAutoESFINS(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic ESF INS reports
  bool setAutoESFINS(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic ESF INS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFINSrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic INS reports
  bool setAutoESFINScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_INS_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic INS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFINS(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and ESF INS is send cyclically already
  void flushESFINS();                                                                                                                                              // Mark all the data as read/stale
  void logESFINS(bool enabled = true);                                                                                                                             // Log data to file buffer

  bool setAutoESFMEAS(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                 // Enable/disable automatic ESF MEAS reports
  bool setAutoESFMEAS(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                            // Enable/disable automatic ESF MEAS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFMEASrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                 // Set the rate for automatic MEAS reports
  bool setAutoESFMEAScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_MEAS_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic MEAS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFMEAS(bool enabled, bool implicitUpdate = true);                                                                                                  // In case no config access to the GPS is possible and ESF MEAS is send cyclically already
  void logESFMEAS(bool enabled = true);                                                                                                                              // Log data to file buffer

  bool setAutoESFRAW(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic ESF RAW reports
  bool setAutoESFRAW(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic ESF RAW reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFRAWrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic RAW reports
  bool setAutoESFRAWcallbackPtr(void (*callbackPointerPtr)(UBX_ESF_RAW_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic RAW reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFRAW(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and ESF RAW is send cyclically already
  void logESFRAW(bool enabled = true);                                                                                                                             // Log data to file buffer
#endif

#ifndef SFE_UBLOX_DISABLE_HNR
  // High navigation rate (HNR)

  bool getHNRAtt(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // HNR ATT Helper
  bool getHNRATT(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // Returns true if the get HNR attitude is successful
  bool setAutoHNRATT(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic HNR Attitude reports at the HNR rate
  bool setAutoHNRATT(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic HNR Attitude reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoHNRATTrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic ATT reports
  bool setAutoHNRATTcallbackPtr(void (*callbackPointerPtr)(UBX_HNR_ATT_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic ATT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoHNRATT(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and HNR Attitude is send cyclically already
  void flushHNRATT();                                                                                                                                              // Mark all the data as read/stale
  void logHNRATT(bool enabled = true);                                                                                                                             // Log data to file buffer

  bool getHNRDyn(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // HNR INS Helper
  bool getHNRINS(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // Returns true if the get HNR dynamics is successful
  bool setAutoHNRINS(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic HNR dynamics reports at the HNR rate
  bool setAutoHNRINS(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic HNR dynamics reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoHNRINSrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic INS reports
  bool setAutoHNRINScallbackPtr(void (*callbackPointerPtr)(UBX_HNR_INS_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic INS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoHNRINS(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and HNR dynamics is send cyclically already
  void flushHNRINS();                                                                                                                                              // Mark all the data as read/stale
  void logHNRINS(bool enabled = true);                                                                                                                             // Log data to file buffer

  bool getHNRPVT(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                     // Returns true if the get HNR PVT is successful
  bool setAutoHNRPVT(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic HNR PVT reports at the HNR rate
  bool setAutoHNRPVT(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic HNR PVT reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoHNRPVTrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic PVT reports
  bool setAutoHNRPVTcallbackPtr(void (*callbackPointerPtr)(UBX_HNR_PVT_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoHNRPVT(bool enabled, bool implicitUpdate = true);                                                                                                 // In case no config access to the GPS is possible and HNR PVT is send cyclically already
  void flushHNRPVT();                                                                                                                                              // Mark all the data as read/stale
  void logHNRPVT(bool enabled = true);                                                                                                                             // Log data to file buffer
#endif

  // Helper functions for CFG RATE

  bool setNavigationFrequency(uint8_t navFreq, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Set the number of nav solutions sent per second
  bool getNavigationFrequency(uint8_t *navFreq, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Get the number of nav solutions sent per second currently being output by module
  uint8_t getNavigationFrequency(uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                   // Unsafe overload
  bool setMeasurementRate(uint16_t rate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);       // Set the elapsed time between GNSS measurements in milliseconds, which defines the rate
  bool getMeasurementRate(uint16_t *measRate, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Return the elapsed time between GNSS measurements in milliseconds
  uint16_t getMeasurementRate(uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                      // Unsafe overload
  bool setNavigationRate(uint16_t rate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);        // Set the ratio between the number of measurements and the number of navigation solutions. Unit is cycles. Max is 127
  bool getNavigationRate(uint16_t *navRate, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);        // Return the ratio between the number of measurements and the number of navigation solutions. Unit is cycles
  uint16_t getNavigationRate(uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                       // Unsafe overload

  // Helper functions for DOP
  // For safety, call these inside an if(getDOP())

  uint16_t getGeometricDOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getPositionDOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getTimeDOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getVerticalDOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getHorizontalDOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getNorthingDOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getEastingDOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  // Helper functions for ATT
  // For safety, call these inside an if(getNAVATT())

  float getATTroll(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returned as degrees
  float getATTpitch(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Returned as degrees
  float getATTheading(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returned as degrees

  // Helper functions for PVT
  // For safety, call these inside an if(getPVT())

  uint32_t getTimeOfWeek(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getYear(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getMonth(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getDay(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getHour(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getMinute(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getSecond(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getMillisecond(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getNanosecond(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint32_t getUnixEpoch(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint32_t getUnixEpoch(uint32_t &microsecond, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  bool getDateValid(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool getTimeValid(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool getTimeFullyResolved(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool getConfirmedDate(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool getConfirmedTime(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  uint8_t getFixType(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning

  bool getGnssFixOk(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Get whether we have a valid fix (i.e within DOP & accuracy masks)
  bool getDiffSoln(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);  // Get whether differential corrections were applied
  bool getHeadVehValid(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getCarrierSolutionType(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns RTK solution: 0=no, 1=float solution, 2=fixed solution

  uint8_t getSIV(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);         // Returns number of sats used in fix
  int32_t getLongitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
  int32_t getLatitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
  int32_t getAltitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returns the current altitude in mm above ellipsoid
  int32_t getAltitudeMSL(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the current altitude in mm above mean sea level
  int32_t getHorizontalAccEst(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getVerticalAccEst(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getNedNorthVel(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getNedEastVel(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getNedDownVel(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getGroundSpeed(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns speed in mm/s
  int32_t getHeading(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);     // Returns heading in degrees * 10^-5
  uint32_t getSpeedAccEst(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint32_t getHeadingAccEst(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getPDOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns positional dillution of precision * 10^-2 (dimensionless)

  bool getInvalidLlh(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  int32_t getHeadVeh(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int16_t getMagDec(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getMagAcc(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  int32_t getGeoidSeparation(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  // Helper functions for HPPOSECEF
  // For safety, call these inside an if(getNAVHPPOSECEF())

  uint32_t getPositionAccuracy(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the 3D accuracy of the current high-precision fix, in mm. Supported on NEO-M8P, ZED-F9P,
  int32_t getHighResECEFX(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Returns the ECEF X coordinate (cm)
  int32_t getHighResECEFY(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Returns the ECEF Y coordinate (cm)
  int32_t getHighResECEFZ(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Returns the ECEF Z coordinate (cm)
  int8_t getHighResECEFXHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);     // Returns the ECEF X coordinate High Precision Component (0.1 mm)
  int8_t getHighResECEFYHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);     // Returns the ECEF Y coordinate High Precision Component (0.1 mm)
  int8_t getHighResECEFZHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);     // Returns the ECEF Z coordinate High Precision Component (0.1 mm)

  // Helper functions for HPPOSLLH
  // For safety, call these inside an if(getHPPOSLLH())

  uint32_t getTimeOfWeekFromHPPOSLLH(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getHighResLongitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getHighResLatitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getElipsoid(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getMeanSeaLevel(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int8_t getHighResLongitudeHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int8_t getHighResLatitudeHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int8_t getElipsoidHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int8_t getMeanSeaLevelHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint32_t getHorizontalAccuracy(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint32_t getVerticalAccuracy(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  // Helper functions for PVAT
  // For safety, call these inside an if(getNAVPVAT())

  int32_t getVehicleRoll(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returns vehicle roll in degrees * 10^-5
  int32_t getVehiclePitch(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Returns vehicle pitch in degrees * 10^-5
  int32_t getVehicleHeading(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns vehicle heading in degrees * 10^-5
  int32_t getMotionHeading(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);  // Returns the motion heading in degrees * 10^-5

  // Helper functions for SVIN
  // For safety, call these inside an if(getSurveyStatus())

  bool getSurveyInActive(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool getSurveyInValid(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getSurveyInObservationTime(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);     // Truncated to 65535 seconds
  uint32_t getSurveyInObservationTimeFull(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Return the full uint32_t
  float getSurveyInMeanAccuracy(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Returned as m

  // Helper functions for TIMELS
  // For safety, call these inside an if(getLeapSecondEvent())

  uint8_t getLeapIndicator(int32_t &timeToLsEvent, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int8_t getCurrentLeapSeconds(sfe_ublox_ls_src_e &source, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  // Helper functions for RELPOSNED
  // For safety, call these inside an if(getRELPOSNED())

  float getRelPosN(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returned as m
  float getRelPosE(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returned as m
  float getRelPosD(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returned as m
  float getRelPosAccN(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returned as m
  float getRelPosAccE(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returned as m
  float getRelPosAccD(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returned as m

  // Helper functions for AOPSTATUS
  // For safety, call these inside an if(getAOPSTATUS())

  uint8_t getAOPSTATUSuseAOP(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the UBX-NAV-AOPSTATUS useAOP flag. Don't confuse this with getAopCfg - which returns the aopCfg byte from UBX-CFG-NAVX5
  uint8_t getAOPSTATUSstatus(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the UBX-NAV-AOPSTATUS status field. A host application can determine the optimal time to shut down the receiver by monitoring the status field for a steady 0.

  // Helper functions for TIM TP
  // For safety, call these inside an if(getTIMTP())

  uint32_t getTIMTPtowMS(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                          // Returns the UBX-TIM-TP towMS time pulse of week (ms)
  uint32_t getTIMTPtowSubMS(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                       // Returns the UBX-TIM-TP submillisecond part of towMS (ms * 2^-32)
  uint16_t getTIMTPweek(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Returns the UBX-TIM-TP time pulse week according to time base
  uint32_t getTIMTPAsEpoch(uint32_t &microsecond, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Convert TIM TP to Unix Epoch - CAUTION! Assumes the time base is UTC and the week number is GPS

  // Helper function for hardware status (including jamming)
  // For safety, call getAntennaStatus inside an if(getMONHW())

  bool getHWstatus(UBX_MON_HW_data_t *data = nullptr, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Get the hardware status using UBX_MON_HW
  sfe_ublox_antenna_status_e getAntennaStatus(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);         // Get the antenna status (aStatus) using UBX_MON_HW

#ifndef SFE_UBLOX_DISABLE_ESF
  // Helper functions for ESF
  // For safety, call getESFroll/pitch/yaw inside an if(getESFALG())

  float getESFroll(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);  // Returned as degrees
  float getESFpitch(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returned as degrees
  float getESFyaw(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Returned as degrees
  bool getSensorFusionMeasurement(UBX_ESF_MEAS_sensorData_t *sensorData, UBX_ESF_MEAS_data_t ubxDataStruct, uint8_t sensor);
  bool getRawSensorMeasurement(UBX_ESF_RAW_sensorData_t *sensorData, UBX_ESF_RAW_data_t ubxDataStruct, uint8_t sensor);
  bool getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, uint8_t sensor, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, UBX_ESF_STATUS_data_t ubxDataStruct, uint8_t sensor);
#endif

#ifndef SFE_UBLOX_DISABLE_HNR
  // Helper functions for HNR
  // For safety, call getHNRroll/pitch/yaw inside an if(getHNRATT())

  bool setHNRNavigationRate(uint8_t rate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns true if the setHNRNavigationRate is successful
  uint8_t getHNRNavigationRate(uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Returns 0 if the getHNRNavigationRate fails
  float getHNRroll(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                           // Returned as degrees
  float getHNRpitch(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                          // Returned as degrees
  float getHNRheading(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                        // Returned as degrees
#endif

  // Helper functions for the NEO-F10N
  bool getLNAMode(sfe_ublox_lna_mode_e *mode, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Get the LNA mode
  bool setLNAMode(sfe_ublox_lna_mode_e mode, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Set the LNA mode
  bool getGPSL5HealthOverride(bool *override, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Get the GPS L5 health override status
  bool setGPSL5HealthOverride(bool override, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Set the GPS L5 health override status

  // Set the mainTalkerId used by NMEA messages - allows all NMEA messages except GSV to be prefixed with GP instead of GN
  bool setMainTalkerID(sfe_ublox_talker_ids_e id = SFE_UBLOX_MAIN_TALKER_ID_DEFAULT, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  // Enable/Disable NMEA High Precision Mode - include extra decimal places in the Lat and Lon
  bool setHighPrecisionMode(bool enable = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  // Helper functions for NMEA logging
  void setNMEALoggingMask(uint32_t messages = SFE_UBLOX_FILTER_NMEA_ALL); // Add selected NMEA messages to file buffer - if enabled. Default to adding ALL messages to the file buffer
  uint32_t getNMEALoggingMask();                                          // Return which NMEA messages are selected for logging to the file buffer - if enabled

  // Helper functions to control which NMEA messages are passed to processNMEA
  void setProcessNMEAMask(uint32_t messages = SFE_UBLOX_FILTER_NMEA_ALL); // Control which NMEA messages are passed to processNMEA. Default to passing ALL messages
  uint32_t getProcessNMEAMask();                                          // Return which NMEA messages are passed to processNMEA

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  // Support for "auto" storage of NMEA messages
  uint8_t getLatestNMEAGPGGA(NMEA_GGA_data_t *data);                           // Return the most recent GPGGA: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGPGGAcallbackPtr(void (*callbackPointerPtr)(NMEA_GGA_data_t *)); // Enable a callback on the arrival of a GPGGA message
  uint8_t getLatestNMEAGNGGA(NMEA_GGA_data_t *data);                           // Return the most recent GNGGA: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGNGGAcallbackPtr(void (*callbackPointerPtr)(NMEA_GGA_data_t *)); // Enable a callback on the arrival of a GNGGA message
  uint8_t getLatestNMEAGPVTG(NMEA_VTG_data_t *data);                           // Return the most recent GPVTG: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGPVTGcallbackPtr(void (*callbackPointerPtr)(NMEA_VTG_data_t *)); // Enable a callback on the arrival of a GPVTG message
  uint8_t getLatestNMEAGNVTG(NMEA_VTG_data_t *data);                           // Return the most recent GNVTG: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGNVTGcallbackPtr(void (*callbackPointerPtr)(NMEA_VTG_data_t *)); // Enable a callback on the arrival of a GNVTG message
  uint8_t getLatestNMEAGPRMC(NMEA_RMC_data_t *data);                           // Return the most recent GPRMC: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGPRMCcallbackPtr(void (*callbackPointerPtr)(NMEA_RMC_data_t *)); // Enable a callback on the arrival of a GPRMC message
  uint8_t getLatestNMEAGNRMC(NMEA_RMC_data_t *data);                           // Return the most recent GNRMC: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGNRMCcallbackPtr(void (*callbackPointerPtr)(NMEA_RMC_data_t *)); // Enable a callback on the arrival of a GNRMC message
  uint8_t getLatestNMEAGPZDA(NMEA_ZDA_data_t *data);                           // Return the most recent GPZDA: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGPZDAcallbackPtr(void (*callbackPointerPtr)(NMEA_ZDA_data_t *)); // Enable a callback on the arrival of a GPZDA message
  uint8_t getLatestNMEAGNZDA(NMEA_ZDA_data_t *data);                           // Return the most recent GNZDA: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGNZDAcallbackPtr(void (*callbackPointerPtr)(NMEA_ZDA_data_t *)); // Enable a callback on the arrival of a GNZDA message
#endif

  // RTCM

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  uint8_t getLatestRTCM1005(RTCM_1005_data_t *data);                           // Return the most recent RTCM 1005: 0 = no data, 1 = stale data, 2 = fresh data
  bool setRTCM1005callbackPtr(void (*callbackPointerPtr)(RTCM_1005_data_t *)); // Configure a callback for the RTCM 1005 Message

  uint8_t getLatestRTCM1005Input(RTCM_1005_data_t *data);                                // Return the most recent RTCM 1005 Input, extracted from pushRawData: 0 = no data, 1 = stale data, 2 = fresh data
  void setRTCM1005InputcallbackPtr(void (*rtcm1005CallbackPointer)(RTCM_1005_data_t *)); // Configure a callback for RTCM 1005 Input - from pushRawData
  uint8_t getLatestRTCM1006Input(RTCM_1006_data_t *data);                                // Return the most recent RTCM 1006 Input, extracted from pushRawData: 0 = no data, 1 = stale data, 2 = fresh data
  void setRTCM1006InputcallbackPtr(void (*rtcm1006CallbackPointer)(RTCM_1006_data_t *)); // Configure a callback for RTCM 1006 Input - from pushRawData

  void extractRTCM1005(RTCM_1005_data_t *destination, uint8_t *source); // Extract RTCM 1005 from source into destination
  void extractRTCM1006(RTCM_1006_data_t *destination, uint8_t *source); // Extract RTCM 1006 from source into destination

  // Helper functions for RTCM logging
  bool setRTCMLoggingMask(uint32_t messages = SFE_UBLOX_FILTER_RTCM_ALL); // Add selected RTCM messages to file buffer - if enabled. Default to adding ALL messages to the file buffer
  uint32_t getRTCMLoggingMask();                                          // Return which RTCM messages are selected for logging to the file buffer - if enabled
#endif

  // UBX Logging - log any UBX message using packetAuto and avoiding having to have and use "Auto" (setAutonnn and lognnn) methods
  void enableUBXlogging(uint8_t UBX_CLASS, uint8_t UBX_ID, bool enable = true);

  // Functions to extract signed and unsigned 8/16/32-bit data from a ubxPacket
  // From v2.0: These are public. The user can call these to extract data from custom packets
  uint64_t extractLongLong(ubxPacket *msg, uint16_t spotToStart);      // Combine eight bytes from payload into uint64_t
  int64_t extractSignedLongLong(ubxPacket *msg, uint16_t spotToStart); // Combine eight bytes from payload into uint64_t
  uint32_t extractLong(ubxPacket *msg, uint16_t spotToStart);          // Combine four bytes from payload into long
  int32_t extractSignedLong(ubxPacket *msg, uint16_t spotToStart);     // Combine four bytes from payload into signed long (avoiding any ambiguity caused by casting)
  uint16_t extractInt(ubxPacket *msg, uint16_t spotToStart);           // Combine two bytes from payload into int
  int16_t extractSignedInt(ubxPacket *msg, uint16_t spotToStart);
  uint8_t extractByte(ubxPacket *msg, uint16_t spotToStart);      // Get byte from payload
  int8_t extractSignedChar(ubxPacket *msg, uint16_t spotToStart); // Get signed 8-bit value from payload
  float extractFloat(ubxPacket *msg, uint16_t spotToStart);       // Get signed 32-bit float (R4) from payload
  double extractDouble(ubxPacket *msg, uint16_t spotToStart);     // Get signed 64-bit double (R8) from payload

  // Functions to help extract RTCM bit fields
  uint64_t extractUnsignedBits(uint8_t *ptr, uint16_t start, uint16_t width);
  int64_t extractSignedBits(uint8_t *ptr, uint16_t start, uint16_t width);

  // Pointers to storage for the "automatic" messages
  // RAM is allocated for these if/when required.

  UBX_NAV_POSECEF_t *packetUBXNAVPOSECEF = nullptr;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_STATUS_t *packetUBXNAVSTATUS = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_DOP_t *packetUBXNAVDOP = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_EOE_t *packetUBXNAVEOE = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_ATT_t *packetUBXNAVATT = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_PVT_t *packetUBXNAVPVT = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_ODO_t *packetUBXNAVODO = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_TIMEUTC_t *packetUBXNAVTIMEUTC = nullptr;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_VELECEF_t *packetUBXNAVVELECEF = nullptr;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_VELNED_t *packetUBXNAVVELNED = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_HPPOSECEF_t *packetUBXNAVHPPOSECEF = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_HPPOSLLH_t *packetUBXNAVHPPOSLLH = nullptr;   // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_PVAT_t *packetUBXNAVPVAT = nullptr;           // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_CLOCK_t *packetUBXNAVCLOCK = nullptr;         // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_TIMELS_t *packetUBXNAVTIMELS = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_SVIN_t *packetUBXNAVSVIN = nullptr;           // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_RELPOSNED_t *packetUBXNAVRELPOSNED = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_AOPSTATUS_t *packetUBXNAVAOPSTATUS = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary

#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  UBX_NAV_SAT_t *packetUBXNAVSAT = nullptr;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_SIG_t *packetUBXNAVSIG = nullptr;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_PMP_t *packetUBXRXMPMP = nullptr;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_PMP_message_t *packetUBXRXMPMPmessage = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_QZSSL6_message_t *packetUBXRXMQZSSL6message = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_COR_t *packetUBXRXMCOR = nullptr;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_SFRBX_t *packetUBXRXMSFRBX = nullptr;                  // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_RAWX_t *packetUBXRXMRAWX = nullptr;                    // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_MEASX_t *packetUBXRXMMEASX = nullptr;                  // Pointer to struct. RAM will be allocated for this if/when necessary
#endif

  UBX_TIM_TM2_t *packetUBXTIMTM2 = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_TIM_TP_t *packetUBXTIMTP = nullptr;   // Pointer to struct. RAM will be allocated for this if/when necessary

  UBX_MON_HW_t *packetUBXMONHW = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary

#ifndef SFE_UBLOX_DISABLE_ESF
  UBX_ESF_ALG_t *packetUBXESFALG = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_INS_t *packetUBXESFINS = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_MEAS_t *packetUBXESFMEAS = nullptr;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_RAW_t *packetUBXESFRAW = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_STATUS_t *packetUBXESFSTATUS = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
#endif

#ifndef SFE_UBLOX_DISABLE_HNR
  UBX_HNR_PVT_t *packetUBXHNRPVT = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_HNR_ATT_t *packetUBXHNRATT = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_HNR_INS_t *packetUBXHNRINS = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
#endif

  UBX_MGA_ACK_DATA0_t *packetUBXMGAACK = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_MGA_DBD_t *packetUBXMGADBD = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  NMEA_GPGGA_t *storageNMEAGPGGA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNGGA_t *storageNMEAGNGGA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GPVTG_t *storageNMEAGPVTG = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNVTG_t *storageNMEAGNVTG = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GPRMC_t *storageNMEAGPRMC = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNRMC_t *storageNMEAGNRMC = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GPZDA_t *storageNMEAGPZDA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNZDA_t *storageNMEAGNZDA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
#endif

  RTCM_1005_t *storageRTCM1005 = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  struct
  {
    union
    {
      uint8_t all;
      struct
      {
        uint8_t dataValid1005 : 1;
        uint8_t dataRead1005 : 1;
        uint8_t dataValid1006 : 1;
        uint8_t dataRead1006 : 1;
      } bits;
    } flags;
    RTCM_1005_data_t rtcm1005; // Latest RTCM 1005 parsed from pushRawData
    RTCM_1006_data_t rtcm1006; // Latest RTCM 1006 parsed from pushRawData
    void (*rtcm1005CallbackPointer)(RTCM_1005_data_t *);
    void (*rtcm1006CallbackPointer)(RTCM_1006_data_t *);
    void init(void) // Initializer / constructor
    {
      flags.all = 0;                     // Clear the RTCM Input flags
      rtcm1005CallbackPointer = nullptr; // Clear the callback pointers
      rtcm1006CallbackPointer = nullptr;
    }
  } rtcmInputStorage; // Latest RTCM parsed from pushRawData
#endif

  uint16_t rtcmFrameCounter = 0; // Tracks the type of incoming byte inside RTCM frame

protected:
  // Depending on the ubx binary response class, store binary responses into different places
  enum classTypes
  {
    CLASS_NONE = 0,
    CLASS_ACK,
    CLASS_NOT_AN_ACK
  } ubxFrameClass = CLASS_NONE;

  // Functions
  bool checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass = 0, uint8_t requestedID = 0); // Checks module with user selected commType
  void addToChecksum(uint8_t incoming);                                                                 // Given an incoming byte, adjust rollingChecksumA/B
  size_t pushAssistNowDataInternal(size_t offset, bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);
  size_t findMGAANOForDateInternal(const uint8_t *dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture);

  // Return true if this "automatic" message has storage allocated for it. Also provide the associated max payload size
  bool autoLookup(uint8_t Class, uint8_t ID, uint16_t *maxSize = nullptr);

  bool initGeofenceParams();  // Allocate RAM for currentGeofenceParams and initialize it
  bool initModuleSWVersion(); // Allocate RAM for moduleSWVersion and initialize it

  // The initPacket functions need to be private as they don't check if memory has already been allocated.
  // Functions like setAutoNAVPOSECEF will check that memory has not been allocated before calling initPacket.
  bool initPacketUBXNAVPOSECEF();       // Allocate RAM for packetUBXNAVPOSECEF and initialize it
  bool initPacketUBXNAVSTATUS();        // Allocate RAM for packetUBXNAVSTATUS and initialize it
  bool initPacketUBXNAVDOP();           // Allocate RAM for packetUBXNAVDOP and initialize it
  bool initPacketUBXNAVATT();           // Allocate RAM for packetUBXNAVATT and initialize it
  bool initPacketUBXNAVPVT();           // Allocate RAM for packetUBXNAVPVT and initialize it
  bool initPacketUBXNAVODO();           // Allocate RAM for packetUBXNAVODO and initialize it
  bool initPacketUBXNAVVELECEF();       // Allocate RAM for packetUBXNAVVELECEF and initialize it
  bool initPacketUBXNAVVELNED();        // Allocate RAM for packetUBXNAVVELNED and initialize it
  bool initPacketUBXNAVHPPOSECEF();     // Allocate RAM for packetUBXNAVHPPOSECEF and initialize it
  bool initPacketUBXNAVHPPOSLLH();      // Allocate RAM for packetUBXNAVHPPOSLLH and initialize it
  bool initPacketUBXNAVPVAT();          // Allocate RAM for packetUBXNAVPVAT and initialize it
  bool initPacketUBXNAVTIMEUTC();       // Allocate RAM for packetUBXNAVTIMEUTC and initialize it
  bool initPacketUBXNAVCLOCK();         // Allocate RAM for packetUBXNAVCLOCK and initialize it
  bool initPacketUBXNAVTIMELS();        // Allocate RAM for packetUBXNAVTIMELS and initialize it
  bool initPacketUBXNAVSVIN();          // Allocate RAM for packetUBXNAVSVIN and initialize it
  bool initPacketUBXNAVRELPOSNED();     // Allocate RAM for packetUBXNAVRELPOSNED and initialize it
  bool initPacketUBXNAVAOPSTATUS();     // Allocate RAM for packetUBXNAVAOPSTATUS and initialize it
  bool initPacketUBXNAVEOE();           // Allocate RAM for packetUBXNAVEOE and initialize it
  bool initPacketUBXNAVSAT();           // Allocate RAM for packetUBXNAVSAT and initialize it
  bool initPacketUBXNAVSIG();           // Allocate RAM for packetUBXNAVSIG and initialize it
  bool initPacketUBXRXMPMP();           // Allocate RAM for packetUBXRXMPMP and initialize it
  bool initPacketUBXRXMPMPmessage();    // Allocate RAM for packetUBXRXMPMPRaw and initialize it
  bool initPacketUBXRXMQZSSL6message(); // Allocate RAM for packetUBXRXMQZSSL6raw and initialize it
  bool initPacketUBXRXMCOR();           // Allocate RAM for packetUBXRXMCOR and initialize it
  bool initPacketUBXRXMSFRBX();         // Allocate RAM for packetUBXRXMSFRBX and initialize it
  bool initPacketUBXRXMRAWX();          // Allocate RAM for packetUBXRXMRAWX and initialize it
  bool initPacketUBXRXMMEASX();         // Allocate RAM for packetUBXRXMMEASX and initialize it
  bool initPacketUBXTIMTM2();           // Allocate RAM for packetUBXTIMTM2 and initialize it
  bool initPacketUBXTIMTP();            // Allocate RAM for packetUBXTIMTP and initialize it
  bool initPacketUBXMONHW();            // Allocate RAM for packetUBXMONHW and initialize it
  bool initPacketUBXESFALG();           // Allocate RAM for packetUBXESFALG and initialize it
  bool initPacketUBXESFSTATUS();        // Allocate RAM for packetUBXESFSTATUS and initialize it
  bool initPacketUBXESFINS();           // Allocate RAM for packetUBXESFINS and initialize it
  bool initPacketUBXESFMEAS();          // Allocate RAM for packetUBXESFMEAS and initialize it
  bool initPacketUBXESFRAW();           // Allocate RAM for packetUBXESFRAW and initialize it
  bool initPacketUBXHNRATT();           // Allocate RAM for packetUBXHNRATT and initialize it
  bool initPacketUBXHNRINS();           // Allocate RAM for packetUBXHNRINS and initialize it
  bool initPacketUBXHNRPVT();           // Allocate RAM for packetUBXHNRPVT and initialize it
  bool initPacketUBXMGAACK();           // Allocate RAM for packetUBXMGAACK and initialize it
  bool initPacketUBXMGADBD();           // Allocate RAM for packetUBXMGADBD and initialize it

  bool initStorageNMEAGPGGA(); // Allocate RAM for incoming NMEA GPGGA messages and initialize it
  bool initStorageNMEAGNGGA(); // Allocate RAM for incoming NMEA GNGGA messages and initialize it
  bool initStorageNMEAGPVTG(); // Allocate RAM for incoming NMEA GPVTG messages and initialize it
  bool initStorageNMEAGNVTG(); // Allocate RAM for incoming NMEA GNVTG messages and initialize it
  bool initStorageNMEAGPRMC(); // Allocate RAM for incoming NMEA GPRMC messages and initialize it
  bool initStorageNMEAGNRMC(); // Allocate RAM for incoming NMEA GNRMC messages and initialize it
  bool initStorageNMEAGPZDA(); // Allocate RAM for incoming NMEA GPZDA messages and initialize it
  bool initStorageNMEAGNZDA(); // Allocate RAM for incoming NMEA GNZDA messages and initialize it

  bool initStorageRTCM(); // Allocate RAM for incoming RTCM messages and initialize it
  bool initStorageNMEA(); // Allocate RAM for incoming non-Auto NMEA messages and initialize it

  bool initStorageRTCM1005(); // Allocate RAM for incoming RTCM 1005 messages and initialize it

  // Variables
  SparkFun_UBLOX_GNSS::GNSSDeviceBus *_sfeBus;

  SparkFun_UBLOX_GNSS::SfePrint _nmeaOutputPort; // The user can assign an output port to print NMEA sentences if they wish
  SparkFun_UBLOX_GNSS::SfePrint _rtcmOutputPort; // The user can assign an output port to print RTCM sentences if they wish
  SparkFun_UBLOX_GNSS::SfePrint _ubxOutputPort;  // The user can assign an output port to print UBX sentences if they wish
  SparkFun_UBLOX_GNSS::SfePrint _outputPort;     // The user can assign an output port to print ALL characters to if they wish
  SparkFun_UBLOX_GNSS::SfePrint _debugSerial;    // The stream to send debug messages to if enabled
  bool _printDebug = false;                      // Flag to print the serial commands we are sending to the Serial port for debug
  bool _printLimitedDebug = false;               // Flag to print limited debug messages. Useful for I2C debugging or high navigation rates

  // The packet buffers
  // These are pointed at from within the ubxPacket
  uint8_t payloadAck[2];           // Holds the requested ACK/NACK
  uint8_t payloadBuf[2];           // Temporary buffer used to screen incoming packets or dump unrequested packets
  size_t packetCfgPayloadSize = 0; // Size for the packetCfg payload. .begin will set this to MAX_PAYLOAD_SIZE if necessary. User can change with setPacketCfgPayloadSize
  uint8_t *payloadCfg = nullptr;
  uint8_t *payloadAuto = nullptr;

  uint8_t *spiBuffer = nullptr;                                // A buffer to store any bytes being recieved back from the device while we are sending via SPI
  size_t spiBufferIndex = 0;                                   // Index into the SPI buffer
  size_t spiBufferSize = SFE_UBLOX_SPI_BUFFER_DEFAULT_SIZE;    // Default size of the SPI buffer
  uint8_t spiTransactionSize = SFE_UBLOX_SPI_TRANSACTION_SIZE; // Default size of SPI transactions

  // Init the packet structures and init them with pointers to the payloadAck, payloadCfg, payloadBuf and payloadAuto arrays
  ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  ubxPacket packetBuf = {0, 0, 0, 0, 0, payloadBuf, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  ubxPacket packetAuto = {0, 0, 0, 0, 0, payloadAuto, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  // Flag if this packet is unrequested (and so should be ignored and not copied into packetCfg or packetAck)
  bool ignoreThisPayload = false;

  // Identify which buffer is in use
  // Data is stored in packetBuf until the requested class and ID can be validated
  // If a match is seen, data is diverted into packetAck or packetCfg
  //"Automatic" messages which have RAM allocated for them are diverted into packetAuto
  sfe_ublox_packet_buffer_e activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;

  // Limit checking of new data to every X ms
  // If we are expecting an update every X Hz then we should check every quarter that amount of time
  // Otherwise we may block ourselves from seeing new data
  uint8_t i2cPollingWait = 100;    // Default to 100ms. Adjusted when user calls setNavigationFrequency() or setHNRNavigationRate() or setMeasurementRate()
  uint8_t i2cPollingWaitNAV = 100; // We need to record the desired polling rate for standard nav messages
  uint8_t i2cPollingWaitHNR = 100; // and for HNR too so we can set i2cPollingWait to the lower of the two

  // The SPI polling wait is a little different. checkUbloxSpi will delay for this amount before returning if
  // there is no data waiting to be read. This prevents waitForACKResponse from pounding the SPI bus too hard.
  uint8_t spiPollingWait = 9; // Default to 9ms; waitForACKResponse delays for 1ms on top of this. User can adjust with setSPIPollingWait.

  unsigned long lastCheck = 0;

  uint16_t ubxFrameCounter; // Count all UBX frame bytes. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]
  uint8_t rollingChecksumA; // Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
  uint8_t rollingChecksumB; // Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

  // NMEA logging / Auto support
  sfe_ublox_nmea_filtering_t _logNMEA;     // Flags to indicate which NMEA messages should be added to the file buffer for logging
  sfe_ublox_nmea_filtering_t _processNMEA; // Flags to indicate which NMEA messages should be passed to processNMEA

  int8_t nmeaByteCounter; // Count all NMEA message bytes.
  // Abort NMEA message reception if nmeaByteCounter exceeds maxNMEAByteCount.
  // The user can adjust maxNMEAByteCount by calling setMaxNMEAByteCount
  int8_t maxNMEAByteCount = SFE_UBLOX_MAX_NMEA_BYTE_COUNT;
  uint8_t nmeaAddressField[6]; // NMEA Address Field - includes the start character (*)
  bool logThisNMEA();          // Return true if we should log this NMEA message
  bool processThisNMEA();      // Return true if we should pass this NMEA message to processNMEA
  bool isNMEAHeaderValid();    // Return true if the six byte NMEA header appears valid. Used to set _signsOfLife

  NMEA_STORAGE_t *_storageNMEA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  bool isThisNMEAauto();                 // Check if the NMEA message (in nmeaAddressField) is "auto" (i.e. has RAM allocated for it)
  bool doesThisNMEAHaveCallback();       // Do we need to copy the data into the callback copy?
  uint8_t *getNMEAWorkingLengthPtr();    // Get a pointer to the working copy length
  uint8_t *getNMEAWorkingNMEAPtr();      // Get a pointer to the working copy NMEA data
  uint8_t *getNMEACompleteLengthPtr();   // Get a pointer to the complete copy length
  uint8_t *getNMEACompleteNMEAPtr();     // Get a pointer to the complete copy NMEA data
  uint8_t *getNMEACallbackLengthPtr();   // Get a pointer to the callback copy length
  uint8_t *getNMEACallbackNMEAPtr();     // Get a pointer to the callback copy NMEA data
  uint8_t getNMEAMaxLength();            // Get the maximum length of this NMEA message
  nmeaAutomaticFlags *getNMEAFlagsPtr(); // Get a pointer to the flags
#endif

  // RTCM logging
  sfe_ublox_rtcm_filtering_t _logRTCM; // Flags to indicate which NMEA messages should be added to the file buffer for logging

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  RTCM_FRAME_t *_storageRTCM = nullptr;              // Pointer to struct. RAM will be allocated for this if/when necessary
  void crc24q(uint8_t incoming, uint32_t *checksum); // Add incoming to checksum as per CRC-24Q
#endif

  // Define the maximum possible message length for packetAuto and enableUBXlogging
  // UBX_NAV_SAT_MAX_LEN is just > UBX_RXM_RAWX_MAX_LEN
  const uint16_t SFE_UBX_MAX_LENGTH = UBX_NAV_SAT_MAX_LEN;

  // UBX logging
  sfe_ublox_ubx_logging_list_t *sfe_ublox_ubx_logging_list_head = nullptr; // Linked list of which messages to log
  bool logThisUBX(uint8_t UBX_CLASS, uint8_t UBX_ID);                      // Returns true if this UBX should be added to the logging buffer

  // Flag to prevent reentry into checkCallbacks
  // Prevent badness if the user accidentally calls checkCallbacks from inside a callback
  volatile bool checkCallbacksReentrant = false;

  // Support for data logging
  uint8_t *ubxFileBuffer = nullptr;                             // Pointer to the file buffer. RAM is allocated for this if required in .begin
  uint16_t fileBufferSize = 0;                                  // The size of the file buffer. This can be changed by calling setFileBufferSize _before_ .begin
  uint16_t fileBufferHead = 0;                                  // The incoming byte is written into the file buffer at this location
  uint16_t fileBufferTail = 0;                                  // The next byte to be read from the buffer will be read from this location
  uint16_t fileBufferMaxAvail = 0;                              // The maximum number of bytes the file buffer has contained. Handy for checking the buffer is large enough to handle all the incoming data.
  bool createFileBuffer(void);                                  // Create the file buffer. Called by .begin
  uint16_t fileBufferSpaceAvailable(void);                      // Check how much space is available in the buffer
  uint16_t fileBufferSpaceUsed(void);                           // Check how much space is used in the buffer
  bool storePacket(ubxPacket *msg);                             // Add a UBX packet to the file buffer
  bool storeFileBytes(uint8_t *theBytes, uint16_t numBytes);    // Add theBytes to the file buffer
  void writeToFileBuffer(uint8_t *theBytes, uint16_t numBytes); // Write theBytes to the file buffer

  // Support for RTCM buffering
  uint8_t *rtcmBuffer = nullptr;                                // Pointer to the RTCM buffer. RAM is allocated for this if required in .begin
  uint16_t rtcmBufferSize = 0;                                  // The size of the RTCM buffer. This can be changed by calling setRTCMBufferSize _before_ .begin
  uint16_t rtcmBufferHead = 0;                                  // The incoming byte is written into the buffer at this location
  uint16_t rtcmBufferTail = 0;                                  // The next byte to be read from the buffer will be read from this location
  bool createRTCMBuffer(void);                                  // Create the RTCM buffer. Called by .begin
  uint16_t rtcmBufferSpaceAvailable(void);                      // Check how much space is available in the buffer
  uint16_t rtcmBufferSpaceUsed(void);                           // Check how much space is used in the buffer
  bool storeRTCMBytes(uint8_t *theBytes, uint16_t numBytes);    // Add theBytes to the buffer
  void writeToRTCMBuffer(uint8_t *theBytes, uint16_t numBytes); // Write theBytes to the buffer

  // .begin will return true if the assumeSuccess parameter is true and if _signsOfLife is true
  // _signsOfLife is set to true when: a valid UBX message is seen; a valig NMEA header is seen.
  bool _signsOfLife;

  // Keep track of how many keys have been added to CfgValset
  uint8_t _numCfgKeys = 0;

  // Keep track of how many keys have been added to CfgValget and what size the response will be
  uint8_t _numGetCfgKeys = 0;
  uint16_t _lenCfgValGetResponse = 0;
  uint8_t *cfgValgetValueSizes = nullptr; // A pointer to a list of the value sizes for each key in the cfgValget
  uint16_t _cfgValgetMaxPayload = 0;

  // Send the current CFG_VALSET message when packetCfg has less than this many bytes available
  size_t _autoSendAtSpaceRemaining = 0;

public:
  // Flag to indicate if currentSentence should be reset on a (I2C) bus error
  bool _resetCurrentSentenceOnBusError = true;

  typedef union
  {
    uint64_t unsigned64;
    int64_t signed64;
  } unsignedSigned64;

  typedef union
  {
    uint32_t unsigned32;
    int32_t signed32;
  } unsignedSigned32;

  typedef union
  {
    uint16_t unsigned16;
    int16_t signed16;
  } unsignedSigned16;

  typedef union
  {
    uint8_t unsigned8;
    int8_t signed8;
  } unsignedSigned8;

  typedef union
  {
    uint32_t unsigned32;
    float flt;
  } unsigned32float;

  typedef union
  {
    uint64_t unsigned64;
    double dbl;
  } unsigned64double;
};
