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

// sfe_bus.h

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

namespace SparkFun_UBLOX_GNSS
{

  // The following abstract class is used an interface for upstream implementation.
  class GNSSDeviceBus
  {
  public:
    // For I2C, ping the _address
    // Not Applicable for SPI and Serial
    virtual bool ping() = 0;

    // For Serial, return Serial.available()
    // For I2C, read registers 0xFD and 0xFE. Return bytes available as uint16_t
    // Not Applicable for SPI
    virtual uint16_t available() = 0;

    // For Serial, do Serial.write
    // For I2C, push data to register 0xFF. Chunkify if necessary. Prevent single byte writes as these are illegal
    // For SPI, writing bytes will also read bytes simultaneously. Read data is _ignored_ here. Use writeReadBytes
    virtual uint8_t writeBytes(uint8_t *data, uint8_t length) = 0;

    // For SPI, writing bytes will also read bytes simultaneously. Read data is returned in readData
    virtual uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length) = 0;
    virtual void startWriteReadByte() = 0;                                  // beginTransaction
    virtual void writeReadByte(const uint8_t *data, uint8_t *readData) = 0; // transfer
    virtual void writeReadByte(const uint8_t data, uint8_t *readData) = 0;  // transfer
    virtual void endWriteReadByte() = 0;                                    // endTransaction

    // For Serial, attempt Serial.read
    // For I2C, read from register 0xFF
    // For SPI, read the byte while writing 0xFF
    virtual uint8_t readBytes(uint8_t *data, uint8_t length) = 0;
  };

  // The SfeI2C device defines behavior for I2C implementation based around the TwoWire class (Wire).
  // This is Arduino specific.
  class SfeI2C : public GNSSDeviceBus
  {
  public:
    SfeI2C(void);

    bool init(uint8_t address);

    bool init(TwoWire &wirePort, uint8_t address, bool bInit = false);

    bool ping();

    uint16_t available();

    uint8_t writeBytes(uint8_t *data, uint8_t length);

    uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length)
    { (void)data; (void)readData; (void)length; return 0; }

    void startWriteReadByte(){};
    void writeReadByte(const uint8_t *data, uint8_t *readData){ (void)data; (void)readData; }
    void writeReadByte(const uint8_t data, uint8_t *readData){ (void)data; (void)readData; }
    void endWriteReadByte(){};

    uint8_t readBytes(uint8_t *data, uint8_t length);

  private:
    TwoWire *_i2cPort;
    uint8_t _address;
  };

  // The SfeSPI class defines behavior for SPI implementation based around the SPIClass class (SPI).
  // This is Arduino specific.
  // Note that writeBytes also reads bytes (into data)
  class SfeSPI : public GNSSDeviceBus
  {
  public:
    SfeSPI(void);

    bool init(uint8_t cs);

    bool init(SPIClass &spiPort, SPISettings &spiSettings, uint8_t cs, bool bInit = false);

    bool init(SPIClass &spiPort, uint32_t spiSpeed, uint8_t cs, bool bInit = false);

    bool ping() { return false; }

    uint16_t available();

    uint8_t writeBytes(uint8_t *data, uint8_t length);

    uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length);

    void startWriteReadByte();
    void writeReadByte(const uint8_t *data, uint8_t *readData);
    void writeReadByte(const uint8_t data, uint8_t *readData);
    void endWriteReadByte();

    uint8_t readBytes(uint8_t *data, uint8_t length);

  private:
    SPIClass *_spiPort;
    // Settings are used for every transaction.
    SPISettings _sfeSPISettings;
    uint8_t _cs;
  };

  // The sfeSerial device defines behavior for Serial (UART) implementation based around the Stream class.
  // This is Arduino specific.
  class SfeSerial : public GNSSDeviceBus
  {
  public:
    SfeSerial(void);

    bool init(Stream &serialPort);

    bool ping() { return false; }

    uint16_t available();

    uint8_t writeBytes(uint8_t *data, uint8_t length);

    uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length)
    { (void)data; (void)readData; (void)length; return 0; }

    void startWriteReadByte(){};
    void writeReadByte(const uint8_t *data, uint8_t *readData){ (void)data; (void)readData; }
    void writeReadByte(const uint8_t data, uint8_t *readData){ (void)data; (void)readData; }
    void endWriteReadByte(){};

    uint8_t readBytes(uint8_t *data, uint8_t length);

  private:
    Stream *_serialPort;
  };

  // The sfePrint device defines behavior for Serial diagnostic prints based around the Stream class.
  // This is Arduino specific.
  class SfePrint
  {
  public:
    SfePrint(void) { _outputPort = nullptr; }

    void init(Print &outputPort) { _outputPort = &outputPort; }
    inline bool operator==(SfePrint const &other) const { return _outputPort == other._outputPort; }
    inline bool operator!=(SfePrint const &other) const { return !(*this == other); }
    
    void write(uint8_t c)
    {
      if (_outputPort != nullptr)
        _outputPort->write(c);
    }
    void print(const char *c)
    {
      if (_outputPort != nullptr)
        _outputPort->print(c);
    }
    void print(const __FlashStringHelper *c)
    {
      if (_outputPort != nullptr)
        _outputPort->print(c);
    }
    void print(unsigned int c, int f)
    {
      if (_outputPort != nullptr)
        _outputPort->print(c, f);
    }
    void print(uint16_t c)
    {
      if (_outputPort != nullptr)
        _outputPort->print(c);
    }
    void println()
    {
      if (_outputPort != nullptr)
        _outputPort->println();
    }
    void println(const char *c)
    {
      if (_outputPort != nullptr)
        _outputPort->println(c);
    }
    void println(const __FlashStringHelper *c)
    {
      if (_outputPort != nullptr)
        _outputPort->println(c);
    }
    void println(size_t c)
    {
      if (_outputPort != nullptr)
        _outputPort->println(c);
    }
    void println(uint8_t c, int f)
    {
      if (_outputPort != nullptr)
        _outputPort->println(c, f);
    }
  
  private: 
    Print *_outputPort;
  };

};
