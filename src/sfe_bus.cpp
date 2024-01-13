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

// sfe_bus.cpp

#include <Arduino.h>
#include "sfe_bus.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
//

namespace SparkFun_UBLOX_GNSS
{

  SfeI2C::SfeI2C(void) : _i2cPort{nullptr}, _address{0}
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // I2C init()
  //
  // Methods to init/setup this device.
  // The caller can provide a Wire Port, or this class will use the default.
  // Always update the address in case the user has changed the I2C address - see Example9
  bool SfeI2C::init(TwoWire &wirePort, uint8_t address, bool bInit)
  {
    // if we don't have a wire port already
    if (!_i2cPort)
    {
      _i2cPort = &wirePort;

      if (bInit)
        _i2cPort->begin();
    }

    _address = address;

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // I2C init()
  //
  // Methods to init/setup this device.
  // The caller can provide a Wire Port, or this class will use the default.
  bool SfeI2C::init(uint8_t address)
  {
    return init(Wire, address);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // ping()
  //
  // Is a device connected?
  bool SfeI2C::ping()
  {

    if (!_i2cPort)
      return false;

    _i2cPort->beginTransmission(_address);
    return _i2cPort->endTransmission() == 0;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()
  //
  // Checks how many bytes are waiting in the GNSS's I2C buffer
  // It does this by reading registers 0xFD and 0xFE
  //
  // From the u-blox integration manual:
  // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
  //  address and thus allows any register to be read. The second "current address" form omits the
  //  register address. If this second form is used, then an address pointer in the receiver is used to
  //  determine which register to read. This address pointer will increment after each read unless it
  //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
  //  unaltered."

  uint16_t SfeI2C::available()
  {

    if (!_i2cPort)
      return false;

    // Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(0xFD);                               // 0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    uint8_t i2cError = _i2cPort->endTransmission(false); // Always send a restart command. Do not release the bus. ESP32 supports this.
    if (i2cError != 0)
    {
      return (0); // Sensor did not ACK
    }

    // Forcing requestFrom to use a restart would be unwise. If bytesAvailable is zero, we want to surrender the bus.
    uint16_t bytesReturned = _i2cPort->requestFrom(_address, static_cast<uint8_t>(2));
    if (bytesReturned != 2)
    {
      return (0); // Sensor did not return 2 bytes
    }
    else // if (_i2cPort->available())
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    return (bytesAvailable);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

  uint8_t SfeI2C::writeBytes(uint8_t *dataToWrite, uint8_t length)
  {
    if (!_i2cPort)
      return 0;

    if (length == 0)
      return 0;

    _i2cPort->beginTransmission(_address);
    uint8_t written = _i2cPort->write((const uint8_t *)dataToWrite, length);
    if (_i2cPort->endTransmission() == 0)
      return written;

    return 0;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // readBytes()

  uint8_t SfeI2C::readBytes(uint8_t *data, uint8_t length)
  {
    if (!_i2cPort)
      return 0;

    if (length == 0)
      return 0;

    uint8_t bytesReturned = _i2cPort->requestFrom(_address, length);

    for (uint8_t i = 0; i < bytesReturned; i++)
      *data++ = _i2cPort->read();

    return bytesReturned;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Constructor
  //

  SfeSPI::SfeSPI(void) : _spiPort{nullptr}
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // SPI init()
  //
  // Methods to init/setup this device. The caller can provide a SPI Port, or this class
  // will use the default

  bool SfeSPI::init(SPIClass &spiPort, SPISettings &ismSPISettings, uint8_t cs, bool bInit)
  {

    // if we don't have a SPI port already
    if (!_spiPort)
    {
      _spiPort = &spiPort;

      if (bInit)
        _spiPort->begin();
    }

    // SPI settings are needed for every transaction
    _sfeSPISettings = ismSPISettings;

    // The chip select pin can vary from platform to platform and project to project
    // and so it must be given by the user.
    if (!cs)
      return false;

    _cs = cs;

    // Initialize the chip select pin
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // SPI init()
  //
  // Methods to init/setup this device. The caller can provide a SPI Port, or this class
  // will use the default
  bool SfeSPI::init(uint8_t cs)
  {

    // If the transaction settings are not provided by the user they are built here.
    SPISettings spiSettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);

    // In addition of the port is not provided by the user, it defaults to SPI here.
    return init(SPI, spiSettings, cs);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // SPI init()
  //
  // Methods to init/setup this device. The caller can provide a SPI Port, or this class
  // will use the default
  bool SfeSPI::init(SPIClass &spiPort, uint32_t spiSpeed, uint8_t cs, bool bInit)
  {

    // If the transaction settings are not provided by the user they are built here.
    SPISettings spiSettings = SPISettings(spiSpeed, MSBFIRST, SPI_MODE0);

    // In addition of the port is not provided by the user, it defaults to SPI here.
    return init(spiPort, spiSettings, cs, bInit);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()
  //
  // available isn't applicable for SPI

  uint16_t SfeSPI::available()
  {
    return (0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

  uint8_t SfeSPI::writeBytes(uint8_t *data, uint8_t length)
  {
    if (!_spiPort)
      return 0;

    if (length == 0)
      return 0;

    uint8_t i;

    // Apply settings
    _spiPort->beginTransaction(_sfeSPISettings);

    // Signal communication start
    digitalWrite(_cs, LOW);

    for (i = 0; i < length; i++)
    {
      _spiPort->transfer(*data++);
    }

    // End communication
    digitalWrite(_cs, HIGH);
    _spiPort->endTransaction();

    return i;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // readBytes()

  uint8_t SfeSPI::readBytes(uint8_t *data, uint8_t length)
  {
    if (!_spiPort)
      return 0;

    if (length == 0)
      return 0;

    uint8_t i; // counter in loop

    // Apply settings
    _spiPort->beginTransaction(_sfeSPISettings);

    // Signal communication start
    digitalWrite(_cs, LOW);

    for (i = 0; i < length; i++)
    {
      *data++ = _spiPort->transfer(0xFF);
    }

    // End transaction
    digitalWrite(_cs, HIGH);
    _spiPort->endTransaction();

    return i;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // writeReadBytes()

  uint8_t SfeSPI::writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length)
  {
    if (!_spiPort)
      return 0;

    if (length == 0)
      return 0;

    uint8_t i; // counter in loop

    // Apply settings
    _spiPort->beginTransaction(_sfeSPISettings);

    // Signal communication start
    digitalWrite(_cs, LOW);

    for (i = 0; i < length; i++)
    {
      *readData = _spiPort->transfer(*data);
      data++;
      readData++;
    }

    // End transaction
    digitalWrite(_cs, HIGH);
    _spiPort->endTransaction();

    return i;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // writeReadBytes()

  void SfeSPI::startWriteReadByte() // beginTransaction
  {
    if (!_spiPort)
      return;

    // Apply settings
    _spiPort->beginTransaction(_sfeSPISettings);

    // Signal communication start
    digitalWrite(_cs, LOW);
  }
  void SfeSPI::writeReadByte(const uint8_t *data, uint8_t *readData)
  {
    *readData = _spiPort->transfer(*data);
  }
  void SfeSPI::writeReadByte(const uint8_t data, uint8_t *readData)
  {
    *readData = _spiPort->transfer(data);
  }
  void SfeSPI::endWriteReadByte() // endTransaction
  {
    digitalWrite(_cs, HIGH);
    _spiPort->endTransaction();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Constructor
  //

  SfeSerial::SfeSerial(void) : _serialPort{nullptr}
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial init()
  //
  // Methods to init/setup this device

  bool SfeSerial::init(Stream &serialPort)
  {
    // if we don't have a port already
    if (!_serialPort)
    {
      _serialPort = &serialPort;
    }

    // Get rid of any stale serial data already in the processor's RX buffer
    while (_serialPort->available())
      _serialPort->read();

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()

  uint16_t SfeSerial::available()
  {

    if (!_serialPort)
      return 0;

    return (_serialPort->available());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

  uint8_t SfeSerial::writeBytes(uint8_t *dataToWrite, uint8_t length)
  {
    if (!_serialPort)
      return 0;

    if (length == 0)
      return 0;

    return _serialPort->write(dataToWrite, length);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // readBytes()

  uint8_t SfeSerial::readBytes(uint8_t *data, uint8_t length)
  {
    if (!_serialPort)
      return 0;

    if (length == 0)
      return 0;

#ifdef PARTICLE
    return _serialPort->readBytes((char *)data, length);
#else
    return _serialPort->readBytes(data, length);
#endif
  }
}
