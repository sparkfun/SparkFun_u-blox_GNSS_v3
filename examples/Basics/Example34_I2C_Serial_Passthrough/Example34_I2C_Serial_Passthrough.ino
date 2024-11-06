/*
  I2C - Serial Passthrough
  By: Paul Clark
  SparkFun Electronics
  Date: August 14th, 2024
  License: MIT. See license file for more information.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Connect the RedBoard to your PC using a USB cable
  Connect with u-center at 115200 baud to communicate with the GNSS

  How it works:
  Data arriving on Serial is added to the uartI2cRingBuffer.
  If the uartI2cRingBuffer contains at least uartI2cSendLimit bytes, these are 'pushed' to the GNSS I2C register 0xFF.
  If the uartI2cRingBuffer contains less than uartI2cSendLimit bytes, and it is more than uartI2cSendInterval_ms
  since the last send, these are 'pushed' to the GNSS I2C register 0xFF.
  Every i2cReadInterval_ms, the number of available bytes in the GNSS I2C buffer is read (from registers 0xFD and 0xFE)
  and stored in i2cBytesAvailable.
  If GNSS data is available, it is read in blocks of i2cReadLimit bytes and stored in i2cUartRingBuffer.
  If the i2cUartRingBuffer contains at least i2cUartSendLimit bytes, these are written to Serial.
  If the i2cUartRingBuffer contains less than i2cUartSendLimit bytes, and it is more than i2cUartSendInterval_ms
  since the last send, these are written to Serial.
*/

#include "Arduino.h"

HardwareSerial &mySerial = Serial; // USB Serial. Change this if needed

#include <Wire.h>

TwoWire &myWire = Wire; // TwoWire (I2C) connection to GNSS. Change this if needed

const uint8_t gnssAddress = 0x42; // GNSS I2C address (unshifted)

const uint16_t ringBufferSize = 512; // Define the size of the two ring buffers
uint8_t i2cUartRingBuffer[ringBufferSize];
uint16_t i2cUartBufferHead = 0;
uint16_t i2cUartBufferTail = 0;
uint8_t uartI2cRingBuffer[ringBufferSize];
uint16_t uartI2cBufferHead = 0;
uint16_t uartI2cBufferTail = 0;

const unsigned long uartI2cSendInterval_ms = 5;
unsigned long uartI2cLastSend_ms = 0;
const uint16_t uartI2cSendLimit = 16;

const unsigned long i2cUartSendInterval_ms = 5;
unsigned long i2cUartLastSend_ms = 0;
const uint16_t i2cUartSendLimit = 16;

const unsigned long i2cReadInterval_ms = 50;
unsigned long i2cLastRead_ms = 0;
uint16_t i2cBytesAvailable = 0;
const uint16_t i2cReadLimit = 16;

void setup()
{

    delay(2000); // Wait for ESP32 and GNSS to start up

    mySerial.begin(115200); // Baud rate for u-center

    myWire.begin();          // Start I2C
    myWire.setClock(400000); // 400kHz

    // Give I2C a kickstart - if needed. Request UBX-MON-VER
    const uint8_t pollUbxMonVer[] = { 0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 };
    for (int i = 0; i < (sizeof(pollUbxMonVer) / sizeof(uint8_t)); i++) {
        addToUartI2cBuffer(pollUbxMonVer[i]);
    }

} // /setup

void loop()
{

    // If it is more than i2cReadInterval_ms since the last read, read how
    // many bytes are available in the GNSS I2C buffer. This will leave the register
    // address pointing at 0xFF.
    if ((millis() > (i2cLastRead_ms + i2cReadInterval_ms)) || (i2cLastRead_ms == 0))
    {
        i2cBytesAvailable = gnssI2cAvailable();
        i2cLastRead_ms = millis();
    }

    // If Serial data is available, add it to the buffer
    if (Serial.available())
        addToUartI2cBuffer(Serial.read());

    // Check how many bytes are available in the uartI2cRingBuffer
    uint16_t uartI2cAvailable = ringBufferAvailable(uartI2cBufferHead, uartI2cBufferTail);
    if (uartI2cAvailable > 0)
    {
        // We must avoid sending a single byte. Send one byte less if needed.
        uint16_t bytesToSend = uartI2cAvailable;
        if (bytesToSend > uartI2cSendLimit) // Limit to uartI2cSendLimit
            bytesToSend = uartI2cSendLimit;
        if ((uartI2cAvailable - bytesToSend) == 1) // If this would leave one byte in the buffer
            bytesToSend--;                         // Send one byte less
        // If uartI2cRingBuffer contains at least uartI2cSendLimit bytes, send them
        if (bytesToSend >= (uartI2cSendLimit - 1))
        {
            sendI2cBytes(bytesToSend);
            uartI2cLastSend_ms = millis();
        }
        // Else if uartI2cRingBuffer contains data and it is more than uartI2cSendInterval_ms
        // since the last send, send them
        else if ((bytesToSend > 0) && (millis() > (uartI2cLastSend_ms + uartI2cSendInterval_ms)))
        {
            sendI2cBytes(bytesToSend);
            uartI2cLastSend_ms = millis();
        }
    }

    // If the GNSS has data, read it now
    if (i2cBytesAvailable > 0)
    {
        // Read a maximum of i2cReadLimit, to prevent the code stalling here
        uint16_t bytesToRead = i2cBytesAvailable;
        if (bytesToRead > i2cReadLimit)
            bytesToRead = i2cReadLimit;
        if (readI2cBytes(bytesToRead))
            i2cBytesAvailable -= bytesToRead;
    }

    // Check how much data is in the i2cUartRingBuffer
    uint16_t i2cUartAvailable = ringBufferAvailable(i2cUartBufferHead, i2cUartBufferTail);
    if (i2cUartAvailable > 0)
    {
        uint16_t bytesToSend = i2cUartAvailable;
        if (bytesToSend > i2cUartSendLimit)
            bytesToSend = i2cUartSendLimit;
        // If the buffer contains i2cUartSendLimit bytes, send them
        if (bytesToSend == i2cUartSendLimit)
        {
            sendUartBytes(bytesToSend);
            i2cUartLastSend_ms = millis();
        }
        // Else if i2cUartRingBuffer contains data and it is more than i2cUartSendInterval_ms
        // since the last send, send them
        else if ((bytesToSend > 0) && (millis() > (i2cUartLastSend_ms + i2cUartSendInterval_ms)))
        {
            sendUartBytes(bytesToSend);
            i2cUartLastSend_ms = millis();
        }
    }

} // /loop

// Read how many bytes are available in the GNSS I2C buffer.
// This will leave the register address pointing at 0xFF.
uint16_t gnssI2cAvailable()
{
    // Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    myWire.beginTransmission(gnssAddress);
    myWire.write(0xFD);                               // 0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    uint8_t i2cError = myWire.endTransmission(false); // Always send a restart command. Do not release the bus. ESP32 supports this.
    if (i2cError != 0)
    {
        return (0); // Sensor did not ACK
    }

    // Forcing requestFrom to use a restart would be unwise. If bytesAvailable is zero, we want to surrender the bus.
    uint16_t bytesReturned = myWire.requestFrom(gnssAddress, static_cast<uint8_t>(2));
    if (bytesReturned != 2)
    {
        return (0); // Sensor did not return 2 bytes
    }
    else // if (myWire.available())
    {
        uint8_t msb = myWire.read();
        uint8_t lsb = myWire.read();
        bytesAvailable = (((uint16_t)msb) << 8) | lsb;
    }

    return (bytesAvailable);
} // /gnssI2cAvailable

// Add b to the uartI2cRingBuffer if space is available
bool addToUartI2cBuffer(uint8_t b)
{
    if (ringBufferSpace(uartI2cBufferHead, uartI2cBufferTail) > 0)
    {
        uartI2cRingBuffer[uartI2cBufferHead++] = b;
        uartI2cBufferHead %= ringBufferSize; // Wrap-around
        return true;
    }

    return false; // Buffer is full
}

// Send numBytes from i2cUartBuffer to Serial
// This function assumes ringBufferAvailable has been called externally
// It will read the bytes regardless
bool sendUartBytes(uint16_t numBytes)
{
    if (numBytes == 0)
        return false;

    if (numBytes > i2cUartSendLimit)
        numBytes = i2cUartSendLimit;

    static uint8_t store[i2cUartSendLimit]; // Store the data temporarily
    for (uint16_t i = 0; i < numBytes; i++)
        store[i] = readI2cUartBuffer();

    return (mySerial.write(store, numBytes) == numBytes);
}

// Send numBytes from uartI2cBuffer to GNSS
// This function assumes ringBufferAvailable has been called externally
// It will read the bytes regardless
// Note: we cannot send a single byte. numBytes must be >= 2.
//       Otherwise the write will set the register address instead.
bool sendI2cBytes(uint16_t numBytes)
{
    if (numBytes < 2)
        return false;

    if (numBytes > uartI2cSendLimit)
        numBytes = uartI2cSendLimit;

    static uint8_t store[uartI2cSendLimit]; // Store the data temporarily
    for (uint16_t i = 0; i < numBytes; i++)
        store[i] = readUartI2cBuffer();

    // Assume the GNSS register address is already set to 0xFF
    myWire.beginTransmission(gnssAddress);
    myWire.write((const uint8_t *)store, numBytes);
    if (myWire.endTransmission() == 0)
        return true;

    return false;
}

// Read numBytes from the GNSS. Store them in i2cUartRingBuffer
bool readI2cBytes(uint16_t numBytes)
{
    if (numBytes == 0)
        return false;

    uint16_t bytesRequested = 0;
    uint16_t bytesLeftToRead = numBytes;

    while ((bytesLeftToRead > 0) && (bytesRequested < numBytes))
    {
        uint8_t bytesToRead;
        if (bytesLeftToRead > 255)
            bytesToRead = 255;
        else
            bytesToRead = bytesLeftToRead;
        if (bytesToRead > i2cReadLimit)
            bytesToRead = i2cReadLimit;

        uint8_t bytesReturned = myWire.requestFrom(gnssAddress, bytesToRead);

        for (uint8_t i = 0; i < bytesReturned; i++)
        {
            uint8_t b = myWire.read();
            if (ringBufferSpace(i2cUartBufferHead, i2cUartBufferTail) > 0)
            {
                i2cUartRingBuffer[i2cUartBufferHead++] = b;
                i2cUartBufferHead %= ringBufferSize; // Wrap-around
            }
            else
            {
                // Buffer is full
            }
        }

        bytesRequested += bytesToRead;
        bytesLeftToRead -= bytesReturned;
    }

    return (bytesLeftToRead == 0);
}

// Read a single byte from the uartI2cRingBuffer
// This function assumes ringBufferAvailable has been called externally
// It will read a byte regardless
uint8_t readUartI2cBuffer()
{
    uint8_t b = uartI2cRingBuffer[uartI2cBufferTail++];
    uartI2cBufferTail %= ringBufferSize; // Wrap-around
    return b;
}

// Read a single byte from the i2cUartRingBuffer
// This function assumes ringBufferAvailable has been called externally
// It will read a byte regardless
uint8_t readI2cUartBuffer()
{
    uint8_t b = i2cUartRingBuffer[i2cUartBufferTail++];
    i2cUartBufferTail %= ringBufferSize; // Wrap-around
    return b;
}

// Calculate how many ring buffer bytes are available
uint16_t ringBufferAvailable(uint16_t head, uint16_t tail)
{
    if (head == tail) // If the buffer is empty
        return 0;
    if (head > tail)
    { // No wrap-around
        return (head - tail);
    }
    // Use uint32_t to make the wrap-around easier
    uint32_t h = head;
    uint32_t t = tail;
    const uint32_t s = ringBufferSize;
    return (uint16_t)((h + s) - t);
}

// Calculate how much space is available in the ring buffer
// Buffer can hold (ringBufferSize - 1) bytes
// Buffer is empty when head == tail
// Buffer is full when head is one byte behind tail
uint16_t ringBufferSpace(uint16_t head, uint16_t tail)
{
    return (ringBufferSize - (ringBufferAvailable(head, tail) + 1));
}