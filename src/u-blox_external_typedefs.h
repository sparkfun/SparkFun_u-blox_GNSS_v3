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

#define kUBLOXGNSSDefaultAddress 0x42 // Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series

// A default of 250ms for maxWait seems fine for I2C but is not enough for SerialUSB.
// If you know you are only going to be using I2C / Qwiic communication, you can
// safely reduce kUBLOXGNSSDefaultMaxWait to 250.
#ifndef kUBLOXGNSSDefaultMaxWait // Let's allow the user to define their own value if they want to
#define kUBLOXGNSSDefaultMaxWait 1100
#endif

// Global Status Returns
typedef enum
{
  SFE_UBLOX_STATUS_SUCCESS,
  SFE_UBLOX_STATUS_FAIL,
  SFE_UBLOX_STATUS_CRC_FAIL,
  SFE_UBLOX_STATUS_TIMEOUT,
  SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
  SFE_UBLOX_STATUS_OUT_OF_RANGE,
  SFE_UBLOX_STATUS_INVALID_ARG,
  SFE_UBLOX_STATUS_INVALID_OPERATION,
  SFE_UBLOX_STATUS_MEM_ERR,
  SFE_UBLOX_STATUS_HW_ERR,
  SFE_UBLOX_STATUS_DATA_SENT,     // This indicates that a 'set' was successful
  SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
  SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
  SFE_UBLOX_STATUS_SPI_COMM_FAILURE,
  SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} sfe_ublox_status_e;

// ubxPacket validity
typedef enum
{
  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
  SFE_UBLOX_PACKET_VALIDITY_VALID,
  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
  SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} sfe_ublox_packet_validity_e;

// Identify which packet buffer is in use:
// packetCfg (or a custom packet), packetAck or packetBuf
// packetAuto is used to store expected "automatic" messages
typedef enum
{
  SFE_UBLOX_PACKET_PACKETCFG,
  SFE_UBLOX_PACKET_PACKETACK,
  SFE_UBLOX_PACKET_PACKETBUF,
  SFE_UBLOX_PACKET_PACKETAUTO
} sfe_ublox_packet_buffer_e;

// Define a struct to allow selective logging / processing of NMEA messages
// Set the individual bits to pass the NMEA messages to the file buffer and/or processNMEA
// Setting bits.all will pass all messages to the file buffer and processNMEA
typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;
      uint32_t UBX_NMEA_DTM : 1;
      uint32_t UBX_NMEA_GAQ : 1;
      uint32_t UBX_NMEA_GBQ : 1;
      uint32_t UBX_NMEA_GBS : 1;
      uint32_t UBX_NMEA_GGA : 1;
      uint32_t UBX_NMEA_GLL : 1;
      uint32_t UBX_NMEA_GLQ : 1;
      uint32_t UBX_NMEA_GNQ : 1;
      uint32_t UBX_NMEA_GNS : 1;
      uint32_t UBX_NMEA_GPQ : 1;
      uint32_t UBX_NMEA_GQQ : 1;
      uint32_t UBX_NMEA_GRS : 1;
      uint32_t UBX_NMEA_GSA : 1;
      uint32_t UBX_NMEA_GST : 1;
      uint32_t UBX_NMEA_GSV : 1;
      uint32_t UBX_NMEA_RLM : 1;
      uint32_t UBX_NMEA_RMC : 1;
      uint32_t UBX_NMEA_TXT : 1;
      uint32_t UBX_NMEA_VLW : 1;
      uint32_t UBX_NMEA_VTG : 1;
      uint32_t UBX_NMEA_ZDA : 1;
      uint32_t UBX_NMEA_THS : 1;
    } bits;
  };
} sfe_ublox_nmea_filtering_t;

// Define an enum to make it easy to enable/disable selected NMEA messages for logging / processing
typedef enum
{
  SFE_UBLOX_FILTER_NMEA_ALL = 0x00000001,
  SFE_UBLOX_FILTER_NMEA_DTM = 0x00000002,
  SFE_UBLOX_FILTER_NMEA_GAQ = 0x00000004,
  SFE_UBLOX_FILTER_NMEA_GBQ = 0x00000008,
  SFE_UBLOX_FILTER_NMEA_GBS = 0x00000010,
  SFE_UBLOX_FILTER_NMEA_GGA = 0x00000020,
  SFE_UBLOX_FILTER_NMEA_GLL = 0x00000040,
  SFE_UBLOX_FILTER_NMEA_GLQ = 0x00000080,
  SFE_UBLOX_FILTER_NMEA_GNQ = 0x00000100,
  SFE_UBLOX_FILTER_NMEA_GNS = 0x00000200,
  SFE_UBLOX_FILTER_NMEA_GPQ = 0x00000400,
  SFE_UBLOX_FILTER_NMEA_GQQ = 0x00000800,
  SFE_UBLOX_FILTER_NMEA_GRS = 0x00001000,
  SFE_UBLOX_FILTER_NMEA_GSA = 0x00002000,
  SFE_UBLOX_FILTER_NMEA_GST = 0x00004000,
  SFE_UBLOX_FILTER_NMEA_GSV = 0x00008000,
  SFE_UBLOX_FILTER_NMEA_RLM = 0x00010000,
  SFE_UBLOX_FILTER_NMEA_RMC = 0x00020000,
  SFE_UBLOX_FILTER_NMEA_TXT = 0x00040000,
  SFE_UBLOX_FILTER_NMEA_VLW = 0x00080000,
  SFE_UBLOX_FILTER_NMEA_VTG = 0x00100000,
  SFE_UBLOX_FILTER_NMEA_ZDA = 0x00200000,
  SFE_UBLOX_FILTER_NMEA_THS = 0x00400000,
} sfe_ublox_nmea_filtering_e;

// Define a struct to allow selective logging of RTCM messages
// Set the individual bits to pass the RTCM messages to the file buffer
// Setting bits.all will pass all messages to the file buffer
typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;
      uint32_t UBX_RTCM_TYPE1001 : 1;
      uint32_t UBX_RTCM_TYPE1002 : 1;
      uint32_t UBX_RTCM_TYPE1003 : 1;
      uint32_t UBX_RTCM_TYPE1004 : 1;
      uint32_t UBX_RTCM_TYPE1005 : 1;
      uint32_t UBX_RTCM_TYPE1006 : 1;
      uint32_t UBX_RTCM_TYPE1007 : 1;
      uint32_t UBX_RTCM_TYPE1009 : 1;
      uint32_t UBX_RTCM_TYPE1010 : 1;
      uint32_t UBX_RTCM_TYPE1011 : 1;
      uint32_t UBX_RTCM_TYPE1012 : 1;
      uint32_t UBX_RTCM_TYPE1033 : 1;
      uint32_t UBX_RTCM_TYPE1074 : 1;
      uint32_t UBX_RTCM_TYPE1075 : 1;
      uint32_t UBX_RTCM_TYPE1077 : 1;
      uint32_t UBX_RTCM_TYPE1084 : 1;
      uint32_t UBX_RTCM_TYPE1085 : 1;
      uint32_t UBX_RTCM_TYPE1087 : 1;
      uint32_t UBX_RTCM_TYPE1094 : 1;
      uint32_t UBX_RTCM_TYPE1095 : 1;
      uint32_t UBX_RTCM_TYPE1097 : 1;
      uint32_t UBX_RTCM_TYPE1124 : 1;
      uint32_t UBX_RTCM_TYPE1125 : 1;
      uint32_t UBX_RTCM_TYPE1127 : 1;
      uint32_t UBX_RTCM_TYPE1230 : 1;
      uint32_t UBX_RTCM_TYPE4072_0 : 1;
      uint32_t UBX_RTCM_TYPE4072_1 : 1;
    } bits;
  };
} sfe_ublox_rtcm_filtering_t;

// Define an enum to make it easy to enable/disable selected NMEA messages for logging / processing
typedef enum
{
  SFE_UBLOX_FILTER_RTCM_ALL = 0x00000001,
  SFE_UBLOX_FILTER_RTCM_TYPE1001 = 0x00000002,
  SFE_UBLOX_FILTER_RTCM_TYPE1002 = 0x00000004,
  SFE_UBLOX_FILTER_RTCM_TYPE1003 = 0x00000008,
  SFE_UBLOX_FILTER_RTCM_TYPE1004 = 0x00000010,
  SFE_UBLOX_FILTER_RTCM_TYPE1005 = 0x00000020,
  SFE_UBLOX_FILTER_RTCM_TYPE1006 = 0x00000040,
  SFE_UBLOX_FILTER_RTCM_TYPE1007 = 0x00000080,
  SFE_UBLOX_FILTER_RTCM_TYPE1009 = 0x00000100,
  SFE_UBLOX_FILTER_RTCM_TYPE1010 = 0x00000200,
  SFE_UBLOX_FILTER_RTCM_TYPE1011 = 0x00000400,
  SFE_UBLOX_FILTER_RTCM_TYPE1012 = 0x00000800,
  SFE_UBLOX_FILTER_RTCM_TYPE1033 = 0x00001000,
  SFE_UBLOX_FILTER_RTCM_TYPE1074 = 0x00002000,
  SFE_UBLOX_FILTER_RTCM_TYPE1075 = 0x00004000,
  SFE_UBLOX_FILTER_RTCM_TYPE1077 = 0x00008000,
  SFE_UBLOX_FILTER_RTCM_TYPE1084 = 0x00010000,
  SFE_UBLOX_FILTER_RTCM_TYPE1085 = 0x00020000,
  SFE_UBLOX_FILTER_RTCM_TYPE1087 = 0x00040000,
  SFE_UBLOX_FILTER_RTCM_TYPE1094 = 0x00080000,
  SFE_UBLOX_FILTER_RTCM_TYPE1095 = 0x00100000,
  SFE_UBLOX_FILTER_RTCM_TYPE1097 = 0x00200000,
  SFE_UBLOX_FILTER_RTCM_TYPE1124 = 0x00400000,
  SFE_UBLOX_FILTER_RTCM_TYPE1125 = 0x00800000,
  SFE_UBLOX_FILTER_RTCM_TYPE1127 = 0x01000000,
  SFE_UBLOX_FILTER_RTCM_TYPE1230 = 0x02000000,
  SFE_UBLOX_FILTER_RTCM_TYPE4072_0 = 0x04000000,
  SFE_UBLOX_FILTER_RTCM_TYPE4072_1 = 0x08000000,
} sfe_ublox_rtcm_filtering_e;

// Define a linked list which defines which UBX messages should be logged automatically
// - without needing to have and use the Auto methods
struct sfe_ublox_ubx_logging_list_t
{
  uint8_t UBX_CLASS;
  uint8_t UBX_ID;
  bool enable;
  sfe_ublox_ubx_logging_list_t *next;
};

//-=-=-=-=-

#ifndef MAX_PAYLOAD_SIZE
// v2.0: keep this for backwards-compatibility, but this is largely superseded by setPacketCfgPayloadSize
#define MAX_PAYLOAD_SIZE 246 // We need ~220 bytes for getProtocolVersion on most ublox modules
// #define MAX_PAYLOAD_SIZE 768 //Worst case: UBX_CFG_VALSET packet with 64 keyIDs each with 64 bit values
#endif

// Limit for SPI transactions
#define SFE_UBLOX_SPI_TRANSACTION_SIZE 32
// Default size of the SPI buffer - 8 larger than packetCfg payload
#define SFE_UBLOX_SPI_BUFFER_DEFAULT_SIZE (MAX_PAYLOAD_SIZE + 8)

// Default maximum NMEA byte count
// maxNMEAByteCount was set to 82: https://en.wikipedia.org/wiki/NMEA_0183#Message_structure
// but the u-blox HP (RTK) GGA messages are 88 bytes long
// The user can adjust maxNMEAByteCount by calling setMaxNMEAByteCount
// To be safe - use 100 like we do for the Auto NMEA messages
#define SFE_UBLOX_MAX_NMEA_BYTE_COUNT 100

//-=-=-=-=- UBX binary specific variables
struct ubxPacket
{
  uint8_t cls;
  uint8_t id;
  uint16_t len;          // Length of the payload. Does not include cls, id, or checksum bytes
  uint16_t counter;      // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  uint16_t startingSpot; // The counter value needed to go past before we begin recording into payload array
  uint8_t *payload;      // We will allocate RAM for the payload if/when needed.
  uint8_t checksumA;     // Given to us from module. Checked against the rolling calculated A/B checksums.
  uint8_t checksumB;
  sfe_ublox_packet_validity_e valid;           // Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};

// Struct to hold the results returned by getGeofenceState (returned by UBX-NAV-GEOFENCE)
typedef struct
{
  uint8_t status;    // Geofencing status: 0 - Geofencing not available or not reliable; 1 - Geofencing active
  uint8_t numFences; // Number of geofences
  uint8_t combState; // Combined (logical OR) state of all geofences: 0 - Unknown; 1 - Inside; 2 - Outside
  uint8_t states[4]; // Geofence states: 0 - Unknown; 1 - Inside; 2 - Outside
} geofenceState;

// Struct to hold the current geofence parameters
typedef struct
{
  uint8_t numFences; // Number of active geofences
  int32_t lats[4];   // Latitudes of geofences (in degrees * 10^-7)
  int32_t longs[4];  // Longitudes of geofences (in degrees * 10^-7)
  uint32_t rads[4];  // Radii of geofences (in m * 10^-2)
} geofenceParams_t;

// Struct to hold the module software version
#define firmwareTypeLen 3 // HPG, SPG, etc.
#define moduleNameMaxLen 13 // Allow for: 4-chars minus 4-chars minus 3-chars
typedef struct
{
  uint8_t protocolVersionLow; // Loaded from getModuleInfo()
  uint8_t protocolVersionHigh;
  uint8_t firmwareVersionLow;
  uint8_t firmwareVersionHigh;
  char firmwareType[firmwareTypeLen + 1]; // Include space for a NULL
  char moduleName[moduleNameMaxLen + 1]; // Include space for a NULL
  bool moduleQueried;
} moduleSWVersion_t;

const uint32_t SFE_UBLOX_DAYS_FROM_1970_TO_2020 = 18262; // Jan 1st 2020 Epoch = 1577836800 seconds
const uint16_t SFE_UBLOX_DAYS_SINCE_2020[80] =
    {
        0, 366, 731, 1096, 1461, 1827, 2192, 2557, 2922, 3288,
        3653, 4018, 4383, 4749, 5114, 5479, 5844, 6210, 6575, 6940,
        7305, 7671, 8036, 8401, 8766, 9132, 9497, 9862, 10227, 10593,
        10958, 11323, 11688, 12054, 12419, 12784, 13149, 13515, 13880, 14245,
        14610, 14976, 15341, 15706, 16071, 16437, 16802, 17167, 17532, 17898,
        18263, 18628, 18993, 19359, 19724, 20089, 20454, 20820, 21185, 21550,
        21915, 22281, 22646, 23011, 23376, 23742, 24107, 24472, 24837, 25203,
        25568, 25933, 26298, 26664, 27029, 27394, 27759, 28125, 28490, 28855};
const uint16_t SFE_UBLOX_DAYS_SINCE_MONTH[2][12] =
    {
        {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}, // Leap Year (Year % 4 == 0)
        {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334}  // Normal Year
};

const uint32_t SFE_UBLOX_JAN_1ST_2020_WEEK = 2086; // GPS Week Number for Jan 1st 2020
const uint32_t SFE_UBLOX_EPOCH_WEEK_2086 = 1577836800 - 259200; // Epoch for the start of GPS week 2086
const uint32_t SFE_UBLOX_SECS_PER_WEEK = 60 * 60 * 24 * 7; // Seconds per week
