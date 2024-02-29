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

#include <Arduino.h>
#include "u-blox_GNSS.h"

DevUBLOXGNSS::DevUBLOXGNSS(void)
{
  // Constructor
  if (debugPin >= 0)
  {
    pinMode((uint8_t)debugPin, OUTPUT);
    digitalWrite((uint8_t)debugPin, HIGH);
  }

  _logNMEA.all = 0;                             // Default to passing no NMEA messages to the file buffer
  _processNMEA.all = SFE_UBLOX_FILTER_NMEA_ALL; // Default to passing all NMEA messages to processNMEA
  _logRTCM.all = 0;                             // Default to passing no RTCM messages to the file buffer

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  rtcmInputStorage.init();
#endif
}

DevUBLOXGNSS::~DevUBLOXGNSS(void)
{
  // Destructor

  end(); // Delete all allocated memory - excluding payloadCfg, payloadAuto and spiBuffer

  if (payloadCfg != nullptr)
  {
    delete[] payloadCfg; // Created with new[]
    payloadCfg = nullptr;
  }

  if (payloadAuto != nullptr)
  {
    delete[] payloadAuto; // Created with new[]
    payloadAuto = nullptr;
  }

  if (spiBuffer != nullptr)
  {
    delete[] spiBuffer; // Created with new[]
    spiBuffer = nullptr;
  }
}

// Stop all automatic message processing. Free all used RAM
void DevUBLOXGNSS::end(void)
{
  // Note: payloadCfg is not deleted

  // Note: payloadAuto is not deleted

  // Note: spiBuffer is not deleted

  if (ubxFileBuffer != nullptr) // Check if RAM has been allocated for the file buffer
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("end: the file buffer has been deleted. You will need to call setFileBufferSize before .begin to create a new one."));
    }
#endif
    delete[] ubxFileBuffer; // Created with new[]
    ubxFileBuffer = nullptr;
    fileBufferSize = 0; // Reset file buffer size. User will have to call setFileBufferSize again
    fileBufferMaxAvail = 0;
  }

  if (rtcmBuffer != nullptr) // Check if RAM has been allocated for the RTCM buffer
  {
    delete[] rtcmBuffer; // Created with new[]
    rtcmBuffer = nullptr;
    rtcmBufferSize = 0; // Reset file buffer size. User will have to call setFileBufferSize again
  }

  if (cfgValgetValueSizes != nullptr)
  {
    delete[] cfgValgetValueSizes;
    cfgValgetValueSizes = nullptr;
  }

  if (moduleSWVersion != nullptr)
  {
    delete moduleSWVersion; // Created with new moduleSWVersion_t
    moduleSWVersion = nullptr;
  }

  if (currentGeofenceParams != nullptr)
  {
    delete currentGeofenceParams; // Created with new geofenceParams_t
    currentGeofenceParams = nullptr;
  }

  if (packetUBXNAVTIMELS != nullptr)
  {
    delete packetUBXNAVTIMELS; // Created with new UBX_NAV_TIMELS_t
    packetUBXNAVTIMELS = nullptr;
  }

  if (packetUBXNAVPOSECEF != nullptr)
  {
    if (packetUBXNAVPOSECEF->callbackData != nullptr)
    {
      delete packetUBXNAVPOSECEF->callbackData; // Created with new UBX_NAV_POSECEF_data_t
    }
    delete packetUBXNAVPOSECEF; // Created with new UBX_NAV_POSECEF_t
    packetUBXNAVPOSECEF = nullptr;
  }

  if (packetUBXNAVSTATUS != nullptr)
  {
    if (packetUBXNAVSTATUS->callbackData != nullptr)
    {
      delete packetUBXNAVSTATUS->callbackData;
    }
    delete packetUBXNAVSTATUS;
    packetUBXNAVSTATUS = nullptr;
  }

  if (packetUBXNAVDOP != nullptr)
  {
    if (packetUBXNAVDOP->callbackData != nullptr)
    {
      delete packetUBXNAVDOP->callbackData;
    }
    delete packetUBXNAVDOP;
    packetUBXNAVDOP = nullptr;
  }

  if (packetUBXNAVPVT != nullptr)
  {
    if (packetUBXNAVPVT->callbackData != nullptr)
    {
      delete packetUBXNAVPVT->callbackData;
    }
    delete packetUBXNAVPVT;
    packetUBXNAVPVT = nullptr;
  }

  if (packetUBXNAVATT != nullptr)
  {
    if (packetUBXNAVATT->callbackData != nullptr)
    {
      delete packetUBXNAVATT->callbackData;
    }
    delete packetUBXNAVATT;
    packetUBXNAVATT = nullptr;
  }

  if (packetUBXNAVODO != nullptr)
  {
    if (packetUBXNAVODO->callbackData != nullptr)
    {
      delete packetUBXNAVODO->callbackData;
    }
    delete packetUBXNAVODO;
    packetUBXNAVODO = nullptr;
  }

  if (packetUBXNAVVELECEF != nullptr)
  {
    if (packetUBXNAVVELECEF->callbackData != nullptr)
    {
      delete packetUBXNAVVELECEF->callbackData;
    }
    delete packetUBXNAVVELECEF;
    packetUBXNAVVELECEF = nullptr;
  }

  if (packetUBXNAVVELNED != nullptr)
  {
    if (packetUBXNAVVELNED->callbackData != nullptr)
    {
      delete packetUBXNAVVELNED->callbackData;
    }
    delete packetUBXNAVVELNED;
    packetUBXNAVVELNED = nullptr;
  }

  if (packetUBXNAVHPPOSECEF != nullptr)
  {
    if (packetUBXNAVHPPOSECEF->callbackData != nullptr)
    {
      delete packetUBXNAVHPPOSECEF->callbackData;
    }
    delete packetUBXNAVHPPOSECEF;
    packetUBXNAVHPPOSECEF = nullptr;
  }

  if (packetUBXNAVHPPOSLLH != nullptr)
  {
    if (packetUBXNAVHPPOSLLH->callbackData != nullptr)
    {
      delete packetUBXNAVHPPOSLLH->callbackData;
    }
    delete packetUBXNAVHPPOSLLH;
    packetUBXNAVHPPOSLLH = nullptr;
  }

  if (packetUBXNAVPVAT != nullptr)
  {
    if (packetUBXNAVPVAT->callbackData != nullptr)
    {
      delete packetUBXNAVPVAT->callbackData;
    }
    delete packetUBXNAVPVAT;
    packetUBXNAVPVAT = nullptr;
  }

  if (packetUBXNAVTIMEUTC != nullptr)
  {
    if (packetUBXNAVTIMEUTC->callbackData != nullptr)
    {
      delete packetUBXNAVTIMEUTC->callbackData;
    }
    delete packetUBXNAVTIMEUTC;
    packetUBXNAVTIMEUTC = nullptr;
  }

  if (packetUBXNAVCLOCK != nullptr)
  {
    if (packetUBXNAVCLOCK->callbackData != nullptr)
    {
      delete packetUBXNAVCLOCK->callbackData;
    }
    delete packetUBXNAVCLOCK;
    packetUBXNAVCLOCK = nullptr;
  }

  if (packetUBXNAVSVIN != nullptr)
  {
    if (packetUBXNAVSVIN->callbackData != nullptr)
    {
      delete packetUBXNAVSVIN->callbackData;
    }
    delete packetUBXNAVSVIN;
    packetUBXNAVSVIN = nullptr;
  }

  if (packetUBXNAVRELPOSNED != nullptr)
  {
    if (packetUBXNAVRELPOSNED->callbackData != nullptr)
    {
      delete packetUBXNAVRELPOSNED->callbackData;
    }
    delete packetUBXNAVRELPOSNED;
    packetUBXNAVRELPOSNED = nullptr;
  }

  if (packetUBXNAVAOPSTATUS != nullptr)
  {
    if (packetUBXNAVAOPSTATUS->callbackData != nullptr)
    {
      delete packetUBXNAVAOPSTATUS->callbackData;
    }
    delete packetUBXNAVAOPSTATUS;
    packetUBXNAVAOPSTATUS = nullptr;
  }

  if (packetUBXNAVEOE != nullptr)
  {
    if (packetUBXNAVEOE->callbackData != nullptr)
    {
      delete packetUBXNAVEOE->callbackData;
    }
    delete packetUBXNAVEOE;
    packetUBXNAVEOE = nullptr;
  }

#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  if (packetUBXNAVSAT != nullptr)
  {
    if (packetUBXNAVSAT->callbackData != nullptr)
    {
      delete packetUBXNAVSAT->callbackData;
    }
    delete packetUBXNAVSAT;
    packetUBXNAVSAT = nullptr;
  }

  if (packetUBXNAVSIG != nullptr)
  {
    if (packetUBXNAVSIG->callbackData != nullptr)
    {
      delete packetUBXNAVSIG->callbackData;
    }
    delete packetUBXNAVSIG;
    packetUBXNAVSIG = nullptr;
  }

  if (packetUBXRXMPMP != nullptr)
  {
    if (packetUBXRXMPMP->callbackData != nullptr)
    {
      delete packetUBXRXMPMP->callbackData;
    }
    delete packetUBXRXMPMP;
    packetUBXRXMPMP = nullptr;
  }

  if (packetUBXRXMPMPmessage != nullptr)
  {
    if (packetUBXRXMPMPmessage->callbackData != nullptr)
    {
      delete packetUBXRXMPMPmessage->callbackData;
    }
    delete packetUBXRXMPMPmessage;
    packetUBXRXMPMPmessage = nullptr;
  }

  if (packetUBXRXMQZSSL6message != nullptr)
  {
    if (packetUBXRXMQZSSL6message->callbackData != nullptr)
    {
      delete[] packetUBXRXMQZSSL6message->callbackData;
    }
    delete packetUBXRXMQZSSL6message;
    packetUBXRXMQZSSL6message = nullptr;
  }

  if (packetUBXRXMCOR != nullptr)
  {
    if (packetUBXRXMCOR->callbackData != nullptr)
    {
      delete packetUBXRXMCOR->callbackData;
    }
    delete packetUBXRXMCOR;
    packetUBXRXMCOR = nullptr;
  }

  if (packetUBXRXMSFRBX != nullptr)
  {
    if (packetUBXRXMSFRBX->callbackData != nullptr)
    {
      delete packetUBXRXMSFRBX->callbackData;
    }
    if (packetUBXRXMSFRBX->callbackMessageData != nullptr)
    {
      delete[] packetUBXRXMSFRBX->callbackMessageData;
    }
    delete packetUBXRXMSFRBX;
    packetUBXRXMSFRBX = nullptr;
  }

  if (packetUBXRXMRAWX != nullptr)
  {
    if (packetUBXRXMRAWX->callbackData != nullptr)
    {
      delete packetUBXRXMRAWX->callbackData;
    }
    delete packetUBXRXMRAWX;
    packetUBXRXMRAWX = nullptr;
  }

  if (packetUBXRXMMEASX != nullptr)
  {
    if (packetUBXRXMMEASX->callbackData != nullptr)
    {
      delete packetUBXRXMMEASX->callbackData;
    }
    delete packetUBXRXMMEASX;
    packetUBXRXMMEASX = nullptr;
  }
#endif

  if (packetUBXTIMTM2 != nullptr)
  {
    if (packetUBXTIMTM2->callbackData != nullptr)
    {
      delete packetUBXTIMTM2->callbackData;
    }
    delete packetUBXTIMTM2;
    packetUBXTIMTM2 = nullptr;
  }

  if (packetUBXTIMTP != nullptr)
  {
    if (packetUBXTIMTP->callbackData != nullptr)
    {
      delete packetUBXTIMTP->callbackData;
    }
    delete packetUBXTIMTP;
    packetUBXTIMTP = nullptr;
  }

  if (packetUBXMONHW != nullptr)
  {
    if (packetUBXMONHW->callbackData != nullptr)
    {
      delete packetUBXMONHW->callbackData;
    }
    delete packetUBXMONHW;
    packetUBXMONHW = nullptr;
  }

#ifndef SFE_UBLOX_DISABLE_ESF
  if (packetUBXESFALG != nullptr)
  {
    if (packetUBXESFALG->callbackData != nullptr)
    {
      delete packetUBXESFALG->callbackData;
    }
    delete packetUBXESFALG;
    packetUBXESFALG = nullptr;
  }

  if (packetUBXESFSTATUS != nullptr)
  {
    if (packetUBXESFSTATUS->callbackData != nullptr)
    {
      delete packetUBXESFSTATUS->callbackData;
    }
    delete packetUBXESFSTATUS;
    packetUBXESFSTATUS = nullptr;
  }

  if (packetUBXESFINS != nullptr)
  {
    if (packetUBXESFINS->callbackData != nullptr)
    {
      delete packetUBXESFINS->callbackData;
    }
    delete packetUBXESFINS;
    packetUBXESFINS = nullptr;
  }

  if (packetUBXESFMEAS != nullptr)
  {
    if (packetUBXESFMEAS->callbackData != nullptr)
    {
      delete[] packetUBXESFMEAS->callbackData;
    }
    delete packetUBXESFMEAS;
    packetUBXESFMEAS = nullptr;
  }

  if (packetUBXESFRAW != nullptr)
  {
    if (packetUBXESFRAW->callbackData != nullptr)
    {
      delete packetUBXESFRAW->callbackData;
    }
    delete packetUBXESFRAW;
    packetUBXESFRAW = nullptr;
  }
#endif

  if (packetUBXMGAACK != nullptr)
  {
    delete packetUBXMGAACK;
    packetUBXMGAACK = nullptr;
  }

  if (packetUBXMGADBD != nullptr)
  {
    delete packetUBXMGADBD;
    packetUBXMGADBD = nullptr;
  }

#ifndef SFE_UBLOX_DISABLE_HNR
  if (packetUBXHNRATT != nullptr)
  {
    if (packetUBXHNRATT->callbackData != nullptr)
    {
      delete packetUBXHNRATT->callbackData;
    }
    delete packetUBXHNRATT;
    packetUBXHNRATT = nullptr;
  }

  if (packetUBXHNRINS != nullptr)
  {
    if (packetUBXHNRINS->callbackData != nullptr)
    {
      delete packetUBXHNRINS->callbackData;
    }
    delete packetUBXHNRINS;
    packetUBXHNRINS = nullptr;
  }

  if (packetUBXHNRPVT != nullptr)
  {
    if (packetUBXHNRPVT->callbackData != nullptr)
    {
      delete packetUBXHNRPVT->callbackData;
    }
    delete packetUBXHNRPVT;
    packetUBXHNRPVT = nullptr;
  }
#endif

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  if (storageNMEAGPGGA != nullptr)
  {
    if (storageNMEAGPGGA->callbackCopy != nullptr)
    {
      delete storageNMEAGPGGA->callbackCopy;
    }
    delete storageNMEAGPGGA;
    storageNMEAGPGGA = nullptr;
  }

  if (storageNMEAGNGGA != nullptr)
  {
    if (storageNMEAGNGGA->callbackCopy != nullptr)
    {
      delete storageNMEAGNGGA->callbackCopy;
    }
    delete storageNMEAGNGGA;
    storageNMEAGNGGA = nullptr;
  }

  if (storageNMEAGPVTG != nullptr)
  {
    if (storageNMEAGPVTG->callbackCopy != nullptr)
    {
      delete storageNMEAGPVTG->callbackCopy;
    }
    delete storageNMEAGPVTG;
    storageNMEAGPVTG = nullptr;
  }

  if (storageNMEAGNVTG != nullptr)
  {
    if (storageNMEAGNVTG->callbackCopy != nullptr)
    {
      delete storageNMEAGNVTG->callbackCopy;
    }
    delete storageNMEAGNVTG;
    storageNMEAGNVTG = nullptr;
  }

  if (storageNMEAGPRMC != nullptr)
  {
    if (storageNMEAGPRMC->callbackCopy != nullptr)
    {
      delete storageNMEAGPRMC->callbackCopy;
    }
    delete storageNMEAGPRMC;
    storageNMEAGPRMC = nullptr;
  }

  if (storageNMEAGNRMC != nullptr)
  {
    if (storageNMEAGNRMC->callbackCopy != nullptr)
    {
      delete storageNMEAGNRMC->callbackCopy;
    }
    delete storageNMEAGNRMC;
    storageNMEAGNRMC = nullptr;
  }

  if (storageNMEAGPZDA != nullptr)
  {
    if (storageNMEAGPZDA->callbackCopy != nullptr)
    {
      delete storageNMEAGPZDA->callbackCopy;
    }
    delete storageNMEAGPZDA;
    storageNMEAGPZDA = nullptr;
  }

  if (storageNMEAGNZDA != nullptr)
  {
    if (storageNMEAGNZDA->callbackCopy != nullptr)
    {
      delete storageNMEAGNZDA->callbackCopy;
    }
    delete storageNMEAGNZDA;
    storageNMEAGNZDA = nullptr;
  }
#endif

  if (_storageNMEA != nullptr)
  {
    if (_storageNMEA->data != nullptr)
    {
      delete[] _storageNMEA->data;
    }
    delete _storageNMEA;
    _storageNMEA = nullptr;
  }

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  if (_storageRTCM != nullptr)
  {
    delete _storageRTCM;
    _storageRTCM = nullptr;
  }
#endif

  if (storageRTCM1005 != nullptr)
  {
    if (storageRTCM1005->callbackData != nullptr)
    {
      delete storageRTCM1005->callbackData;
    }
    delete storageRTCM1005;
    storageRTCM1005 = nullptr;
  }

  if (sfe_ublox_ubx_logging_list_head != nullptr)
  {
    while (sfe_ublox_ubx_logging_list_head->next != nullptr)
    {
      // Step through the list, find the tail
      sfe_ublox_ubx_logging_list_t *sfe_ublox_ubx_logging_list_ptr_previous = sfe_ublox_ubx_logging_list_head;
      sfe_ublox_ubx_logging_list_t *sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_head->next;
      while (sfe_ublox_ubx_logging_list_ptr->next != nullptr)
      {
        sfe_ublox_ubx_logging_list_ptr_previous = sfe_ublox_ubx_logging_list_ptr;
        sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_ptr->next;
      }
      // Delete the tail
      delete sfe_ublox_ubx_logging_list_ptr;
      sfe_ublox_ubx_logging_list_ptr_previous->next = nullptr;
    }
    // Finally, delete the head
    delete sfe_ublox_ubx_logging_list_head;
    sfe_ublox_ubx_logging_list_head = nullptr;
  }

  deleteLock(); // Delete the lock semaphore - if required
}

// Allow the user to change packetCfgPayloadSize. Handy if you want to process big messages like RAWX
// This can be called before .begin if required / desired
bool DevUBLOXGNSS::setPacketCfgPayloadSize(size_t payloadSize)
{
  bool success = true;

  if ((payloadSize == 0) && (payloadCfg != nullptr))
  {
    // Zero payloadSize? Dangerous! But we'll free the memory anyway...
    delete[] payloadCfg; // Created with new[]
    payloadCfg = nullptr;
    packetCfg.payload = payloadCfg;
    packetCfgPayloadSize = payloadSize;
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setPacketCfgPayloadSize: Zero payloadSize!"));
  }

  else if (payloadCfg == nullptr) // Memory has not yet been allocated - so use new
  {
    payloadCfg = new uint8_t[payloadSize];
    packetCfg.payload = payloadCfg;

    if (payloadCfg == nullptr)
    {
      success = false;
      packetCfgPayloadSize = 0;
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial.println(F("setPacketCfgPayloadSize: RAM alloc failed!"));
    }
    else
      packetCfgPayloadSize = payloadSize;

    if ((packetCfgPayloadSize + 8) > spiBufferSize) // Warn the user if spiBuffer is now smaller than the packetCfg payload. Could result in lost data
    {
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial.println(F("setPacketCfgPayloadSize: packetCfgPayloadSize > spiBufferSize!"));
    }
  }

  else // Memory has already been allocated - so resize
  {
    uint8_t *newPayload = new uint8_t[payloadSize];

    if (newPayload == nullptr) // Check if the alloc was successful
    {
      success = false;                                           // Report failure. Don't change payloadCfg, packetCfg.payload or packetCfgPayloadSize
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial.println(F("setPacketCfgPayloadSize: RAM resize failed!"));
    }
    else
    {
      memcpy(newPayload, payloadCfg, payloadSize <= packetCfgPayloadSize ? payloadSize : packetCfgPayloadSize); // Copy as much existing data as we can
      delete[] payloadCfg;                                                                                      // Free payloadCfg. Created with new[]
      payloadCfg = newPayload;                                                                                  // Point to the newPayload
      packetCfg.payload = payloadCfg;                                                                           // Update the packet pointer
      packetCfgPayloadSize = payloadSize;                                                                       // Update the packet payload size
    }

    if ((packetCfgPayloadSize + 8) > spiBufferSize) // Warn the user if spiBuffer is now smaller than the packetCfg payload. Could result in lost data
    {
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial.println(F("setPacketCfgPayloadSize: packetCfgPayloadSize > spiBufferSize!"));
    }
  }

  return (success);
}

// Return the number of free bytes remaining in packetCfgPayload
size_t DevUBLOXGNSS::getPacketCfgSpaceRemaining()
{
  return (packetCfgPayloadSize - packetCfg.len);
}

// New in v3.0: hardware interface is abstracted
void DevUBLOXGNSS::setCommunicationBus(SparkFun_UBLOX_GNSS::GNSSDeviceBus &theBus)
{
  _sfeBus = &theBus;
}
// For Serial, return Serial.available()
// For I2C, read registers 0xFD and 0xFE. Return bytes available as uint16_t
// Not Applicable for SPI
uint16_t DevUBLOXGNSS::available()
{
  return _sfeBus->available();
}
// For I2C, ping the _address
// Not Applicable for SPI and Serial
bool DevUBLOXGNSS::ping()
{
  if (!lock())
    return false;
  bool ok = _sfeBus->ping();
  unlock();
  return ok;
}
// For Serial, do Serial.write
// For I2C, push data to register 0xFF. Chunkify if necessary. Prevent single byte writes as these are illegal
// For SPI, writing bytes will also read bytes simultaneously. Read data is _ignored_ here. Use writeReadBytes
uint8_t DevUBLOXGNSS::writeBytes(uint8_t *data, uint8_t length)
{
  return _sfeBus->writeBytes(data, length);
}
// For SPI, writing bytes will also read bytes simultaneously. Read data is returned in readData
uint8_t DevUBLOXGNSS::writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length)
{
  return _sfeBus->writeReadBytes(data, readData, length);
}
void DevUBLOXGNSS::startWriteReadByte()
{
  _sfeBus->startWriteReadByte();
}
void DevUBLOXGNSS::writeReadByte(const uint8_t *data, uint8_t *readData)
{
  _sfeBus->writeReadByte(data, readData);
}
void DevUBLOXGNSS::writeReadByte(const uint8_t data, uint8_t *readData)
{
  _sfeBus->writeReadByte(data, readData);
}
void DevUBLOXGNSS::endWriteReadByte()
{
  _sfeBus->endWriteReadByte();
}
// For Serial, attempt Serial.read
// For I2C, read from register 0xFF
// For SPI, read the byte while writing 0xFF
uint8_t DevUBLOXGNSS::readBytes(uint8_t *data, uint8_t length)
{
  return _sfeBus->readBytes(data, length);
}

bool DevUBLOXGNSS::init(uint16_t maxWait, bool assumeSuccess)
{
  createLock(); // Create the lock semaphore - if needed

  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  // New in v2.0: allocate memory for the file buffer - if required. (The user should have called setFileBufferSize already)
  createFileBuffer();

  // Create storage for RTCM data - only useful on systems where the GNSS is interfaced via SPI and you may want to write
  // data to another SPI device (e.g. Ethernet) in a safe way, avoiding processRTCM (which would be called _during_ checkUblox).
  createRTCMBuffer();

  if (_commType == COMM_TYPE_SPI)
  {
    // Create the SPI buffer
    if (spiBuffer == nullptr) // Memory has not yet been allocated - so use new
    {
      spiBuffer = new uint8_t[spiBufferSize];
    }

    if (spiBuffer == nullptr)
    {
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.println(F("begin (SPI): memory allocation failed for SPI Buffer!"));
        return (false);
      }
    }
    else
    {
      // Initialize/clear the SPI buffer - fill it with 0xFF as this is what is received from the UBLOX module if there's no data to be processed
      for (size_t i = 0; i < spiBufferSize; i++)
      {
        spiBuffer[i] = 0xFF;
      }
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.print(F("begin (SPI): spiBuffer size is "));
        _debugSerial.println(spiBufferSize);
      }
      if ((packetCfgPayloadSize + 8) > spiBufferSize) // Warn the user if spiBuffer is now smaller than the packetCfg payload. Could result in lost data
      {
        _debugSerial.println(F("begin (SPI): packetCfgPayloadSize > spiBufferSize!"));
      }
#endif
    }
  }

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("begin: isConnected - second attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("begin: isConnected - third attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("begin: third attempt failed. Assuming success..."));
    }
#endif
    return (true);
  }

  return (connected);
}

// Allow the user to change I2C polling wait (the minimum interval between I2C data requests - to avoid pounding the bus)
// i2cPollingWait defaults to 100ms and is adjusted automatically when setNavigationFrequency()
// or setHNRNavigationRate() are called. But if the user is using callbacks, it might be advantageous
// to be able to set the polling wait manually.
void DevUBLOXGNSS::setI2CpollingWait(uint8_t newPollingWait_ms)
{
  i2cPollingWait = newPollingWait_ms;
}

// Allow the user to change SPI polling wait
// (the minimum interval between SPI data requests when no data is available - to avoid pounding the bus)
void DevUBLOXGNSS::setSPIpollingWait(uint8_t newPollingWait_ms)
{
  spiPollingWait = newPollingWait_ms;
}

// Sets the global size for I2C transactions
// Most platforms use 32 bytes (the default) but this allows users to increase the transaction
// size if the platform supports it
// Note: If the transaction size is set larger than the platforms buffer size, bad things will happen.
void DevUBLOXGNSS::setI2CTransactionSize(uint8_t transactionSize)
{
  if (transactionSize < 8)
    transactionSize = 8; // Ensure transactionSize is at least 8 bytes otherwise sendI2cCommand will have problems!

  i2cTransactionSize = transactionSize;
}
uint8_t DevUBLOXGNSS::getI2CTransactionSize(void)
{
  return (i2cTransactionSize);
}

// Sets the global size for SPI transactions.
// Call this **before** begin()!
void DevUBLOXGNSS::setSpiTransactionSize(uint8_t transactionSize)
{
  if (spiBuffer == nullptr)
  {
    if (transactionSize < spiTransactionSize)
      transactionSize = spiTransactionSize;
    spiTransactionSize = transactionSize;

    if (spiBufferSize < spiTransactionSize) // Ensure the buffer is at least spiTransactionSize
      spiBufferSize = spiTransactionSize;
  }
  else
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("setSpiTransactionSize: you need to call setSpiTransactionSize _before_ begin!"));
    }
#endif
  }
}
uint8_t DevUBLOXGNSS::getSpiTransactionSize(void)
{
  return (spiTransactionSize);
}

// Sets the global size for the SPI buffer.
// Call this **before** begin()!
// Note: if the buffer size is too small, incoming characters may be lost if the message sent
// is larger than this buffer. If too big, you may run out of SRAM on constrained architectures!
void DevUBLOXGNSS::setSpiBufferSize(size_t bufferSize)
{
  if (spiBuffer == nullptr)
  {
    if (bufferSize < spiTransactionSize) // Ensure the buffer is at least spiTransactionSize
      bufferSize = spiTransactionSize;
    spiBufferSize = bufferSize;
  }
  else
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("setSpiBufferSize: you need to call setSpiBufferSize _before_ begin!"));
    }
#endif
  }
}
size_t DevUBLOXGNSS::getSpiBufferSize(void)
{
  return (spiBufferSize);
}

// Sets the size of maxNMEAByteCount
// Call this before .begin to avoid badness with _storageNMEA
void DevUBLOXGNSS::setMaxNMEAByteCount(int8_t newMax)
{
  maxNMEAByteCount = newMax;
}
int8_t DevUBLOXGNSS::getMaxNMEAByteCount(void)
{
  return (maxNMEAByteCount);
}

// Returns true if I2C device ack's
bool DevUBLOXGNSS::isConnected(uint16_t maxWait)
{
  if (_commType == COMM_TYPE_I2C)
  {
    if (!ping())
      return false; // Sensor did not ack
  }

  // Query port configuration to see whether we get a meaningful response
  // We could simply request the config for any port but, just for giggles, let's request the config for most appropriate port
  uint8_t en;
  if (_commType == COMM_TYPE_I2C)
    return getVal8(UBLOX_CFG_I2CINPROT_UBX, &en, VAL_LAYER_RAM, maxWait);
  else if ((_commType == COMM_TYPE_SERIAL) && (!_UART2))
    return getVal8(UBLOX_CFG_UART1INPROT_UBX, &en, VAL_LAYER_RAM, maxWait);
  else if ((_commType == COMM_TYPE_SERIAL) && (_UART2))
    return getVal8(UBLOX_CFG_UART2INPROT_UBX, &en, VAL_LAYER_RAM, maxWait);
  else if (_commType == COMM_TYPE_SPI)
    return getVal8(UBLOX_CFG_SPIINPROT_UBX, &en, VAL_LAYER_RAM, maxWait);
  else
    return false;
}

// Enable or disable the printing of sent/response HEX values.
// Use this in conjunction with 'Transport Logging' from the Universal Reader Assistant to see what they're doing that we're not
void DevUBLOXGNSS::enableDebugging(Print &debugPort, bool printLimitedDebug)
{
  _debugSerial.init(debugPort); // Grab which port the user wants us to use for debugging
  if (printLimitedDebug == false)
  {
    _printDebug = true; // Should we print the commands we send? Good for debugging
  }
  else
  {
    _printLimitedDebug = true; // Should we print limited debug messages? Good for debugging high navigation rates
  }
}
void DevUBLOXGNSS::disableDebugging(void)
{
  _printDebug = false; // Turn off extra print statements
  _printLimitedDebug = false;
}

// Safely print messages
void DevUBLOXGNSS::debugPrint(char *message)
{
  if (_printDebug == true)
  {
    _debugSerial.print(message);
  }
}
// Safely print messages
void DevUBLOXGNSS::debugPrintln(char *message)
{
  if (_printDebug == true)
  {
    _debugSerial.println(message);
  }
}

const char *DevUBLOXGNSS::statusString(sfe_ublox_status_e stat)
{
  switch (stat)
  {
  case SFE_UBLOX_STATUS_SUCCESS:
    return "Success";
    break;
  case SFE_UBLOX_STATUS_FAIL:
    return "General Failure";
    break;
  case SFE_UBLOX_STATUS_CRC_FAIL:
    return "CRC Fail";
    break;
  case SFE_UBLOX_STATUS_TIMEOUT:
    return "Timeout";
    break;
  case SFE_UBLOX_STATUS_COMMAND_NACK:
    return "Command not acknowledged (NACK)";
    break;
  case SFE_UBLOX_STATUS_OUT_OF_RANGE:
    return "Out of range";
    break;
  case SFE_UBLOX_STATUS_INVALID_ARG:
    return "Invalid Arg";
    break;
  case SFE_UBLOX_STATUS_INVALID_OPERATION:
    return "Invalid operation";
    break;
  case SFE_UBLOX_STATUS_MEM_ERR:
    return "Memory Error";
    break;
  case SFE_UBLOX_STATUS_HW_ERR:
    return "Hardware Error";
    break;
  case SFE_UBLOX_STATUS_DATA_SENT:
    return "Data Sent";
    break;
  case SFE_UBLOX_STATUS_DATA_RECEIVED:
    return "Data Received";
    break;
  case SFE_UBLOX_STATUS_I2C_COMM_FAILURE:
    return "I2C Comm Failure";
    break;
  case SFE_UBLOX_STATUS_DATA_OVERWRITTEN:
    return "Data Packet Overwritten";
    break;
  default:
    return "Unknown Status";
    break;
  }
  return "None";
}

// Check for the arrival of new I2C/Serial/SPI data
// Called regularly to check for available bytes on the user' specified port

bool DevUBLOXGNSS::checkUblox(uint8_t requestedClass, uint8_t requestedID)
{
  return checkUbloxInternal(&packetCfg, requestedClass, requestedID);
}

// PRIVATE: Called regularly to check for available bytes on the user' specified port
bool DevUBLOXGNSS::checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (!lock())
    return false;

  bool ok = false;
  if (_commType == COMM_TYPE_I2C)
    ok = (checkUbloxI2C(incomingUBX, requestedClass, requestedID));
  else if (_commType == COMM_TYPE_SERIAL)
    ok = (checkUbloxSerial(incomingUBX, requestedClass, requestedID));
  else if (_commType == COMM_TYPE_SPI)
    ok = (checkUbloxSpi(incomingUBX, requestedClass, requestedID));

  unlock();

  return ok;
}

// Polls I2C for data, passing any new bytes to process()
// Returns true if new bytes are available
bool DevUBLOXGNSS::checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (millis() - lastCheck >= i2cPollingWait)
  {
    // Get the number of bytes available from the module
    // From the u-blox integration manual:
    // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
    //  address and thus allows any register to be read. The second "current address" form omits the
    //  register address. If this second form is used, then an address pointer in the receiver is used to
    //  determine which register to read. This address pointer will increment after each read unless it
    //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
    //  unaltered."
    uint16_t bytesAvailable = available();

    if (bytesAvailable == 0)
    {
      // #ifndef SFE_UBLOX_REDUCED_PROG_MEM
      //       if (_printDebug == true)
      //       {
      //         _debugSerial.println(F("checkUbloxI2C: OK, zero bytes available"));
      //       }
      // #endif
      lastCheck = millis(); // Put off checking to avoid I2C bus traffic
      return (false);
    }

    // Check for undocumented bit error. We found this doing logic scans.
    // This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
    // then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
    if (bytesAvailable & ((uint16_t)1 << 15))
    {
      // Clear the MSbit
      bytesAvailable &= ~((uint16_t)1 << 15);
    }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.print(F("checkUbloxI2C: "));
      _debugSerial.print(bytesAvailable);
      _debugSerial.println(F(" bytes available"));
    }
#endif

    while (bytesAvailable)
    {
      // Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable; // 16-bit
      if (bytesToRead > i2cTransactionSize)  // Limit for i2cTransactionSize is 8-bit
        bytesToRead = i2cTransactionSize;

      // Here it would be desireable to use a restart where possible / supported, but only if there will be multiple reads.
      // However, if an individual requestFrom fails, we could end up leaving the bus hanging.
      // On balance, it is probably safest to not use restarts here.
      uint8_t buf[i2cTransactionSize];
      uint8_t bytesReturned = readBytes(buf, (uint8_t)bytesToRead);
      if ((uint16_t)bytesReturned == bytesToRead)
      {
        for (uint16_t x = 0; x < bytesToRead; x++)
        {
          process(buf[x], incomingUBX, requestedClass, requestedID); // Process this valid character
        }
      }
      else
      {
        // Something has gone very wrong. Sensor did not respond - or a bus error happened...
        if (_resetCurrentSentenceOnBusError)
          currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being looking for a new start char
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial.println(F("checkUbloxI2C: bus error? bytesReturned != bytesToRead"));
        }
#endif
        return (false);
      }

      bytesAvailable -= bytesToRead;
    }
  }

  return (true);

} // end checkUbloxI2C()

// Checks Serial for data, passing any new bytes to process()
bool DevUBLOXGNSS::checkUbloxSerial(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (available())
  {
    while (available())
    {
      uint8_t chr;
      readBytes(&chr, 1);
      process(chr, incomingUBX, requestedClass, requestedID);
    }
    return (true);
  }
  return (false);
} // end checkUbloxSerial()

bool DevUBLOXGNSS::processSpiBuffer(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  bool retVal = false;
  // Process the contents of the SPI buffer if not empty!
  if (spiBuffer != nullptr)
  {
    if (spiBufferIndex > 0)
    {
      retVal = true;
      for (size_t i = 0; i < spiBufferIndex; i++)
      {
        process(spiBuffer[i], incomingUBX, requestedClass, requestedID);
      }
      spiBufferIndex = 0;
    }
  }

  return retVal;
}

// Checks SPI for data, passing any new bytes to process()
bool DevUBLOXGNSS::checkUbloxSpi(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  bool retVal = processSpiBuffer(incomingUBX, requestedClass, requestedID);

  // SPI has no available(). We need to keep reading bytes one at a time and stop when we hit 0xFF
  startWriteReadByte();

  uint8_t byteReturned = 0xFF;
  writeReadByte(0xFF, &byteReturned);

  // Note to future self: I think the 0xFF check might cause problems when attempting to process (e.g.) RAWX data
  // which could legitimately contain 0xFF within the data stream. But the currentSentence check will certainly help!

  // If we are not receiving a sentence (currentSentence == NONE) and the byteReturned is 0xFF,
  // i.e. the module has no data for us, then delay and return
  if ((byteReturned == 0xFF) && (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NONE))
  {
    endWriteReadByte();
    delay(spiPollingWait);
    return (retVal);
  }

  while ((byteReturned != 0xFF) || (currentSentence != SFE_UBLOX_SENTENCE_TYPE_NONE))
  {
    process(byteReturned, incomingUBX, requestedClass, requestedID);
    writeReadByte(0xFF, &byteReturned);
  }
  endWriteReadByte();
  return (true);

} // end checkUbloxSpi()

// PRIVATE: Check if we have storage allocated for an incoming "automatic" message
// Also calculate how much RAM is needed to store the payload for a given automatic message
bool DevUBLOXGNSS::autoLookup(uint8_t Class, uint8_t ID, uint16_t *maxSize)
{
  if (maxSize != nullptr)
    *maxSize = 0;

  switch (Class)
  {
  case UBX_CLASS_NAV:
    if (ID == UBX_NAV_POSECEF)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_POSECEF_LEN;
      return (packetUBXNAVPOSECEF != nullptr);
    }
    else if (ID == UBX_NAV_STATUS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_STATUS_LEN;
      return (packetUBXNAVSTATUS != nullptr);
    }
    else if (ID == UBX_NAV_DOP)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_DOP_LEN;
      return (packetUBXNAVDOP != nullptr);
    }
    else if (ID == UBX_NAV_ATT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_ATT_LEN;
      return (packetUBXNAVATT != nullptr);
    }
    else if (ID == UBX_NAV_PVT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_PVT_LEN;
      return (packetUBXNAVPVT != nullptr);
    }
    else if (ID == UBX_NAV_ODO)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_ODO_LEN;
      return (packetUBXNAVODO != nullptr);
    }
    else if (ID == UBX_NAV_VELECEF)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_VELECEF_LEN;
      return (packetUBXNAVVELECEF != nullptr);
    }
    else if (ID == UBX_NAV_VELNED)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_VELNED_LEN;
      return (packetUBXNAVVELNED != nullptr);
    }
    else if (ID == UBX_NAV_HPPOSECEF)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_HPPOSECEF_LEN;
      return (packetUBXNAVHPPOSECEF != nullptr);
    }
    else if (ID == UBX_NAV_HPPOSLLH)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_HPPOSLLH_LEN;
      return (packetUBXNAVHPPOSLLH != nullptr);
    }
    else if (ID == UBX_NAV_PVAT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_PVAT_LEN;
      return (packetUBXNAVPVAT != nullptr);
    }
    else if (ID == UBX_NAV_TIMEUTC)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_TIMEUTC_LEN;
      return (packetUBXNAVTIMEUTC != nullptr);
    }
    else if (ID == UBX_NAV_CLOCK)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_CLOCK_LEN;
      return (packetUBXNAVCLOCK != nullptr);
    }
    else if (ID == UBX_NAV_TIMELS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_TIMELS_LEN;
      return (packetUBXNAVTIMELS != nullptr);
    }
    else if (ID == UBX_NAV_SVIN)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_SVIN_LEN;
      return (packetUBXNAVSVIN != nullptr);
    }
    else if (ID == UBX_NAV_RELPOSNED)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_RELPOSNED_LEN_F9;
      return (packetUBXNAVRELPOSNED != nullptr);
    }
    else if (ID == UBX_NAV_AOPSTATUS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_AOPSTATUS_LEN;
      return (packetUBXNAVAOPSTATUS != nullptr);
    }
    else if (ID == UBX_NAV_EOE)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_EOE_LEN;
      return (packetUBXNAVEOE != nullptr);
    }
#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
    else if (ID == UBX_NAV_SAT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_SAT_MAX_LEN;
      return (packetUBXNAVSAT != nullptr);
    }
    else if (ID == UBX_NAV_SIG)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_SIG_MAX_LEN;
      return (packetUBXNAVSIG != nullptr);
    }
#endif
    break;
  case UBX_CLASS_RXM:
#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
    if (ID == UBX_RXM_SFRBX)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_SFRBX_MAX_LEN;
      return (packetUBXRXMSFRBX != nullptr);
    }
    else if (ID == UBX_RXM_RAWX)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_RAWX_MAX_LEN;
      return (packetUBXRXMRAWX != nullptr);
    }
    else if (ID == UBX_RXM_QZSSL6)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_QZSSL6_MAX_LEN;
      return (packetUBXRXMQZSSL6message != nullptr);
    }
    else if (ID == UBX_RXM_COR)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_COR_LEN;
      return (packetUBXRXMCOR != nullptr);
    }
    else if (ID == UBX_RXM_MEASX)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_MEASX_MAX_LEN;
      return (packetUBXRXMMEASX != nullptr);
    }
    else if (ID == UBX_RXM_PMP)
    {
      // PMP is a special case as it has both struct and message packages
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_PMP_MAX_LEN;
      return ((packetUBXRXMPMP != nullptr) || (packetUBXRXMPMPmessage != nullptr));
    }
#endif
    break;
  case UBX_CLASS_TIM:
    if (ID == UBX_TIM_TM2)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_TIM_TM2_LEN;
      return (packetUBXTIMTM2 != nullptr);
    }
    else if (ID == UBX_TIM_TP)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_TIM_TP_LEN;
      return (packetUBXTIMTP != nullptr);
    }
    break;
  case UBX_CLASS_MON:
    if (ID == UBX_MON_HW)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_MON_HW_LEN;
      return (packetUBXMONHW != nullptr);
    }
    break;
  case UBX_CLASS_ESF:
#ifndef SFE_UBLOX_DISABLE_ESF
    if (ID == UBX_ESF_ALG)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_ALG_LEN;
      return (packetUBXESFALG != nullptr);
    }
    else if (ID == UBX_ESF_INS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_INS_LEN;
      return (packetUBXESFINS != nullptr);
    }
    else if (ID == UBX_ESF_MEAS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_MEAS_MAX_LEN;
      return (packetUBXESFMEAS != nullptr);
    }
    else if (ID == UBX_ESF_RAW)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_RAW_MAX_LEN;
      return (packetUBXESFRAW != nullptr);
    }
    else if (ID == UBX_ESF_STATUS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_STATUS_MAX_LEN;
      return (packetUBXESFSTATUS != nullptr);
    }
#endif
    break;
  case UBX_CLASS_MGA:
    if (ID == UBX_MGA_ACK_DATA0)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_MGA_ACK_DATA0_LEN;
      return (packetUBXMGAACK != nullptr);
    }
    else if (ID == UBX_MGA_DBD)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_MGA_DBD_LEN;
      return (packetUBXMGADBD != nullptr);
    }
    break;
  case UBX_CLASS_HNR:
#ifndef SFE_UBLOX_DISABLE_HNR
    if (ID == UBX_HNR_PVT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_HNR_PVT_LEN;
      return (packetUBXHNRPVT != nullptr);
    }
    else if (ID == UBX_HNR_ATT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_HNR_ATT_LEN;
      return (packetUBXHNRATT != nullptr);
    }
    else if (ID == UBX_HNR_INS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_HNR_INS_LEN;
      return (packetUBXHNRINS != nullptr);
    }
#endif
    break;
  default:
    return false;
    break;
  }
  return false;
}

// Processes NMEA, RTCM and UBX binary sentences one byte at a time
// Take a given byte and file it into the proper array
void DevUBLOXGNSS::process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  // Update storedClass and storedID if either requestedClass or requestedID is non-zero,
  // otherwise leave unchanged. This allows calls of checkUblox() (which defaults to checkUblox(0,0))
  // by other threads without overwriting the requested / expected Class and ID.
  volatile static uint8_t storedClass = 0;
  volatile static uint8_t storedID = 0;
  if (requestedClass || requestedID) // If either is non-zero, store the requested Class and ID
  {
    storedClass = requestedClass;
    storedID = requestedID;
  }

  _outputPort.write(incoming); // Echo this byte to the serial port

  if ((currentSentence == SFE_UBLOX_SENTENCE_TYPE_NONE) || (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA))
  {
    if (incoming == UBX_SYNCH_1) // UBX binary frames start with 0xB5, aka μ
    {
      // This is the start of a binary sentence. Reset flags.
      // We still don't know the response class
      ubxFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_UBX;
      // Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
      packetBuf.counter = 0;
      ignoreThisPayload = false; // We should not ignore this payload - yet
      // Store data in packetBuf until we know if we have a stored class and ID match
      activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
    }
    else if (incoming == '$')
    {
      nmeaByteCounter = 0; // Reset the NMEA byte counter
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NMEA;
    }
    else if (incoming == 0xD3) // RTCM frames start with 0xD3
    {
      rtcmFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_RTCM;
    }
    else
    {
      // This character is unknown or we missed the previous start of a sentence
      // Or it could be a 0xFF from a SPI transaction
    }
  }

  uint16_t maxPayload = 0;

  // Depending on the sentence, pass the character to the individual processor
  if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_UBX)
  {
    // Decide what type of response this is
    if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1))      // ISO 'μ'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2)) // ASCII 'b'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    // Note to future self:
    // There may be some duplication / redundancy in the next few lines as processUBX will also
    // load information into packetBuf, but we'll do it here too for clarity
    else if (ubxFrameCounter == 2) // Class
    {
      // Record the class in packetBuf until we know what to do with it
      packetBuf.cls = incoming; // (Duplication)
      rollingChecksumA = 0;     // Reset our rolling checksums here (not when we receive the 0xB5)
      rollingChecksumB = 0;
      packetBuf.counter = 0;                                   // Reset the packetBuf.counter (again)
      packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
      packetBuf.startingSpot = incomingUBX->startingSpot;      // Copy the startingSpot
    }
    else if (ubxFrameCounter == 3) // ID
    {
      // Record the ID in packetBuf until we know what to do with it
      packetBuf.id = incoming; // (Duplication)
      // We can now identify the type of response
      // If the packet we are receiving is not an ACK then check for a class and ID match
      if (packetBuf.cls != UBX_CLASS_ACK)
      {
        bool logBecauseAuto = autoLookup(packetBuf.cls, packetBuf.id, &maxPayload);
        bool logBecauseEnabled = logThisUBX(packetBuf.cls, packetBuf.id);

        // This is not an ACK so check for a class and ID match
        if ((packetBuf.cls == storedClass) && (packetBuf.id == storedID))
        {
          // This is not an ACK and we have a class and ID match
          // So start diverting data into incomingUBX (usually packetCfg)
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
        }
        // This is not an ACK and we do not have a complete class and ID match
        // So let's check if this is an "automatic" message which has its own storage defined
        else if (logBecauseAuto || logBecauseEnabled)
        {
          // This is not the message we were expecting but it has its own storage and so we should process it anyway.
          // We'll try to use packetAuto to buffer the message (so it can't overwrite anything in packetCfg).
          // We need to allocate memory for the packetAuto payload (payloadAuto) - and delete it once
          // reception is complete.
          if (logBecauseAuto && (maxPayload == 0))
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial.print(F("process: autoLookup returned ZERO maxPayload!! Class: 0x"));
              _debugSerial.print(packetBuf.cls, HEX);
              _debugSerial.print(F(" ID: 0x"));
              _debugSerial.println(packetBuf.id, HEX);
            }
#endif
          }
          if (payloadAuto != nullptr) // Check if memory is already allocated - this should be impossible!
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial.println(F("process: memory is already allocated for payloadAuto! Deleting..."));
            }
#endif
            delete[] payloadAuto; // Created with new[]
            payloadAuto = nullptr;
            packetAuto.payload = payloadAuto;
          }
          if ((!logBecauseAuto) && (logBecauseEnabled))
            maxPayload = SFE_UBX_MAX_LENGTH;
          payloadAuto = new uint8_t[maxPayload]; // Allocate RAM for payloadAuto
          packetAuto.payload = payloadAuto;
          if (payloadAuto == nullptr) // Check if the alloc failed
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial.print(F("process: memory allocation failed for \"automatic\" message: Class: 0x"));
              _debugSerial.print(packetBuf.cls, HEX);
              _debugSerial.print(F(" ID: 0x"));
              _debugSerial.println(packetBuf.id, HEX);
              _debugSerial.println(F("process: \"automatic\" message could overwrite data"));
            }
#endif
            // The RAM allocation failed so fall back to using incomingUBX (usually packetCfg) even though we risk overwriting data
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
            incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
            incomingUBX->id = packetBuf.id;
            incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
          }
          else
          {
            // The RAM allocation was successful so we start diverting data into packetAuto and process it
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETAUTO;
            packetAuto.cls = packetBuf.cls; // Copy the class and ID into packetAuto
            packetAuto.id = packetBuf.id;
            packetAuto.counter = packetBuf.counter;           // Copy over the .counter too
            packetAuto.startingSpot = packetBuf.startingSpot; // And the starting spot? (Probably redundant)
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if (_printDebug == true)
            {
              _debugSerial.print(F("process: incoming \"automatic\" message: Class: 0x"));
              _debugSerial.print(packetBuf.cls, HEX);
              _debugSerial.print(F(" ID: 0x"));
              _debugSerial.print(packetBuf.id, HEX);
              _debugSerial.print(F(" logBecauseAuto:"));
              _debugSerial.print(logBecauseAuto);
              _debugSerial.print(F(" logBecauseEnabled:"));
              _debugSerial.println(logBecauseEnabled);
            }
#endif
          }
        }
        else
        {
          // This is not an ACK and we do not have a class and ID match
          // so we should keep diverting data into packetBuf and ignore the payload
          ignoreThisPayload = true;
        }
      }
      else
      {
        // This is an ACK so it is to early to do anything with it
        // We need to wait until we have received the length and data bytes
        // So we should keep diverting data into packetBuf
      }
    }
    else if (ubxFrameCounter == 4) // Length LSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len = incoming; // (Duplication)
    }
    else if (ubxFrameCounter == 5) // Length MSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len |= incoming << 8; // (Duplication)
    }
    else if (ubxFrameCounter == 6) // This should be the first byte of the payload unless .len is zero
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial.print(F("process: ZERO LENGTH packet received: Class: 0x"));
          _debugSerial.print(packetBuf.cls, HEX);
          _debugSerial.print(F(" ID: 0x"));
          _debugSerial.println(packetBuf.id, HEX);
        }
#endif
        // If length is zero (!) this will be the first byte of the checksum so record it
        packetBuf.checksumA = incoming;
      }
      else
      {
        // The length is not zero so record this byte in the payload
        packetBuf.payload[0] = incoming;
      }
    }
    else if (ubxFrameCounter == 7) // This should be the second byte of the payload unless .len is zero or one
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
        // If length is zero (!) this will be the second byte of the checksum so record it
        packetBuf.checksumB = incoming;
      }
      else if (packetBuf.len == 1) // Check if length is one
      {
        // The length is one so this is the first byte of the checksum
        packetBuf.checksumA = incoming;
      }
      else // Length is >= 2 so this must be a payload byte
      {
        packetBuf.payload[1] = incoming;
      }
      // Now that we have received two payload bytes, we can check for a matching ACK/NACK
      if ((activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
          && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
          && (packetBuf.payload[0] == storedClass)           // and if the class matches
          && (packetBuf.payload[1] == storedID))             // and if the ID matches
      {
        if (packetBuf.len == 2) // Check if .len is 2
        {
          // Then this is a matching ACK so copy it into packetAck
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETACK;
          packetAck.cls = packetBuf.cls;
          packetAck.id = packetBuf.id;
          packetAck.len = packetBuf.len;
          packetAck.counter = packetBuf.counter;
          packetAck.payload[0] = packetBuf.payload[0];
          packetAck.payload[1] = packetBuf.payload[1];
        }
        else // Length is not 2 (hopefully this is impossible!)
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial.print(F("process: ACK received with .len != 2: Class: 0x"));
            _debugSerial.print(packetBuf.payload[0], HEX);
            _debugSerial.print(F(" ID: 0x"));
            _debugSerial.print(packetBuf.payload[1], HEX);
            _debugSerial.print(F(" len: "));
            _debugSerial.println(packetBuf.len);
          }
#endif
        }
      }
    }

    // Divert incoming into the correct buffer
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETACK)
      processUBX(incoming, &packetAck, storedClass, storedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
      processUBX(incoming, incomingUBX, storedClass, storedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
      processUBX(incoming, &packetBuf, storedClass, storedID);
    else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
      processUBX(incoming, &packetAuto, storedClass, storedID);

    // If user has assigned an output port then pipe the characters there,
    // but only if the port is different (otherwise we'll output each character twice!)
    if (_outputPort != _ubxOutputPort)
      _ubxOutputPort.write(incoming); // Echo this byte to the serial port

    // Finally, increment the frame counter
    ubxFrameCounter++;
  }
  else if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA) // Process incoming NMEA mesages. Selectively log if desired.
  {
    if ((nmeaByteCounter == 0) && (incoming != '$'))
    {
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset. (Almost certainly redundant!)
    }
    else if ((nmeaByteCounter == 1) && (incoming != 'G'))
    {
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.
    }
    else if ((nmeaByteCounter >= 0) && (nmeaByteCounter <= 5))
    {
      nmeaAddressField[nmeaByteCounter] = incoming; // Store the start character and NMEA address field
    }

    if (nmeaByteCounter == 5)
    {
      if (!_signsOfLife) // If _signsOfLife is not already true, set _signsOfLife to true if the NMEA header is valid
      {
        _signsOfLife = isNMEAHeaderValid();
      }

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      // Check if we have automatic storage for this message
      if (isThisNMEAauto())
      {
        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();
        *lengthPtr = 6;                           // Set the working copy length
        memset(nmeaPtr, 0, nmeaMaxLength);        // Clear the working copy
        memcpy(nmeaPtr, &nmeaAddressField[0], 6); // Copy the start character and address field into the working copy
      }
      else
#endif
      {
        // if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        // {
        //   _debugSerial.println(F("process: non-auto NMEA message"));
        // }
      }

      // We've just received the end of the address field. Check if it is selected for logging
      if (logThisNMEA())
      {
        memcpy(_storageNMEA->data, &nmeaAddressField[0], 6); // Add start character and address field to the storage
        _storageNMEA->length = 6;
      }
      // Check if it should be passed to processNMEA
      if (processThisNMEA())
      {
        for (uint8_t i = 0; i < 6; i++)
        {
          processNMEA(nmeaAddressField[i]); // Process the start character and address field
          // If user has assigned an output port then pipe the characters there,
          // but only if the port is different (otherwise we'll output each character twice!)
          if (_outputPort != _nmeaOutputPort)
            _nmeaOutputPort.write(nmeaAddressField[i]); // Echo this byte to the serial port
        }
      }
    }

    if ((nmeaByteCounter > 5) || (nmeaByteCounter < 0)) // Should we add incoming to the file buffer and/or pass it to processNMEA?
    {
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      if (isThisNMEAauto())
      {
        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();
        if (*lengthPtr < nmeaMaxLength)
        {
          *(nmeaPtr + *lengthPtr) = incoming; // Store the character
          *lengthPtr = *lengthPtr + 1;        // Increment the length
          if (*lengthPtr == nmeaMaxLength)
          {
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial.println(F("process: NMEA buffer is full!"));
            }
          }
        }
      }
#endif
      if (logThisNMEA())
      {
        // This check is probably redundant.
        // currentSentence is set to SFE_UBLOX_SENTENCE_TYPE_NONE below if nmeaByteCounter == maxNMEAByteCount
        if (_storageNMEA->length < maxNMEAByteCount) // Check we have room for it
        {
          _storageNMEA->data[_storageNMEA->length] = incoming; // Store the byte
          _storageNMEA->length = _storageNMEA->length + 1;
        }
      }
      if (processThisNMEA())
      {
        processNMEA(incoming); // Pass incoming to processNMEA
        // If user has assigned an output port then pipe the characters there,
        // but only if the port is different (otherwise we'll output each character twice!)
        if (_outputPort != _nmeaOutputPort)
          _nmeaOutputPort.write(incoming); // Echo this byte to the serial port
      }
    }

    if (incoming == '*')
      nmeaByteCounter = -5; // We are expecting * plus two checksum bytes plus CR and LF

    nmeaByteCounter++; // Increment the byte counter

    if (nmeaByteCounter == maxNMEAByteCount)          // Check if we have processed too many bytes
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.

    if (nmeaByteCounter == 0) // Check if we are done
    {
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      if (isThisNMEAauto())
      {
        uint8_t *workingLengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *workingNMEAPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();

        // Check the checksum: the checksum is the exclusive-OR of all characters between the $ and the *
        uint8_t nmeaChecksum = 0;
        uint8_t charsChecked = 1; // Start after the $
        uint8_t thisChar = '\0';
        while ((charsChecked < (nmeaMaxLength - 1)) && (charsChecked < ((*workingLengthPtr) - 4)) && (thisChar != '*'))
        {
          thisChar = *(workingNMEAPtr + charsChecked); // Get a char from the working copy
          if (thisChar != '*')                         // Ex-or the char into the checksum - but not if it is the '*'
            nmeaChecksum ^= thisChar;
          charsChecked++; // Increment the counter
        }
        if (thisChar == '*') // Make sure we found the *
        {
          uint8_t expectedChecksum1 = (nmeaChecksum >> 4) + '0';
          if (expectedChecksum1 >= ':') // Handle Hex correctly
            expectedChecksum1 += 'A' - ':';
          uint8_t expectedChecksum2 = (nmeaChecksum & 0x0F) + '0';
          if (expectedChecksum2 >= ':') // Handle Hex correctly
            expectedChecksum2 += 'A' - ':';
          if ((expectedChecksum1 == *(workingNMEAPtr + charsChecked)) && (expectedChecksum2 == *(workingNMEAPtr + charsChecked + 1)))
          {
            uint8_t *completeLengthPtr = getNMEACompleteLengthPtr();    // Get a pointer to the complete copy length
            uint8_t *completeNMEAPtr = getNMEACompleteNMEAPtr();        // Get a pointer to the complete copy NMEA data
            memset(completeNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous complete copy
            memcpy(completeNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the complete copy
            *completeLengthPtr = *workingLengthPtr;                     // Update the length
            nmeaAutomaticFlags *flagsPtr = getNMEAFlagsPtr();           // Get a pointer to the flags
            nmeaAutomaticFlags flagsCopy = *flagsPtr;
            flagsCopy.flags.bits.completeCopyValid = 1; // Set the complete copy valid flag
            flagsCopy.flags.bits.completeCopyRead = 0;  // Clear the complete copy read flag
            *flagsPtr = flagsCopy;                      // Update the flags
            // Callback
            if (doesThisNMEAHaveCallback()) // Do we need to copy the data into the callback copy?
            {
              if (flagsCopy.flags.bits.callbackCopyValid == 0) // Has the callback copy valid flag been cleared (by checkCallbacks)
              {
                uint8_t *callbackLengthPtr = getNMEACallbackLengthPtr();    // Get a pointer to the callback copy length
                uint8_t *callbackNMEAPtr = getNMEACallbackNMEAPtr();        // Get a pointer to the callback copy NMEA data
                memset(callbackNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous callback copy
                memcpy(callbackNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the callback copy
                *callbackLengthPtr = *workingLengthPtr;                     // Update the length
                flagsCopy.flags.bits.callbackCopyValid = 1;                 // Set the callback copy valid flag
                *flagsPtr = flagsCopy;                                      // Update the flags
              }
            }
          }
          else
          {
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial.print(F("process: NMEA checksum fail (2)! Expected "));
              _debugSerial.write(expectedChecksum1);
              _debugSerial.write(expectedChecksum2);
              _debugSerial.print(F(" Got "));
              _debugSerial.write(*(workingNMEAPtr + charsChecked));
              _debugSerial.write(*(workingNMEAPtr + charsChecked + 1));
              _debugSerial.println();
            }
          }
        }
        else
        {
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial.println(F("process: NMEA checksum fail (1)!"));
          }
        }
      }
#endif
      if (logThisNMEA())
      {
        // Check the checksum: the checksum is the exclusive-OR of all characters between the $ and the *
        uint8_t nmeaChecksum = 0;
        int8_t charsChecked = 1; // Start after the $
        uint8_t thisChar = '\0';
        while ((charsChecked < maxNMEAByteCount) && (charsChecked < (_storageNMEA->length - 4)) && (thisChar != '*'))
        {
          thisChar = _storageNMEA->data[charsChecked]; // Get a char from the storage
          if (thisChar != '*')                         // Ex-or the char into the checksum - but not if it is the '*'
            nmeaChecksum ^= thisChar;
          charsChecked++; // Increment the counter
        }
        if (thisChar == '*') // Make sure we found the *
        {
          uint8_t expectedChecksum1 = (nmeaChecksum >> 4) + '0';
          if (expectedChecksum1 >= ':') // Handle Hex correctly
            expectedChecksum1 += 'A' - ':';
          uint8_t expectedChecksum2 = (nmeaChecksum & 0x0F) + '0';
          if (expectedChecksum2 >= ':') // Handle Hex correctly
            expectedChecksum2 += 'A' - ':';
          if ((expectedChecksum1 == _storageNMEA->data[charsChecked]) && (expectedChecksum2 == _storageNMEA->data[charsChecked + 1]))
          {
            storeFileBytes(_storageNMEA->data, _storageNMEA->length); // Add NMEA to the file buffer
          }
          else if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial.println(F("process: _storageNMEA checksum fail!"));
          }
        }
      }
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // All done!
    }
  }
  else if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_RTCM)
  {

    // RTCM Logging
#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
    if (_storageRTCM != nullptr) // Check if RTCM logging storage exists
    {
      if (rtcmFrameCounter == 0)
      {
        _storageRTCM->dataMessage[0] = incoming;
        _storageRTCM->rollingChecksum = 0; // Initialize the checksum. Seed is 0x000000
      }
      else if (rtcmFrameCounter == 1)
      {
        _storageRTCM->dataMessage[1] = incoming;
        _storageRTCM->messageLength = (uint16_t)(incoming & 0x03) << 8;
      }
      else if (rtcmFrameCounter == 2)
      {
        _storageRTCM->dataMessage[2] = incoming;
        _storageRTCM->messageLength |= incoming;
      }

      // Store the mesage data (and CRC) - now that the message length is known
      if ((rtcmFrameCounter >= 3) && (rtcmFrameCounter < (_storageRTCM->messageLength + 6)) && (rtcmFrameCounter < (3 + SFE_UBLOX_MAX_RTCM_MSG_LEN + 3)))
        _storageRTCM->dataMessage[rtcmFrameCounter] = incoming;

      // Add incoming header and data bytes to the checksum
      if ((rtcmFrameCounter < 3) || ((rtcmFrameCounter >= 3) && (rtcmFrameCounter < (_storageRTCM->messageLength + 3))))
        crc24q(incoming, &_storageRTCM->rollingChecksum);

      // Check if all bytes have been received
      if ((rtcmFrameCounter >= 3) && (rtcmFrameCounter == _storageRTCM->messageLength + 5))
      {
        uint32_t expectedChecksum = _storageRTCM->dataMessage[_storageRTCM->messageLength + 3];
        expectedChecksum <<= 8;
        expectedChecksum |= _storageRTCM->dataMessage[_storageRTCM->messageLength + 4];
        expectedChecksum <<= 8;
        expectedChecksum |= _storageRTCM->dataMessage[_storageRTCM->messageLength + 5];

        if (expectedChecksum == _storageRTCM->rollingChecksum) // Does the checksum match?
        {
          // Extract the message type and check if it should be logged

          // Extract the message number from the first 12 bits
          uint16_t messageType = ((uint16_t)_storageRTCM->dataMessage[3]) << 4;
          messageType |= _storageRTCM->dataMessage[4] >> 4;
          uint16_t messageSubType = ((uint16_t)_storageRTCM->dataMessage[4] & 0x0F) << 8;
          messageSubType |= _storageRTCM->dataMessage[5];
          bool logThisRTCM = false;

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true)
          {
            _debugSerial.print(F("process: valid RTCM message type: "));
            _debugSerial.print(messageType);
            if (messageType == 4072)
            {
              _debugSerial.print(F("_"));
              _debugSerial.print(messageSubType);
            }
            _debugSerial.println(F(""));
          }
#endif

          if (!logThisRTCM)
            logThisRTCM = (messageType == 1001) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1001 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1002) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1002 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1003) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1003 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1004) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1004 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1005) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1005 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1006) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1006 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1007) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1007 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1009) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1009 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1010) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1010 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1011) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1011 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1012) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1012 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1033) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1033 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1074) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1074 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1075) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1075 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1077) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1077 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1084) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1084 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1085) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1085 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1087) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1087 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1094) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1094 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1095) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1095 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1097) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1097 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1124) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1124 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1125) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1125 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1127) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1127 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1230) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1230 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 4072) && (messageSubType == 0) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE4072_0 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 4072) && (messageSubType == 1) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE4072_1 == 1));

          if (logThisRTCM) // Should we log this message?
          {
            storeFileBytes(_storageRTCM->dataMessage, _storageRTCM->messageLength + 6);
          }

          // If there is space in the RTCM buffer, store the data there too
          if (rtcmBufferSpaceAvailable() >= _storageRTCM->messageLength + 6)
            storeRTCMBytes(_storageRTCM->dataMessage, _storageRTCM->messageLength + 6);

          // Check "Auto" RTCM
          if ((messageType == 1005) && (_storageRTCM->messageLength == RTCM_1005_MSG_LEN_BYTES) && (storageRTCM1005 != nullptr))
          {
            extractRTCM1005(&storageRTCM1005->data, &_storageRTCM->dataMessage[3]);

            storageRTCM1005->automaticFlags.flags.bits.dataValid = 1; // Mark the data as valid and unread
            storageRTCM1005->automaticFlags.flags.bits.dataRead = 0;

            if (storageRTCM1005->callbackData != nullptr)                              // Should we copy the data for the callback?
              if (storageRTCM1005->callbackPointerPtr != nullptr)                      // Has the callback been defined?
                if (storageRTCM1005->automaticFlags.flags.bits.callbackDataValid == 0) // Only overwrite the callback copy if it has been read
                {
                  memcpy(storageRTCM1005->callbackData, &storageRTCM1005->data, sizeof(RTCM_1005_data_t));
                  storageRTCM1005->automaticFlags.flags.bits.callbackDataValid = 1;
                }
          }
        }
        else
        {
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial.println(F("process: RTCM checksum fail!"));
          }
        }
      }
    }
#endif

    currentSentence = processRTCMframe(incoming, &rtcmFrameCounter); // Deal with RTCM bytes

    // If user has assigned an output port then pipe the characters there,
    // but only if the port is different (otherwise we'll output each character twice!)
    if (_outputPort != _rtcmOutputPort)
      _rtcmOutputPort.write(incoming); // Echo this byte to the serial port
  }
}

// PRIVATE: Return true if we should add this NMEA message to the file buffer for logging
bool DevUBLOXGNSS::logThisNMEA()
{
  bool logMe = false;
  if (_logNMEA.bits.all == 1)
    logMe = true;
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M') && (_logNMEA.bits.UBX_NMEA_DTM == 1))
    logMe = true;
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GAQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GBQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GBS == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_GGA == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L') && (_logNMEA.bits.UBX_NMEA_GLL == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GLQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GNQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GNS == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GPQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GQQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GRS == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_GSA == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T') && (_logNMEA.bits.UBX_NMEA_GST == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V') && (_logNMEA.bits.UBX_NMEA_GSV == 1))
      logMe = true;
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M') && (_logNMEA.bits.UBX_NMEA_RLM == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C') && (_logNMEA.bits.UBX_NMEA_RMC == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'H') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_THS == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T') && (_logNMEA.bits.UBX_NMEA_TXT == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W') && (_logNMEA.bits.UBX_NMEA_VLW == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G') && (_logNMEA.bits.UBX_NMEA_VTG == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_ZDA == 1))
    logMe = true;

  if (logMe)                   // Message should be logged.
    logMe = initStorageNMEA(); // Check we have non-Auto storage for it
  return (logMe);
}

// PRIVATE: Return true if the NMEA header is valid
bool DevUBLOXGNSS::isNMEAHeaderValid()
{
  if (nmeaAddressField[0] != '*')
    return (false);
  if (nmeaAddressField[1] != 'G')
    return (false);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M'))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A'))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L'))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V'))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M'))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C'))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'H') && (nmeaAddressField[5] == 'S'))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T'))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W'))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G'))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A'))
    return (true);
  return (false);
}

// PRIVATE: Return true if we should pass this NMEA message to processNMEA
bool DevUBLOXGNSS::processThisNMEA()
{
  if (_processNMEA.bits.all == 1)
    return (true);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M') && (_processNMEA.bits.UBX_NMEA_DTM == 1))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GAQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GBQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GBS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_GGA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L') && (_processNMEA.bits.UBX_NMEA_GLL == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GLQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GNQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GNS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GPQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GQQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GRS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_GSA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T') && (_processNMEA.bits.UBX_NMEA_GST == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V') && (_processNMEA.bits.UBX_NMEA_GSV == 1))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M') && (_processNMEA.bits.UBX_NMEA_RLM == 1))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C') && (_processNMEA.bits.UBX_NMEA_RMC == 1))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'H') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_THS == 1))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T') && (_processNMEA.bits.UBX_NMEA_TXT == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W') && (_processNMEA.bits.UBX_NMEA_VLW == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G') && (_processNMEA.bits.UBX_NMEA_VTG == 1))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_ZDA == 1))
    return (true);
  return (false);
}

// This is the default or generic NMEA processor. We're only going to pipe the data to serial port so we can see it.
// User could overwrite this function to pipe characters to nmea.process(c) of tinyGPS or MicroNMEA
// Or user could pipe each character to a buffer, radio, etc.
void DevUBLOXGNSS::processNMEA(char incoming)
{
  (void)incoming;
}

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
// Check if the NMEA message (in nmeaAddressField) is "auto" (i.e. has dedicated RAM allocated for it)
bool DevUBLOXGNSS::isThisNMEAauto()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPGGA != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNGGA != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPVTG != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNVTG != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPRMC != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNRMC != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPZDA != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNZDA != nullptr)
      return true;
  }

  return false;
}

// Do we need to copy the data into the callback copy?
bool DevUBLOXGNSS::doesThisNMEAHaveCallback()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPGGA != nullptr)
      if (storageNMEAGPGGA->callbackCopy != nullptr)
        if (storageNMEAGPGGA->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNGGA != nullptr)
      if (storageNMEAGNGGA->callbackCopy != nullptr)
        if (storageNMEAGNGGA->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPVTG != nullptr)
      if (storageNMEAGPVTG->callbackCopy != nullptr)
        if (storageNMEAGPVTG->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNVTG != nullptr)
      if (storageNMEAGNVTG->callbackCopy != nullptr)
        if (storageNMEAGNVTG->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPRMC != nullptr)
      if (storageNMEAGPRMC->callbackCopy != nullptr)
        if (storageNMEAGPRMC->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNRMC != nullptr)
      if (storageNMEAGNRMC->callbackCopy != nullptr)
        if (storageNMEAGNRMC->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPZDA != nullptr)
      if (storageNMEAGPZDA->callbackCopy != nullptr)
        if (storageNMEAGPZDA->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNZDA != nullptr)
      if (storageNMEAGNZDA->callbackCopy != nullptr)
        if (storageNMEAGNZDA->callbackPointerPtr != nullptr)
          return true;
  }

  return false;
}

// Get a pointer to the working copy length
uint8_t *DevUBLOXGNSS::getNMEAWorkingLengthPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->workingCopy.length;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->workingCopy.length;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->workingCopy.length;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->workingCopy.length;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->workingCopy.length;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->workingCopy.length;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->workingCopy.length;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->workingCopy.length;
  }

  return nullptr;
}

// Get a pointer to the working copy NMEA data
uint8_t *DevUBLOXGNSS::getNMEAWorkingNMEAPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->workingCopy.nmea[0];
  }

  return nullptr;
}

// Get a pointer to the complete copy length
uint8_t *DevUBLOXGNSS::getNMEACompleteLengthPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->completeCopy.length;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->completeCopy.length;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->completeCopy.length;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->completeCopy.length;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->completeCopy.length;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->completeCopy.length;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->completeCopy.length;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->completeCopy.length;
  }

  return nullptr;
}

// Get a pointer to the complete copy NMEA data
uint8_t *DevUBLOXGNSS::getNMEACompleteNMEAPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->completeCopy.nmea[0];
  }

  return nullptr;
}

// Get a pointer to the callback copy length
uint8_t *DevUBLOXGNSS::getNMEACallbackLengthPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->callbackCopy->length;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->callbackCopy->length;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->callbackCopy->length;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->callbackCopy->length;
  }

  return nullptr;
}

// Get a pointer to the callback copy NMEA data
uint8_t *DevUBLOXGNSS::getNMEACallbackNMEAPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->callbackCopy->nmea[0];
  }

  return nullptr;
}

// Get the maximum length of this NMEA message
uint8_t DevUBLOXGNSS::getNMEAMaxLength()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_GGA_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_GGA_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_VTG_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_VTG_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_RMC_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_RMC_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_ZDA_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_ZDA_MAX_LENGTH;
  }

  return 0;
}

// Get a pointer to the automatic NMEA flags
nmeaAutomaticFlags *DevUBLOXGNSS::getNMEAFlagsPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->automaticFlags;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->automaticFlags;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->automaticFlags;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->automaticFlags;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->automaticFlags;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->automaticFlags;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->automaticFlags;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->automaticFlags;
  }

  return nullptr;
}
#endif

// We need to be able to identify an RTCM packet and then the length
// so that we know when the RTCM message is completely received and we then start
// listening for other sentences (like NMEA or UBX)
// RTCM packet structure is very odd. I never found RTCM STANDARD 10403.2 but
// http://d1.amobbs.com/bbs_upload782111/files_39/ourdev_635123CK0HJT.pdf is good
// https://dspace.cvut.cz/bitstream/handle/10467/65205/F3-BP-2016-Shkalikava-Anastasiya-Prenos%20polohove%20informace%20prostrednictvim%20datove%20site.pdf?sequence=-1
// Lead me to: https://forum.u-blox.com/index.php/4348/how-to-read-rtcm-messages-from-neo-m8p
// RTCM 3.2 bytes look like this:
// Byte 0: Always 0xD3
// Byte 1: 6-bits of zero
// Byte 2: 10-bits of length of this packet including the first two-ish header bytes, + 6.
// byte 3 + 4 bits: Msg type 12 bits
// Example: D3 00 7C 43 F0 ... / 0x7C = 124+6 = 130 bytes in this packet, 0x43F = Msg type 1087
DevUBLOXGNSS::sfe_ublox_sentence_types_e DevUBLOXGNSS::processRTCMframe(uint8_t incoming, uint16_t *rtcmFrameCounter)
{
  static uint16_t rtcmLen = 0; // Static - length is retained between calls

  if (*rtcmFrameCounter == 1)
  {
    rtcmLen = (incoming & 0x03) << 8; // Get the last two bits of this byte. Bits 8&9 of 10-bit length
  }
  else if (*rtcmFrameCounter == 2)
  {
    rtcmLen |= incoming; // Bits 0-7 of packet length
    rtcmLen += 6;        // There are 6 additional bytes of what we presume is header, msgType, CRC, and stuff
  }
  /*else if (rtcmFrameCounter == 3)
  {
    rtcmMsgType = incoming << 4; //Message Type, MS 4 bits
  }
  else if (rtcmFrameCounter == 4)
  {
    rtcmMsgType |= (incoming >> 4); //Message Type, bits 0-7
  }*/

  *rtcmFrameCounter = *rtcmFrameCounter + 1; // Increment rtcmFrameCounter

  processRTCM(incoming); // Here is where we expose this byte to the user

  // If rtcmLen is not yet known, return SFE_UBLOX_SENTENCE_TYPE_RTCM
  if (*rtcmFrameCounter <= 2) // If this is header byte 0 or 1 (rtcmFrameCounter has been incremented)
    return SFE_UBLOX_SENTENCE_TYPE_RTCM;

  // Reset and start looking for next sentence type when done
  return (*rtcmFrameCounter == rtcmLen) ? SFE_UBLOX_SENTENCE_TYPE_NONE : SFE_UBLOX_SENTENCE_TYPE_RTCM;
}

// This function is called for each byte of an RTCM frame
// Ths user can overwrite this function and process the RTCM frame as they please
// Bytes can be piped to Serial or other interface. The consumer could be a radio or the internet (Ntrip broadcaster)
void DevUBLOXGNSS::processRTCM(uint8_t incoming)
{
  (void)incoming;
}

// Given a character, file it away into the uxb packet structure
// Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
// The payload portion of the packet can be 100s of bytes but the max array size is packetCfgPayloadSize bytes.
// startingSpot can be set so we only record a subset of bytes within a larger packet.
void DevUBLOXGNSS::processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  // If incomingUBX is a user-defined custom packet, then the payload size could be different to packetCfgPayloadSize.
  // TO DO: update this to prevent an overrun when receiving an automatic message
  //        and the incomingUBX payload size is smaller than packetCfgPayloadSize.
  uint16_t maximum_payload_size;
  if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
    maximum_payload_size = packetCfgPayloadSize;
  else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
  {
    // Calculate maximum payload size once Class and ID have been received
    // (This check is probably redundant as activePacketBuffer can only be SFE_UBLOX_PACKET_PACKETAUTO
    //  when ubxFrameCounter >= 3)
    // if (incomingUBX->counter >= 2)
    //{

    bool logBecauseAuto = autoLookup(incomingUBX->cls, incomingUBX->id, &maximum_payload_size);
    bool logBecauseEnabled = logThisUBX(incomingUBX->cls, incomingUBX->id);
    if ((!logBecauseAuto) && (logBecauseEnabled))
      maximum_payload_size = SFE_UBX_MAX_LENGTH;
    if (maximum_payload_size == 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.print(F("processUBX: autoLookup returned ZERO maxPayload!! Class: 0x"));
        _debugSerial.print(incomingUBX->cls, HEX);
        _debugSerial.print(F(" ID: 0x"));
        _debugSerial.println(incomingUBX->id, HEX);
      }
#endif
    }
    //}
    // else
    //  maximum_payload_size = 2;
  }
  else
    maximum_payload_size = 2;

  bool overrun = false;

  // Add all incoming bytes to the rolling checksum
  // Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
  if (incomingUBX->counter < (incomingUBX->len + 4))
    addToChecksum(incoming);

  if (incomingUBX->counter == 0)
  {
    incomingUBX->cls = incoming;
  }
  else if (incomingUBX->counter == 1)
  {
    incomingUBX->id = incoming;
  }
  else if (incomingUBX->counter == 2) // Len LSB
  {
    incomingUBX->len = incoming;
  }
  else if (incomingUBX->counter == 3) // Len MSB
  {
    incomingUBX->len |= incoming << 8;
  }
  else if (incomingUBX->counter == incomingUBX->len + 4) // ChecksumA
  {
    incomingUBX->checksumA = incoming;
  }
  else if (incomingUBX->counter == incomingUBX->len + 5) // ChecksumB
  {
    incomingUBX->checksumB = incoming;

    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // We're done! Reset the sentence to being looking for a new start char

    // Validate this sentence
    if ((incomingUBX->checksumA == rollingChecksumA) && (incomingUBX->checksumB == rollingChecksumB))
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid
      _signsOfLife = true;                                  // The checksum is valid, so set the _signsOfLife flag

      // Let's check if the class and ID match the requestedClass and requestedID
      // Remember - this could be a data packet or an ACK packet
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_NACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("processUBX: NACK received: Requested Class: 0x"));
          _debugSerial.print(incomingUBX->payload[0], HEX);
          _debugSerial.print(F(" Requested ID: 0x"));
          _debugSerial.println(incomingUBX->payload[1], HEX);
        }
#endif
      }

      // This is not an ACK and we do not have a complete class and ID match
      // So let's check for an "automatic" message arriving
      else if ((autoLookup(incomingUBX->cls, incomingUBX->id)) || (logThisUBX(incomingUBX->cls, incomingUBX->id)))
      {
        // This isn't the message we are looking for...
        // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("processUBX: incoming \"automatic\" message: Class: 0x"));
          _debugSerial.print(incomingUBX->cls, HEX);
          _debugSerial.print(F(" ID: 0x"));
          _debugSerial.println(incomingUBX->id, HEX);
        }
#endif
      }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial.print(F("Incoming: Size: "));
        _debugSerial.print(incomingUBX->len);
        _debugSerial.print(F(" Received: "));
        printPacket(incomingUBX);

        if (incomingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial.println(F("packetCfg now valid"));
        }
        if (packetAck.valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial.println(F("packetAck now valid"));
        }
        if (incomingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial.println(F("packetCfg classAndIDmatch"));
        }
        if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial.println(F("packetAck classAndIDmatch"));
        }
      }
#endif

      // We've got a valid packet, now do something with it but only if ignoreThisPayload is false
      if (ignoreThisPayload == false)
      {
        processUBXpacket(incomingUBX);
      }
    }
    else // Checksum failure
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

      // Let's check if the class and ID match the requestedClass and requestedID.
      // This is potentially risky as we are saying that we saw the requested Class and ID
      // but that the packet checksum failed. Potentially it could be the class or ID bytes
      // that caused the checksum error!
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }
      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }

      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        // Drive an external pin to allow for easier logic analyzation
        if (debugPin >= 0)
        {
          digitalWrite((uint8_t)debugPin, LOW);
          delay(10);
          digitalWrite((uint8_t)debugPin, HIGH);
        }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        _debugSerial.print(F("Checksum failed:"));
        _debugSerial.print(F(" checksumA: "));
        _debugSerial.print(incomingUBX->checksumA);
        _debugSerial.print(F(" checksumB: "));
        _debugSerial.print(incomingUBX->checksumB);

        _debugSerial.print(F(" rollingChecksumA: "));
        _debugSerial.print(rollingChecksumA);
        _debugSerial.print(F(" rollingChecksumB: "));
        _debugSerial.print(rollingChecksumB);
        _debugSerial.println();
#endif
      }
    }

    // Now that the packet is complete and has been processed, we need to delete the memory
    // allocated for packetAuto
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
    {
      delete[] payloadAuto; // Created with new[]
      payloadAuto = nullptr;
      packetAuto.payload = payloadAuto;
    }
  }
  else // Load this byte into the payload array
  {
    // If an automatic packet comes in asynchronously, we need to fudge the startingSpot
    uint16_t startingSpot = incomingUBX->startingSpot;
    if (autoLookup(incomingUBX->cls, incomingUBX->id))
      startingSpot = 0;
    // Check if this is payload data which should be ignored
    if (ignoreThisPayload == false)
    {
      // Begin recording if counter goes past startingSpot
      if ((incomingUBX->counter - 4) >= startingSpot)
      {
        // Check to see if we have room for this byte
        if (((incomingUBX->counter - 4) - startingSpot) < maximum_payload_size) // If counter = 208, starting spot = 200, we're good to record.
        {
          incomingUBX->payload[(incomingUBX->counter - 4) - startingSpot] = incoming; // Store this byte into payload array
        }
        else
        {
          overrun = true;
        }
      }
    }
  }

  // incomingUBX->counter should never reach maximum_payload_size + class + id + len[2] + checksum[2]
  if (overrun || ((incomingUBX->counter == maximum_payload_size + 6) && (ignoreThisPayload == false)))
  {
    // Something has gone very wrong
    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being looking for a new start char
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      if (overrun)
        _debugSerial.print(F("processUBX: buffer overrun detected!"));
      else
        _debugSerial.print(F("processUBX: counter hit maximum_payload_size + 6!"));
      _debugSerial.print(F(" activePacketBuffer: "));
      _debugSerial.print(activePacketBuffer);
      _debugSerial.print(F(" maximum_payload_size: "));
      _debugSerial.println(maximum_payload_size);
    }
#endif
  }

  // Increment the counter
  incomingUBX->counter++;
}

// Once a packet has been received and validated, identify this packet's class/id and update internal flags
void DevUBLOXGNSS::processUBXpacket(ubxPacket *msg)
{
  bool addedToFileBuffer = false;
  switch (msg->cls)
  {
  case UBX_CLASS_NAV:
    if (msg->id == UBX_NAV_POSECEF && msg->len == UBX_NAV_POSECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPOSECEF != nullptr)
      {
        packetUBXNAVPOSECEF->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPOSECEF->data.ecefX = extractSignedLong(msg, 4);
        packetUBXNAVPOSECEF->data.ecefY = extractSignedLong(msg, 8);
        packetUBXNAVPOSECEF->data.ecefZ = extractSignedLong(msg, 12);
        packetUBXNAVPOSECEF->data.pAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPOSECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPOSECEF->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPOSECEF->callbackData->iTOW, &packetUBXNAVPOSECEF->data.iTOW, sizeof(UBX_NAV_POSECEF_data_t));
          packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPOSECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_STATUS && msg->len == UBX_NAV_STATUS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSTATUS != nullptr)
      {
        packetUBXNAVSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVSTATUS->data.gpsFix = extractByte(msg, 4);
        packetUBXNAVSTATUS->data.flags.all = extractByte(msg, 5);
        packetUBXNAVSTATUS->data.fixStat.all = extractByte(msg, 6);
        packetUBXNAVSTATUS->data.flags2.all = extractByte(msg, 7);
        packetUBXNAVSTATUS->data.ttff = extractLong(msg, 8);
        packetUBXNAVSTATUS->data.msss = extractLong(msg, 12);

        // Mark all datums as fresh (not read before)
        packetUBXNAVSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSTATUS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSTATUS->callbackData->iTOW, &packetUBXNAVSTATUS->data.iTOW, sizeof(UBX_NAV_STATUS_data_t));
          packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_DOP && msg->len == UBX_NAV_DOP_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVDOP != nullptr)
      {
        packetUBXNAVDOP->data.iTOW = extractLong(msg, 0);
        packetUBXNAVDOP->data.gDOP = extractInt(msg, 4);
        packetUBXNAVDOP->data.pDOP = extractInt(msg, 6);
        packetUBXNAVDOP->data.tDOP = extractInt(msg, 8);
        packetUBXNAVDOP->data.vDOP = extractInt(msg, 10);
        packetUBXNAVDOP->data.hDOP = extractInt(msg, 12);
        packetUBXNAVDOP->data.nDOP = extractInt(msg, 14);
        packetUBXNAVDOP->data.eDOP = extractInt(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVDOP->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVDOP->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVDOP->callbackData->iTOW, &packetUBXNAVDOP->data.iTOW, sizeof(UBX_NAV_DOP_data_t));
          packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVDOP->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_ATT && msg->len == UBX_NAV_ATT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVATT != nullptr)
      {
        packetUBXNAVATT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVATT->data.version = extractByte(msg, 4);
        packetUBXNAVATT->data.roll = extractSignedLong(msg, 8);
        packetUBXNAVATT->data.pitch = extractSignedLong(msg, 12);
        packetUBXNAVATT->data.heading = extractSignedLong(msg, 16);
        packetUBXNAVATT->data.accRoll = extractLong(msg, 20);
        packetUBXNAVATT->data.accPitch = extractLong(msg, 24);
        packetUBXNAVATT->data.accHeading = extractLong(msg, 28);

        // Mark all datums as fresh (not read before)
        packetUBXNAVATT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVATT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVATT->callbackData->iTOW, &packetUBXNAVATT->data.iTOW, sizeof(UBX_NAV_ATT_data_t));
          packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVATT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_PVT && msg->len == UBX_NAV_PVT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVT != nullptr)
      {
        packetUBXNAVPVT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVT->data.year = extractInt(msg, 4);
        packetUBXNAVPVT->data.month = extractByte(msg, 6);
        packetUBXNAVPVT->data.day = extractByte(msg, 7);
        packetUBXNAVPVT->data.hour = extractByte(msg, 8);
        packetUBXNAVPVT->data.min = extractByte(msg, 9);
        packetUBXNAVPVT->data.sec = extractByte(msg, 10);
        packetUBXNAVPVT->data.valid.all = extractByte(msg, 11);
        packetUBXNAVPVT->data.tAcc = extractLong(msg, 12);
        packetUBXNAVPVT->data.nano = extractSignedLong(msg, 16); // Includes milliseconds
        packetUBXNAVPVT->data.fixType = extractByte(msg, 20);
        packetUBXNAVPVT->data.flags.all = extractByte(msg, 21);
        packetUBXNAVPVT->data.flags2.all = extractByte(msg, 22);
        packetUBXNAVPVT->data.numSV = extractByte(msg, 23);
        packetUBXNAVPVT->data.lon = extractSignedLong(msg, 24);
        packetUBXNAVPVT->data.lat = extractSignedLong(msg, 28);
        packetUBXNAVPVT->data.height = extractSignedLong(msg, 32);
        packetUBXNAVPVT->data.hMSL = extractSignedLong(msg, 36);
        packetUBXNAVPVT->data.hAcc = extractLong(msg, 40);
        packetUBXNAVPVT->data.vAcc = extractLong(msg, 44);
        packetUBXNAVPVT->data.velN = extractSignedLong(msg, 48);
        packetUBXNAVPVT->data.velE = extractSignedLong(msg, 52);
        packetUBXNAVPVT->data.velD = extractSignedLong(msg, 56);
        packetUBXNAVPVT->data.gSpeed = extractSignedLong(msg, 60);
        packetUBXNAVPVT->data.headMot = extractSignedLong(msg, 64);
        packetUBXNAVPVT->data.sAcc = extractLong(msg, 68);
        packetUBXNAVPVT->data.headAcc = extractLong(msg, 72);
        packetUBXNAVPVT->data.pDOP = extractInt(msg, 76);
        packetUBXNAVPVT->data.flags3.all = extractByte(msg, 78);
        packetUBXNAVPVT->data.headVeh = extractSignedLong(msg, 84);
        packetUBXNAVPVT->data.magDec = extractSignedInt(msg, 88);
        packetUBXNAVPVT->data.magAcc = extractInt(msg, 90);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVT->callbackData->iTOW, &packetUBXNAVPVT->data.iTOW, sizeof(UBX_NAV_PVT_data_t));
          packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_ODO && msg->len == UBX_NAV_ODO_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVODO != nullptr)
      {
        packetUBXNAVODO->data.version = extractByte(msg, 0);
        packetUBXNAVODO->data.iTOW = extractLong(msg, 4);
        packetUBXNAVODO->data.distance = extractLong(msg, 8);
        packetUBXNAVODO->data.totalDistance = extractLong(msg, 12);
        packetUBXNAVODO->data.distanceStd = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVODO->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVODO->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVODO->callbackData->version, &packetUBXNAVODO->data.version, sizeof(UBX_NAV_ODO_data_t));
          packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVODO->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_VELECEF && msg->len == UBX_NAV_VELECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVVELECEF != nullptr)
      {
        packetUBXNAVVELECEF->data.iTOW = extractLong(msg, 0);
        packetUBXNAVVELECEF->data.ecefVX = extractSignedLong(msg, 4);
        packetUBXNAVVELECEF->data.ecefVY = extractSignedLong(msg, 8);
        packetUBXNAVVELECEF->data.ecefVZ = extractSignedLong(msg, 12);
        packetUBXNAVVELECEF->data.sAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVVELECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVVELECEF->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVVELECEF->callbackData->iTOW, &packetUBXNAVVELECEF->data.iTOW, sizeof(UBX_NAV_VELECEF_data_t));
          packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVVELECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_VELNED && msg->len == UBX_NAV_VELNED_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVVELNED != nullptr)
      {
        packetUBXNAVVELNED->data.iTOW = extractLong(msg, 0);
        packetUBXNAVVELNED->data.velN = extractSignedLong(msg, 4);
        packetUBXNAVVELNED->data.velE = extractSignedLong(msg, 8);
        packetUBXNAVVELNED->data.velD = extractSignedLong(msg, 12);
        packetUBXNAVVELNED->data.speed = extractLong(msg, 16);
        packetUBXNAVVELNED->data.gSpeed = extractLong(msg, 20);
        packetUBXNAVVELNED->data.heading = extractSignedLong(msg, 24);
        packetUBXNAVVELNED->data.sAcc = extractLong(msg, 28);
        packetUBXNAVVELNED->data.cAcc = extractLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXNAVVELNED->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVVELNED->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVVELNED->callbackData->iTOW, &packetUBXNAVVELNED->data.iTOW, sizeof(UBX_NAV_VELNED_data_t));
          packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVVELNED->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_HPPOSECEF && msg->len == UBX_NAV_HPPOSECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVHPPOSECEF != nullptr)
      {
        packetUBXNAVHPPOSECEF->data.version = extractByte(msg, 0);
        packetUBXNAVHPPOSECEF->data.iTOW = extractLong(msg, 4);
        packetUBXNAVHPPOSECEF->data.ecefX = extractSignedLong(msg, 8);
        packetUBXNAVHPPOSECEF->data.ecefY = extractSignedLong(msg, 12);
        packetUBXNAVHPPOSECEF->data.ecefZ = extractSignedLong(msg, 16);
        packetUBXNAVHPPOSECEF->data.ecefXHp = extractSignedChar(msg, 20);
        packetUBXNAVHPPOSECEF->data.ecefYHp = extractSignedChar(msg, 21);
        packetUBXNAVHPPOSECEF->data.ecefZHp = extractSignedChar(msg, 22);
        packetUBXNAVHPPOSECEF->data.flags.all = extractByte(msg, 23);
        packetUBXNAVHPPOSECEF->data.pAcc = extractLong(msg, 24);

        // Mark all datums as fresh (not read before)
        packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVHPPOSECEF->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVHPPOSECEF->callbackData->version, &packetUBXNAVHPPOSECEF->data.version, sizeof(UBX_NAV_HPPOSECEF_data_t));
          packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_HPPOSLLH && msg->len == UBX_NAV_HPPOSLLH_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVHPPOSLLH != nullptr)
      {
        packetUBXNAVHPPOSLLH->data.version = extractByte(msg, 0);
        packetUBXNAVHPPOSLLH->data.flags.all = extractByte(msg, 3);
        packetUBXNAVHPPOSLLH->data.iTOW = extractLong(msg, 4);
        packetUBXNAVHPPOSLLH->data.lon = extractSignedLong(msg, 8);
        packetUBXNAVHPPOSLLH->data.lat = extractSignedLong(msg, 12);
        packetUBXNAVHPPOSLLH->data.height = extractSignedLong(msg, 16);
        packetUBXNAVHPPOSLLH->data.hMSL = extractSignedLong(msg, 20);
        packetUBXNAVHPPOSLLH->data.lonHp = extractSignedChar(msg, 24);
        packetUBXNAVHPPOSLLH->data.latHp = extractSignedChar(msg, 25);
        packetUBXNAVHPPOSLLH->data.heightHp = extractSignedChar(msg, 26);
        packetUBXNAVHPPOSLLH->data.hMSLHp = extractSignedChar(msg, 27);
        packetUBXNAVHPPOSLLH->data.hAcc = extractLong(msg, 28);
        packetUBXNAVHPPOSLLH->data.vAcc = extractLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVHPPOSLLH->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVHPPOSLLH->callbackData->version, &packetUBXNAVHPPOSLLH->data.version, sizeof(UBX_NAV_HPPOSLLH_data_t));
          packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_PVAT && msg->len == UBX_NAV_PVAT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVAT != nullptr)
      {
        packetUBXNAVPVAT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVAT->data.version = extractByte(msg, 4);
        packetUBXNAVPVAT->data.valid.all = extractByte(msg, 5);
        packetUBXNAVPVAT->data.year = extractInt(msg, 6);
        packetUBXNAVPVAT->data.month = extractByte(msg, 8);
        packetUBXNAVPVAT->data.day = extractByte(msg, 9);
        packetUBXNAVPVAT->data.hour = extractByte(msg, 10);
        packetUBXNAVPVAT->data.min = extractByte(msg, 11);
        packetUBXNAVPVAT->data.sec = extractByte(msg, 12);
        packetUBXNAVPVAT->data.tAcc = extractLong(msg, 16);
        packetUBXNAVPVAT->data.nano = extractSignedLong(msg, 20); // Includes milliseconds
        packetUBXNAVPVAT->data.fixType = extractByte(msg, 24);
        packetUBXNAVPVAT->data.flags.all = extractByte(msg, 25);
        packetUBXNAVPVAT->data.flags2.all = extractByte(msg, 26);
        packetUBXNAVPVAT->data.numSV = extractByte(msg, 27);
        packetUBXNAVPVAT->data.lon = extractSignedLong(msg, 28);
        packetUBXNAVPVAT->data.lat = extractSignedLong(msg, 32);
        packetUBXNAVPVAT->data.height = extractSignedLong(msg, 36);
        packetUBXNAVPVAT->data.hMSL = extractSignedLong(msg, 40);
        packetUBXNAVPVAT->data.hAcc = extractLong(msg, 44);
        packetUBXNAVPVAT->data.vAcc = extractLong(msg, 48);
        packetUBXNAVPVAT->data.velN = extractSignedLong(msg, 52);
        packetUBXNAVPVAT->data.velE = extractSignedLong(msg, 56);
        packetUBXNAVPVAT->data.velD = extractSignedLong(msg, 60);
        packetUBXNAVPVAT->data.gSpeed = extractSignedLong(msg, 64);
        packetUBXNAVPVAT->data.sAcc = extractLong(msg, 68);
        packetUBXNAVPVAT->data.vehRoll = extractSignedLong(msg, 72);
        packetUBXNAVPVAT->data.vehPitch = extractSignedLong(msg, 76);
        packetUBXNAVPVAT->data.vehHeading = extractSignedLong(msg, 80);
        packetUBXNAVPVAT->data.motHeading = extractSignedLong(msg, 84);
        packetUBXNAVPVAT->data.accRoll = extractInt(msg, 88);
        packetUBXNAVPVAT->data.accPitch = extractInt(msg, 90);
        packetUBXNAVPVAT->data.accHeading = extractInt(msg, 92);
        packetUBXNAVPVAT->data.magDec = extractSignedInt(msg, 94);
        packetUBXNAVPVAT->data.magAcc = extractInt(msg, 96);
        packetUBXNAVPVAT->data.errEllipseOrient = extractInt(msg, 98);
        packetUBXNAVPVAT->data.errEllipseMajor = extractLong(msg, 100);
        packetUBXNAVPVAT->data.errEllipseMinor = extractLong(msg, 104);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVAT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVAT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVAT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVAT->callbackData->iTOW, &packetUBXNAVPVAT->data.iTOW, sizeof(UBX_NAV_PVAT_data_t));
          packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVAT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_TIMEUTC && msg->len == UBX_NAV_TIMEUTC_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVTIMEUTC != nullptr)
      {
        packetUBXNAVTIMEUTC->data.iTOW = extractLong(msg, 0);
        packetUBXNAVTIMEUTC->data.tAcc = extractLong(msg, 4);
        packetUBXNAVTIMEUTC->data.nano = extractSignedLong(msg, 8);
        packetUBXNAVTIMEUTC->data.year = extractInt(msg, 12);
        packetUBXNAVTIMEUTC->data.month = extractByte(msg, 14);
        packetUBXNAVTIMEUTC->data.day = extractByte(msg, 15);
        packetUBXNAVTIMEUTC->data.hour = extractByte(msg, 16);
        packetUBXNAVTIMEUTC->data.min = extractByte(msg, 17);
        packetUBXNAVTIMEUTC->data.sec = extractByte(msg, 18);
        packetUBXNAVTIMEUTC->data.valid.all = extractByte(msg, 19);

        // Mark all datums as fresh (not read before)
        packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVTIMEUTC->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVTIMEUTC->callbackData->iTOW, &packetUBXNAVTIMEUTC->data.iTOW, sizeof(UBX_NAV_TIMEUTC_data_t));
          packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_CLOCK && msg->len == UBX_NAV_CLOCK_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVCLOCK != nullptr)
      {
        packetUBXNAVCLOCK->data.iTOW = extractLong(msg, 0);
        packetUBXNAVCLOCK->data.clkB = extractSignedLong(msg, 4);
        packetUBXNAVCLOCK->data.clkD = extractSignedLong(msg, 8);
        packetUBXNAVCLOCK->data.tAcc = extractLong(msg, 12);
        packetUBXNAVCLOCK->data.fAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVCLOCK->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVCLOCK->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVCLOCK->callbackData->iTOW, &packetUBXNAVCLOCK->data.iTOW, sizeof(UBX_NAV_CLOCK_data_t));
          packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVCLOCK->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_TIMELS && msg->len == UBX_NAV_TIMELS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVTIMELS != nullptr)
      {
        packetUBXNAVTIMELS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVTIMELS->data.version = extractByte(msg, 4);
        packetUBXNAVTIMELS->data.srcOfCurrLs = extractByte(msg, 8);
        packetUBXNAVTIMELS->data.currLs = extractSignedChar(msg, 9);
        packetUBXNAVTIMELS->data.srcOfLsChange = extractByte(msg, 10);
        packetUBXNAVTIMELS->data.lsChange = extractSignedChar(msg, 11);
        packetUBXNAVTIMELS->data.timeToLsEvent = extractSignedLong(msg, 12);
        packetUBXNAVTIMELS->data.dateOfLsGpsWn = extractInt(msg, 16);
        packetUBXNAVTIMELS->data.dateOfLsGpsDn = extractInt(msg, 18);
        packetUBXNAVTIMELS->data.valid.all = extractSignedChar(msg, 23);

        // Mark all datums as fresh (not read before)
        packetUBXNAVTIMELS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;
      }
    }
    else if (msg->id == UBX_NAV_SVIN && msg->len == UBX_NAV_SVIN_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSVIN != nullptr)
      {
        packetUBXNAVSVIN->data.version = extractByte(msg, 0);
        packetUBXNAVSVIN->data.iTOW = extractLong(msg, 4);
        packetUBXNAVSVIN->data.dur = extractLong(msg, 8);
        packetUBXNAVSVIN->data.meanX = extractSignedLong(msg, 12);
        packetUBXNAVSVIN->data.meanY = extractSignedLong(msg, 16);
        packetUBXNAVSVIN->data.meanZ = extractSignedLong(msg, 20);
        packetUBXNAVSVIN->data.meanXHP = extractSignedChar(msg, 24);
        packetUBXNAVSVIN->data.meanYHP = extractSignedChar(msg, 25);
        packetUBXNAVSVIN->data.meanZHP = extractSignedChar(msg, 26);
        packetUBXNAVSVIN->data.meanAcc = extractLong(msg, 28);
        packetUBXNAVSVIN->data.obs = extractLong(msg, 32);
        packetUBXNAVSVIN->data.valid = extractSignedChar(msg, 36);
        packetUBXNAVSVIN->data.active = extractSignedChar(msg, 37);

        // Mark all datums as fresh (not read before)
        packetUBXNAVSVIN->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSVIN->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSVIN->callbackData->version, &packetUBXNAVSVIN->data.version, sizeof(UBX_NAV_SVIN_data_t));
          packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSVIN->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
    else if (msg->id == UBX_NAV_SAT) // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSAT != nullptr)
      {
        packetUBXNAVSAT->data.header.iTOW = extractLong(msg, 0);
        packetUBXNAVSAT->data.header.version = extractByte(msg, 4);
        packetUBXNAVSAT->data.header.numSvs = extractByte(msg, 5);

        // The NAV SAT message could contain data for 255 SVs max. (numSvs is uint8_t. UBX_NAV_SAT_MAX_BLOCKS is 255)
        for (uint16_t i = 0; (i < UBX_NAV_SAT_MAX_BLOCKS) && (i < ((uint16_t)packetUBXNAVSAT->data.header.numSvs)) && ((i * 12) < (msg->len - 8)); i++)
        {
          uint16_t offset = (i * 12) + 8;
          packetUBXNAVSAT->data.blocks[i].gnssId = extractByte(msg, offset + 0);
          packetUBXNAVSAT->data.blocks[i].svId = extractByte(msg, offset + 1);
          packetUBXNAVSAT->data.blocks[i].cno = extractByte(msg, offset + 2);
          packetUBXNAVSAT->data.blocks[i].elev = extractSignedChar(msg, offset + 3);
          packetUBXNAVSAT->data.blocks[i].azim = extractSignedInt(msg, offset + 4);
          packetUBXNAVSAT->data.blocks[i].prRes = extractSignedInt(msg, offset + 6);
          packetUBXNAVSAT->data.blocks[i].flags.all = extractLong(msg, offset + 8);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVSAT->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSAT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSAT->callbackData->header.iTOW, &packetUBXNAVSAT->data.header.iTOW, sizeof(UBX_NAV_SAT_data_t));
          packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSAT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_SIG) // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSIG != nullptr)
      {
        packetUBXNAVSIG->data.header.iTOW = extractLong(msg, 0);
        packetUBXNAVSIG->data.header.version = extractByte(msg, 4);
        packetUBXNAVSIG->data.header.numSigs = extractByte(msg, 5);

        // The NAV SIG message could potentially contain data for 255 signals. (numSigs is uint8_t. UBX_NAV_SIG_MAX_BLOCKS is 92)
        for (uint16_t i = 0; (i < UBX_NAV_SIG_MAX_BLOCKS) && (i < ((uint16_t)packetUBXNAVSIG->data.header.numSigs)) && ((i * 16) < (msg->len - 8)); i++)
        {
          uint16_t offset = (i * 16) + 8;
          packetUBXNAVSIG->data.blocks[i].gnssId = extractByte(msg, offset + 0);
          packetUBXNAVSIG->data.blocks[i].svId = extractByte(msg, offset + 1);
          packetUBXNAVSIG->data.blocks[i].sigId = extractByte(msg, offset + 2);
          packetUBXNAVSIG->data.blocks[i].freqId = extractByte(msg, offset + 3);
          packetUBXNAVSIG->data.blocks[i].prRes = extractSignedInt(msg, offset + 4);
          packetUBXNAVSIG->data.blocks[i].cno = extractByte(msg, offset + 6);
          packetUBXNAVSIG->data.blocks[i].qualityInd = extractByte(msg, offset + 7);
          packetUBXNAVSIG->data.blocks[i].corrSource = extractByte(msg, offset + 8);
          packetUBXNAVSIG->data.blocks[i].ionoModel = extractByte(msg, offset + 9);
          packetUBXNAVSIG->data.blocks[i].sigFlags.all = extractInt(msg, offset + 10);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVSIG->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSIG->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSIG->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSIG->callbackData->header.iTOW, &packetUBXNAVSIG->data.header.iTOW, sizeof(UBX_NAV_SIG_data_t));
          packetUBXNAVSIG->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSIG->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
#endif
    else if (msg->id == UBX_NAV_RELPOSNED && ((msg->len == UBX_NAV_RELPOSNED_LEN) || (msg->len == UBX_NAV_RELPOSNED_LEN_F9)))
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVRELPOSNED != nullptr)
      {
        // Note:
        //   RELPOSNED on the M8 is only 40 bytes long
        //   RELPOSNED on the F9 is 64 bytes long and contains much more information

        packetUBXNAVRELPOSNED->data.version = extractByte(msg, 0);
        packetUBXNAVRELPOSNED->data.refStationId = extractInt(msg, 2);
        packetUBXNAVRELPOSNED->data.iTOW = extractLong(msg, 4);
        packetUBXNAVRELPOSNED->data.relPosN = extractSignedLong(msg, 8);
        packetUBXNAVRELPOSNED->data.relPosE = extractSignedLong(msg, 12);
        packetUBXNAVRELPOSNED->data.relPosD = extractSignedLong(msg, 16);

        if (msg->len == UBX_NAV_RELPOSNED_LEN)
        {
          // The M8 version does not contain relPosLength or relPosHeading
          packetUBXNAVRELPOSNED->data.relPosLength = 0;
          packetUBXNAVRELPOSNED->data.relPosHeading = 0;
          packetUBXNAVRELPOSNED->data.relPosHPN = extractSignedChar(msg, 20);
          packetUBXNAVRELPOSNED->data.relPosHPE = extractSignedChar(msg, 21);
          packetUBXNAVRELPOSNED->data.relPosHPD = extractSignedChar(msg, 22);
          packetUBXNAVRELPOSNED->data.relPosHPLength = 0; // The M8 version does not contain relPosHPLength
          packetUBXNAVRELPOSNED->data.accN = extractLong(msg, 24);
          packetUBXNAVRELPOSNED->data.accE = extractLong(msg, 28);
          packetUBXNAVRELPOSNED->data.accD = extractLong(msg, 32);
          // The M8 version does not contain accLength or accHeading
          packetUBXNAVRELPOSNED->data.accLength = 0;
          packetUBXNAVRELPOSNED->data.accHeading = 0;
          packetUBXNAVRELPOSNED->data.flags.all = extractLong(msg, 36);
        }
        else
        {
          packetUBXNAVRELPOSNED->data.relPosLength = extractSignedLong(msg, 20);
          packetUBXNAVRELPOSNED->data.relPosHeading = extractSignedLong(msg, 24);
          packetUBXNAVRELPOSNED->data.relPosHPN = extractSignedChar(msg, 32);
          packetUBXNAVRELPOSNED->data.relPosHPE = extractSignedChar(msg, 33);
          packetUBXNAVRELPOSNED->data.relPosHPD = extractSignedChar(msg, 34);
          packetUBXNAVRELPOSNED->data.relPosHPLength = extractSignedChar(msg, 35);
          packetUBXNAVRELPOSNED->data.accN = extractLong(msg, 36);
          packetUBXNAVRELPOSNED->data.accE = extractLong(msg, 40);
          packetUBXNAVRELPOSNED->data.accD = extractLong(msg, 44);
          packetUBXNAVRELPOSNED->data.accLength = extractLong(msg, 48);
          packetUBXNAVRELPOSNED->data.accHeading = extractLong(msg, 52);
          packetUBXNAVRELPOSNED->data.flags.all = extractLong(msg, 60);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVRELPOSNED->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVRELPOSNED->callbackData->version, &packetUBXNAVRELPOSNED->data.version, sizeof(UBX_NAV_RELPOSNED_data_t));
          packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_AOPSTATUS && msg->len == UBX_NAV_AOPSTATUS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVAOPSTATUS != nullptr)
      {
        packetUBXNAVAOPSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVAOPSTATUS->data.aopCfg.all = extractByte(msg, 4);
        packetUBXNAVAOPSTATUS->data.status = extractByte(msg, 5);

        // Mark all datums as fresh (not read before)
        packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVAOPSTATUS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVAOPSTATUS->callbackData->iTOW, &packetUBXNAVAOPSTATUS->data.iTOW, sizeof(UBX_NAV_AOPSTATUS_data_t));
          packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_EOE && msg->len == UBX_NAV_EOE_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVEOE != nullptr)
      {
        packetUBXNAVEOE->data.iTOW = extractLong(msg, 0);

        // Mark all datums as fresh (not read before)
        packetUBXNAVEOE->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVEOE->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVEOE->callbackData->iTOW, &packetUBXNAVEOE->data.iTOW, sizeof(UBX_NAV_EOE_data_t));
          packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVEOE->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  case UBX_CLASS_RXM:
    if (msg->id == UBX_RXM_PMP)
    // Note: length is variable with version 0x01
    // Note: the field positions depend on the version
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it.
      // By default, new PMP data will always overwrite 'old' data (data which is valid but which has not yet been read by the callback).
      // To prevent this, uncomment the line two lines below
      if ((packetUBXRXMPMP != nullptr) && (packetUBXRXMPMP->callbackData != nullptr)
          //&& (packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMPMP->callbackData->version = extractByte(msg, 0);
        packetUBXRXMPMP->callbackData->numBytesUserData = extractInt(msg, 2);
        packetUBXRXMPMP->callbackData->timeTag = extractLong(msg, 4);
        packetUBXRXMPMP->callbackData->uniqueWord[0] = extractLong(msg, 8);
        packetUBXRXMPMP->callbackData->uniqueWord[1] = extractLong(msg, 12);
        packetUBXRXMPMP->callbackData->serviceIdentifier = extractInt(msg, 16);
        packetUBXRXMPMP->callbackData->spare = extractByte(msg, 18);
        packetUBXRXMPMP->callbackData->uniqueWordBitErrors = extractByte(msg, 19);

        if (packetUBXRXMPMP->callbackData->version == 0x00)
        {
          packetUBXRXMPMP->callbackData->fecBits = extractInt(msg, 524);
          packetUBXRXMPMP->callbackData->ebno = extractByte(msg, 526);
        }
        else // if (packetUBXRXMPMP->data.version == 0x01)
        {
          packetUBXRXMPMP->callbackData->fecBits = extractInt(msg, 20);
          packetUBXRXMPMP->callbackData->ebno = extractByte(msg, 22);
        }

        uint16_t userDataStart = (packetUBXRXMPMP->callbackData->version == 0x00) ? 20 : 24;
        uint16_t userDataLength = (packetUBXRXMPMP->callbackData->version == 0x00) ? 504 : (packetUBXRXMPMP->callbackData->numBytesUserData);
        for (uint16_t i = 0; (i < userDataLength) && (i < 504); i++)
        {
          packetUBXRXMPMP->callbackData->userData[i] = extractByte(msg, i + userDataStart);
        }

        packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }

      // Full PMP message, including Class, ID and checksum
      // By default, new PMP data will always overwrite 'old' data (data which is valid but which has not yet been read by the callback).
      // To prevent this, uncomment the line two lines below
      if ((packetUBXRXMPMPmessage != nullptr) && (packetUBXRXMPMPmessage->callbackData != nullptr)
          //&& (packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMPMPmessage->callbackData->sync1 = UBX_SYNCH_1;
        packetUBXRXMPMPmessage->callbackData->sync2 = UBX_SYNCH_2;
        packetUBXRXMPMPmessage->callbackData->cls = UBX_CLASS_RXM;
        packetUBXRXMPMPmessage->callbackData->ID = UBX_RXM_PMP;
        packetUBXRXMPMPmessage->callbackData->lengthLSB = msg->len & 0xFF;
        packetUBXRXMPMPmessage->callbackData->lengthMSB = msg->len >> 8;

        memcpy(packetUBXRXMPMPmessage->callbackData->payload, msg->payload, msg->len);

        packetUBXRXMPMPmessage->callbackData->checksumA = msg->checksumA;
        packetUBXRXMPMPmessage->callbackData->checksumB = msg->checksumB;

        packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }
    }
    else if (msg->id == UBX_RXM_QZSSL6)
    // Note: length is variable with version 0x01
    // Note: the field positions depend on the version
    {
      // Full QZSSL6 message, including Class, ID and checksum
      for (int ch = 0; ch < UBX_RXM_QZSSL6_NUM_CHANNELS; ch++)
      {
        if (0 == (packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid & (1 << ch)))
        {

          packetUBXRXMQZSSL6message->callbackData[ch].sync1 = UBX_SYNCH_1;
          packetUBXRXMQZSSL6message->callbackData[ch].sync2 = UBX_SYNCH_2;
          packetUBXRXMQZSSL6message->callbackData[ch].cls = UBX_CLASS_RXM;
          packetUBXRXMQZSSL6message->callbackData[ch].ID = UBX_RXM_QZSSL6;
          packetUBXRXMQZSSL6message->callbackData[ch].lengthLSB = msg->len & 0xFF;
          packetUBXRXMQZSSL6message->callbackData[ch].lengthMSB = msg->len >> 8;

          memcpy(packetUBXRXMQZSSL6message->callbackData[ch].payload, msg->payload, msg->len);

          packetUBXRXMQZSSL6message->callbackData[ch].checksumA = msg->checksumA;
          packetUBXRXMQZSSL6message->callbackData[ch].checksumB = msg->checksumB;

          packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid |= (1 << ch);
          break; // abort when added
        }
      }
    }
    else if (msg->id == UBX_RXM_COR)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if ((packetUBXRXMCOR != nullptr) && (packetUBXRXMCOR->callbackData != nullptr)
          //&& (packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMCOR->callbackData->version = extractByte(msg, 0);
        packetUBXRXMCOR->callbackData->ebno = extractByte(msg, 1);
        packetUBXRXMCOR->callbackData->statusInfo.all = extractLong(msg, 4);
        packetUBXRXMCOR->callbackData->msgType = extractInt(msg, 8);
        packetUBXRXMCOR->callbackData->msgSubType = extractInt(msg, 10);

        packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }
    }
    else if (msg->id == UBX_RXM_SFRBX)
    // Note: length is variable
    // Note: on protocol version 17: numWords is (0..16)
    //       on protocol version 18+: numWords is (0..10)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMSFRBX != nullptr)
      {
        packetUBXRXMSFRBX->data.gnssId = extractByte(msg, 0);
        packetUBXRXMSFRBX->data.svId = extractByte(msg, 1);
        packetUBXRXMSFRBX->data.freqId = extractByte(msg, 3);
        packetUBXRXMSFRBX->data.numWords = extractByte(msg, 4);
        packetUBXRXMSFRBX->data.chn = extractByte(msg, 5);
        packetUBXRXMSFRBX->data.version = extractByte(msg, 6);

        for (uint8_t i = 0; (i < UBX_RXM_SFRBX_MAX_WORDS) && (i < packetUBXRXMSFRBX->data.numWords) && ((i * 4) < (msg->len - 8)); i++)
        {
          packetUBXRXMSFRBX->data.dwrd[i] = extractLong(msg, 8 + (i * 4));
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMSFRBX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if (packetUBXRXMSFRBX->callbackData != nullptr) // If RAM has been allocated for the copies of the data
        {
          for (uint32_t i = 0; i < UBX_RXM_SFRBX_CALLBACK_BUFFERS; i++) // Check all available buffers
          {
            if ((packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid & (1 << i)) == 0) // AND the buffer is empty
            {
              memcpy(&packetUBXRXMSFRBX->callbackData[i].gnssId, &packetUBXRXMSFRBX->data.gnssId, sizeof(UBX_RXM_SFRBX_data_t));
              packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid |= (1 << i);
              break; // Only copy once - into first available buffer
            }
          }
        }

        // Check if we need to copy the data for the message callbacks
        if (packetUBXRXMSFRBX->callbackMessageData != nullptr) // If RAM has been allocated for the copies of the data
        {
          for (uint32_t i = 0; i < UBX_RXM_SFRBX_CALLBACK_BUFFERS; i++) // Check all available buffers
          {
            if ((packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackMessageCopyValid & (1 << i)) == 0) // AND the buffer is empty
            {
              packetUBXRXMSFRBX->callbackMessageData[i].sync1 = UBX_SYNCH_1;
              packetUBXRXMSFRBX->callbackMessageData[i].sync2 = UBX_SYNCH_2;
              packetUBXRXMSFRBX->callbackMessageData[i].cls = UBX_CLASS_RXM;
              packetUBXRXMSFRBX->callbackMessageData[i].ID = UBX_RXM_SFRBX;
              packetUBXRXMSFRBX->callbackMessageData[i].lengthLSB = msg->len & 0xFF;
              packetUBXRXMSFRBX->callbackMessageData[i].lengthMSB = msg->len >> 8;

              memcpy(&packetUBXRXMSFRBX->callbackMessageData[i].payload, msg->payload, msg->len);

              packetUBXRXMSFRBX->callbackMessageData[i].checksumA = msg->checksumA;
              packetUBXRXMSFRBX->callbackMessageData[i].checksumB = msg->checksumB;

              packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackMessageCopyValid |= (1 << i);
              break; // Only copy once - into first available buffer
            }
          }
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMSFRBX->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_RXM_RAWX)
    // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMRAWX != nullptr)
      {
        for (uint8_t i = 0; i < 8; i++)
        {
          packetUBXRXMRAWX->data.header.rcvTow[i] = extractByte(msg, i);
        }
        packetUBXRXMRAWX->data.header.week = extractInt(msg, 8);
        packetUBXRXMRAWX->data.header.leapS = extractSignedChar(msg, 10);
        packetUBXRXMRAWX->data.header.numMeas = extractByte(msg, 11);
        packetUBXRXMRAWX->data.header.recStat.all = extractByte(msg, 12);
        packetUBXRXMRAWX->data.header.version = extractByte(msg, 13);

        for (uint8_t i = 0; (i < UBX_RXM_RAWX_MAX_BLOCKS) && (i < packetUBXRXMRAWX->data.header.numMeas) && ((((uint16_t)i) * 32) < (msg->len - 16)); i++)
        {
          uint16_t offset = (((uint16_t)i) * 32) + 16;
          for (uint8_t j = 0; j < 8; j++)
          {
            packetUBXRXMRAWX->data.blocks[i].prMes[j] = extractByte(msg, offset + j);
            packetUBXRXMRAWX->data.blocks[i].cpMes[j] = extractByte(msg, offset + 8 + j);
            if (j < 4)
              packetUBXRXMRAWX->data.blocks[i].doMes[j] = extractByte(msg, offset + 16 + j);
          }
          packetUBXRXMRAWX->data.blocks[i].gnssId = extractByte(msg, offset + 20);
          packetUBXRXMRAWX->data.blocks[i].svId = extractByte(msg, offset + 21);
          packetUBXRXMRAWX->data.blocks[i].sigId = extractByte(msg, offset + 22);
          packetUBXRXMRAWX->data.blocks[i].freqId = extractByte(msg, offset + 23);
          packetUBXRXMRAWX->data.blocks[i].lockTime = extractInt(msg, offset + 24);
          packetUBXRXMRAWX->data.blocks[i].cno = extractByte(msg, offset + 26);
          packetUBXRXMRAWX->data.blocks[i].prStdev = extractByte(msg, offset + 27);
          packetUBXRXMRAWX->data.blocks[i].cpStdev = extractByte(msg, offset + 28);
          packetUBXRXMRAWX->data.blocks[i].doStdev = extractByte(msg, offset + 29);
          packetUBXRXMRAWX->data.blocks[i].trkStat.all = extractByte(msg, offset + 30);
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMRAWX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXRXMRAWX->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXRXMRAWX->callbackData->header.rcvTow[0], &packetUBXRXMRAWX->data.header.rcvTow[0], sizeof(UBX_RXM_RAWX_data_t));
          packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMRAWX->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_RXM_MEASX)
    // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMMEASX != nullptr)
      {
        packetUBXRXMMEASX->data.header.version = extractByte(msg, 0);
        packetUBXRXMMEASX->data.header.gpsTOW = extractLong(msg, 4);
        packetUBXRXMMEASX->data.header.gloTOW = extractLong(msg, 8);
        packetUBXRXMMEASX->data.header.bdsTOW = extractLong(msg, 12);
        packetUBXRXMMEASX->data.header.qzssTOW = extractLong(msg, 20);
        packetUBXRXMMEASX->data.header.gpsTOWacc = extractInt(msg, 24);
        packetUBXRXMMEASX->data.header.gloTOWacc = extractInt(msg, 26);
        packetUBXRXMMEASX->data.header.bdsTOWacc = extractInt(msg, 28);
        packetUBXRXMMEASX->data.header.qzssTOWacc = extractInt(msg, 32);
        packetUBXRXMMEASX->data.header.numSV = extractByte(msg, 34);
        packetUBXRXMMEASX->data.header.flags.all = extractByte(msg, 35);

        for (uint8_t i = 0; (i < UBX_RXM_MEASX_MAX_BLOCKS) && (i < packetUBXRXMMEASX->data.header.numSV) && ((((uint16_t)i) * 24) < (msg->len - 44)); i++)
        {
          uint16_t offset = (((uint16_t)i) * 24) + 44;
          packetUBXRXMMEASX->data.blocks[i].gnssId = extractByte(msg, offset + 0);
          packetUBXRXMMEASX->data.blocks[i].svId = extractByte(msg, offset + 1);
          packetUBXRXMMEASX->data.blocks[i].cNo = extractByte(msg, offset + 2);
          packetUBXRXMMEASX->data.blocks[i].mpathIndic = extractByte(msg, offset + 3);
          packetUBXRXMMEASX->data.blocks[i].dopplerMS = extractSignedLong(msg, offset + 4);
          packetUBXRXMMEASX->data.blocks[i].dopplerHz = extractSignedLong(msg, offset + 8);
          packetUBXRXMMEASX->data.blocks[i].wholeChips = extractInt(msg, offset + 12);
          packetUBXRXMMEASX->data.blocks[i].fracChips = extractInt(msg, offset + 14);
          packetUBXRXMMEASX->data.blocks[i].codePhase = extractLong(msg, offset + 16);
          packetUBXRXMMEASX->data.blocks[i].intCodePhase = extractByte(msg, offset + 20);
          packetUBXRXMMEASX->data.blocks[i].pseuRangeRMSErr = extractByte(msg, offset + 21);
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMMEASX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXRXMMEASX->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXRXMMEASX->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXRXMMEASX->callbackData->header.version, &packetUBXRXMMEASX->data.header.version, sizeof(UBX_RXM_MEASX_data_t));
          packetUBXRXMMEASX->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMMEASX->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
    break;
#endif
  case UBX_CLASS_TIM:
    if (msg->id == UBX_TIM_TM2 && msg->len == UBX_TIM_TM2_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXTIMTM2 != nullptr)
      {
        packetUBXTIMTM2->data.ch = extractByte(msg, 0);
        packetUBXTIMTM2->data.flags.all = extractByte(msg, 1);
        packetUBXTIMTM2->data.count = extractInt(msg, 2);
        packetUBXTIMTM2->data.wnR = extractInt(msg, 4);
        packetUBXTIMTM2->data.wnF = extractInt(msg, 6);
        packetUBXTIMTM2->data.towMsR = extractLong(msg, 8);
        packetUBXTIMTM2->data.towSubMsR = extractLong(msg, 12);
        packetUBXTIMTM2->data.towMsF = extractLong(msg, 16);
        packetUBXTIMTM2->data.towSubMsF = extractLong(msg, 20);
        packetUBXTIMTM2->data.accEst = extractLong(msg, 24);

        // Mark all datums as fresh (not read before)
        packetUBXTIMTM2->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXTIMTM2->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXTIMTM2->callbackData->ch, &packetUBXTIMTM2->data.ch, sizeof(UBX_TIM_TM2_data_t));
          packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXTIMTM2->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_TIM_TP && msg->len == UBX_TIM_TP_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXTIMTP != nullptr)
      {
        packetUBXTIMTP->data.towMS = extractLong(msg, 0);
        packetUBXTIMTP->data.towSubMS = extractLong(msg, 4);
        packetUBXTIMTP->data.qErr = extractSignedLong(msg, 8);
        packetUBXTIMTP->data.week = extractInt(msg, 12);
        packetUBXTIMTP->data.flags.all = extractByte(msg, 14);
        packetUBXTIMTP->data.refInfo.all = extractByte(msg, 15);

        // Mark all datums as fresh (not read before)
        packetUBXTIMTP->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXTIMTP->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXTIMTP->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXTIMTP->callbackData->towMS, &packetUBXTIMTP->data.towMS, sizeof(UBX_TIM_TP_data_t));
          packetUBXTIMTP->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXTIMTP->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_MON:
    if (msg->id == UBX_MON_HW && msg->len == UBX_MON_HW_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMONHW != nullptr)
      {
        packetUBXMONHW->data.pinSel = extractLong(msg, 0);
        packetUBXMONHW->data.pinBank = extractLong(msg, 4);
        packetUBXMONHW->data.pinDir = extractLong(msg, 8);
        packetUBXMONHW->data.pinVal = extractLong(msg, 12);
        packetUBXMONHW->data.noisePerMS = extractInt(msg, 16);
        packetUBXMONHW->data.agcCnt = extractInt(msg, 18);
        packetUBXMONHW->data.aStatus = extractByte(msg, 20);
        packetUBXMONHW->data.aPower = extractByte(msg, 21);
        packetUBXMONHW->data.flags.all = extractByte(msg, 22);
        packetUBXMONHW->data.usedMask = extractLong(msg, 24);
        for (uint8_t i = 0; i < 17; i++)
          packetUBXMONHW->data.VP[i] = extractByte(msg, 28 + i);
        packetUBXMONHW->data.jamInd = extractByte(msg, 45);
        packetUBXMONHW->data.pinIrq = extractLong(msg, 48);
        packetUBXMONHW->data.pullH = extractLong(msg, 52);
        packetUBXMONHW->data.pullL = extractLong(msg, 56);

        // Mark all datums as fresh (not read before)
        packetUBXMONHW->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXMONHW->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXMONHW->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXMONHW->callbackData->pinSel, &packetUBXMONHW->data.pinSel, sizeof(UBX_MON_HW_data_t));
          packetUBXMONHW->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXMONHW->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
#ifndef SFE_UBLOX_DISABLE_ESF
  case UBX_CLASS_ESF:
    if (msg->id == UBX_ESF_ALG && msg->len == UBX_ESF_ALG_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFALG != nullptr)
      {
        packetUBXESFALG->data.iTOW = extractLong(msg, 0);
        packetUBXESFALG->data.version = extractByte(msg, 4);
        packetUBXESFALG->data.flags.all = extractByte(msg, 5);
        packetUBXESFALG->data.error.all = extractByte(msg, 6);
        packetUBXESFALG->data.yaw = extractLong(msg, 8);
        packetUBXESFALG->data.pitch = extractSignedInt(msg, 12);
        packetUBXESFALG->data.roll = extractSignedInt(msg, 14);

        // Mark all datums as fresh (not read before)
        packetUBXESFALG->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFALG->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFALG->callbackData->iTOW, &packetUBXESFALG->data.iTOW, sizeof(UBX_ESF_ALG_data_t));
          packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFALG->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_INS && msg->len == UBX_ESF_INS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFINS != nullptr)
      {
        packetUBXESFINS->data.bitfield0.all = extractLong(msg, 0);
        packetUBXESFINS->data.iTOW = extractLong(msg, 8);
        packetUBXESFINS->data.xAngRate = extractSignedLong(msg, 12);
        packetUBXESFINS->data.yAngRate = extractSignedLong(msg, 16);
        packetUBXESFINS->data.zAngRate = extractSignedLong(msg, 20);
        packetUBXESFINS->data.xAccel = extractSignedLong(msg, 24);
        packetUBXESFINS->data.yAccel = extractSignedLong(msg, 28);
        packetUBXESFINS->data.zAccel = extractSignedLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXESFINS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFINS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFINS->callbackData->bitfield0.all, &packetUBXESFINS->data.bitfield0.all, sizeof(UBX_ESF_INS_data_t));
          packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFINS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_MEAS)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFMEAS != nullptr)
      {
        packetUBXESFMEAS->data.timeTag = extractLong(msg, 0);
        packetUBXESFMEAS->data.flags.all = extractInt(msg, 4);
        packetUBXESFMEAS->data.id = extractInt(msg, 6);
        for (uint16_t i = 0; (i < DEF_MAX_NUM_ESF_MEAS) && (i < packetUBXESFMEAS->data.flags.bits.numMeas) && ((i * 4) < (msg->len - 8)); i++)
        {
          packetUBXESFMEAS->data.data[i].data.all = extractLong(msg, 8 + (i * 4));
        }
        if ((uint16_t)msg->len > (uint16_t)(8 + (packetUBXESFMEAS->data.flags.bits.numMeas * 4)))
          packetUBXESFMEAS->data.calibTtag = extractLong(msg, 8 + (packetUBXESFMEAS->data.flags.bits.numMeas * 4));

        // Check if we need to copy the data for the callback
        if (packetUBXESFMEAS->callbackData != nullptr) // If RAM has been allocated for the copy of the data
        {
          for (uint16_t i = 0; i < UBX_ESF_MEAS_CALLBACK_BUFFERS; i++)
          {
            if ((packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid & (1 << i)) == 0) // AND the data is stale
            {
              memcpy(&packetUBXESFMEAS->callbackData[i].timeTag, &packetUBXESFMEAS->data.timeTag, sizeof(UBX_ESF_MEAS_data_t));
              packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid |= (1 << i);
              break; // Only copy once
            }
          }
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFMEAS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_RAW)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFRAW != nullptr)
      {
        packetUBXESFRAW->data.numEsfRawBlocks = (msg->len - 4) / 8; // Record how many blocks were received. Could be 7 or 70 (ZED-F9R vs. NEO-M8U)
        for (uint16_t i = 0; (i < (DEF_NUM_SENS * DEF_MAX_NUM_ESF_RAW_REPEATS)) && ((i * 8) < (msg->len - 4)); i++)
        {
          packetUBXESFRAW->data.data[i].data.all = extractLong(msg, 4 + (i * 8));
          packetUBXESFRAW->data.data[i].sTag = extractLong(msg, 8 + (i * 8));
        }

        // Check if we need to copy the data for the callback
        if ((packetUBXESFRAW->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFRAW->callbackData->data[0].data.all, &packetUBXESFRAW->data.data[0].data.all, sizeof(UBX_ESF_RAW_data_t));
          packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFRAW->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_STATUS)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFSTATUS != nullptr)
      {
        packetUBXESFSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXESFSTATUS->data.version = extractByte(msg, 4);
        packetUBXESFSTATUS->data.fusionMode = extractByte(msg, 12);
        packetUBXESFSTATUS->data.numSens = extractByte(msg, 15);
        for (uint16_t i = 0; (i < DEF_NUM_SENS) && (i < packetUBXESFSTATUS->data.numSens) && ((i * 4) < (msg->len - 16)); i++)
        {
          packetUBXESFSTATUS->data.status[i].sensStatus1.all = extractByte(msg, 16 + (i * 4) + 0);
          packetUBXESFSTATUS->data.status[i].sensStatus2.all = extractByte(msg, 16 + (i * 4) + 1);
          packetUBXESFSTATUS->data.status[i].freq = extractByte(msg, 16 + (i * 4) + 2);
          packetUBXESFSTATUS->data.status[i].faults.all = extractByte(msg, 16 + (i * 4) + 3);
        }

        // Mark all datums as fresh (not read before)
        packetUBXESFSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFSTATUS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFSTATUS->callbackData->iTOW, &packetUBXESFSTATUS->data.iTOW, sizeof(UBX_ESF_STATUS_data_t));
          packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
#endif
  case UBX_CLASS_MGA:
    if (msg->id == UBX_MGA_ACK_DATA0 && msg->len == UBX_MGA_ACK_DATA0_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMGAACK != nullptr)
      {
        // Calculate how many ACKs are already stored in the ring buffer
        uint8_t ackBufferContains;
        if (packetUBXMGAACK->head >= packetUBXMGAACK->tail) // Check if wrap-around has occurred
        {
          // Wrap-around has not occurred so do a simple subtraction
          ackBufferContains = packetUBXMGAACK->head - packetUBXMGAACK->tail;
        }
        else
        {
          // Wrap-around has occurred so do a simple subtraction but add in the buffer length (UBX_MGA_ACK_RINGBUFFER_LEN)
          ackBufferContains = ((uint8_t)(((uint16_t)packetUBXMGAACK->head + (uint16_t)UBX_MGA_ACK_DATA0_RINGBUFFER_LEN) - (uint16_t)packetUBXMGAACK->tail));
        }
        // Have we got space to store this ACK?
        if (ackBufferContains < (UBX_MGA_ACK_DATA0_RINGBUFFER_LEN - 1))
        {
          // Yes, we have, so store it
          packetUBXMGAACK->data[packetUBXMGAACK->head].type = extractByte(msg, 0);
          packetUBXMGAACK->data[packetUBXMGAACK->head].version = extractByte(msg, 1);
          packetUBXMGAACK->data[packetUBXMGAACK->head].infoCode = extractByte(msg, 2);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgId = extractByte(msg, 3);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[0] = extractByte(msg, 4);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[1] = extractByte(msg, 5);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[2] = extractByte(msg, 6);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[3] = extractByte(msg, 7);
          // Increment the head
          packetUBXMGAACK->head++;
          if (packetUBXMGAACK->head == UBX_MGA_ACK_DATA0_RINGBUFFER_LEN)
            packetUBXMGAACK->head = 0;
        }
        else
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial.println(F("processUBXpacket: packetUBXMGAACK is full. ACK will be lost!"));
          }
#endif
        }
      }
    }
    else if (msg->id == UBX_MGA_DBD && msg->len <= UBX_MGA_DBD_LEN) // Message length may be less than UBX_MGA_DBD_LEN. UBX_MGA_DBD_LEN is the maximum it will be.
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMGADBD != nullptr)
      {
        // Calculate how many DBDs are already stored in the ring buffer
        uint8_t dbdBufferContains;
        if (packetUBXMGADBD->head >= packetUBXMGADBD->tail) // Check if wrap-around has occurred
        {
          // Wrap-around has not occurred so do a simple subtraction
          dbdBufferContains = packetUBXMGADBD->head - packetUBXMGADBD->tail;
        }
        else
        {
          // Wrap-around has occurred so do a simple subtraction but add in the buffer length (UBX_MGA_DBD_RINGBUFFER_LEN)
          dbdBufferContains = ((uint8_t)(((uint16_t)packetUBXMGADBD->head + (uint16_t)UBX_MGA_DBD_RINGBUFFER_LEN) - (uint16_t)packetUBXMGADBD->tail));
        }
        // Have we got space to store this DBD?
        if (dbdBufferContains < (UBX_MGA_DBD_RINGBUFFER_LEN - 1))
        {
          // Yes, we have, so store it
          // We need to save the entire message - header, payload and checksum
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryHeader1 = UBX_SYNCH_1;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryHeader2 = UBX_SYNCH_2;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryClass = UBX_CLASS_MGA;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryID = UBX_MGA_DBD;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryLenLSB = (uint8_t)(msg->len & 0xFF); // We need to store the length of the DBD entry. The entry itself does not contain a length...
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryLenMSB = (uint8_t)((msg->len >> 8) & 0xFF);
          for (uint16_t i = 0; i < msg->len; i++)
          {
            packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntry[i] = extractByte(msg, i);
          }
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryChecksumA = msg->checksumA;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryChecksumB = msg->checksumB;
          // Increment the head
          packetUBXMGADBD->head++;
          if (packetUBXMGADBD->head == UBX_MGA_DBD_RINGBUFFER_LEN)
            packetUBXMGADBD->head = 0;
        }
        else
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial.println(F("processUBXpacket: packetUBXMGADBD is full. DBD data will be lost!"));
          }
#endif
        }
      }
    }
    break;
#ifndef SFE_UBLOX_DISABLE_HNR
  case UBX_CLASS_HNR:
    if (msg->id == UBX_HNR_PVT && msg->len == UBX_HNR_PVT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRPVT != nullptr)
      {
        packetUBXHNRPVT->data.iTOW = extractLong(msg, 0);
        packetUBXHNRPVT->data.year = extractInt(msg, 4);
        packetUBXHNRPVT->data.month = extractByte(msg, 6);
        packetUBXHNRPVT->data.day = extractByte(msg, 7);
        packetUBXHNRPVT->data.hour = extractByte(msg, 8);
        packetUBXHNRPVT->data.min = extractByte(msg, 9);
        packetUBXHNRPVT->data.sec = extractByte(msg, 10);
        packetUBXHNRPVT->data.valid.all = extractByte(msg, 11);
        packetUBXHNRPVT->data.nano = extractSignedLong(msg, 12);
        packetUBXHNRPVT->data.gpsFix = extractByte(msg, 16);
        packetUBXHNRPVT->data.flags.all = extractByte(msg, 17);
        packetUBXHNRPVT->data.lon = extractSignedLong(msg, 20);
        packetUBXHNRPVT->data.lat = extractSignedLong(msg, 24);
        packetUBXHNRPVT->data.height = extractSignedLong(msg, 28);
        packetUBXHNRPVT->data.hMSL = extractSignedLong(msg, 32);
        packetUBXHNRPVT->data.gSpeed = extractSignedLong(msg, 36);
        packetUBXHNRPVT->data.speed = extractSignedLong(msg, 40);
        packetUBXHNRPVT->data.headMot = extractSignedLong(msg, 44);
        packetUBXHNRPVT->data.headVeh = extractSignedLong(msg, 48);
        packetUBXHNRPVT->data.hAcc = extractLong(msg, 52);
        packetUBXHNRPVT->data.vAcc = extractLong(msg, 56);
        packetUBXHNRPVT->data.sAcc = extractLong(msg, 60);
        packetUBXHNRPVT->data.headAcc = extractLong(msg, 64);

        // Mark all datums as fresh (not read before)
        packetUBXHNRPVT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRPVT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRPVT->callbackData->iTOW, &packetUBXHNRPVT->data.iTOW, sizeof(UBX_HNR_PVT_data_t));
          packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRPVT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_HNR_ATT && msg->len == UBX_HNR_ATT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRATT != nullptr)
      {
        packetUBXHNRATT->data.iTOW = extractLong(msg, 0);
        packetUBXHNRATT->data.version = extractByte(msg, 4);
        packetUBXHNRATT->data.roll = extractSignedLong(msg, 8);
        packetUBXHNRATT->data.pitch = extractSignedLong(msg, 12);
        packetUBXHNRATT->data.heading = extractSignedLong(msg, 16);
        packetUBXHNRATT->data.accRoll = extractLong(msg, 20);
        packetUBXHNRATT->data.accPitch = extractLong(msg, 24);
        packetUBXHNRATT->data.accHeading = extractLong(msg, 28);

        // Mark all datums as fresh (not read before)
        packetUBXHNRATT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRATT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRATT->callbackData->iTOW, &packetUBXHNRATT->data.iTOW, sizeof(UBX_HNR_ATT_data_t));
          packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRATT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_HNR_INS && msg->len == UBX_HNR_INS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRINS != nullptr)
      {
        packetUBXHNRINS->data.bitfield0.all = extractLong(msg, 0);
        packetUBXHNRINS->data.iTOW = extractLong(msg, 8);
        packetUBXHNRINS->data.xAngRate = extractSignedLong(msg, 12);
        packetUBXHNRINS->data.yAngRate = extractSignedLong(msg, 16);
        packetUBXHNRINS->data.zAngRate = extractSignedLong(msg, 20);
        packetUBXHNRINS->data.xAccel = extractSignedLong(msg, 24);
        packetUBXHNRINS->data.yAccel = extractSignedLong(msg, 28);
        packetUBXHNRINS->data.zAccel = extractSignedLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXHNRINS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRINS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRINS->callbackData->bitfield0.all, &packetUBXHNRINS->data.bitfield0.all, sizeof(UBX_HNR_INS_data_t));
          packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRINS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
#endif
  }

  // Check if this UBX message should be added to the file buffer - if it has not been added already
  if ((!addedToFileBuffer) && (logThisUBX(msg->cls, msg->id)))
    storePacket(msg);
}

// UBX Logging - without needing to have or use "Auto" methods
void DevUBLOXGNSS::enableUBXlogging(uint8_t UBX_CLASS, uint8_t UBX_ID, bool enable)
{
  // If the list is empty
  if (sfe_ublox_ubx_logging_list_head == nullptr)
  {
    // Start the list with this CLASS + ID
    sfe_ublox_ubx_logging_list_head = new sfe_ublox_ubx_logging_list_t;
    sfe_ublox_ubx_logging_list_head->UBX_CLASS = UBX_CLASS;
    sfe_ublox_ubx_logging_list_head->UBX_ID = UBX_ID;
    sfe_ublox_ubx_logging_list_head->enable = enable;
    sfe_ublox_ubx_logging_list_head->next = nullptr;
    return;
  }

  // Check if this CLASS + ID is already registered in the linked list
  sfe_ublox_ubx_logging_list_t *sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_head;

  // Step through the list, check for CLASS + ID
  bool keepGoing = true;
  while (keepGoing)
  {
    if ((sfe_ublox_ubx_logging_list_ptr->UBX_CLASS == UBX_CLASS) // Check for a match
        && (sfe_ublox_ubx_logging_list_ptr->UBX_ID == UBX_ID))
    {
      sfe_ublox_ubx_logging_list_ptr->enable = enable; // Update enable
      return;
    }

    if (sfe_ublox_ubx_logging_list_ptr->next == nullptr)
      keepGoing = false;
    else
      sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_ptr->next;
  }

  // CLASS + ID not found. Add them.
  sfe_ublox_ubx_logging_list_ptr->next = new sfe_ublox_ubx_logging_list_t;
  sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_ptr->next;
  sfe_ublox_ubx_logging_list_ptr->UBX_CLASS = UBX_CLASS;
  sfe_ublox_ubx_logging_list_ptr->UBX_ID = UBX_ID;
  sfe_ublox_ubx_logging_list_ptr->enable = enable;
  sfe_ublox_ubx_logging_list_ptr->next = nullptr;
}

// PRIVATE: Returns true if this UBX should be added to the logging buffer
bool DevUBLOXGNSS::logThisUBX(uint8_t UBX_CLASS, uint8_t UBX_ID)
{
  // If the list is empty
  if (sfe_ublox_ubx_logging_list_head == nullptr)
    return false;

  // Step through the list, check for CLASS + ID
  sfe_ublox_ubx_logging_list_t *sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_head;
  bool keepGoing = true;
  while (keepGoing)
  {
    if ((sfe_ublox_ubx_logging_list_ptr->UBX_CLASS == UBX_CLASS) // Check for a match
        && (sfe_ublox_ubx_logging_list_ptr->UBX_ID == UBX_ID))
    {
      return sfe_ublox_ubx_logging_list_ptr->enable;
    }

    if (sfe_ublox_ubx_logging_list_ptr->next == nullptr)
      keepGoing = false;
    else
      sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_ptr->next;
  }

  return false;
}

// Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
// This is called before we send a command message
void DevUBLOXGNSS::calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

// Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
// This is used when receiving messages from module
void DevUBLOXGNSS::addToChecksum(uint8_t incoming)
{
  rollingChecksumA += incoming;
  rollingChecksumB += rollingChecksumA;
}

// Given a packet and payload, send everything including CRC bytes via I2C port
sfe_ublox_status_e DevUBLOXGNSS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait, bool expectACKonly)
{
  if (!lock())
    return SFE_UBLOX_STATUS_FAIL;

  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

  calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("\nSending: "));
    printPacket(outgoingUBX, true); // Always print payload
  }
#endif

  if (_commType == COMM_TYPE_I2C)
  {
    retVal = sendI2cCommand(outgoingUBX);
    if (retVal != SFE_UBLOX_STATUS_SUCCESS)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial.println(F("Send I2C Command failed"));
      }
#endif
      unlock();
      return retVal;
    }
  }
  else if (_commType == COMM_TYPE_SERIAL)
  {
    sendSerialCommand(outgoingUBX);
  }
  else if (_commType == COMM_TYPE_SPI)
  {
    sendSpiCommand(outgoingUBX);
  }

  unlock();

  if (maxWait > 0)
  {
    // Depending on what we just sent, either we need to look for an ACK or not
    if ((outgoingUBX->cls == UBX_CLASS_CFG) || (expectACKonly == true))
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial.println(F("sendCommand: Waiting for ACK response"));
      }
#endif
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
    else
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial.println(F("sendCommand: Waiting for No ACK response"));
      }
#endif
      retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
  }
  else
  {
    processSpiBuffer(&packetCfg, 0, 0); // Process any SPI data received during the sendSpiCommand - but only if not checking for a response
  }

  return retVal;
}

// Returns false if sensor fails to respond to I2C traffic
sfe_ublox_status_e DevUBLOXGNSS::sendI2cCommand(ubxPacket *outgoingUBX)
{
  // From the integration guide:
  // "The receiver does not provide any write access except for writing UBX and NMEA messages to the
  //  receiver, such as configuration or aiding data. Therefore, the register set mentioned in section Read
  //  Access is not writeable. Following the start condition from the master, the 7-bit device address and
  //  the RW bit (which is a logic low for write access) are clocked onto the bus by the master transmitter.
  //  The receiver answers with an acknowledge (logic low) to indicate that it is responsible for the given
  //  address. Now, the master can write 2 to N bytes to the receiver, generating a stop condition after the
  //  last byte being written. The number of data bytes must be at least 2 to properly distinguish from
  //  the write access to set the address counter in random read accesses."
  // I take two things from this:
  // 1) We do not need to write 0xFF to point at register 0xFF. We're already pointing at it.
  // 2) We must always write at least 2 bytes, otherwise it looks like we are starting to do a read.
  // Point 2 is important. It means:
  // * In this function:
  //     if we do multiple writes (because we're trying to write more than i2cTransactionSize),
  //     we may need to write one byte less in the penultimate write to ensure we always have two bytes left for the final write.
  // * In pushRawData:
  //     if there is one byte to write, or one byte left to write, we need to do the same thing and may need to store a single
  //     byte until pushRawData is called again.

  // The total number of bytes to be written is: payload len + 8
  // UBX_SYNCH_1
  // UBX_SYNCH_2
  // cls
  // id
  // len (MSB)
  // len (LSB)
  // < payload >
  // checksumA
  // checksumB

  // i2cTransactionSize will be at least 8. We don't need to check for smaller values than that.

  uint16_t bytesLeftToSend = outgoingUBX->len; // How many bytes remain to be sent
  uint16_t startSpot = 0;                      // Payload pointer

  // Check if we can send all the data in one transfer?
  if (bytesLeftToSend + 8 <= i2cTransactionSize)
  {
    uint8_t buf[i2cTransactionSize];
    buf[0] = UBX_SYNCH_1; // μ - oh ublox, you're funny. I will call you micro-blox from now on.
    buf[1] = UBX_SYNCH_2; // b
    buf[2] = outgoingUBX->cls;
    buf[3] = outgoingUBX->id;
    buf[4] = outgoingUBX->len & 0xFF; // LSB
    buf[5] = outgoingUBX->len >> 8;   // MSB
    uint16_t i = 0;
    for (; i < outgoingUBX->len; i++)
      buf[i + 6] = outgoingUBX->payload[startSpot + i];
    buf[i + 6] = outgoingUBX->checksumA;
    buf[i + 7] = outgoingUBX->checksumB;

    if (writeBytes(buf, bytesLeftToSend + 8) != bytesLeftToSend + 8)
      return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK
  }

  else
  {
    uint8_t buf[6];
    buf[0] = UBX_SYNCH_1; // μ - oh ublox, you're funny. I will call you micro-blox from now on.
    buf[1] = UBX_SYNCH_2; // b
    buf[2] = outgoingUBX->cls;
    buf[3] = outgoingUBX->id;
    buf[4] = outgoingUBX->len & 0xFF; // LSB
    buf[5] = outgoingUBX->len >> 8;   // MSB

    if (writeBytes(buf, 6) != 6)
      return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK

    // If bytesLeftToSend is zero, that's OK.
    // If bytesLeftToSend is >= 2, that's OK.
    // But if bytesLeftToSend is 1, we need to carry that byte over and send it with the checksum bytes
    while (bytesLeftToSend > 1)
    {
      uint16_t len = bytesLeftToSend; // How many bytes should we actually write?
      if (len > i2cTransactionSize)   // Limit len to i2cTransactionSize
        len = i2cTransactionSize;

      bytesLeftToSend -= len; // Calculate how many bytes will be left after we do this write

      // Write a portion of the payload to the bus.
      // Keep going until we've sent as many bytes as we can in this transmission (x == len)
      // or until we reach the end of the payload ((startSpot + x) == (outgoingUBX->len))
      uint16_t x = len;
      if ((startSpot + x) >= (outgoingUBX->len))
        x = outgoingUBX->len - startSpot;

      if (writeBytes(&outgoingUBX->payload[startSpot], x) != x)
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK

      startSpot += x;
    }

    // Finally, write any left-over bytes plus the checksum
    if (bytesLeftToSend == 1)
    {
      buf[0] = outgoingUBX->payload[startSpot];
      buf[1] = outgoingUBX->checksumA;
      buf[2] = outgoingUBX->checksumB;

      if (writeBytes(buf, 3) != 3)
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK
    }
    else
    {
      buf[0] = outgoingUBX->checksumA;
      buf[1] = outgoingUBX->checksumB;

      if (writeBytes(buf, 2) != 2)
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK
    }
  }

  return (SFE_UBLOX_STATUS_SUCCESS);
}

// Given a packet and payload, send everything including CRC bytesA via Serial port
void DevUBLOXGNSS::sendSerialCommand(ubxPacket *outgoingUBX)
{
  uint8_t buf[6];
  buf[0] = UBX_SYNCH_1; // μ - oh ublox, you're funny. I will call you micro-blox from now on.
  buf[1] = UBX_SYNCH_2; // b
  buf[2] = outgoingUBX->cls;
  buf[3] = outgoingUBX->id;
  buf[4] = outgoingUBX->len & 0xFF; // LSB
  buf[5] = outgoingUBX->len >> 8;   // MSB
  writeBytes(buf, 6);

  // Write payload
  writeBytes(outgoingUBX->payload, outgoingUBX->len);

  buf[0] = outgoingUBX->checksumA;
  buf[1] = outgoingUBX->checksumB;
  writeBytes(buf, 2);
}

// Transfer a byte to SPI. Also capture any bytes received from the UBLOX device during sending and capture them in a small buffer so that
// they can be processed later with process
void DevUBLOXGNSS::spiTransfer(const uint8_t byteToTransfer)
{
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  static bool printOnce = false;
  if (spiBufferIndex == 0)
    printOnce = false;
#endif

  // If we start to receive something, we need to keep receiving and buffering
  // otherwise 0xFF bytes will be ignored if currentSentence == SFE_UBLOX_SENTENCE_TYPE_NONE
  static bool receivedSomething = false;
  if (spiBufferIndex == 0)
    receivedSomething = false;

  uint8_t returnedByte = 0xFF;

  writeReadByte(byteToTransfer, &returnedByte);

  if ((returnedByte != 0xFF) || (currentSentence != SFE_UBLOX_SENTENCE_TYPE_NONE) || receivedSomething)
  {
    if (spiBufferIndex < spiBufferSize)
    {
      spiBuffer[spiBufferIndex] = returnedByte;
      spiBufferIndex++;
      receivedSomething = true;
    }
    else
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (((_printDebug == true) || (_printLimitedDebug == true)) && !printOnce) // This is important. Print this if doing limited debugging
      {
        _debugSerial.print(F("spiTransfer: spiBuffer is full!"));
        printOnce = true;
      }
#endif
    }
  }
}

// Send a command via SPI
sfe_ublox_status_e DevUBLOXGNSS::sendSpiCommand(ubxPacket *outgoingUBX)
{
  if (spiBuffer == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.print(F("sendSpiCommand: no memory allocation for SPI Buffer!"));
    }
#endif
    return (SFE_UBLOX_STATUS_MEM_ERR);
  }

  // Start at the beginning of the SPI buffer
  // spiBufferIndex = 0;

  uint16_t bytesLeftToSend = outgoingUBX->len; // How many bytes remain to be sent

  startWriteReadByte();

  spiTransfer(UBX_SYNCH_1); // μ - oh ublox, you're funny. I will call you micro-blox from now on.
  spiTransfer(UBX_SYNCH_2); // b
  spiTransfer(outgoingUBX->cls);
  spiTransfer(outgoingUBX->id);
  spiTransfer(outgoingUBX->len & 0xFF); // LSB
  spiTransfer(outgoingUBX->len >> 8);   // MSB

  // Check if we can send all the data in one transfer?
  if ((bytesLeftToSend + 8) <= spiTransactionSize)
  {
    for (uint16_t i = 0; i < bytesLeftToSend; i++)
      spiTransfer(outgoingUBX->payload[i]);
  }

  else
  {
    endWriteReadByte();

    uint16_t bytesSent = 0;

    while (bytesLeftToSend > 0)
    {
      uint16_t len = bytesLeftToSend; // How many bytes should we actually write?
      if (len > spiTransactionSize)   // Limit len to spiTransactionSize
        len = spiTransactionSize;

      bytesLeftToSend -= len; // Calculate how many bytes will be left after we do this write

      // Write a portion of the payload to the bus.

      startWriteReadByte();

      for (uint16_t i = 0; i < len; i++)
        spiTransfer(outgoingUBX->payload[bytesSent + i]);

      bytesSent += len;

      endWriteReadByte();
    }

    startWriteReadByte();
  }

  // Finally, write the checksum
  spiTransfer(outgoingUBX->checksumA);
  spiTransfer(outgoingUBX->checksumB);

  endWriteReadByte();

  return (SFE_UBLOX_STATUS_SUCCESS);
}

// Pretty prints the current ubxPacket
void DevUBLOXGNSS::printPacket(ubxPacket *packet, bool alwaysPrintPayload)
{
  // Only print the payload is ignoreThisPayload is false otherwise
  // we could be printing gibberish from beyond the end of packetBuf
  // (These two lines get rid of a pesky compiler warning)
  bool printPayload = (ignoreThisPayload == false);
  printPayload |= (alwaysPrintPayload == true);

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("CLS:"));
    if (packet->cls == UBX_CLASS_NAV) // 1
      _debugSerial.print(F("NAV"));
    else if (packet->cls == UBX_CLASS_ACK) // 5
      _debugSerial.print(F("ACK"));
    else if (packet->cls == UBX_CLASS_CFG) // 6
      _debugSerial.print(F("CFG"));
    else if (packet->cls == UBX_CLASS_MON) // 0x0A
      _debugSerial.print(F("MON"));
    else
    {
      _debugSerial.print(F("0x"));
      _debugSerial.print(packet->cls, HEX);
    }

    _debugSerial.print(F(" ID:"));
    if (packet->cls == UBX_CLASS_NAV && packet->id == UBX_NAV_PVT)
      _debugSerial.print(F("PVT"));
    else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_CFG)
      _debugSerial.print(F("SAVE"));
    else
    {
      _debugSerial.print(F("0x"));
      _debugSerial.print(packet->id, HEX);
    }

    _debugSerial.print(F(" Len: 0x"));
    _debugSerial.print(packet->len, HEX);

    if (printPayload)
    {
      _debugSerial.print(F(" Payload:"));

      for (uint16_t x = 0; x < packet->len; x++)
      {
        _debugSerial.print(F(" "));
        _debugSerial.print(packet->payload[x], HEX);
      }
    }
    else
    {
      _debugSerial.print(F(" Payload: IGNORED"));
    }
    _debugSerial.println();
  }
#else
  if (_printDebug == true)
  {
    _debugSerial.print(F("Len: 0x"));
    _debugSerial.print(packet->len, HEX);
  }
#endif
}

// When messages from the class CFG are sent to the receiver, the receiver will send an "acknowledge"(UBX - ACK - ACK) or a
//"not acknowledge"(UBX-ACK-NAK) message back to the sender, depending on whether or not the message was processed correctly.
// Some messages from other classes also use the same acknowledgement mechanism.

// When we poll or get a setting, we will receive _both_ a config packet and an ACK
// If the poll or get request is not valid, we will receive _only_ a NACK

// If we are trying to get or poll a setting, then packetCfg.len will be 0 or 1 when the packetCfg is _sent_.
// If we poll the setting for a particular port using UBX-CFG-PRT then .len will be 1 initially
// For all other gets or polls, .len will be 0 initially
//(It would be possible for .len to be 2 _if_ we were using UBX-CFG-MSG to poll the settings for a particular message - but we don't use that (currently))

// If the get or poll _fails_, i.e. is NACK'd, then packetCfg.len could still be 0 or 1 after the NACK is received
// But if the get or poll is ACK'd, then packetCfg.len will have been updated by the incoming data and will always be at least 2

// If we are going to set the value for a setting, then packetCfg.len will be at least 3 when the packetCfg is _sent_.
//(UBX-CFG-MSG appears to have the shortest set length of 3 bytes)

// We need to think carefully about how interleaved PVT packets affect things.
// It is entirely possible that our packetCfg and packetAck were received successfully
// but while we are still in the "if (checkUblox() == true)" loop a PVT packet is processed
// or _starts_ to arrive (remember that Serial data can arrive very slowly).

// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got an ACK and a valid packetCfg (module is responding with register content)
// Returns SFE_UBLOX_STATUS_DATA_SENT if we got an ACK and no packetCfg (no valid packetCfg needed, module absorbs new register data)
// Returns SFE_UBLOX_STATUS_FAIL if something very bad happens (e.g. a double checksum failure)
// Returns SFE_UBLOX_STATUS_COMMAND_NACK if the packet was not-acknowledged (NACK)
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we had a checksum failure
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an ACK and a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e DevUBLOXGNSS::waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() < (startTime + (unsigned long)maxTime))
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {
      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForACKResponse: valid data and valid ACK received after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and a correct ACK!
      }

      // We can be confident that the data packet (if we are going to get one) will always arrive
      // before the matching ACK. So if we sent a config packet which only produces an ACK
      // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
      // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
      // as these may have been changed by an automatic packet.
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForACKResponse: no data and valid ACK after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_SENT); // We got an ACK but no data...
      }

      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by an automatic packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForACKResponse: data being OVERWRITTEN after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
      // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
      else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForACKResponse: CRC failed after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_CRC_FAIL); // Checksum fail
      }

      // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
      // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
      // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
      // but outgoingUBX->cls and outgoingUBX->id would not match...
      // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
      // the packet was definitely NACK'd otherwise we are possibly just guessing...
      // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
      else if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_COMMAND_NACK); // We received a NACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
      // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
      // If we were playing safe, we should return FAIL instead
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForACKResponse: VALID data and INVALID ACK received after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
      // then we return a FAIL. This must be a double checksum failure?
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForACKResponse: INVALID data and INVALID ACK received after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_FAIL); // We received invalid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
      // then the ACK has not yet been received and we should keep waiting for it
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial.print(F("waitForACKResponse: valid data after "));
        //   _debugSerial.print(millis() - startTime);
        //   _debugSerial.println(F(" msec. Waiting for ACK."));
        // }
      }

    } // checkUbloxInternal == true

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }           // while (millis() < (startTime + (unsigned long)maxTime))

  // We have timed out...
  // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
  // even though we did not get an ACK
  if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.print(F("waitForACKResponse: TIMEOUT with valid data after "));
      _debugSerial.print(millis() - startTime);
      _debugSerial.println(F(" msec. "));
    }
#endif
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data... But no ACK!
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("waitForACKResponse: TIMEOUT after "));
    _debugSerial.print(millis() - startTime);
    _debugSerial.println(F(" msec."));
  }
#endif

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

// For non-CFG queries no ACK is sent so we use this function
// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of response data that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e DevUBLOXGNSS::waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() - startTime < maxTime)
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {

      // If outgoingUBX->classAndIDmatch is VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForNoACKResponse: valid data with CLS/ID match after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data!
      }

      // If the outgoingUBX->classAndIDmatch is VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForNoACKResponse: data being OVERWRITTEN after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If outgoingUBX->classAndIDmatch is NOT_DEFINED
      // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial.print(F("waitForNoACKResponse: valid but UNWANTED data after "));
        //   _debugSerial.print(millis() - startTime);
        //   _debugSerial.print(F(" msec. Class: 0x"));
        //   _debugSerial.print(outgoingUBX->cls, HEX);
        //   _debugSerial.print(F(" ID: 0x"));
        //   _debugSerial.print(outgoingUBX->id, HEX);
        // }
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
      else if (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial.print(F("waitForNoACKResponse: CLS/ID match but failed CRC after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_CRC_FAIL); // We received invalid data
      }
    }

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("waitForNoACKResponse: TIMEOUT after "));
    _debugSerial.print(millis() - startTime);
    _debugSerial.println(F(" msec. No packet received."));
  }
#endif

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

// Check if any callbacks are waiting to be processed
void DevUBLOXGNSS::checkCallbacks(void)
{
  if (checkCallbacksReentrant == true) // Check for reentry (i.e. checkCallbacks has been called from inside a callback)
    return;

  checkCallbacksReentrant = true;

  if (packetUBXNAVSTATUS != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVSTATUS->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVSTATUS->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVSTATUS->callbackPointerPtr(packetUBXNAVSTATUS->callbackData); // Call the callback
        }
        packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVDOP != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVDOP->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVDOP->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVDOP->callbackPointerPtr(packetUBXNAVDOP->callbackData); // Call the callback
        }
        packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVATT != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVATT->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVATT->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVATT->callbackPointerPtr(packetUBXNAVATT->callbackData); // Call the callback
        }
        packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVPVT != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVPVT->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVPVT->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVPVT->callbackPointerPtr(packetUBXNAVPVT->callbackData); // Call the callback
        }
        packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVODO != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVODO->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVODO->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVODO->callbackPointerPtr(packetUBXNAVODO->callbackData); // Call the callback
        }
        packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVVELECEF != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVVELECEF->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVVELECEF->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVVELECEF->callbackPointerPtr(packetUBXNAVVELECEF->callbackData); // Call the callback
        }
        packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVVELNED != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVVELNED->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVVELNED->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVVELNED->callbackPointerPtr(packetUBXNAVVELNED->callbackData); // Call the callback
        }
        packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVHPPOSECEF != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVHPPOSECEF->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVHPPOSECEF->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVHPPOSECEF->callbackPointerPtr(packetUBXNAVHPPOSECEF->callbackData); // Call the callback
        }
        packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVHPPOSLLH != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVHPPOSLLH->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVHPPOSLLH->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVHPPOSLLH->callbackPointerPtr(packetUBXNAVHPPOSLLH->callbackData); // Call the callback
        }
        packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVPVAT != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVPVAT->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVPVAT->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVPVAT->callbackPointerPtr(packetUBXNAVPVAT->callbackData); // Call the callback
        }
        packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVTIMEUTC != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVTIMEUTC->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVTIMEUTC->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVTIMEUTC->callbackPointerPtr(packetUBXNAVTIMEUTC->callbackData); // Call the callback
        }
        packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVCLOCK != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVCLOCK->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVCLOCK->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVCLOCK->callbackPointerPtr(packetUBXNAVCLOCK->callbackData); // Call the callback
        }
        packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVSVIN != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVSVIN->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVSVIN->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVSVIN->callbackPointerPtr(packetUBXNAVSVIN->callbackData); // Call the callback
        }
        packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  if (packetUBXNAVSAT != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVSAT->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVSAT->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVSAT->callbackPointerPtr(packetUBXNAVSAT->callbackData); // Call the callback
        }
        packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVSIG != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVSIG->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVSIG->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVSIG->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVSIG->callbackPointerPtr(packetUBXNAVSIG->callbackData); // Call the callback
        }
        packetUBXNAVSIG->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }
#endif

  if (packetUBXNAVRELPOSNED != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVRELPOSNED->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVRELPOSNED->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVRELPOSNED->callbackPointerPtr(packetUBXNAVRELPOSNED->callbackData); // Call the callback
        }
        packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVAOPSTATUS != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVAOPSTATUS->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVAOPSTATUS->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVAOPSTATUS->callbackPointerPtr(packetUBXNAVAOPSTATUS->callbackData); // Call the callback
        }
        packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXNAVEOE != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVEOE->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVEOE->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVEOE->callbackPointerPtr(packetUBXNAVEOE->callbackData); // Call the callback
        }
        packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  if (packetUBXRXMPMP != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXRXMPMP->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXRXMPMP->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXRXMPMP->callbackPointerPtr(packetUBXRXMPMP->callbackData); // Call the callback
        }
        packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXRXMPMPmessage != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXRXMPMPmessage->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXRXMPMPmessage->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXRXMPMPmessage->callbackPointerPtr(packetUBXRXMPMPmessage->callbackData); // Call the callback
        }
        packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXRXMQZSSL6message != nullptr)                         // If RAM has been allocated for message storage
    if (packetUBXRXMQZSSL6message->callbackData != nullptr)         // If RAM has been allocated for the copy of the data
      if (packetUBXRXMQZSSL6message->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
      {
        for (int ch = 0; ch < UBX_RXM_QZSSL6_NUM_CHANNELS; ch++)
        {
          if (packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid & (1 << ch)) // If the copy of the data is valid
          {
            packetUBXRXMQZSSL6message->callbackPointerPtr(&packetUBXRXMQZSSL6message->callbackData[ch]); // Call the callback
            packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid &= ~(1 << ch);        // clear it
          }
        }
      }

  if (packetUBXRXMCOR != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXRXMCOR->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXRXMCOR->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXRXMCOR->callbackPointerPtr(packetUBXRXMCOR->callbackData); // Call the callback
        }
        packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXRXMSFRBX != nullptr) // If RAM has been allocated for message storage
  {
    if (packetUBXRXMSFRBX->callbackData != nullptr) // If RAM has been allocated for the copies of the data
    {
      for (uint32_t i = 0; i < UBX_RXM_SFRBX_CALLBACK_BUFFERS; i++)
      {
        if ((packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid & (1 << i)) != 0) // If the copy of the data is valid
        {
          if (packetUBXRXMSFRBX->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
          {
            packetUBXRXMSFRBX->callbackPointerPtr(&packetUBXRXMSFRBX->callbackData[i]); // Call the callback
          }
          packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid &= ~(1 << i); // Mark the data as stale
        }
      }
    }
    if (packetUBXRXMSFRBX->callbackMessageData != nullptr) // If RAM has been allocated for the copies of the data
    {
      for (uint32_t i = 0; i < UBX_RXM_SFRBX_CALLBACK_BUFFERS; i++)
      {
        if ((packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackMessageCopyValid & (1 << i)) != 0) // If the copy of the data is valid
        {
          if (packetUBXRXMSFRBX->callbackMessagePointerPtr != nullptr) // If the pointer to the callback has been defined
          {
            packetUBXRXMSFRBX->callbackMessagePointerPtr(&packetUBXRXMSFRBX->callbackMessageData[i]); // Call the callback
          }
          packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackMessageCopyValid &= ~(1 << i); // Mark the data as stale
        }
      }
    }
  }

  if (packetUBXRXMRAWX != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXRXMRAWX->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXRXMRAWX->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXRXMRAWX->callbackPointerPtr(packetUBXRXMRAWX->callbackData); // Call the callback
        }
        packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXRXMMEASX != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXRXMMEASX->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXRXMMEASX->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXRXMMEASX->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXRXMMEASX->callbackPointerPtr(packetUBXRXMMEASX->callbackData); // Call the callback
        }
        packetUBXRXMMEASX->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }
#endif

  if (packetUBXTIMTM2 != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXTIMTM2->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXTIMTM2->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXTIMTM2->callbackPointerPtr(packetUBXTIMTM2->callbackData); // Call the callback
        }
        packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXTIMTP != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXTIMTP->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXTIMTP->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXTIMTP->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXTIMTP->callbackPointerPtr(packetUBXTIMTP->callbackData); // Call the callback
        }
        packetUBXTIMTP->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXMONHW != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXMONHW->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXMONHW->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXMONHW->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXMONHW->callbackPointerPtr(packetUBXMONHW->callbackData); // Call the callback
        }
        packetUBXMONHW->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

#ifndef SFE_UBLOX_DISABLE_ESF
  if (packetUBXESFALG != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXESFALG->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXESFALG->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXESFALG->callbackPointerPtr(packetUBXESFALG->callbackData); // Call the callback
        }
        packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXESFINS != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXESFINS->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXESFINS->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXESFINS->callbackPointerPtr(packetUBXESFINS->callbackData); // Call the callback
        }
        packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXESFMEAS != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXESFMEAS->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      for (uint16_t i = 0; i < UBX_ESF_MEAS_CALLBACK_BUFFERS; i++)
      {
        if ((packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid & (1 << i)) != 0) // If the copy of the data is valid
        {
          if (packetUBXESFMEAS->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
          {
            packetUBXESFMEAS->callbackPointerPtr(&packetUBXESFMEAS->callbackData[i]); // Call the callback
          }
          packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid &= ~(1 << i); // Mark the data as stale
        }
      }

  if (packetUBXESFRAW != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXESFRAW->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXESFRAW->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXESFRAW->callbackPointerPtr(packetUBXESFRAW->callbackData); // Call the callback
        }
        packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXESFSTATUS != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXESFSTATUS->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXESFSTATUS->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXESFSTATUS->callbackPointerPtr(packetUBXESFSTATUS->callbackData); // Call the callback
        }
        packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }
#endif
#ifndef SFE_UBLOX_DISABLE_HNR
  if (packetUBXHNRATT != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXHNRATT->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXHNRATT->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXHNRATT->callbackPointerPtr(packetUBXHNRATT->callbackData); // Call the callback
        }
        packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXHNRINS != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXHNRINS->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXHNRINS->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXHNRINS->callbackPointerPtr(packetUBXHNRINS->callbackData); // Call the callback
        }
        packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }

  if (packetUBXHNRPVT != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXHNRPVT->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXHNRPVT->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXHNRPVT->callbackPointerPtr(packetUBXHNRPVT->callbackData); // Call the callback
        }
        packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }
#endif

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  if (storageNMEAGPGGA != nullptr)                                            // If RAM has been allocated for message storage
    if (storageNMEAGPGGA->callbackCopy != nullptr)                            // If RAM has been allocated for the copy of the data
      if (storageNMEAGPGGA->automaticFlags.flags.bits.callbackCopyValid == 1) // If the copy of the data is valid
      {
        if (storageNMEAGPGGA->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          storageNMEAGPGGA->callbackPointerPtr(storageNMEAGPGGA->callbackCopy); // Call the callback
        }
        storageNMEAGPGGA->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
      }

  if (storageNMEAGNGGA != nullptr)                                            // If RAM has been allocated for message storage
    if (storageNMEAGNGGA->callbackCopy != nullptr)                            // If RAM has been allocated for the copy of the data
      if (storageNMEAGNGGA->automaticFlags.flags.bits.callbackCopyValid == 1) // If the copy of the data is valid
      {
        if (storageNMEAGNGGA->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          storageNMEAGNGGA->callbackPointerPtr(storageNMEAGNGGA->callbackCopy); // Call the callback
        }
        storageNMEAGNGGA->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
      }

  if (storageNMEAGPVTG != nullptr)                                            // If RAM has been allocated for message storage
    if (storageNMEAGPVTG->callbackCopy != nullptr)                            // If RAM has been allocated for the copy of the data
      if (storageNMEAGPVTG->automaticFlags.flags.bits.callbackCopyValid == 1) // If the copy of the data is valid
      {
        if (storageNMEAGPVTG->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          storageNMEAGPVTG->callbackPointerPtr(storageNMEAGPVTG->callbackCopy); // Call the callback
        }
        storageNMEAGPVTG->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
      }

  if (storageNMEAGNVTG != nullptr)                                            // If RAM has been allocated for message storage
    if (storageNMEAGNVTG->callbackCopy != nullptr)                            // If RAM has been allocated for the copy of the data
      if (storageNMEAGNVTG->automaticFlags.flags.bits.callbackCopyValid == 1) // If the copy of the data is valid
      {
        if (storageNMEAGNVTG->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          storageNMEAGNVTG->callbackPointerPtr(storageNMEAGNVTG->callbackCopy); // Call the callback
        }
        storageNMEAGNVTG->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
      }

  if (storageNMEAGPRMC != nullptr)                                            // If RAM has been allocated for message storage
    if (storageNMEAGPRMC->callbackCopy != nullptr)                            // If RAM has been allocated for the copy of the data
      if (storageNMEAGPRMC->automaticFlags.flags.bits.callbackCopyValid == 1) // If the copy of the data is valid
      {
        if (storageNMEAGPRMC->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          storageNMEAGPRMC->callbackPointerPtr(storageNMEAGPRMC->callbackCopy); // Call the callback
        }
        storageNMEAGPRMC->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
      }

  if (storageNMEAGNRMC != nullptr)                                            // If RAM has been allocated for message storage
    if (storageNMEAGNRMC->callbackCopy != nullptr)                            // If RAM has been allocated for the copy of the data
      if (storageNMEAGNRMC->automaticFlags.flags.bits.callbackCopyValid == 1) // If the copy of the data is valid
      {
        if (storageNMEAGNRMC->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          storageNMEAGNRMC->callbackPointerPtr(storageNMEAGNRMC->callbackCopy); // Call the callback
        }
        storageNMEAGNRMC->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
      }

  if (storageNMEAGPZDA != nullptr)                                            // If RAM has been allocated for message storage
    if (storageNMEAGPZDA->callbackCopy != nullptr)                            // If RAM has been allocated for the copy of the data
      if (storageNMEAGPZDA->automaticFlags.flags.bits.callbackCopyValid == 1) // If the copy of the data is valid
      {
        if (storageNMEAGPZDA->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          storageNMEAGPZDA->callbackPointerPtr(storageNMEAGPZDA->callbackCopy); // Call the callback
        }
        storageNMEAGPZDA->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
      }

  if (storageNMEAGNZDA != nullptr)                                            // If RAM has been allocated for message storage
    if (storageNMEAGNZDA->callbackCopy != nullptr)                            // If RAM has been allocated for the copy of the data
      if (storageNMEAGNZDA->automaticFlags.flags.bits.callbackCopyValid == 1) // If the copy of the data is valid
      {
        if (storageNMEAGNZDA->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          // if (_printDebug == true)
          //   _debugSerial.println(F("checkCallbacks: calling callbackPtr for GNZDA"));
          storageNMEAGNZDA->callbackPointerPtr(storageNMEAGNZDA->callbackCopy); // Call the callback
        }
        storageNMEAGNZDA->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
      }
#endif

  if (storageRTCM1005 != nullptr)                                            // If RAM has been allocated for message storage
    if (storageRTCM1005->callbackData != nullptr)                            // If RAM has been allocated for the copy of the data
      if (storageRTCM1005->automaticFlags.flags.bits.callbackDataValid == 1) // If the copy of the data is valid
      {
        if (storageRTCM1005->callbackPointerPtr != nullptr)                   // If the pointer to the callback has been defined
          storageRTCM1005->callbackPointerPtr(storageRTCM1005->callbackData); // Call the callback
        storageRTCM1005->automaticFlags.flags.bits.callbackDataValid = 0;     // Mark the data as stale
      }

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  if (rtcmInputStorage.rtcm1005CallbackPointer != nullptr) // If the pointer to the callback has been defined
    if (rtcmInputStorage.flags.bits.dataValid1005 == 1)    // If the copy of the data is valid
      if (rtcmInputStorage.flags.bits.dataRead1005 == 0)   // If the data has not been read
      {
        rtcmInputStorage.rtcm1005CallbackPointer(&rtcmInputStorage.rtcm1005); // Call the callback
        rtcmInputStorage.flags.bits.dataRead1005 = 1;                         // Mark the data as read
      }

  if (rtcmInputStorage.rtcm1006CallbackPointer != nullptr) // If the pointer to the callback has been defined
    if (rtcmInputStorage.flags.bits.dataValid1006 == 1)    // If the copy of the data is valid
      if (rtcmInputStorage.flags.bits.dataRead1006 == 0)   // If the data has not been read
      {
        rtcmInputStorage.rtcm1006CallbackPointer(&rtcmInputStorage.rtcm1006); // Call the callback
        rtcmInputStorage.flags.bits.dataRead1006 = 1;                         // Mark the data as read
      }
#endif

  checkCallbacksReentrant = false;
}

// Push (e.g.) RTCM data directly to the module
// Returns true if all numDataBytes were pushed successfully
// Warning: this function does not check that the data is valid. It is the user's responsibility to ensure the data is valid before pushing.
bool DevUBLOXGNSS::pushRawData(uint8_t *dataBytes, size_t numDataBytes, bool callProcessBuffer)
{
  // Return now if numDataBytes is zero
  if (numDataBytes == 0)
    return false; // Indicate to the user that there was no data to push

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  parseRTCM1005(dataBytes, numDataBytes);
  parseRTCM1006(dataBytes, numDataBytes);
#endif

  if (!lock())
    return false;

  bool ok = false;
  if (_commType == COMM_TYPE_SERIAL)
  {
    // Serial: divide pushes up into 16 byte chunks and call checkUbloxSerial between pushes
    // (if callProcessBuffer is true) to try and avoid data loss
    ok = true;
    size_t bytesLeftToWrite = numDataBytes;
    while (bytesLeftToWrite > 0)
    {
      uint8_t bytesToWrite;

      if (bytesLeftToWrite < 16)
        bytesToWrite = (uint8_t)bytesLeftToWrite;
      else
        bytesToWrite = 16;

      ok &= (writeBytes(dataBytes, bytesToWrite) == bytesToWrite); // Will set ok to false if any one write fails

      bytesLeftToWrite -= (size_t)bytesToWrite;
      dataBytes += bytesToWrite;

      if (callProcessBuffer)                // Try and prevent data loss during large pushes by calling checkUbloxSerial between chunks
        checkUbloxSerial(&packetCfg, 0, 0); // Don't call checkUbloxInternal as we already have the lock!
    }
  }
  else if (_commType == COMM_TYPE_I2C)
  {
    // We can not write a single data byte to I2C as it would look like the address of a random read.
    // If numDataBytes is 1, we should probably just reject the data and return false.
    // But we'll be nice and store the byte until the next time pushRawData is called.

    // Storage just in case the user tries to push a single byte using pushRawBytes
    static bool _pushSingleByte = false;
    static uint8_t _pushThisSingleByte = 0;

    if ((numDataBytes == 1) && (_pushSingleByte == false))
    {
      _pushThisSingleByte = *dataBytes;
      _pushSingleByte = true;
      ok = false; // Indicate to the user that their data has not been pushed yet
    }
    else
    {

      // I2C: split the data up into packets of i2cTransactionSize
      size_t bytesLeftToWrite = numDataBytes;
      size_t bytesWrittenTotal = 0;

      if (_pushSingleByte == true) // Increment bytesLeftToWrite if we have a single byte waiting to be pushed
        bytesLeftToWrite++;

      while (bytesLeftToWrite > 0)
      {
        size_t bytesToWrite; // Limit bytesToWrite to i2cTransactionSize

        if (bytesLeftToWrite > i2cTransactionSize)
          bytesToWrite = i2cTransactionSize;
        else
          bytesToWrite = bytesLeftToWrite;

        // If there would be one byte left to be written next time, send one byte less now
        if ((bytesLeftToWrite - bytesToWrite) == 1)
          bytesToWrite--;

        size_t bytesWritten = 0;

        if (_pushSingleByte == true)
        {
          uint8_t buf[i2cTransactionSize];

          buf[0] = _pushThisSingleByte;

          for (uint16_t x = 1; x < bytesToWrite; x++)
            buf[x] = dataBytes[x - 1];

          bytesWritten += writeBytes(buf, bytesToWrite); // Write the bytes
          dataBytes += bytesToWrite - 1;                 // Point to fresh data
          _pushSingleByte = false;                       // Clear the flag
        }
        else
        {
          bytesWritten += writeBytes(dataBytes, bytesToWrite); // Write the bytes
          dataBytes += bytesToWrite;                           // Point to fresh data
        }

        bytesWrittenTotal += bytesWritten; // Update the totals
        bytesLeftToWrite -= bytesToWrite;
      }

      ok = (bytesWrittenTotal == numDataBytes); // Return true if the correct number of bytes were written
    }
  }
  else if (_commType == COMM_TYPE_SPI)
  {
    // We've got to be careful here...
    // We could be pushing a lot of data.
    // And we're supposed to be reading the same amount of data at the same time.
    // If numDataBytes is > spiBuffer, we'll lose data
    // We'll call processSpiBuffer between transactions to try and prevent lost data
    size_t bytesLeftToWrite = numDataBytes;

    while (bytesLeftToWrite > 0)
    {
      size_t bytesToWrite; // Limit bytesToWrite to spiTransactionSize

      if (bytesLeftToWrite > spiTransactionSize)
        bytesToWrite = spiTransactionSize;
      else
        bytesToWrite = bytesLeftToWrite;

      startWriteReadByte();

      for (size_t i = 0; i < bytesToWrite; i++)
      {
        spiTransfer(*dataBytes);
        dataBytes++;
      }

      endWriteReadByte();

      bytesLeftToWrite -= bytesToWrite; // Update the totals

      if (callProcessBuffer)
        processSpiBuffer(&packetCfg, 0, 0); // This will hopefully prevent any lost data?
    }

    ok = true;
  }

  unlock();

  return ok;
}

// Push MGA AssistNow data to the module.
// Check for UBX-MGA-ACK responses if required (if mgaAck is YES or ENQUIRE).
// Wait for maxWait millis after sending each packet (if mgaAck is NO).
// Return how many bytes were pushed successfully.
// If skipTime is true, any UBX-MGA-INI-TIME_UTC or UBX-MGA-INI-TIME_GNSS packets found in the data will be skipped,
// allowing the user to override with their own time data with setUTCTimeAssistance.
size_t DevUBLOXGNSS::pushAssistNowData(const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(0, false, (const uint8_t *)dataBytes.c_str(), numDataBytes, mgaAck, maxWait));
}
size_t DevUBLOXGNSS::pushAssistNowData(const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(0, false, dataBytes, numDataBytes, mgaAck, maxWait));
}
size_t DevUBLOXGNSS::pushAssistNowData(bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(0, skipTime, (const uint8_t *)dataBytes.c_str(), numDataBytes, mgaAck, maxWait));
}
size_t DevUBLOXGNSS::pushAssistNowData(bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(0, skipTime, dataBytes, numDataBytes, mgaAck, maxWait));
}
size_t DevUBLOXGNSS::pushAssistNowData(size_t offset, bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(offset, skipTime, (const uint8_t *)dataBytes.c_str(), numDataBytes, mgaAck, maxWait));
}
size_t DevUBLOXGNSS::pushAssistNowData(size_t offset, bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(offset, skipTime, dataBytes, numDataBytes, mgaAck, maxWait));
}
size_t DevUBLOXGNSS::pushAssistNowDataInternal(size_t offset, bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  size_t dataPtr = offset;     // Pointer into dataBytes
  size_t packetsProcessed = 0; // Keep count of how many packets have been processed
  size_t bytesPushed = 0;      // Keep count

  bool checkForAcks = (mgaAck == SFE_UBLOX_MGA_ASSIST_ACK_YES); // If mgaAck is YES, always check for Acks

  // If mgaAck is ENQUIRE, we need to check UBX-CFG-NAVX5 ackAiding to determine if UBX-MGA-ACK's are expected
  if (mgaAck == SFE_UBLOX_MGA_ASSIST_ACK_ENQUIRE)
  {
    uint8_t ackAiding = getAckAiding(maxWait); // Enquire if we should expect Acks
    if (ackAiding == 1)
      checkForAcks = true;

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.print(F("pushAssistNowData: mgaAck is ENQUIRE. getAckAiding returned "));
      _debugSerial.println(ackAiding);
    }
#endif
  }

  // If checkForAcks is true, then we need to set up storage for the UBX-MGA-ACK-DATA0 messages
  if (checkForAcks)
  {
    if (packetUBXMGAACK == nullptr)
      initPacketUBXMGAACK();        // Check that RAM has been allocated for the MGA_ACK data
    if (packetUBXMGAACK == nullptr) // Bail if the RAM allocation failed
      return (0);
  }

  while (dataPtr < (offset + numDataBytes)) // Keep going until we have processed all the bytes
  {
    // Start by checking the validity of the packet being pointed to
    bool dataIsOK = true;

    dataIsOK &= (*(dataBytes + dataPtr + 0) == UBX_SYNCH_1);   // Check for 0xB5
    dataIsOK &= (*(dataBytes + dataPtr + 1) == UBX_SYNCH_2);   // Check for 0x62
    dataIsOK &= (*(dataBytes + dataPtr + 2) == UBX_CLASS_MGA); // Check for class UBX-MGA

    size_t packetLength = ((size_t) * (dataBytes + dataPtr + 4)) | (((size_t) * (dataBytes + dataPtr + 5)) << 8); // Extract the length

    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    // Calculate the checksum bytes
    // Keep going until the end of the packet is reached (payloadPtr == (dataPtr + packetLength))
    // or we reach the end of the AssistNow data (payloadPtr == offset + numDataBytes)
    for (size_t payloadPtr = dataPtr + ((size_t)2); (payloadPtr < (dataPtr + packetLength + ((size_t)6))) && (payloadPtr < (offset + numDataBytes)); payloadPtr++)
    {
      checksumA += *(dataBytes + payloadPtr);
      checksumB += checksumA;
    }
    // Check the checksum bytes
    dataIsOK &= (checksumA == *(dataBytes + dataPtr + packetLength + ((size_t)6)));
    dataIsOK &= (checksumB == *(dataBytes + dataPtr + packetLength + ((size_t)7)));

    dataIsOK &= ((dataPtr + packetLength + ((size_t)8)) <= (offset + numDataBytes)); // Check we haven't overrun

    // If the data is valid, push it
    if (dataIsOK)
    {
      // Check if this is time assistance data which should be skipped
      if ((skipTime) && ((*(dataBytes + dataPtr + 3) == UBX_MGA_INI_TIME_UTC) || (*(dataBytes + dataPtr + 3) == UBX_MGA_INI_TIME_GNSS)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial.print(F("pushAssistNowData: skipped INI_TIME ID 0x"));
          if (*(dataBytes + dataPtr + 3) < 0x10)
            _debugSerial.print(F("0"));
          _debugSerial.println(*(dataBytes + dataPtr + 3), HEX);
        }
#endif
      }
      else
      {
        bool pushResult = pushRawData((uint8_t *)(dataBytes + dataPtr), packetLength + ((size_t)8), checkForAcks ? false : true); // Push the data. Don't call processSpiBuffer / checkUbloxSerial when using ACKs

        if (pushResult)
          bytesPushed += packetLength + ((size_t)8); // Increment bytesPushed if the push was successful

        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial.print(F("pushAssistNowData: packet ID 0x"));
          if (*(dataBytes + dataPtr + 3) < 0x10)
            _debugSerial.print(F("0"));
          _debugSerial.print(*(dataBytes + dataPtr + 3), HEX);
          _debugSerial.print(F(" length "));
          _debugSerial.println(packetLength);
        }

        if (checkForAcks)
        {
          unsigned long startTime = millis();
          bool keepGoing = true;
          while (keepGoing && (millis() < (startTime + maxWait))) // Keep checking for the ACK until we time out
          {
            checkUbloxInternal(&packetCfg, 0, 0);               // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID. We could be pushing this from another thread...
            if (packetUBXMGAACK->head != packetUBXMGAACK->tail) // Does the MGA ACK ringbuffer contain any ACK's?
            {
              bool dataAckd = true;                                                                                        // Check if we've received the correct ACK
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgId == *(dataBytes + dataPtr + 3));              // Check if the message ID matches
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[0] == *(dataBytes + dataPtr + 6)); // Check if the first four data bytes match
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[1] == *(dataBytes + dataPtr + 7));
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[2] == *(dataBytes + dataPtr + 8));
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[3] == *(dataBytes + dataPtr + 9));

              if (dataAckd) // Is this the ACK we are looking for?
              {
                if ((packetUBXMGAACK->data[packetUBXMGAACK->tail].type == (uint8_t)1) && (packetUBXMGAACK->data[packetUBXMGAACK->tail].infoCode == (uint8_t)SFE_UBLOX_MGA_ACK_INFOCODE_ACCEPTED))
                {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
                  if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
                  {
                    _debugSerial.print(F("pushAssistNowData: packet was accepted after "));
                    _debugSerial.print(millis() - startTime);
                    _debugSerial.println(F(" ms"));
                  }
#endif
                  packetsProcessed++;
                }
                else
                {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
                  if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
                  {
                    _debugSerial.print(F("pushAssistNowData: packet was _not_ accepted. infoCode is "));
                    _debugSerial.println(packetUBXMGAACK->data[packetUBXMGAACK->tail].infoCode);
                  }
#endif
                }
                keepGoing = false;
              }
              // Increment the tail
              packetUBXMGAACK->tail++;
              if (packetUBXMGAACK->tail == UBX_MGA_ACK_DATA0_RINGBUFFER_LEN)
                packetUBXMGAACK->tail = 0;
            }
          }
          if (keepGoing) // If keepGoing is still true, we must have timed out
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial.println(F("pushAssistNowData: packet ack timed out!"));
            }
#endif
          }
        }
        else
        {
          // We are not checking for Acks, so let's assume the send was successful?
          packetsProcessed++;
          // We are not checking for Acks, so delay for maxWait millis unless we've reached the end of the data
          if ((dataPtr + packetLength + ((size_t)8)) < (offset + numDataBytes))
          {
            delay(maxWait);
          }
        }
      }

      dataPtr += packetLength + ((size_t)8); // Point to the next message
    }
    else
    {

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      // The data was invalid. Send a debug message and then try to find the next 0xB5
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.print(F("pushAssistNowData: bad data - ignored! dataPtr is "));
        _debugSerial.println(dataPtr);
      }
#endif

      while ((dataPtr < (offset + numDataBytes)) && (*(dataBytes + ++dataPtr) != UBX_SYNCH_1))
      {
        ; // Increment dataPtr until we are pointing at the next 0xB5 - or we reach the end of the data
      }
    }
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
  {
    _debugSerial.print(F("pushAssistNowData: packetsProcessed: "));
    _debugSerial.println(packetsProcessed);
  }
#endif

  return (bytesPushed); // Return the number of valid bytes successfully pushed
}

// PRIVATE: Allocate RAM for packetUBXMGAACK and initialize it
bool DevUBLOXGNSS::initPacketUBXMGAACK()
{
  packetUBXMGAACK = new UBX_MGA_ACK_DATA0_t; // Allocate RAM for the main struct
  if (packetUBXMGAACK == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXMGAACK: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXMGAACK->head = 0; // Initialize the ring buffer pointers
  packetUBXMGAACK->tail = 0;
  return (true);
}

// Provide initial time assistance
bool DevUBLOXGNSS::setUTCTimeAssistance(uint16_t year, uint8_t month, uint8_t day,
                                        uint8_t hour, uint8_t minute, uint8_t second, uint32_t nanos,
                                        uint16_t tAccS, uint32_t tAccNs, uint8_t source,
                                        sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  uint8_t iniTimeUTC[32];       // Create the UBX-MGA-INI-TIME_UTC message by hand
  memset(iniTimeUTC, 0x00, 32); // Set all unused / reserved bytes and the checksum to zero

  iniTimeUTC[0] = UBX_SYNCH_1;              // Sync char 1
  iniTimeUTC[1] = UBX_SYNCH_2;              // Sync char 2
  iniTimeUTC[2] = UBX_CLASS_MGA;            // Class
  iniTimeUTC[3] = UBX_MGA_INI_TIME_UTC;     // ID
  iniTimeUTC[4] = 24;                       // Length LSB
  iniTimeUTC[5] = 0x00;                     // Length MSB
  iniTimeUTC[6] = 0x10;                     // type
  iniTimeUTC[7] = 0x00;                     // version
  iniTimeUTC[8] = source;                   // ref (source)
  iniTimeUTC[9] = 0x80;                     // leapSecs. Set to 0x80 = unknown
  iniTimeUTC[10] = (uint8_t)(year & 0xFF);  // year LSB
  iniTimeUTC[11] = (uint8_t)(year >> 8);    // year MSB
  iniTimeUTC[12] = month;                   // month starting at 1
  iniTimeUTC[13] = day;                     // day starting at 1
  iniTimeUTC[14] = hour;                    // hour 0:23
  iniTimeUTC[15] = minute;                  // minute 0:59
  iniTimeUTC[16] = second;                  // seconds 0:59
  iniTimeUTC[18] = (uint8_t)(nanos & 0xFF); // nanoseconds LSB
  iniTimeUTC[19] = (uint8_t)((nanos >> 8) & 0xFF);
  iniTimeUTC[20] = (uint8_t)((nanos >> 16) & 0xFF);
  iniTimeUTC[21] = (uint8_t)(nanos >> 24);   // nanoseconds MSB
  iniTimeUTC[22] = (uint8_t)(tAccS & 0xFF);  // seconds part of the accuracy LSB
  iniTimeUTC[23] = (uint8_t)(tAccS >> 8);    // seconds part of the accuracy MSB
  iniTimeUTC[26] = (uint8_t)(tAccNs & 0xFF); // nanoseconds part of the accuracy LSB
  iniTimeUTC[27] = (uint8_t)((tAccNs >> 8) & 0xFF);
  iniTimeUTC[28] = (uint8_t)((tAccNs >> 16) & 0xFF);
  iniTimeUTC[29] = (uint8_t)(tAccNs >> 24); // nanoseconds part of the accuracy MSB

  for (uint8_t i = 2; i < 30; i++) // Calculate the checksum
  {
    iniTimeUTC[30] += iniTimeUTC[i];
    iniTimeUTC[31] += iniTimeUTC[30];
  }

  // Return true if the one packet was pushed successfully
  return (pushAssistNowDataInternal(0, false, iniTimeUTC, 32, mgaAck, maxWait) == 32);
}

// Provide initial position assistance
// The units for ecefX/Y/Z and posAcc (stddev) are cm.
bool DevUBLOXGNSS::setPositionAssistanceXYZ(int32_t ecefX, int32_t ecefY, int32_t ecefZ, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  uint8_t iniPosXYZ[28];       // Create the UBX-MGA-INI-POS_XYZ message by hand
  memset(iniPosXYZ, 0x00, 28); // Set all unused / reserved bytes and the checksum to zero

  iniPosXYZ[0] = UBX_SYNCH_1;         // Sync char 1
  iniPosXYZ[1] = UBX_SYNCH_2;         // Sync char 2
  iniPosXYZ[2] = UBX_CLASS_MGA;       // Class
  iniPosXYZ[3] = UBX_MGA_INI_POS_XYZ; // ID
  iniPosXYZ[4] = 20;                  // Length LSB
  iniPosXYZ[5] = 0x00;                // Length MSB
  iniPosXYZ[6] = 0x00;                // type
  iniPosXYZ[7] = 0x00;                // version

  union // Use a union to convert from int32_t to uint32_t
  {
    int32_t signedLong;
    uint32_t unsignedLong;
  } signedUnsigned;

  signedUnsigned.signedLong = ecefX;
  iniPosXYZ[10] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosXYZ[11] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosXYZ[12] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosXYZ[13] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  signedUnsigned.signedLong = ecefY;
  iniPosXYZ[14] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosXYZ[15] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosXYZ[16] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosXYZ[17] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  signedUnsigned.signedLong = ecefZ;
  iniPosXYZ[18] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosXYZ[19] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosXYZ[20] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosXYZ[21] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  iniPosXYZ[22] = (uint8_t)(posAcc & 0xFF); // LSB
  iniPosXYZ[23] = (uint8_t)((posAcc >> 8) & 0xFF);
  iniPosXYZ[24] = (uint8_t)((posAcc >> 16) & 0xFF);
  iniPosXYZ[25] = (uint8_t)(posAcc >> 24); // MSB

  for (uint8_t i = 2; i < 26; i++) // Calculate the checksum
  {
    iniPosXYZ[26] += iniPosXYZ[i];
    iniPosXYZ[27] += iniPosXYZ[26];
  }

  // Return true if the one packet was pushed successfully
  return (pushAssistNowDataInternal(0, false, iniPosXYZ, 28, mgaAck, maxWait) == 28);
}

// The units for lat and lon are degrees * 1e-7 (WGS84)
// The units for alt (WGS84) and posAcc (stddev) are cm.
bool DevUBLOXGNSS::setPositionAssistanceLLH(int32_t lat, int32_t lon, int32_t alt, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  uint8_t iniPosLLH[28];       // Create the UBX-MGA-INI-POS_LLH message by hand
  memset(iniPosLLH, 0x00, 28); // Set all unused / reserved bytes and the checksum to zero

  iniPosLLH[0] = UBX_SYNCH_1;         // Sync char 1
  iniPosLLH[1] = UBX_SYNCH_2;         // Sync char 2
  iniPosLLH[2] = UBX_CLASS_MGA;       // Class
  iniPosLLH[3] = UBX_MGA_INI_POS_LLH; // ID
  iniPosLLH[4] = 20;                  // Length LSB
  iniPosLLH[5] = 0x00;                // Length MSB
  iniPosLLH[6] = 0x01;                // type
  iniPosLLH[7] = 0x00;                // version

  union // Use a union to convert from int32_t to uint32_t
  {
    int32_t signedLong;
    uint32_t unsignedLong;
  } signedUnsigned;

  signedUnsigned.signedLong = lat;
  iniPosLLH[10] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosLLH[11] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosLLH[12] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosLLH[13] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  signedUnsigned.signedLong = lon;
  iniPosLLH[14] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosLLH[15] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosLLH[16] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosLLH[17] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  signedUnsigned.signedLong = alt;
  iniPosLLH[18] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosLLH[19] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosLLH[20] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosLLH[21] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  iniPosLLH[22] = (uint8_t)(posAcc & 0xFF); // LSB
  iniPosLLH[23] = (uint8_t)((posAcc >> 8) & 0xFF);
  iniPosLLH[24] = (uint8_t)((posAcc >> 16) & 0xFF);
  iniPosLLH[25] = (uint8_t)(posAcc >> 24); // MSB

  for (uint8_t i = 2; i < 26; i++) // Calculate the checksum
  {
    iniPosLLH[26] += iniPosLLH[i];
    iniPosLLH[27] += iniPosLLH[26];
  }

  // Return true if the one packet was pushed successfully
  return (pushAssistNowDataInternal(0, false, iniPosLLH, 28, mgaAck, maxWait) == 28);
}

// Find the start of the AssistNow Offline (UBX_MGA_ANO) data for the chosen day
// The daysIntoFture parameter makes it easy to get the data for (e.g.) tomorrow based on today's date
// Returns numDataBytes if unsuccessful
// TO DO: enhance this so it will find the nearest data for the chosen day - instead of an exact match
size_t DevUBLOXGNSS::findMGAANOForDate(const String &dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture)
{
  return (findMGAANOForDateInternal((const uint8_t *)dataBytes.c_str(), numDataBytes, year, month, day, daysIntoFuture));
}
size_t DevUBLOXGNSS::findMGAANOForDate(const uint8_t *dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture)
{
  return (findMGAANOForDateInternal(dataBytes, numDataBytes, year, month, day, daysIntoFuture));
}
size_t DevUBLOXGNSS::findMGAANOForDateInternal(const uint8_t *dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture)
{
  size_t dataPtr = 0;     // Pointer into dataBytes
  bool dateFound = false; // Flag to indicate when the date has been found

  // Calculate matchDay, matchMonth and matchYear
  uint8_t matchDay = day;
  uint8_t matchMonth = month;
  uint8_t matchYear = (uint8_t)(year - 2000);

  // Add on daysIntoFuture
  uint8_t daysIntoFutureCopy = daysIntoFuture;
  while (daysIntoFutureCopy > 0)
  {
    matchDay++;
    daysIntoFutureCopy--;
    switch (matchMonth)
    {
    case 1:
    case 3:
    case 5:
    case 7:
    case 8:
    case 10:
    case 12:
      if (matchDay == 32)
      {
        matchDay = 1;
        matchMonth++;
        if (matchMonth == 13)
        {
          matchMonth = 1;
          matchYear++;
        }
      }
      break;
    case 4:
    case 6:
    case 9:
    case 11:
      if (matchDay == 31)
      {
        matchDay = 1;
        matchMonth++;
      }
      break;
    default: // February
      if (((matchYear % 4) == 0) && (matchDay == 30))
      {
        matchDay = 1;
        matchMonth++;
      }
      else if (((matchYear % 4) > 0) && (matchDay == 29))
      {
        matchDay = 1;
        matchMonth++;
      }
      break;
    }
  }

  while ((!dateFound) && (dataPtr < numDataBytes)) // Keep going until we have found the date or processed all the bytes
  {
    // Start by checking the validity of the packet being pointed to
    bool dataIsOK = true;

    dataIsOK &= (*(dataBytes + dataPtr + 0) == UBX_SYNCH_1);   // Check for 0xB5
    dataIsOK &= (*(dataBytes + dataPtr + 1) == UBX_SYNCH_2);   // Check for 0x62
    dataIsOK &= (*(dataBytes + dataPtr + 2) == UBX_CLASS_MGA); // Check for class UBX-MGA

    size_t packetLength = ((size_t) * (dataBytes + dataPtr + 4)) | (((size_t) * (dataBytes + dataPtr + 5)) << 8); // Extract the length

    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    // Calculate the checksum bytes
    // Keep going until the end of the packet is reached (payloadPtr == (dataPtr + packetLength))
    // or we reach the end of the AssistNow data (payloadPtr == numDataBytes)
    for (size_t payloadPtr = dataPtr + ((size_t)2); (payloadPtr < (dataPtr + packetLength + ((size_t)6))) && (payloadPtr < numDataBytes); payloadPtr++)
    {
      checksumA += *(dataBytes + payloadPtr);
      checksumB += checksumA;
    }
    // Check the checksum bytes
    dataIsOK &= (checksumA == *(dataBytes + dataPtr + packetLength + ((size_t)6)));
    dataIsOK &= (checksumB == *(dataBytes + dataPtr + packetLength + ((size_t)7)));

    dataIsOK &= ((dataPtr + packetLength + ((size_t)8)) <= numDataBytes); // Check we haven't overrun

    // If the data is valid, check for a date match
    if (dataIsOK)
    {
      if ((*(dataBytes + dataPtr + 3) == UBX_MGA_ANO) && (*(dataBytes + dataPtr + 10) == matchYear) && (*(dataBytes + dataPtr + 11) == matchMonth) && (*(dataBytes + dataPtr + 12) == matchDay))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial.print(F("findMGAANOForDate: found date match at location "));
          _debugSerial.println(dataPtr);
        }
#endif
        dateFound = true;
      }
      else
      {
        // The data is valid, but these are not the droids we are looking for...
        dataPtr += packetLength + ((size_t)8); // Point to the next message
      }
    }
    else
    {

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      // The data was invalid. Send a debug message and then try to find the next 0xB5
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.print(F("findMGAANOForDate: bad data - ignored! dataPtr is "));
        _debugSerial.println(dataPtr);
      }
#endif

      while ((dataPtr < numDataBytes) && (*(dataBytes + ++dataPtr) != UBX_SYNCH_1))
      {
        ; // Increment dataPtr until we are pointing at the next 0xB5 - or we reach the end of the data
      }
    }
  }

  return (dataPtr);
}

// Read the whole navigation data base. The receiver will send all available data from its internal database.
// Data is written to dataBytes. Set maxNumDataBytes to the (maximum) size of dataBytes.
// If the database exceeds maxNumDataBytes, the excess bytes will be lost.
// The function returns the number of database bytes written to dataBytes.
// The return value will be equal to maxNumDataBytes if excess data was received.
// The function will timeout after maxWait milliseconds - in case the final UBX-MGA-ACK was missed.
size_t DevUBLOXGNSS::readNavigationDatabase(uint8_t *dataBytes, size_t maxNumDataBytes, uint16_t maxWait)
{
  // Allocate RAM to store the MGA ACK message
  if (packetUBXMGAACK == nullptr)
    initPacketUBXMGAACK();        // Check that RAM has been allocated for the MGA_ACK data
  if (packetUBXMGAACK == nullptr) // Bail if the RAM allocation failed
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("readNavigationDatabase: packetUBXMGAACK RAM allocation failed!"));
    }
#endif
    return ((size_t)0);
  }
  if (packetUBXMGAACK->head != packetUBXMGAACK->tail) // Does the MGA ACK ringbuffer contain any data?
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("readNavigationDatabase: packetUBXMGAACK contains unprocessed data. Clearing it."));
    }
#endif
    packetUBXMGAACK->tail = packetUBXMGAACK->head; // Clear the buffer by setting the tail equal to the head
  }

  // Allocate RAM to store the MGA DBD messages
  if (packetUBXMGADBD == nullptr)
    initPacketUBXMGADBD();        // Check that RAM has been allocated for the MGA_DBD data
  if (packetUBXMGADBD == nullptr) // Bail if the RAM allocation failed
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("readNavigationDatabase: packetUBXMGADBD RAM allocation failed!"));
    }
#endif
    return ((size_t)0);
  }
  if (packetUBXMGADBD->head != packetUBXMGADBD->tail) // Does the MGA DBD ringbuffer contain any data?
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("readNavigationDatabase: packetUBXMGADBD contains unprocessed data. Clearing it."));
    }
#endif
    packetUBXMGADBD->tail = packetUBXMGADBD->head; // Clear the buffer by setting the tail equal to the head
  }

  // Record what ackAiding is currently set to so we can restore it
  uint8_t currentAckAiding = getAckAiding();
  if (currentAckAiding == 255)
    currentAckAiding = 0; // If the get failed, disable the ACKs when returning
  // Enable ackAiding
  setAckAiding(1);

  // Record what i2cPollingWait is currently set to so we can restore it
  uint8_t currentI2cPollingWait = i2cPollingWait;
  // Set the I2C polling wait to 1ms
  i2cPollingWait = 1;

  // Construct the poll message:
  uint8_t pollNaviDatabase[8];       // Create the UBX-MGA-DBD message by hand
  memset(pollNaviDatabase, 0x00, 8); // Set all unused / reserved bytes and the checksum to zero

  pollNaviDatabase[0] = UBX_SYNCH_1;   // Sync char 1
  pollNaviDatabase[1] = UBX_SYNCH_2;   // Sync char 2
  pollNaviDatabase[2] = UBX_CLASS_MGA; // Class
  pollNaviDatabase[3] = UBX_MGA_DBD;   // ID
  pollNaviDatabase[4] = 0x00;          // Length LSB
  pollNaviDatabase[5] = 0x00;          // Length MSB

  for (uint8_t i = 2; i < 6; i++) // Calculate the checksum
  {
    pollNaviDatabase[6] += pollNaviDatabase[i];
    pollNaviDatabase[7] += pollNaviDatabase[6];
  }

  // Push the poll message to the module.
  // Do not Wait for an ACK - the DBD data will start arriving immediately.
  size_t pushResult = pushAssistNowDataInternal(0, false, pollNaviDatabase, (size_t)8, SFE_UBLOX_MGA_ASSIST_ACK_NO, 0);

  // Check pushResult == 8
  if (pushResult != 8)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("readNavigationDatabase: pushAssistNowDataInternal failed!"));
    }
#endif
    i2cPollingWait = currentI2cPollingWait; // Restore i2cPollingWait
    setAckAiding(currentAckAiding);         // Restore Ack Aiding
    return ((size_t)0);
  }

  // Now keep checking for the arrival of UBX-MGA-DBD packets and write them to dataBytes
  bool keepGoing = true;
  unsigned long startTime = millis();
  uint32_t databaseEntriesRX = 0; // Keep track of how many database entries are received
  size_t numBytesReceived = 0;    // Keep track of how many bytes are received

  while (keepGoing && (millis() < (startTime + maxWait)))
  {
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID. We could be pushing this from another thread...

    while (packetUBXMGADBD->head != packetUBXMGADBD->tail) // Does the MGA DBD ringbuffer contain any data?
    {
      // The data will be valid - process will have already checked it. So we can simply copy the data into dataBuffer.
      // We do not need to check if there is room to store the entire database entry. pushAssistNowData will check the data before pushing it.
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryHeader1;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryHeader2;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryClass;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryID;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryLenLSB;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryLenMSB;
      size_t msgLen = (((size_t)packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryLenMSB) * 256) + ((size_t)packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryLenLSB);
      for (size_t i = 0; i < msgLen; i++)
      {
        if (numBytesReceived < maxNumDataBytes)
          *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntry[i];
      }
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryChecksumA;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryChecksumB;

      // Increment the tail
      packetUBXMGADBD->tail++;
      if (packetUBXMGADBD->tail == UBX_MGA_DBD_RINGBUFFER_LEN)
        packetUBXMGADBD->tail = 0;

      databaseEntriesRX++; // Increment the number of entries received
    }

    // The final MGA-ACK is sent at the end of the DBD packets. So, we need to check the ACK buffer _after_ the DBD buffer.
    while (packetUBXMGAACK->head != packetUBXMGAACK->tail) // Does the MGA ACK ringbuffer contain any data?
    {
      // Check if we've received the correct ACK
      bool idMatch = (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgId == UBX_MGA_DBD); // Check if the message ID matches

      bool dataAckd = true;
      dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[0] == (uint8_t)(databaseEntriesRX & 0xFF)); // Check if the ACK contents match databaseEntriesRX
      dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[1] == (uint8_t)((databaseEntriesRX >> 8) & 0xFF));
      dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[2] == (uint8_t)((databaseEntriesRX >> 16) & 0xFF));
      dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[3] == (uint8_t)((databaseEntriesRX >> 24) & 0xFF));

      if (idMatch && dataAckd) // Is the ACK valid?
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial.print(F("readNavigationDatabase: ACK received. databaseEntriesRX is "));
          _debugSerial.print(databaseEntriesRX);
          _debugSerial.print(F(". numBytesReceived is "));
          _debugSerial.print(numBytesReceived);
          _debugSerial.print(F(". DBD read complete after "));
          _debugSerial.print(millis() - startTime);
          _debugSerial.println(F(" ms"));
        }
#endif
        keepGoing = false;
      }
      else if (idMatch)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial.print(F("readNavigationDatabase: unexpected ACK received. databaseEntriesRX is 0x"));
          _debugSerial.print(databaseEntriesRX, HEX);
          _debugSerial.print(F(". msgPayloadStart is 0x"));
          for (uint8_t i = 4; i > 0; i--)
          {
            if (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[i - 1] < 0x10)
              _debugSerial.print(F("0"));
            _debugSerial.print(packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[i - 1], HEX);
          }
          _debugSerial.println();
        }
#endif
      }

      // Increment the tail
      packetUBXMGAACK->tail++;
      if (packetUBXMGAACK->tail == UBX_MGA_ACK_DATA0_RINGBUFFER_LEN)
        packetUBXMGAACK->tail = 0;
    }
  }

  if (keepGoing) // If keepGoing is still true, we must have timed out
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("readNavigationDatabase: DBD RX timed out!"));
    }
#endif
  }

  i2cPollingWait = currentI2cPollingWait; // Restore i2cPollingWait
  setAckAiding(currentAckAiding);         // Restore Ack Aiding

  return (numBytesReceived);
}

// PRIVATE: Allocate RAM for packetUBXMGADBD and initialize it
bool DevUBLOXGNSS::initPacketUBXMGADBD()
{
  packetUBXMGADBD = new UBX_MGA_DBD_t; // Allocate RAM for the main struct
  if (packetUBXMGADBD == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXMGADBD: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXMGADBD->head = 0; // Initialize the ring buffer pointers
  packetUBXMGADBD->tail = 0;
  return (true);
}

// Support for data logging

// Set the file buffer size. This must be called _before_ .begin
void DevUBLOXGNSS::setFileBufferSize(uint16_t bufferSize)
{
  fileBufferSize = bufferSize;
}

// Return the file buffer size
uint16_t DevUBLOXGNSS::getFileBufferSize(void)
{
  return (fileBufferSize);
}

// Extract numBytes of data from the file buffer. Copy it to destination.
// It is the user's responsibility to ensure destination is large enough.
// Returns the number of bytes extracted - which may be less than numBytes.
uint16_t DevUBLOXGNSS::extractFileBufferData(uint8_t *destination, uint16_t numBytes)
{
  // Check how many bytes are available in the buffer
  uint16_t bytesAvailable = fileBufferSpaceUsed();
  if (numBytes > bytesAvailable) // Limit numBytes if required
    numBytes = bytesAvailable;

  // Start copying at fileBufferTail. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = fileBufferSize - fileBufferTail; // How much space is available 'above' Tail?
  if (bytesBeforeWrapAround > numBytes)                             // Will we need to wrap-around?
  {
    bytesBeforeWrapAround = numBytes; // We need to wrap-around
  }
  memcpy(destination, &ubxFileBuffer[fileBufferTail], bytesBeforeWrapAround); // Copy the data out of the buffer

  // Is there any data leftover which we need to copy from the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&destination[bytesBeforeWrapAround], &ubxFileBuffer[0], bytesLeftToCopy); // Copy the remaining data out of the buffer
    fileBufferTail = bytesLeftToCopy;                                                // Update Tail. The next byte to be read will be read from here.
  }
  else
  {
    fileBufferTail += numBytes; // Only update Tail. The next byte to be read will be read from here.
  }

  return (numBytes); // Return the number of bytes extracted
}

// Returns the number of bytes available in file buffer which are waiting to be read
uint16_t DevUBLOXGNSS::fileBufferAvailable(void)
{
  return (fileBufferSpaceUsed());
}

// Returns the maximum number of bytes which the file buffer contained.
// Handy for checking the buffer is large enough to handle all the incoming data.
uint16_t DevUBLOXGNSS::getMaxFileBufferAvail(void)
{
  return (fileBufferMaxAvail);
}

// Clear the file buffer - discard all contents
void DevUBLOXGNSS::clearFileBuffer(void)
{
  if (fileBufferSize == 0) // Bail if the user has not called setFileBufferSize (probably redundant)
    return;
  fileBufferTail = fileBufferHead;
}

// Reset fileBufferMaxAvail
void DevUBLOXGNSS::clearMaxFileBufferAvail(void)
{
  fileBufferMaxAvail = 0;
}

// PRIVATE: Create the file buffer. Called by .begin
bool DevUBLOXGNSS::createFileBuffer(void)
{
  if (fileBufferSize == 0) // Bail if the user has not called setFileBufferSize
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("createFileBuffer: Warning. fileBufferSize is zero. Data logging is not possible."));
    }
#endif
    return (false);
  }

  if (ubxFileBuffer != nullptr) // Bail if RAM has already been allocated for the file buffer
  {                             // This will happen if you call .begin more than once - without calling .end first
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("createFileBuffer: Warning. File buffer already exists. Skipping..."));
    }
#endif
    return (false);
  }

  ubxFileBuffer = new uint8_t[fileBufferSize]; // Allocate RAM for the buffer

  if (ubxFileBuffer == nullptr) // Check if the new (alloc) was successful
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("createFileBuffer: RAM alloc failed!"));
    }
    fileBufferSize = 0; // Set file buffer size so user can check with getFileBufferSize (ubxFileBuffer is protected)
    return (false);
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("createFileBuffer: fileBufferSize is: "));
    _debugSerial.println(fileBufferSize);
  }
#endif

  fileBufferHead = 0; // Initialize head and tail
  fileBufferTail = 0;

  return (true);
}

// PRIVATE: Check how much space is available in the buffer
uint16_t DevUBLOXGNSS::fileBufferSpaceAvailable(void)
{
  return (fileBufferSize - fileBufferSpaceUsed());
}

// PRIVATE: Check how much space is used in the buffer
uint16_t DevUBLOXGNSS::fileBufferSpaceUsed(void)
{
  if (fileBufferHead >= fileBufferTail) // Check if wrap-around has occurred
  {
    // Wrap-around has not occurred so do a simple subtraction
    return (fileBufferHead - fileBufferTail);
  }
  else
  {
    // Wrap-around has occurred so do a simple subtraction but add in the fileBufferSize
    return ((uint16_t)(((uint32_t)fileBufferHead + (uint32_t)fileBufferSize) - (uint32_t)fileBufferTail));
  }
}

// PRIVATE: Add a UBX packet to the file buffer
bool DevUBLOXGNSS::storePacket(ubxPacket *msg)
{
  // First, check that the file buffer has been created
  if ((ubxFileBuffer == nullptr) || (fileBufferSize == 0))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("storePacket: file buffer not available!"));
    }
#endif
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  uint16_t totalLength = msg->len + 8; // Total length. Include sync chars, class, id, length and checksum bytes
  if (totalLength > fileBufferSpaceAvailable())
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("storePacket: insufficient space available! Data will be lost!"));
    }
#endif
    return (false);
  }

  // Store the two sync chars
  uint8_t sync_chars[] = {UBX_SYNCH_1, UBX_SYNCH_2};
  writeToFileBuffer(sync_chars, 2);

  // Store the Class & ID
  writeToFileBuffer(&msg->cls, 1);
  writeToFileBuffer(&msg->id, 1);

  // Store the length. Ensure length is little-endian
  uint8_t msg_length[2];
  msg_length[0] = msg->len & 0xFF;
  msg_length[1] = msg->len >> 8;
  writeToFileBuffer(msg_length, 2);

  // Store the payload
  writeToFileBuffer(msg->payload, msg->len);

  // Store the checksum
  writeToFileBuffer(&msg->checksumA, 1);
  writeToFileBuffer(&msg->checksumB, 1);

  return (true);
}

// PRIVATE: Add theBytes to the file buffer
bool DevUBLOXGNSS::storeFileBytes(uint8_t *theBytes, uint16_t numBytes)
{
  // First, check that the file buffer has been created
  if ((ubxFileBuffer == nullptr) || (fileBufferSize == 0))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("storeFileBytes: file buffer not available!"));
    }
#endif
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  if (numBytes > fileBufferSpaceAvailable())
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("storeFileBytes: insufficient space available! Data will be lost!"));
    }
#endif
    return (false);
  }

  // There is room for all the data in the buffer so copy the data into the buffer
  writeToFileBuffer(theBytes, numBytes);

  return (true);
}

// PRIVATE: Write theBytes to the file buffer
void DevUBLOXGNSS::writeToFileBuffer(uint8_t *theBytes, uint16_t numBytes)
{
  // Start writing at fileBufferHead. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = fileBufferSize - fileBufferHead; // How much space is available 'above' Head?
  if (bytesBeforeWrapAround > numBytes)                             // Is there enough room for all the data?
  {
    bytesBeforeWrapAround = numBytes; // There is enough room for all the data
  }
  memcpy(&ubxFileBuffer[fileBufferHead], theBytes, bytesBeforeWrapAround); // Copy the data into the buffer

  // Is there any data leftover which we need to copy to the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&ubxFileBuffer[0], &theBytes[bytesBeforeWrapAround], bytesLeftToCopy); // Copy the remaining data into the buffer
    fileBufferHead = bytesLeftToCopy;                                             // Update Head. The next byte written will be written here.
  }
  else
  {
    fileBufferHead += numBytes; // Only update Head. The next byte written will be written here.
  }

  // Update fileBufferMaxAvail if required
  uint16_t bytesInBuffer = fileBufferSpaceUsed();
  if (bytesInBuffer > fileBufferMaxAvail)
    fileBufferMaxAvail = bytesInBuffer;
}

// Support for RTCM buffering

// Set the RTCM buffer size. This must be called _before_ .begin
void DevUBLOXGNSS::setRTCMBufferSize(uint16_t bufferSize)
{
  rtcmBufferSize = bufferSize;
}

// Return the RTCM buffer size
uint16_t DevUBLOXGNSS::getRTCMBufferSize(void)
{
  return (rtcmBufferSize);
}

// Extract numBytes of data from the RTCM buffer. Copy it to destination.
// It is the user's responsibility to ensure destination is large enough.
// Returns the number of bytes extracted - which may be less than numBytes.
uint16_t DevUBLOXGNSS::extractRTCMBufferData(uint8_t *destination, uint16_t numBytes)
{
  // Check how many bytes are available in the buffer
  uint16_t bytesAvailable = rtcmBufferSpaceUsed();
  if (numBytes > bytesAvailable) // Limit numBytes if required
    numBytes = bytesAvailable;

  // Start copying at rtcmBufferTail. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = rtcmBufferSize - rtcmBufferTail; // How much space is available 'above' Tail?
  if (bytesBeforeWrapAround > numBytes)                             // Will we need to wrap-around?
  {
    bytesBeforeWrapAround = numBytes; // We need to wrap-around
  }
  memcpy(destination, &rtcmBuffer[rtcmBufferTail], bytesBeforeWrapAround); // Copy the data out of the buffer

  // Is there any data leftover which we need to copy from the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&destination[bytesBeforeWrapAround], &rtcmBuffer[0], bytesLeftToCopy); // Copy the remaining data out of the buffer
    rtcmBufferTail = bytesLeftToCopy;                                             // Update Tail. The next byte to be read will be read from here.
  }
  else
  {
    rtcmBufferTail += numBytes; // Only update Tail. The next byte to be read will be read from here.
  }

  return (numBytes); // Return the number of bytes extracted
}

// Returns the number of bytes available in RTCM buffer which are waiting to be read
uint16_t DevUBLOXGNSS::rtcmBufferAvailable(void)
{
  return (rtcmBufferSpaceUsed());
}

// Clear the RTCM buffer - discard all contents
void DevUBLOXGNSS::clearRTCMBuffer(void)
{
  if (rtcmBufferSize == 0) // Bail if the user has not called setRTCMBufferSize (probably redundant)
    return;
  rtcmBufferTail = rtcmBufferHead;
}

// PRIVATE: Create the RTCM buffer. Called by .begin
bool DevUBLOXGNSS::createRTCMBuffer(void)
{
  if (rtcmBufferSize == 0) // Bail if the user has not called setRTCMBufferSize
  {
    return (false);
  }

  if (rtcmBuffer != nullptr) // Bail if RAM has already been allocated for the buffer
  {                          // This will happen if you call .begin more than once - without calling .end first
    return (false);
  }

  rtcmBuffer = new uint8_t[rtcmBufferSize]; // Allocate RAM for the buffer

  if (rtcmBuffer == nullptr) // Check if the new (alloc) was successful
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("createRTCMBuffer: RAM alloc failed!"));
    }
    rtcmBufferSize = 0; // Set buffer size so user can check with getRTCMBufferSize (rtcmBuffer is protected)
    return (false);
  }

  rtcmBufferHead = 0; // Initialize head and tail
  rtcmBufferTail = 0;

  return (true);
}

// PRIVATE: Check how much space is available in the buffer
uint16_t DevUBLOXGNSS::rtcmBufferSpaceAvailable(void)
{
  return (rtcmBufferSize - rtcmBufferSpaceUsed());
}

// PRIVATE: Check how much space is used in the buffer
uint16_t DevUBLOXGNSS::rtcmBufferSpaceUsed(void)
{
  if (rtcmBufferHead >= rtcmBufferTail) // Check if wrap-around has occurred
  {
    // Wrap-around has not occurred so do a simple subtraction
    return (rtcmBufferHead - rtcmBufferTail);
  }
  else
  {
    // Wrap-around has occurred so do a simple subtraction but add in the rtcmBufferSize
    return ((uint16_t)(((uint32_t)rtcmBufferHead + (uint32_t)rtcmBufferSize) - (uint32_t)rtcmBufferTail));
  }
}

// PRIVATE: Add theBytes to the RTCM buffer
bool DevUBLOXGNSS::storeRTCMBytes(uint8_t *theBytes, uint16_t numBytes)
{
  // First, check that the file buffer has been created
  if ((rtcmBuffer == nullptr) || (rtcmBufferSize == 0))
  {
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  if (numBytes > rtcmBufferSpaceAvailable())
  {
    return (false);
  }

  // There is room for all the data in the buffer so copy the data into the buffer
  writeToRTCMBuffer(theBytes, numBytes);

  return (true);
}

// PRIVATE: Write theBytes to the RTCM buffer
void DevUBLOXGNSS::writeToRTCMBuffer(uint8_t *theBytes, uint16_t numBytes)
{
  // Start writing at fileBufferHead. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = rtcmBufferSize - rtcmBufferHead; // How much space is available 'above' Head?
  if (bytesBeforeWrapAround > numBytes)                             // Is there enough room for all the data?
  {
    bytesBeforeWrapAround = numBytes; // There is enough room for all the data
  }
  memcpy(&rtcmBuffer[rtcmBufferHead], theBytes, bytesBeforeWrapAround); // Copy the data into the buffer

  // Is there any data leftover which we need to copy to the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&rtcmBuffer[0], &theBytes[bytesBeforeWrapAround], bytesLeftToCopy); // Copy the remaining data into the buffer
    rtcmBufferHead = bytesLeftToCopy;                                          // Update Head. The next byte written will be written here.
  }
  else
  {
    rtcmBufferHead += numBytes; // Only update Head. The next byte written will be written here.
  }
}

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
void DevUBLOXGNSS::extractRTCM1005(RTCM_1005_data_t *destination, uint8_t *source)
{
  destination->MessageNumber = extractUnsignedBits(source, 0, 12);
  destination->ReferenceStationID = extractUnsignedBits(source, 12, 12);
  destination->ITRFRealizationYear = extractUnsignedBits(source, 24, 6);
  destination->GPSIndicator = extractUnsignedBits(source, 30, 1);
  destination->GLONASSIndicator = extractUnsignedBits(source, 31, 1);
  destination->GalileoIndicator = extractUnsignedBits(source, 32, 1);
  destination->ReferenceStationIndicator = extractUnsignedBits(source, 33, 1);
  destination->AntennaReferencePointECEFX = extractSignedBits(source, 34, 38);
  destination->SingleReceiverOscillatorIndicator = extractUnsignedBits(source, 72, 1);
  destination->Reserved = extractUnsignedBits(source, 73, 1);
  destination->AntennaReferencePointECEFY = extractSignedBits(source, 74, 38);
  destination->QuarterCycleIndicator = extractUnsignedBits(source, 112, 2);
  destination->AntennaReferencePointECEFZ = extractSignedBits(source, 114, 38);
}

void DevUBLOXGNSS::extractRTCM1006(RTCM_1006_data_t *destination, uint8_t *source)
{
  destination->MessageNumber = extractUnsignedBits(source, 0, 12);
  destination->ReferenceStationID = extractUnsignedBits(source, 12, 12);
  destination->ITRFRealizationYear = extractUnsignedBits(source, 24, 6);
  destination->GPSIndicator = extractUnsignedBits(source, 30, 1);
  destination->GLONASSIndicator = extractUnsignedBits(source, 31, 1);
  destination->GalileoIndicator = extractUnsignedBits(source, 32, 1);
  destination->ReferenceStationIndicator = extractUnsignedBits(source, 33, 1);
  destination->AntennaReferencePointECEFX = extractSignedBits(source, 34, 38);
  destination->SingleReceiverOscillatorIndicator = extractUnsignedBits(source, 72, 1);
  destination->Reserved = extractUnsignedBits(source, 73, 1);
  destination->AntennaReferencePointECEFY = extractSignedBits(source, 74, 38);
  destination->QuarterCycleIndicator = extractUnsignedBits(source, 112, 2);
  destination->AntennaReferencePointECEFZ = extractSignedBits(source, 114, 38);
  destination->AntennaHeight = extractUnsignedBits(source, 152, 16);
}

void DevUBLOXGNSS::parseRTCM1005(uint8_t *dataBytes, size_t numDataBytes)
{
  // This is called from inside pushRawData. It thoroughly examines dataBytes and will copy any RTCM 1005 messages it finds into storage.
  // It keeps a local copy of the data so it does not matter if the message spans multiple calls to pushRawData.

  static uint8_t rtcm1005store[RTCM_1005_MSG_LEN_BYTES + 6];
  static uint8_t bytesStored;

  enum parse1005states
  {
    waitingForD3,
    expecting00,
    expecting13,
    expecting3E,
    expectingDn,
    storingBytes,
  };
  static parse1005states parse1005state = waitingForD3;

  for (size_t i = 0; i < numDataBytes; i++) // Step through each byte
  {
    switch (parse1005state)
    {
    case waitingForD3:
      if (*(dataBytes + i) == 0xD3)
      {
        rtcm1005store[0] = 0xD3;
        parse1005state = expecting00;
      }
      break;
    case expecting00:
      if (*(dataBytes + i) == 0x00)
      {
        rtcm1005store[1] = 0x00;
        parse1005state = expecting13;
      }
      else
      {
        parse1005state = waitingForD3;
      }
      break;
    case expecting13:
      if (*(dataBytes + i) == 0x13)
      {
        rtcm1005store[2] = 0x13;
        parse1005state = expecting3E;
      }
      else
      {
        parse1005state = waitingForD3;
      }
      break;
    case expecting3E:
      if (*(dataBytes + i) == 0x3E)
      {
        rtcm1005store[3] = 0x3E;
        parse1005state = expectingDn;
      }
      else
      {
        parse1005state = waitingForD3;
      }
      break;
    case expectingDn:
      if (((*(dataBytes + i)) & 0xF0) == 0xD0)
      {
        rtcm1005store[4] = *(dataBytes + i);
        parse1005state = storingBytes;
        bytesStored = 5;
      }
      else
      {
        parse1005state = waitingForD3;
      }
      break;
    case storingBytes:
      rtcm1005store[bytesStored++] = *(dataBytes + i);
      if (bytesStored == RTCM_1005_MSG_LEN_BYTES + 6) // All data received?
      {
        parse1005state = waitingForD3;
        uint32_t checksum = 0;
        for (size_t j = 0; j < (RTCM_1005_MSG_LEN_BYTES + 3); j++)
          crc24q(rtcm1005store[j], &checksum);
        if (rtcm1005store[RTCM_1005_MSG_LEN_BYTES + 3] == ((checksum >> 16) & 0xFF)) // Check the checksum
          if (rtcm1005store[RTCM_1005_MSG_LEN_BYTES + 4] == ((checksum >> 8) & 0xFF))
            if (rtcm1005store[RTCM_1005_MSG_LEN_BYTES + 5] == (checksum & 0xFF))
            {
              extractRTCM1005(&rtcmInputStorage.rtcm1005, &rtcm1005store[3]);
              rtcmInputStorage.flags.bits.dataValid1005 = 1;
              rtcmInputStorage.flags.bits.dataRead1005 = 0;
              return; // Return now - to avoid processing the remainder of the data
            }
      }
      break;
    }
  }
}

void DevUBLOXGNSS::parseRTCM1006(uint8_t *dataBytes, size_t numDataBytes)
{
  // This is called from inside pushRawData. It thoroughly examines dataBytes and will copy any RTCM 1006 messages it finds into storage.
  // It keeps a local copy of the data so it does not matter if the message spans multiple calls to pushRawData.

  static uint8_t rtcm1006store[RTCM_1006_MSG_LEN_BYTES + 6];
  static uint8_t bytesStored;

  enum parse1006states
  {
    waitingForD3,
    expecting00,
    expecting15,
    expecting3E,
    expectingEn,
    storingBytes,
  };
  static parse1006states parse1006state = waitingForD3;

  for (size_t i = 0; i < numDataBytes; i++) // Step through each byte
  {
    switch (parse1006state)
    {
    case waitingForD3:
      if (*(dataBytes + i) == 0xD3)
      {
        rtcm1006store[0] = 0xD3;
        parse1006state = expecting00;
      }
      break;
    case expecting00:
      if (*(dataBytes + i) == 0x00)
      {
        rtcm1006store[1] = 0x00;
        parse1006state = expecting15;
      }
      else
      {
        parse1006state = waitingForD3;
      }
      break;
    case expecting15:
      if (*(dataBytes + i) == 0x15)
      {
        rtcm1006store[2] = 0x15;
        parse1006state = expecting3E;
      }
      else
      {
        parse1006state = waitingForD3;
      }
      break;
    case expecting3E:
      if (*(dataBytes + i) == 0x3E)
      {
        rtcm1006store[3] = 0x3E;
        parse1006state = expectingEn;
      }
      else
      {
        parse1006state = waitingForD3;
      }
      break;
    case expectingEn:
      if (((*(dataBytes + i)) & 0xF0) == 0xE0)
      {
        rtcm1006store[4] = *(dataBytes + i);
        parse1006state = storingBytes;
        bytesStored = 5;
      }
      else
      {
        parse1006state = waitingForD3;
      }
      break;
    case storingBytes:
      rtcm1006store[bytesStored++] = *(dataBytes + i);
      if (bytesStored == RTCM_1006_MSG_LEN_BYTES + 6) // All data received?
      {
        parse1006state = waitingForD3;
        uint32_t checksum = 0;
        for (size_t j = 0; j < (RTCM_1006_MSG_LEN_BYTES + 3); j++)
          crc24q(rtcm1006store[j], &checksum);

        if (rtcm1006store[RTCM_1006_MSG_LEN_BYTES + 3] == ((checksum >> 16) & 0xFF)) // Check the checksum
          if (rtcm1006store[RTCM_1006_MSG_LEN_BYTES + 4] == ((checksum >> 8) & 0xFF))
            if (rtcm1006store[RTCM_1006_MSG_LEN_BYTES + 5] == (checksum & 0xFF))
            {
              extractRTCM1006(&rtcmInputStorage.rtcm1006, &rtcm1006store[3]);
              rtcmInputStorage.flags.bits.dataValid1006 = 1;
              rtcmInputStorage.flags.bits.dataRead1006 = 0;
              return; // Return now - to avoid processing the remainder of the data
            }
      }
      break;
    }
  }
}
#endif

//=-=-=-=-=-=-=-= Specific commands =-=-=-=-=-=-=-==-=-=-=-=-=-=-=
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Changes the I2C address that the u-blox module responds to
// 0x42 is the default but can be changed with this command
// Note: the module stores the address in shifted format - not unshifted.
// We need to shift left by one bit to compensate.
bool DevUBLOXGNSS::setI2CAddress(uint8_t deviceAddress, uint8_t layer, uint16_t maxWait)
{
  return setVal8(UBLOX_CFG_I2C_ADDRESS, deviceAddress << 1, layer, maxWait); // Change the I2C address. Shift left by one bit.
}

// Changes the serial baud rate of the u-blox module, can't return success/fail 'cause ACK from modem
// is lost due to baud rate change
bool DevUBLOXGNSS::setSerialRate(uint32_t baudrate, uint8_t uartPort, uint8_t layer, uint16_t maxWait)
{
  if (uartPort == COM_PORT_UART1)
    return setVal32(UBLOX_CFG_UART1_BAUDRATE, baudrate, layer, maxWait);
  else if (uartPort == COM_PORT_UART2)
    return setVal32(UBLOX_CFG_UART2_BAUDRATE, baudrate, layer, maxWait);
  else
    return false;
}

// Configure a port to output UBX, NMEA, RTCM3 or a combination thereof
bool DevUBLOXGNSS::setI2COutput(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_I2COUTPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_I2COUTPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_I2COUTPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support RTCM3
  return result;
}
bool DevUBLOXGNSS::setUART1Output(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_UART1OUTPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_UART1OUTPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_UART1OUTPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support RTCM3
  return result;
}
bool DevUBLOXGNSS::setUART2Output(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_UART2OUTPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_UART2OUTPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_UART2OUTPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support RTCM3
  return result;
}
bool DevUBLOXGNSS::setUSBOutput(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_USBOUTPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_USBOUTPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_USBOUTPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support RTCM3
  return result;
}
bool DevUBLOXGNSS::setSPIOutput(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_SPIOUTPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_SPIOUTPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_SPIOUTPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support RTCM3
  return result;
}

// Configure a port to input UBX, NMEA, RTCM3, SPARTN or a combination thereof
bool DevUBLOXGNSS::setI2CInput(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_I2CINPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_I2CINPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_I2CINPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait);  // This will be NACK'd if the module does not support RTCM3
  result |= setVal8(UBLOX_CFG_I2CINPROT_SPARTN, (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support SPARTN
  return result;
}
bool DevUBLOXGNSS::setUART1Input(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_UART1INPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_UART1INPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_UART1INPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait);  // This will be NACK'd if the module does not support RTCM3
  result |= setVal8(UBLOX_CFG_UART1INPROT_SPARTN, (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support SPARTN
  return result;
}
bool DevUBLOXGNSS::setUART2Input(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_UART2INPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_UART2INPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_UART2INPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait);  // This will be NACK'd if the module does not support RTCM3
  result |= setVal8(UBLOX_CFG_UART2INPROT_SPARTN, (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support SPARTN
  return result;
}
bool DevUBLOXGNSS::setUSBInput(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_USBINPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_USBINPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_USBINPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait);  // This will be NACK'd if the module does not support RTCM3
  result |= setVal8(UBLOX_CFG_USBINPROT_SPARTN, (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support SPARTN
  return result;
}
bool DevUBLOXGNSS::setSPIInput(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_SPIINPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_SPIINPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_SPIINPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait);  // This will be NACK'd if the module does not support RTCM3
  result |= setVal8(UBLOX_CFG_SPIINPROT_SPARTN, (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support SPARTN
  return result;
}

// Want to see the NMEA messages on the Serial port? Here's how
void DevUBLOXGNSS::setNMEAOutputPort(Print &outputPort)
{
  _nmeaOutputPort.init(outputPort); // Store the port from user
}

// Want to see the RTCM messages on the Serial port? Here's how
void DevUBLOXGNSS::setRTCMOutputPort(Print &outputPort)
{
  _rtcmOutputPort.init(outputPort); // Store the port from user
}

// Want to see the UBX messages on the Serial port? Here's how
void DevUBLOXGNSS::setUBXOutputPort(Print &outputPort)
{
  _ubxOutputPort.init(outputPort); // Store the port from user
}

void DevUBLOXGNSS::setOutputPort(Print &outputPort)
{
  _outputPort.init(outputPort); // Store the port from user
}

// Reset to defaults

void DevUBLOXGNSS::factoryReset()
{
  // Copy default settings to permanent
  // Note: this does not load the permanent configuration into the current configuration. Calling factoryDefault() will do that.
  uint8_t clearMemory[13] = {0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0xff};
  cfgCfg(clearMemory, 13, 0);
  hardReset(); // cause factory default config to actually be loaded and used cleanly
}

void DevUBLOXGNSS::hardReset()
{
  // Issue hard reset
  uint8_t softwareResetGNSS[4] = {0xff, 0xff, 0, 0};
  cfgRst(softwareResetGNSS, 4);
}

void DevUBLOXGNSS::softwareResetGNSSOnly()
{
  // Issue controlled software reset (GNSS only)
  uint8_t softwareResetGNSS[4] = {0, 0, 0x02, 0};
  cfgRst(softwareResetGNSS, 4);
}

void DevUBLOXGNSS::softwareEnableGNSS(bool enable)
{
  // Issue controlled software reset (GNSS only)
  uint8_t softwareEnable[4] = {0, 0, 0, 0};
  softwareEnable[2] = enable ? 0x09 : 0x08; // 0x09 = start GNSS, 0x08 = stop GNSS
  cfgRst(softwareEnable, 4);
}

void DevUBLOXGNSS::cfgRst(uint8_t *data, uint8_t len)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RST;
  packetCfg.len = len;
  packetCfg.startingSpot = 0;
  for (uint8_t i = 0; i < len; i++)
    payloadCfg[i] = *data++;
  sendCommand(&packetCfg, 0); // don't expect ACK
}

// Reset module to factory defaults
// This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
bool DevUBLOXGNSS::factoryDefault(uint16_t maxWait)
{
  uint8_t configSelective[12];

  // Clear packet payload
  memset(configSelective, 0, 12);

  configSelective[0] = 0xFF; // Set any bit in the clearMask field to clear saved config
  configSelective[1] = 0xFF;
  configSelective[8] = 0xFF; // Set any bit in the loadMask field to discard current config and rebuild from lower non-volatile memory layers
  configSelective[9] = 0xFF;

  return (cfgCfg(configSelective, 12, maxWait));
}

// Save configuration to BBR / Flash

// Save current configuration to flash and BBR (battery backed RAM)
// This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
bool DevUBLOXGNSS::saveConfiguration(uint16_t maxWait)
{
  uint8_t configSelective[12];

  // Clear packet payload
  memset(configSelective, 0, 12);

  configSelective[4] = 0xFF; // Set any bit in the saveMask field to save current config to Flash and BBR
  configSelective[5] = 0xFF;

  return (cfgCfg(configSelective, 12, maxWait));
}

// Save the selected configuration sub-sections to flash and BBR (battery backed RAM)
// This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
bool DevUBLOXGNSS::saveConfigSelective(uint32_t configMask, uint16_t maxWait)
{
  uint8_t configSelective[12];

  // Clear packet payload
  memset(configSelective, 0, 12);

  configSelective[4] = configMask & 0xFF; // Set the appropriate bits in the saveMask field to save current config to Flash and BBR
  configSelective[5] = (configMask >> 8) & 0xFF;
  configSelective[6] = (configMask >> 16) & 0xFF;
  configSelective[7] = (configMask >> 24) & 0xFF;

  return (cfgCfg(configSelective, 12, maxWait));
}

bool DevUBLOXGNSS::cfgCfg(uint8_t *data, uint8_t len, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_CFG;
  packetCfg.len = len;
  packetCfg.startingSpot = 0;
  for (uint8_t i = 0; i < len; i++)
    payloadCfg[i] = *data++;
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Functions used for RTK and base station setup

// Control Survey-In for NEO-M8P
bool DevUBLOXGNSS::setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint8_t layer, uint16_t maxWait)
{
  return (setSurveyModeFull(mode, (uint32_t)observationTime, requiredAccuracy, layer, maxWait));
}
bool DevUBLOXGNSS::setSurveyModeFull(uint8_t mode, uint32_t observationTime, float requiredAccuracy, uint8_t layer, uint16_t maxWait)
{
  uint32_t svinAccLimit = (uint32_t)(requiredAccuracy * 10000.0); // Convert m to 0.1mm

  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_TMODE_MODE, mode);
  result &= addCfgValset32(UBLOX_CFG_TMODE_SVIN_MIN_DUR, observationTime);
  result &= addCfgValset32(UBLOX_CFG_TMODE_SVIN_ACC_LIMIT, svinAccLimit);
  result &= sendCfgValset(maxWait);

  return result;
}

// Begin Survey-In for NEO-M8P
bool DevUBLOXGNSS::enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint8_t layer, uint16_t maxWait)
{
  return (setSurveyModeFull(SVIN_MODE_ENABLE, (uint32_t)observationTime, requiredAccuracy, layer, maxWait));
}
bool DevUBLOXGNSS::enableSurveyModeFull(uint32_t observationTime, float requiredAccuracy, uint8_t layer, uint16_t maxWait)
{
  return (setSurveyModeFull(SVIN_MODE_ENABLE, observationTime, requiredAccuracy, layer, maxWait));
}

// Stop Survey-In for NEO-M8P
bool DevUBLOXGNSS::disableSurveyMode(uint8_t layer, uint16_t maxWait)
{
  return (setSurveyMode(SVIN_MODE_DISABLE, 0, 0, layer, maxWait));
}

// Set the ECEF or Lat/Long coordinates of a receiver
// This imediately puts the receiver in TIME mode (fixed) and will begin outputting RTCM sentences if enabled
// This is helpful once an antenna's position has been established. See this tutorial: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station#gather-raw-gnss-data
//  For ECEF the units are: cm, 0.1mm, cm, 0.1mm, cm, 0.1mm
//  For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
bool DevUBLOXGNSS::setStaticPosition(int32_t ecefXOrLat, int8_t ecefXOrLatHP, int32_t ecefYOrLon, int8_t ecefYOrLonHP, int32_t ecefZOrAlt, int8_t ecefZOrAltHP, bool latLong, uint8_t layer, uint16_t maxWait)
{
  unsignedSigned32 converter32;
  unsignedSigned8 converter8;
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_TMODE_MODE, SVIN_MODE_FIXED);
  result &= addCfgValset8(UBLOX_CFG_TMODE_POS_TYPE, (uint8_t)latLong);
  converter32.signed32 = ecefXOrLat;
  result &= addCfgValset32(latLong ? UBLOX_CFG_TMODE_LAT : UBLOX_CFG_TMODE_ECEF_X, converter32.unsigned32);
  converter32.signed32 = ecefYOrLon;
  result &= addCfgValset32(latLong ? UBLOX_CFG_TMODE_LON : UBLOX_CFG_TMODE_ECEF_Y, converter32.unsigned32);
  converter32.signed32 = ecefZOrAlt;
  result &= addCfgValset32(latLong ? UBLOX_CFG_TMODE_HEIGHT : UBLOX_CFG_TMODE_ECEF_Z, converter32.unsigned32);
  converter8.signed8 = ecefXOrLatHP;
  result &= addCfgValset8(latLong ? UBLOX_CFG_TMODE_LAT_HP : UBLOX_CFG_TMODE_ECEF_X_HP, converter8.unsigned8);
  converter8.signed8 = ecefYOrLonHP;
  result &= addCfgValset8(latLong ? UBLOX_CFG_TMODE_LON_HP : UBLOX_CFG_TMODE_ECEF_Y_HP, converter8.unsigned8);
  converter8.signed8 = ecefZOrAltHP;
  result &= addCfgValset8(latLong ? UBLOX_CFG_TMODE_HEIGHT_HP : UBLOX_CFG_TMODE_ECEF_Z_HP, converter8.unsigned8);
  result &= sendCfgValset(maxWait);

  return result;
}

bool DevUBLOXGNSS::setStaticPosition(int32_t ecefXOrLat, int32_t ecefYOrLon, int32_t ecefZOrAlt, bool latlong, uint8_t layer, uint16_t maxWait)
{
  return (setStaticPosition(ecefXOrLat, 0, ecefYOrLon, 0, ecefZOrAlt, 0, latlong, layer, maxWait));
}

// Set the DGNSS differential mode
bool DevUBLOXGNSS::setDGNSSConfiguration(sfe_ublox_dgnss_mode_e dgnssMode, uint8_t layer, uint16_t maxWait)
{
  return setVal8(UBLOX_CFG_NAVHPG_DGNSSMODE, (uint8_t)dgnssMode, layer, maxWait);
}

// Module Protocol Version

// Get the current protocol version of the u-blox module we're communicating with
// This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
uint8_t DevUBLOXGNSS::getProtocolVersionHigh(uint16_t maxWait)
{
  if (!prepareModuleInfo(maxWait))
    return 0;
  return (moduleSWVersion->protocolVersionHigh);
}
uint8_t DevUBLOXGNSS::getProtocolVersionLow(uint16_t maxWait)
{
  if (!prepareModuleInfo(maxWait))
    return 0;
  return (moduleSWVersion->protocolVersionLow);
}

// Get the firmware version of the u-blox module we're communicating with
uint8_t DevUBLOXGNSS::getFirmwareVersionHigh(uint16_t maxWait)
{
  if (!prepareModuleInfo(maxWait))
    return 0;
  return (moduleSWVersion->firmwareVersionHigh);
}
uint8_t DevUBLOXGNSS::getFirmwareVersionLow(uint16_t maxWait)
{
  if (!prepareModuleInfo(maxWait))
    return 0;
  return (moduleSWVersion->firmwareVersionLow);
}

// Get the firmware type
const char *DevUBLOXGNSS::getFirmwareType(uint16_t maxWait)
{
  static const char unknownFirmware[4] = {'T', 'B', 'D', '\0'};
  if (!prepareModuleInfo(maxWait))
    return unknownFirmware;
  return ((const char *)moduleSWVersion->firmwareType);
}

// Get the module name
const char *DevUBLOXGNSS::getModuleName(uint16_t maxWait)
{
  static const char unknownModule[4] = {'T', 'B', 'D', '\0'};
  if (!prepareModuleInfo(maxWait))
    return unknownModule;
  return ((const char *)moduleSWVersion->moduleName);
}

// PRIVATE: Common code to initialize moduleSWVersion
bool DevUBLOXGNSS::prepareModuleInfo(uint16_t maxWait)
{
  if (moduleSWVersion == nullptr)
    initModuleSWVersion();        // Check that RAM has been allocated for the SW version
  if (moduleSWVersion == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (moduleSWVersion->moduleQueried == false)
    getModuleInfo(maxWait);

  return moduleSWVersion->moduleQueried;
}

// Get the current protocol version of the u-blox module we're communicating with
// This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
bool DevUBLOXGNSS::getProtocolVersion(uint16_t maxWait) // Old name - deprecated
{
  return getModuleInfo(maxWait);
}

bool DevUBLOXGNSS::getModuleInfo(uint16_t maxWait)
{
  if (moduleSWVersion == nullptr)
    initModuleSWVersion();        // Check that RAM has been allocated for the SW version
  if (moduleSWVersion == nullptr) // Bail if the RAM allocation failed
    return (false);

  // Send packet with only CLS and ID, length of zero. This will cause the module to respond with the contents of that CLS/ID.
  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_VER;

  packetCfg.len = 0;
  packetCfg.startingSpot = 40; // Start at first "extended software information" string

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
    return (false);                                                       // If command send fails then bail

  // Payload should now contain ~220 characters (depends on module type)

  // We will step through the payload looking at each extension field of 30 bytes
  char *ptr;
  uint8_t fwProtMod = 0; // Flags to show if we extracted the FWVER, PROTVER and MOD data
  for (uint8_t extensionNumber = 0; extensionNumber < ((packetCfg.len - 40) / 30); extensionNumber++)
  {
    ptr = strstr((const char *)&payloadCfg[(30 * extensionNumber)], "FWVER="); // Check for FWVER (should be in extension 1)
    if (ptr != nullptr)
    {
      ptr += strlen("FWVER="); // Point to the firmware type (HPG etc.)
      int i = 0;
      while ((i < firmwareTypeLen) && (*ptr != '\0') && (*ptr != ' ')) // Extract the firmware type (3-7 chars)
        moduleSWVersion->firmwareType[i++] = *ptr++;
      moduleSWVersion->firmwareType[i] = '\0'; // NULL-terminate

      if (*ptr == ' ')
        ptr++; // Skip the space

      int firmwareHi = 0;
      int firmwareLo = 0;
      int scanned = sscanf(ptr, "%d.%d", &firmwareHi, &firmwareLo);
      if (scanned == 2) // Check we extracted the firmware version successfully
      {
        moduleSWVersion->firmwareVersionHigh = firmwareHi;
        moduleSWVersion->firmwareVersionLow = firmwareLo;
        fwProtMod |= 0x01; // Record that we got the FWVER
      }
    }
    ptr = strstr((const char *)&payloadCfg[(30 * extensionNumber)], "PROTVER="); // Check for PROTVER (should be in extension 2)
    if (ptr != nullptr)
    {
      ptr += strlen("PROTVER="); // Point to the protocol version
      int protHi = 0;
      int protLo = 0;
      int scanned = sscanf(ptr, "%d.%d", &protHi, &protLo);
      if (scanned == 2) // Check we extracted the firmware version successfully
      {
        moduleSWVersion->protocolVersionHigh = protHi;
        moduleSWVersion->protocolVersionLow = protLo;
        fwProtMod |= 0x02; // Record that we got the PROTVER
      }
    }
    ptr = strstr((const char *)&payloadCfg[(30 * extensionNumber)], "MOD="); // Check for MOD (should be in extension 3)
    if (ptr != nullptr)
    {
      ptr += strlen("MOD="); // Point to the module name
      int i = 0;
      while ((i < moduleNameMaxLen) && (*ptr != '\0') && (*ptr != ' ')) // Copy the module name
        moduleSWVersion->moduleName[i++] = *ptr++;
      moduleSWVersion->moduleName[i] = '\0'; // NULL-terminate
      fwProtMod |= 0x04;                     // Record that we got the MOD
    }
  }

  if (fwProtMod == 0x07) // Did we extract all three?
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.print(F("getModuleInfo: FWVER: "));
      _debugSerial.print(moduleSWVersion->firmwareVersionHigh);
      _debugSerial.print(F("."));
      _debugSerial.println(moduleSWVersion->firmwareVersionLow);
      _debugSerial.print(F("getModuleInfo: PROTVER: "));
      _debugSerial.print(moduleSWVersion->protocolVersionHigh);
      _debugSerial.print(F("."));
      _debugSerial.println(moduleSWVersion->protocolVersionLow);
      _debugSerial.print(F("getModuleInfo: MOD: "));
      _debugSerial.println(moduleSWVersion->moduleName);
    }
#endif

    moduleSWVersion->moduleQueried = true; // Mark this data as new

    return (true);
  }

  return (false); // We failed
}

// PRIVATE: Allocate RAM for moduleSWVersion and initialize it
bool DevUBLOXGNSS::initModuleSWVersion()
{
  moduleSWVersion = new moduleSWVersion_t; // Allocate RAM for the main struct
  if (moduleSWVersion == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initModuleSWVersion: RAM alloc failed!"));
#endif
    return (false);
  }
  moduleSWVersion->protocolVersionHigh = 0; // Clear the contents
  moduleSWVersion->protocolVersionLow = 0;
  moduleSWVersion->firmwareVersionHigh = 0;
  moduleSWVersion->firmwareVersionLow = 0;
  moduleSWVersion->firmwareType[0] = 0;
  moduleSWVersion->moduleName[0] = 0;
  moduleSWVersion->moduleQueried = false;
  return (true);
}

// Geofences

// Add a new geofence using UBX-CFG-GEOFENCE
bool DevUBLOXGNSS::addGeofence(int32_t latitude, int32_t longitude, uint32_t radius, uint8_t confidence, bool pinPolarity, uint8_t pin, uint8_t layer, uint16_t maxWait)
{
  if (currentGeofenceParams == nullptr)
    initGeofenceParams();               // Check if RAM has been allocated for currentGeofenceParams
  if (currentGeofenceParams == nullptr) // Abort if the RAM allocation failed
    return (false);

  if (currentGeofenceParams->numFences >= 4)
    return (false); // Quit if we already have four geofences defined

  // Store the new geofence parameters
  currentGeofenceParams->lats[currentGeofenceParams->numFences] = latitude;
  currentGeofenceParams->longs[currentGeofenceParams->numFences] = longitude;
  currentGeofenceParams->rads[currentGeofenceParams->numFences] = radius;
  currentGeofenceParams->numFences += 1; // Increment the number of fences

  unsignedSigned32 converter32;
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_GEOFENCE_CONFLVL, confidence);
  if (pin > 0)
  {
    result &= addCfgValset8(UBLOX_CFG_GEOFENCE_PINPOL, (uint8_t)pinPolarity);
    result &= addCfgValset8(UBLOX_CFG_GEOFENCE_PIN, pin);
    result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_PIO, 1);
  }
  else
  {
    result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_PIO, 0);
  }
  result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_FENCE1, 1);
  converter32.signed32 = currentGeofenceParams->lats[0];
  result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE1_LAT, converter32.unsigned32);
  converter32.signed32 = currentGeofenceParams->longs[0];
  result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE1_LON, converter32.unsigned32);
  result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE1_RAD, currentGeofenceParams->rads[0]);
  result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_FENCE2, currentGeofenceParams->numFences > 1 ? 1 : 0);
  if (currentGeofenceParams->numFences > 1)
  {
    converter32.signed32 = currentGeofenceParams->lats[1];
    result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE2_LAT, converter32.unsigned32);
    converter32.signed32 = currentGeofenceParams->longs[1];
    result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE2_LON, converter32.unsigned32);
    result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE2_RAD, currentGeofenceParams->rads[1]);
  }
  result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_FENCE3, currentGeofenceParams->numFences > 2 ? 1 : 0);
  if (currentGeofenceParams->numFences > 2)
  {
    converter32.signed32 = currentGeofenceParams->lats[2];
    result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE3_LAT, converter32.unsigned32);
    converter32.signed32 = currentGeofenceParams->longs[2];
    result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE3_LON, converter32.unsigned32);
    result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE3_RAD, currentGeofenceParams->rads[2]);
  }
  result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_FENCE4, currentGeofenceParams->numFences > 3 ? 1 : 0);
  if (currentGeofenceParams->numFences > 3)
  {
    converter32.signed32 = currentGeofenceParams->lats[3];
    result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE4_LAT, converter32.unsigned32);
    converter32.signed32 = currentGeofenceParams->longs[3];
    result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE4_LON, converter32.unsigned32);
    result &= addCfgValset32(UBLOX_CFG_GEOFENCE_FENCE4_RAD, currentGeofenceParams->rads[3]);
  }
  result &= sendCfgValset(maxWait);

  return result;
}

// Clear all geofences using UBX-CFG-GEOFENCE
bool DevUBLOXGNSS::clearGeofences(uint8_t layer, uint16_t maxWait)
{
  if (currentGeofenceParams == nullptr)
    initGeofenceParams();               // Check if RAM has been allocated for currentGeofenceParams
  if (currentGeofenceParams == nullptr) // Abort if the RAM allocation failed
    return (false);

  currentGeofenceParams->numFences = 0; // Zero the number of geofences currently in use

  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_FENCE1, 0);
  result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_FENCE2, 0);
  result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_FENCE3, 0);
  result &= addCfgValset8(UBLOX_CFG_GEOFENCE_USE_FENCE4, 0);
  result &= sendCfgValset(maxWait);

  return result;
}

// Returns the combined geofence state using UBX-NAV-GEOFENCE
bool DevUBLOXGNSS::getGeofenceState(geofenceState &currentGeofenceState, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_GEOFENCE;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the geofence status. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  currentGeofenceState.status = payloadCfg[5];    // Extract the status
  currentGeofenceState.numFences = payloadCfg[6]; // Extract the number of geofences
  currentGeofenceState.combState = payloadCfg[7]; // Extract the combined state of all geofences
  if (currentGeofenceState.numFences > 0)
    currentGeofenceState.states[0] = payloadCfg[8]; // Extract geofence 1 state
  if (currentGeofenceState.numFences > 1)
    currentGeofenceState.states[1] = payloadCfg[10]; // Extract geofence 2 state
  if (currentGeofenceState.numFences > 2)
    currentGeofenceState.states[2] = payloadCfg[12]; // Extract geofence 3 state
  if (currentGeofenceState.numFences > 3)
    currentGeofenceState.states[3] = payloadCfg[14]; // Extract geofence 4 state

  return (true);
}

// PRIVATE: Allocate RAM for currentGeofenceParams and initialize it
bool DevUBLOXGNSS::initGeofenceParams()
{
  currentGeofenceParams = new geofenceParams_t; // Allocate RAM for the main struct
  if (currentGeofenceParams == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initGeofenceParams: RAM alloc failed!"));
#endif
    return (false);
  }
  currentGeofenceParams->numFences = 0;
  return (true);
}

// Powers off the GPS device for a given duration to reduce power consumption.
// NOTE: Querying the device before the duration is complete, for example by "getLatitude()" will wake it up!
// Returns true if command has not been not acknowledged.
// Returns false if command has not been acknowledged or maxWait = 0.
bool DevUBLOXGNSS::powerOff(uint32_t durationInMs, uint16_t maxWait)
{
  // use durationInMs = 0 for infinite duration
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("Powering off for "));
    _debugSerial.print(durationInMs);
    _debugSerial.println(" ms");
  }
#endif

  // Power off device using UBX-RXM-PMREQ
  packetCfg.cls = UBX_CLASS_RXM; // 0x02
  packetCfg.id = UBX_RXM_PMREQ;  // 0x41
  packetCfg.len = 8;
  packetCfg.startingSpot = 0;

  // duration
  // big endian to little endian, switch byte order
  for (uint8_t i = 0; i < 4; i++)
    payloadCfg[i] = durationInMs >> (8 * i); // Value

  payloadCfg[4] = 0x02; // Flags : set the backup bit
  payloadCfg[5] = 0x00; // Flags
  payloadCfg[6] = 0x00; // Flags
  payloadCfg[7] = 0x00; // Flags

  if (maxWait != 0)
  {
    // check for "not acknowledged" command
    return (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_COMMAND_NACK);
  }
  else
  {
    sendCommand(&packetCfg, maxWait);
    return false; // can't tell if command not acknowledged if maxWait = 0
  }
}

// Powers off the GPS device for a given duration to reduce power consumption.
// While powered off it can be woken up by creating a falling or rising voltage edge on the specified pin.
// NOTE: The GPS seems to be sensitve to signals on the pins while powered off. Works best when Microcontroller is in deepsleep.
// NOTE: Querying the device before the duration is complete, for example by "getLatitude()" will wake it up!
// Returns true if command has not been not acknowledged.
// Returns false if command has not been acknowledged or maxWait = 0.
bool DevUBLOXGNSS::powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources, bool forceWhileUsb, uint16_t maxWait)
{
  // use durationInMs = 0 for infinite duration
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("Powering off for "));
    _debugSerial.print(durationInMs);
    _debugSerial.println(" ms");
  }
#endif

  // Power off device using UBX-RXM-PMREQ
  packetCfg.cls = UBX_CLASS_RXM; // 0x02
  packetCfg.id = UBX_RXM_PMREQ;  // 0x41
  packetCfg.len = 16;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0x00; // message version

  // bytes 1-3 are reserved - and must be set to zero
  payloadCfg[1] = 0x00;
  payloadCfg[2] = 0x00;
  payloadCfg[3] = 0x00;

  // duration
  // big endian to little endian, switch byte order
  for (uint8_t i = 0; i < 4; i++)
    payloadCfg[4 + i] = durationInMs >> (8 * i); // Value

  // flags

  // disables USB interface when powering off, defaults to true
  if (forceWhileUsb)
  {
    payloadCfg[8] = 0x06; // force | backup
  }
  else
  {
    payloadCfg[8] = 0x02; // backup only (leave the force bit clear - module will stay on if USB is connected)
  }

  payloadCfg[9] = 0x00;
  payloadCfg[10] = 0x00;
  payloadCfg[11] = 0x00;

  // wakeUpSources

  // wakeupPin mapping, defaults to VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0

  // Possible values are:
  // VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX
  // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0
  // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1
  // VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS

  for (uint8_t i = 0; i < 4; i++)
    payloadCfg[12 + i] = wakeupSources >> (8 * i); // Value

  if (maxWait != 0)
  {
    // check for "not acknowledged" command
    return (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_COMMAND_NACK);
  }
  else
  {
    sendCommand(&packetCfg, maxWait);
    return false; // can't tell if command not acknowledged if maxWait = 0
  }
}

// Dynamic Platform Model

// Change the dynamic platform model using UBX-CFG-NAV5
// Possible values are:
// PORTABLE,STATIONARY,PEDESTRIAN,AUTOMOTIVE,SEA,
// AIRBORNE1g,AIRBORNE2g,AIRBORNE4g,WRIST,BIKE
// WRIST is not supported in protocol versions less than 18
// BIKE is supported in protocol versions 19.2
bool DevUBLOXGNSS::setDynamicModel(dynModel newDynamicModel, uint8_t layer, uint16_t maxWait)
{
  return setVal8(UBLOX_CFG_NAVSPG_DYNMODEL, (uint8_t)newDynamicModel, layer, maxWait);
}

// Get the dynamic platform model using UBX-CFG-NAV5
// Returns DYN_MODEL_UNKNOWN (255) if the sendCommand fails
uint8_t DevUBLOXGNSS::getDynamicModel(uint8_t layer, uint16_t maxWait)
{
  uint8_t model;

  if (!getVal8(UBLOX_CFG_NAVSPG_DYNMODEL, &model, layer, maxWait))
    return (DYN_MODEL_UNKNOWN);

  return (model); // Return the dynamic model
}

// Reset the odometer
bool DevUBLOXGNSS::resetOdometer(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_RESETODO;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // This is a special case as we are only expecting an ACK but this is not a CFG message
  return (sendCommand(&packetCfg, maxWait, true) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Enable / disable the odometer
bool DevUBLOXGNSS::enableOdometer(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setVal8(UBLOX_CFG_ODO_USE_ODO, (uint8_t)enable, layer, maxWait);
}

// Read the odometer configuration
bool DevUBLOXGNSS::getOdometerConfig(uint8_t *flags, uint8_t *odoCfg, uint8_t *cogMaxSpeed, uint8_t *cogMaxPosAcc, uint8_t *velLpGain, uint8_t *cogLpGain, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValget(layer);
  result &= addCfgValget(UBLOX_CFG_ODO_USE_ODO);
  result &= addCfgValget(UBLOX_CFG_ODO_USE_COG);
  result &= addCfgValget(UBLOX_CFG_ODO_OUTLPVEL);
  result &= addCfgValget(UBLOX_CFG_ODO_OUTLPCOG);
  result &= addCfgValget(UBLOX_CFG_ODO_PROFILE);
  result &= addCfgValget(UBLOX_CFG_ODO_COGMAXSPEED);
  result &= addCfgValget(UBLOX_CFG_ODO_COGMAXPOSACC);
  result &= addCfgValget(UBLOX_CFG_ODO_VELLPGAIN);
  result &= addCfgValget(UBLOX_CFG_ODO_COGLPGAIN);
  result &= sendCfgValget(maxWait);

  if (result)
  {
    uint8_t flagsBit = 0;
    uint8_t flagsByte = 0;
    result &= extractConfigValueByKey(&packetCfg, UBLOX_CFG_ODO_USE_ODO, &flagsBit, 1);
    if (flagsBit)
      flagsByte |= UBX_CFG_ODO_USE_ODO;
    result &= extractConfigValueByKey(&packetCfg, UBLOX_CFG_ODO_USE_COG, &flagsBit, 1);
    if (flagsBit)
      flagsByte |= UBX_CFG_ODO_USE_COG;
    result &= extractConfigValueByKey(&packetCfg, UBLOX_CFG_ODO_OUTLPVEL, &flagsBit, 1);
    if (flagsBit)
      flagsByte |= UBX_CFG_ODO_OUT_LP_VEL;
    result &= extractConfigValueByKey(&packetCfg, UBLOX_CFG_ODO_OUTLPCOG, &flagsBit, 1);
    if (flagsBit)
      flagsByte |= UBX_CFG_ODO_OUT_LP_COG;
    *flags = flagsByte;

    result &= extractConfigValueByKey(&packetCfg, UBLOX_CFG_ODO_PROFILE, odoCfg, 1);
    result &= extractConfigValueByKey(&packetCfg, UBLOX_CFG_ODO_COGMAXSPEED, cogMaxSpeed, 1);
    result &= extractConfigValueByKey(&packetCfg, UBLOX_CFG_ODO_COGMAXPOSACC, cogMaxPosAcc, 1);
    result &= extractConfigValueByKey(&packetCfg, UBLOX_CFG_ODO_VELLPGAIN, velLpGain, 1);
    result &= extractConfigValueByKey(&packetCfg, UBLOX_CFG_ODO_COGLPGAIN, cogLpGain, 1);
  }

  return result;
}

// Configure the odometer
bool DevUBLOXGNSS::setOdometerConfig(uint8_t flags, uint8_t odoCfg, uint8_t cogMaxSpeed, uint8_t cogMaxPosAcc, uint8_t velLpGain, uint8_t cogLpGain, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_ODO_USE_ODO, flags & UBX_CFG_ODO_USE_ODO ? 1 : 0);
  result &= addCfgValset8(UBLOX_CFG_ODO_USE_COG, flags & UBX_CFG_ODO_USE_COG ? 1 : 0);
  result &= addCfgValset8(UBLOX_CFG_ODO_OUTLPVEL, flags & UBX_CFG_ODO_OUT_LP_VEL ? 1 : 0);
  result &= addCfgValset8(UBLOX_CFG_ODO_OUTLPCOG, flags & UBX_CFG_ODO_OUT_LP_COG ? 1 : 0);
  result &= addCfgValset8(UBLOX_CFG_ODO_PROFILE, odoCfg);
  result &= addCfgValset8(UBLOX_CFG_ODO_COGMAXSPEED, cogMaxSpeed);
  result &= addCfgValset8(UBLOX_CFG_ODO_COGMAXPOSACC, cogMaxPosAcc);
  result &= addCfgValset8(UBLOX_CFG_ODO_VELLPGAIN, velLpGain);
  result &= addCfgValset8(UBLOX_CFG_ODO_COGLPGAIN, cogLpGain);
  result &= sendCfgValset(maxWait);
  return result;
}

uint32_t DevUBLOXGNSS::getEnableGNSSConfigKey(sfe_ublox_gnss_ids_e id)
{
  const uint32_t gnssConfigKeys[(uint8_t)SFE_UBLOX_GNSS_ID_UNKNOWN] = {
      UBLOX_CFG_SIGNAL_GPS_ENA,
      UBLOX_CFG_SIGNAL_SBAS_ENA,
      UBLOX_CFG_SIGNAL_GAL_ENA,
      UBLOX_CFG_SIGNAL_BDS_ENA,
      0, // IMES has no ENA key
      UBLOX_CFG_SIGNAL_QZSS_ENA,
      UBLOX_CFG_SIGNAL_GLO_ENA};

  if (id >= SFE_UBLOX_GNSS_ID_UNKNOWN)
    return 0;
  else
    return (gnssConfigKeys[(uint8_t)id]);
}

// Enable/Disable individual GNSS systems using UBX-CFG-GNSS
bool DevUBLOXGNSS::enableGNSS(bool enable, sfe_ublox_gnss_ids_e id, uint8_t layer, uint16_t maxWait)
{
  uint32_t key = getEnableGNSSConfigKey(id);
  return (setVal8(key, enable ? 1 : 0, layer, maxWait));
}

// Check if an individual GNSS system is enabled
bool DevUBLOXGNSS::isGNSSenabled(sfe_ublox_gnss_ids_e id, bool *enabled, uint8_t layer, uint16_t maxWait)
{
  uint32_t key = getEnableGNSSConfigKey(id);
  return (getVal8(key, (uint8_t *)enabled, layer, maxWait));
}
bool DevUBLOXGNSS::isGNSSenabled(sfe_ublox_gnss_ids_e id, uint8_t layer, uint16_t maxWait) // Unsafe
{
  uint32_t key = getEnableGNSSConfigKey(id);
  uint8_t enabled;
  getVal8(key, &enabled, layer, maxWait);
  return ((bool)enabled);
}

// Reset ESF automatic IMU-mount alignment
bool DevUBLOXGNSS::resetIMUalignment(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_ESF;
  packetCfg.id = UBX_ESF_RESETALG;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // This is a special case as we are only expecting an ACK but this is not a CFG message
  return (sendCommand(&packetCfg, maxWait, true) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Enable/disable esfAutoAlignment
bool DevUBLOXGNSS::getESFAutoAlignment(bool *enabled, uint8_t layer, uint16_t maxWait)
{
  return getVal8(UBLOX_CFG_SFIMU_AUTO_MNTALG_ENA, (uint8_t *)enabled, layer, maxWait);
}
bool DevUBLOXGNSS::getESFAutoAlignment(uint8_t layer, uint16_t maxWait) // Unsafe overload
{
  uint8_t result;
  getVal8(UBLOX_CFG_SFIMU_AUTO_MNTALG_ENA, &result, layer, maxWait);
  return (bool)result;
}
bool DevUBLOXGNSS::setESFAutoAlignment(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setVal8(UBLOX_CFG_SFIMU_AUTO_MNTALG_ENA, (uint8_t)enable, layer, maxWait);
}

// Get the RF information using UBX_MON_RF
bool DevUBLOXGNSS::getRFinformation(UBX_MON_RF_data_t *data, uint16_t maxWait)
{
  if (data == nullptr) // Check if the user forgot to include the data pointer
    return (false);    // Bail

  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_RF;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Extract the data
  data->header.version = extractByte(&packetCfg, 0);
  data->header.nBlocks = extractByte(&packetCfg, 1);

  // Extract the RF information blocks
  for (uint8_t block = 0; (block < data->header.nBlocks) && (block < UBX_MON_RF_MAX_BLOCKS); block++)
  {
    data->blocks[block].blockId = extractByte(&packetCfg, 4 + (block * 24));
    data->blocks[block].flags.all = extractByte(&packetCfg, 5 + (block * 24));
    data->blocks[block].antStatus = extractByte(&packetCfg, 6 + (block * 24));
    data->blocks[block].antPower = extractByte(&packetCfg, 7 + (block * 24));
    data->blocks[block].postStatus = extractLong(&packetCfg, 8 + (block * 24));
    data->blocks[block].noisePerMS = extractInt(&packetCfg, 16 + (block * 24));
    data->blocks[block].agcCnt = extractInt(&packetCfg, 18 + (block * 24));
    data->blocks[block].jamInd = extractByte(&packetCfg, 20 + (block * 24));
    data->blocks[block].ofsI = extractSignedChar(&packetCfg, 21 + (block * 24));
    data->blocks[block].magI = extractByte(&packetCfg, 22 + (block * 24));
    data->blocks[block].ofsQ = extractSignedChar(&packetCfg, 23 + (block * 24));
    data->blocks[block].magQ = extractByte(&packetCfg, 24 + (block * 24));
  }

  return (true);
}

// Get the extended hardware status using UBX_MON_HW2
bool DevUBLOXGNSS::getHW2status(UBX_MON_HW2_data_t *data, uint16_t maxWait)
{
  if (data == nullptr) // Check if the user forgot to include the data pointer
    return (false);    // Bail

  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_HW2;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Extract the data
  data->ofsI = extractSignedChar(&packetCfg, 0);
  data->magI = extractByte(&packetCfg, 1);
  data->ofsQ = extractSignedChar(&packetCfg, 2);
  data->magQ = extractByte(&packetCfg, 3);
  data->cfgSource = extractByte(&packetCfg, 4);
  data->lowLevCfg = extractLong(&packetCfg, 8); // Low-level configuration (obsolete for protocol versions greater than 15.00)
  data->postStatus = extractLong(&packetCfg, 20);

  return (true);
}

// UBX-CFG-NAVX5 - get/set the ackAiding byte. If ackAiding is 1, UBX-MGA-ACK messages will be sent by the module to acknowledge the MGA data
uint8_t DevUBLOXGNSS::getAckAiding(uint8_t layer, uint16_t maxWait) // Get the ackAiding byte - returns 255 if the sendCommand fails
{
  uint8_t enabled;
  bool success = getVal8(UBLOX_CFG_NAVSPG_ACKAIDING, &enabled, layer, maxWait);
  if (success)
    return enabled;
  return 255;
}
bool DevUBLOXGNSS::setAckAiding(uint8_t ackAiding, uint8_t layer, uint16_t maxWait) // Set the ackAiding byte
{
  return setVal8(UBLOX_CFG_NAVSPG_ACKAIDING, ackAiding, layer, maxWait);
}

// AssistNow Autonomous support
// UBX-CFG-NAVX5 - get the AssistNow Autonomous configuration (aopCfg) - returns 255 if the sendCommand fails
uint8_t DevUBLOXGNSS::getAopCfg(uint8_t layer, uint16_t maxWait)
{
  uint8_t enabled;
  bool success = getVal8(UBLOX_CFG_ANA_USE_ANA, &enabled, layer, maxWait);
  if (success)
    return enabled;
  return 255;
}
// Set the aopCfg byte and the aopOrdMaxErr word
bool DevUBLOXGNSS::setAopCfg(uint8_t aopCfg, uint16_t aopOrbMaxErr, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_ANA_USE_ANA, aopCfg);
  if ((aopOrbMaxErr >= 5) && (aopOrbMaxErr <= 1000)) // Maximum acceptable (modeled) orbit error in m. Range is from 5 to 1000.
    result &= addCfgValset16(UBLOX_CFG_ANA_ORBMAXERR, aopOrbMaxErr);
  result &= sendCfgValset(maxWait);
  return result;
}

// SPARTN dynamic keys
//"When the receiver boots, the host should send 'current' and 'next' keys in one message." - Use setDynamicSPARTNKeys for this.
//"Every time the 'current' key is expired, 'next' takes its place."
//"Therefore the host should then retrieve the new 'next' key and send only that." - Use setDynamicSPARTNKey for this.
// The key can be provided in binary (uint8_t) format or in ASCII Hex (char) format, but in both cases keyLengthBytes _must_ represent the binary key length in bytes.
bool DevUBLOXGNSS::setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const char *key)
{
  uint8_t *binaryKey = new uint8_t[keyLengthBytes]; // Allocate memory to store the binaryKey

  if (binaryKey == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
      _debugSerial.println(F("setDynamicSPARTNKey: binaryKey RAM allocation failed!"));
#endif
    return (false);
  }

  bool ok = true;

  // Convert the ASCII Hex const char to binary uint8_t
  for (uint16_t i = 0; i < ((uint16_t)keyLengthBytes * 2); i += 2)
  {
    if ((key[i] >= '0') && (key[i] <= '9'))
    {
      binaryKey[i >> 1] = (key[i] - '0') << 4;
    }
    else if ((key[i] >= 'a') && (key[i] <= 'f'))
    {
      binaryKey[i >> 1] = (key[i] + 10 - 'a') << 4;
    }
    else if ((key[i] >= 'A') && (key[i] <= 'F'))
    {
      binaryKey[i >> 1] = (key[i] + 10 - 'A') << 4;
    }
    else
    {
      ok = false;
    }

    if ((key[i + 1] >= '0') && (key[i + 1] <= '9'))
    {
      binaryKey[i >> 1] |= key[i + 1] - '0';
    }
    else if ((key[i + 1] >= 'a') && (key[i + 1] <= 'f'))
    {
      binaryKey[i >> 1] |= key[i + 1] + 10 - 'a';
    }
    else if ((key[i + 1] >= 'A') && (key[i + 1] <= 'F'))
    {
      binaryKey[i >> 1] |= key[i + 1] + 10 - 'A';
    }
    else
    {
      ok = false;
    }
  }

  if (ok)
    ok = setDynamicSPARTNKey(keyLengthBytes, validFromWno, validFromTow, (const uint8_t *)binaryKey);

  delete[] binaryKey; // Free the memory allocated for binaryKey

  return (ok);
}

bool DevUBLOXGNSS::setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const uint8_t *key)
{
  // Check if there is room for the key in packetCfg. Resize the buffer if not.
  size_t payloadLength = (size_t)keyLengthBytes + 12;
  if (packetCfgPayloadSize < payloadLength)
  {
    if (!setPacketCfgPayloadSize(payloadLength)) // Check if the resize was successful
    {
      return (false);
    }
  }

  // Copy the key etc. into packetCfg
  packetCfg.cls = UBX_CLASS_RXM;
  packetCfg.id = UBX_RXM_SPARTNKEY;
  packetCfg.len = payloadLength;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0x01; // version
  payloadCfg[1] = 0x01; // numKeys
  payloadCfg[2] = 0x00; // reserved0
  payloadCfg[3] = 0x00; // reserved0
  payloadCfg[4] = 0x00; // reserved1
  payloadCfg[5] = keyLengthBytes;
  payloadCfg[6] = validFromWno & 0xFF; // validFromWno little-endian
  payloadCfg[7] = validFromWno >> 8;
  payloadCfg[8] = validFromTow & 0xFF; // validFromTow little-endian
  payloadCfg[9] = (validFromTow >> 8) & 0xFF;
  payloadCfg[10] = (validFromTow >> 16) & 0xFF;
  payloadCfg[11] = (validFromTow >> 24) & 0xFF;

  memcpy(&payloadCfg[12], key, keyLengthBytes);

  return (sendCommand(&packetCfg, 0) == SFE_UBLOX_STATUS_SUCCESS); // UBX-RXM-SPARTNKEY is silent. It does not ACK (or NACK)
}

bool DevUBLOXGNSS::setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const char *key1,
                                        uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const char *key2)
{
  uint8_t *binaryKey1 = new uint8_t[keyLengthBytes1]; // Allocate memory to store binaryKey1

  if (binaryKey1 == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
      _debugSerial.println(F("setDynamicSPARTNKeys: binaryKey1 RAM allocation failed!"));
#endif
    return (false);
  }

  uint8_t *binaryKey2 = new uint8_t[keyLengthBytes2]; // Allocate memory to store binaryKey2

  if (binaryKey2 == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
      _debugSerial.println(F("setDynamicSPARTNKeys: binaryKey2 RAM allocation failed!"));
#endif
    delete[] binaryKey1;
    return (false);
  }

  bool ok = true;

  // Convert the ASCII Hex const char to binary uint8_t
  for (uint16_t i = 0; i < ((uint16_t)keyLengthBytes1 * 2); i += 2)
  {
    if ((key1[i] >= '0') && (key1[i] <= '9'))
    {
      binaryKey1[i >> 1] = (key1[i] - '0') << 4;
    }
    else if ((key1[i] >= 'a') && (key1[i] <= 'f'))
    {
      binaryKey1[i >> 1] = (key1[i] + 10 - 'a') << 4;
    }
    else if ((key1[i] >= 'A') && (key1[i] <= 'F'))
    {
      binaryKey1[i >> 1] = (key1[i] + 10 - 'A') << 4;
    }
    else
    {
      ok = false;
    }

    if ((key1[i + 1] >= '0') && (key1[i + 1] <= '9'))
    {
      binaryKey1[i >> 1] |= key1[i + 1] - '0';
    }
    else if ((key1[i + 1] >= 'a') && (key1[i + 1] <= 'f'))
    {
      binaryKey1[i >> 1] |= key1[i + 1] + 10 - 'a';
    }
    else if ((key1[i + 1] >= 'A') && (key1[i + 1] <= 'F'))
    {
      binaryKey1[i >> 1] |= key1[i + 1] + 10 - 'A';
    }
    else
    {
      ok = false;
    }
  }

  // Convert the ASCII Hex const char to binary uint8_t
  for (uint16_t i = 0; i < ((uint16_t)keyLengthBytes2 * 2); i += 2)
  {
    if ((key2[i] >= '0') && (key2[i] <= '9'))
    {
      binaryKey2[i >> 1] = (key2[i] - '0') << 4;
    }
    else if ((key2[i] >= 'a') && (key2[i] <= 'f'))
    {
      binaryKey2[i >> 1] = (key2[i] + 10 - 'a') << 4;
    }
    else if ((key2[i] >= 'A') && (key2[i] <= 'F'))
    {
      binaryKey2[i >> 1] = (key2[i] + 10 - 'A') << 4;
    }
    else
    {
      ok = false;
    }

    if ((key2[i + 1] >= '0') && (key2[i + 1] <= '9'))
    {
      binaryKey2[i >> 1] |= key2[i + 1] - '0';
    }
    else if ((key2[i + 1] >= 'a') && (key2[i + 1] <= 'f'))
    {
      binaryKey2[i >> 1] |= key2[i + 1] + 10 - 'a';
    }
    else if ((key2[i + 1] >= 'A') && (key2[i + 1] <= 'F'))
    {
      binaryKey2[i >> 1] |= key2[i + 1] + 10 - 'A';
    }
    else
    {
      ok = false;
    }
  }

  if (ok)
    ok = setDynamicSPARTNKeys(keyLengthBytes1, validFromWno1, validFromTow1, (const uint8_t *)binaryKey1,
                              keyLengthBytes2, validFromWno2, validFromTow2, (const uint8_t *)binaryKey2);

  delete[] binaryKey1; // Free the memory allocated for binaryKey1
  delete[] binaryKey2; // Free the memory allocated for binaryKey2

  return (ok);
}

bool DevUBLOXGNSS::setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const uint8_t *key1,
                                        uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const uint8_t *key2)
{
  // Check if there is room for the key in packetCfg. Resize the buffer if not.
  size_t payloadLength = (size_t)keyLengthBytes1 + (size_t)keyLengthBytes2 + 20;
  if (packetCfgPayloadSize < payloadLength)
  {
    if (!setPacketCfgPayloadSize(payloadLength)) // Check if the resize was successful
    {
      return (false);
    }
  }

  // Copy the key etc. into packetCfg
  packetCfg.cls = UBX_CLASS_RXM;
  packetCfg.id = UBX_RXM_SPARTNKEY;
  packetCfg.len = payloadLength;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0x01; // version
  payloadCfg[1] = 0x02; // numKeys
  payloadCfg[2] = 0x00; // reserved0
  payloadCfg[3] = 0x00; // reserved0
  payloadCfg[4] = 0x00; // reserved1
  payloadCfg[5] = keyLengthBytes1;
  payloadCfg[6] = validFromWno1 & 0xFF; // validFromWno little-endian
  payloadCfg[7] = validFromWno1 >> 8;
  payloadCfg[8] = validFromTow1 & 0xFF; // validFromTow little-endian
  payloadCfg[9] = (validFromTow1 >> 8) & 0xFF;
  payloadCfg[10] = (validFromTow1 >> 16) & 0xFF;
  payloadCfg[11] = (validFromTow1 >> 24) & 0xFF;
  payloadCfg[12] = 0x00; // reserved1
  payloadCfg[13] = keyLengthBytes2;
  payloadCfg[14] = validFromWno2 & 0xFF; // validFromWno little-endian
  payloadCfg[15] = validFromWno2 >> 8;
  payloadCfg[16] = validFromTow2 & 0xFF; // validFromTow little-endian
  payloadCfg[17] = (validFromTow2 >> 8) & 0xFF;
  payloadCfg[18] = (validFromTow2 >> 16) & 0xFF;
  payloadCfg[19] = (validFromTow2 >> 24) & 0xFF;

  memcpy(&payloadCfg[20], key1, keyLengthBytes1);
  memcpy(&payloadCfg[20 + keyLengthBytes1], key2, keyLengthBytes2);

  return (sendCommand(&packetCfg, 0) == SFE_UBLOX_STATUS_SUCCESS); // UBX-RXM-SPARTNKEY is silent. It does not ACK (or NACK)
}

// Support for SPARTN parsing
// Mostly stolen from https://github.com/u-blox/ubxlib/blob/master/common/spartn/src/u_spartn_crc.c

uint8_t DevUBLOXGNSS::uSpartnCrc4(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint8_t u8TableRemainder;
    uint8_t u8Remainder = 0; // Initial remainder

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u8TableRemainder = pU8Msg[x] ^ u8Remainder;
        u8Remainder = sfe_ublox_u8Crc4Table[u8TableRemainder];
    }

    return u8Remainder;
}

uint8_t DevUBLOXGNSS::uSpartnCrc8(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint8_t u8TableRemainder;
    uint8_t u8Remainder = 0; // Initial remainder

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u8TableRemainder = pU8Msg[x] ^ u8Remainder;
        u8Remainder = sfe_ublox_u8Crc8Table[u8TableRemainder];
    }

    return u8Remainder;
}

uint16_t DevUBLOXGNSS::uSpartnCrc16(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint16_t u16TableRemainder;
    uint16_t u16Remainder = 0; // Initial remainder
    uint8_t  u8NumBitsInCrc = (8 * sizeof(uint16_t));

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u16TableRemainder = pU8Msg[x] ^ (u16Remainder >> (u8NumBitsInCrc - 8));
        u16Remainder = sfe_ublox_u16Crc16Table[u16TableRemainder] ^ (u16Remainder << 8);
    }

    return u16Remainder;
}

uint32_t DevUBLOXGNSS::uSpartnCrc24(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint32_t u32TableRemainder;
    uint32_t u32Remainder = 0; // Initial remainder
    uint8_t u8NumBitsInCrc = (8 * sizeof(uint8_t) * 3);

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u32TableRemainder = pU8Msg[x] ^ (u32Remainder >> (u8NumBitsInCrc - 8));
        u32Remainder = sfe_ublox_u32Crc24Table[u32TableRemainder] ^ (u32Remainder << 8);
        u32Remainder = u32Remainder & 0x00FFFFFF; // Only interested in 24 bits
    }

    return u32Remainder;
}

uint32_t DevUBLOXGNSS::uSpartnCrc32(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint32_t u32TableRemainder;
    uint32_t u32Remainder = 0xFFFFFFFFU; // Initial remainder
    uint8_t u8NumBitsInCrc = (8 * sizeof(uint32_t));
    uint32_t u32FinalXORValue = 0xFFFFFFFFU;

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u32TableRemainder = pU8Msg[x] ^ (u32Remainder >> (u8NumBitsInCrc - 8));
        u32Remainder = sfe_ublox_u32Crc32Table[u32TableRemainder] ^ (u32Remainder << 8);
    }

    u32Remainder = u32Remainder ^ u32FinalXORValue;

    return u32Remainder;
}

// Parse SPARTN data
uint8_t * DevUBLOXGNSS::parseSPARTN(uint8_t incoming, bool &valid, uint16_t &len, sfe_ublox_spartn_header_t *header)
{
  typedef enum {
    waitingFor73,
    TF002_TF006,
    TF007,
    TF009,
    TF016,
    TF017,
    TF018
  } parseStates;
  static parseStates parseState = waitingFor73;

  static uint8_t spartn[1100];

  static sfe_ublox_spartn_header_t _header;
  static uint16_t frameCount;
  static uint16_t crcBytes;
  static uint16_t TF007toTF016;

  valid = false;

  switch(parseState)
  {
    case waitingFor73:
      if (incoming == 0x73)
      {
        parseState = TF002_TF006;
        frameCount = 0;
        spartn[0] = incoming;
      }
      break;
    case TF002_TF006:
      spartn[1 + frameCount] = incoming;
      if (frameCount == 0)
      {
        _header.messageType = incoming >> 1;
        _header.payloadLength = incoming & 0x01;
      }
      if (frameCount == 1)
      {
        _header.payloadLength <<= 8;
        _header.payloadLength |= incoming;
      }
      if (frameCount == 2)
      {
        _header.payloadLength <<= 1;
        _header.payloadLength |= incoming >> 7;
        _header.EAF = (incoming >> 6) & 0x01;
        _header.crcType = (incoming >> 4) & 0x03;
        switch (_header.crcType)
        {
          case 0:
            crcBytes = 1;
            break;
          case 1:
            crcBytes = 2;
            break;
          case 2:
            crcBytes = 3;
            break;
          default:
            crcBytes = 4;
            break;
        }
        _header.frameCRC = incoming & 0x0F;
        spartn[3] = spartn[3] & 0xF0; // Zero the 4 LSBs before calculating the CRC
        if (uSpartnCrc4(&spartn[1], 3) == _header.frameCRC)
        {
          spartn[3] = incoming; // Restore TF005 and TF006 now we know the data is valid
          parseState = TF007;
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true)
          {
            _debugSerial.print(F("SPARTN Header CRC is valid: payloadLength "));
            _debugSerial.print(_header.payloadLength);
            _debugSerial.print(F(" EAF "));
            _debugSerial.print(_header.EAF);
            _debugSerial.print(F(" crcType "));
            _debugSerial.println(_header.crcType);
          }
#endif
        }
        else
        {
          parseState = waitingFor73;
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true)
          {
            _debugSerial.println(F("SPARTN Header CRC is INVALID"));
          }
#endif
        }
      }
      frameCount++;
      break;
    case TF007:
      spartn[4] = incoming;
      _header.messageSubtype = incoming >> 4;
      _header.timeTagType = (incoming >> 3) & 0x01;
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial.print(F("SPARTN timeTagType "));
        _debugSerial.println(_header.timeTagType);
      }
#endif
      if (_header.timeTagType == 0)
        TF007toTF016 = 4;
      else
        TF007toTF016 = 6;
      if (_header.EAF > 0)
        TF007toTF016 += 2;
      parseState = TF009;
      frameCount = 1;          
      break;
    case TF009:
      spartn[4 + frameCount] = incoming;
      frameCount++;
      if (frameCount == TF007toTF016)
      {
        if (_header.EAF == 0)
        {
          _header.authenticationIndicator = 0;
          _header.embeddedApplicationLengthBytes = 0;
        }
        else
        {
          _header.authenticationIndicator = (incoming >> 3) & 0x07;
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true)
          {
            _debugSerial.print(F("SPARTN authenticationIndicator "));
            _debugSerial.println(_header.authenticationIndicator);
          }
#endif
          if (_header.authenticationIndicator <= 1)
            _header.embeddedApplicationLengthBytes = 0;
          else
          {
            switch(incoming & 0x07)
            {
              case 0:
                _header.embeddedApplicationLengthBytes = 8; // 64 bits
                break;
              case 1:
                _header.embeddedApplicationLengthBytes = 12; // 96 bits
                break;
              case 2:
                _header.embeddedApplicationLengthBytes = 16; // 128 bits
                break;
              case 3:
                _header.embeddedApplicationLengthBytes = 32; // 256 bits
                break;
              default:
                _header.embeddedApplicationLengthBytes = 64; // 512 / TBD bits
                break;
            }
          }
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true)
          {
            _debugSerial.print(F("SPARTN embeddedApplicationLengthBytes "));
            _debugSerial.println(_header.embeddedApplicationLengthBytes);
          }
#endif
        }
        parseState = TF016;
        frameCount = 0;                  
      }
      break;
    case TF016:
      spartn[4 + TF007toTF016 + frameCount] = incoming;
      frameCount++;
      if (frameCount == _header.payloadLength)
      {
        if (_header.embeddedApplicationLengthBytes > 0)
        {
          parseState = TF017;
          frameCount = 0;
        }
        else               
        {
          parseState = TF018;
          frameCount = 0;
        }
      }
      break;
    case TF017:
      spartn[4 + TF007toTF016 + _header.payloadLength + frameCount] = incoming;
      frameCount++;
      if (frameCount == _header.embeddedApplicationLengthBytes)
      {
        parseState = TF018;
        frameCount = 0;        
      }
      break;
    case TF018:
      spartn[4 + TF007toTF016 + _header.payloadLength + _header.embeddedApplicationLengthBytes + frameCount] = incoming;
      frameCount++;
      if (frameCount == crcBytes)
      {
          parseState = waitingFor73;
          uint16_t numBytes = 4 + TF007toTF016 + _header.payloadLength + _header.embeddedApplicationLengthBytes;
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true)
          {
            _debugSerial.print(F("SPARTN numBytes "));
            _debugSerial.println(numBytes);
          }
#endif
          uint8_t *ptr = &spartn[numBytes];
          switch (_header.crcType)
          {
            case 0:
            {
              uint8_t expected = *ptr;
              if (uSpartnCrc8(&spartn[1], numBytes - 1) == expected) // Don't include the preamble in the CRC
              {
                valid = true;
                len = numBytes + 1;
              }
            }
            break;
            case 1:
            {
              uint16_t expected = *ptr++;
              expected <<= 8;
              expected |= *ptr;
              if (uSpartnCrc16(&spartn[1], numBytes - 1) == expected) // Don't include the preamble in the CRC
              {
                valid = true;
                len = numBytes + 2;
              }
            }
            break;
            case 2:
            {
              uint32_t expected = *ptr++;
              expected <<= 8;
              expected |= *ptr++;
              expected <<= 8;
              expected |= *ptr;
              uint32_t crc = uSpartnCrc24(&spartn[1], numBytes - 1); // Don't include the preamble in the CRC
              if (crc == expected)
              {
                valid = true;
                len = numBytes + 3;
              }
              else
              {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
                if (_printDebug == true)
                {
                  _debugSerial.print(F("SPARTN CRC-24 is INVALID: 0x"));
                  _debugSerial.print(expected, HEX);
                  _debugSerial.print(F(" vs 0x"));
                  _debugSerial.println(crc, HEX);
                }
#endif
              }
            }
            break;
            default:
            {
              uint32_t expected = *ptr++;
              expected <<= 8;
              expected |= *ptr++;
              expected <<= 8;
              expected |= *ptr++;
              expected <<= 8;
              expected |= *ptr;
              if (uSpartnCrc32(&spartn[1], numBytes - 1) == expected)
              {
                valid = true;
                len = numBytes + 4;
              }
            }
            break;
          }
      }
      break;
  }

  if (header != nullptr)
    memcpy(header, &_header, sizeof(sfe_ublox_spartn_header_t));

  return &spartn[0];
}

// Get the unique chip ID using UBX-SEC-UNIQID
// The ID is five bytes on the F9 and M9 (version 1) but six bytes on the M10 (version 2)
bool DevUBLOXGNSS::getUniqueChipId(UBX_SEC_UNIQID_data_t *data, uint16_t maxWait)
{
  if (data == nullptr) // Check if the user forgot to include the data pointer
    return (false);    // Bail

  packetCfg.cls = UBX_CLASS_SEC;
  packetCfg.id = UBX_SEC_UNIQID;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Extract the data
  data->version = extractByte(&packetCfg, 0);
  for (uint8_t i = 0; i < 5; i++)
    data->uniqueId[i] = extractByte(&packetCfg, i + 4);

  // The ID is five bytes on the F9 and M9 (version 1) but six bytes on the M10 (version 2)
  if ((data->version == 2) && (packetCfg.len == UBX_SEC_UNIQID_LEN_VERSION2))
    data->uniqueId[5] = extractByte(&packetCfg, 9);
  else
    data->uniqueId[5] = 0;

  return (true);
}
// Get the unique chip ID as text
const char *DevUBLOXGNSS::getUniqueChipIdStr(UBX_SEC_UNIQID_data_t *data, uint16_t maxWait)
{
  static char uniqueId[13] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '\0'};
  bool valid = true;
  bool provided = (data != nullptr);

  if (!provided)
  {
    data = new UBX_SEC_UNIQID_data_t;
    valid = getUniqueChipId(data, maxWait);
  }

  if (valid)
  {
    for (uint8_t i = 0; (i < (data->version + 4)) && (i < 6); i++)
    {
      uint8_t nibble = data->uniqueId[i] >> 4;
      if (nibble < 10)
        uniqueId[(i * 2) + 0] = nibble + '0';
      else
        uniqueId[(i * 2) + 0] = nibble + 'A' - 10;
      nibble = data->uniqueId[i] & 0x0F;
      if (nibble < 10)
        uniqueId[(i * 2) + 1] = nibble + '0';
      else
        uniqueId[(i * 2) + 1] = nibble + 'A' - 10;
      uniqueId[(i * 2) + 2] = 0; // NULL-terminate
    }
  }

  if (!provided)
    delete data;

  return ((const char *)uniqueId);
}

// CONFIGURATION INTERFACE (protocol v27 and above)

// Given a key, load the payload with data that can then be extracted to 8, 16, or 32 bits
// This function takes a full 32-bit key
// Default layer is RAM
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
sfe_ublox_status_e DevUBLOXGNSS::getVal(uint32_t key, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALGET;
  packetCfg.len = 4 + 4 * 1; // While multiple keys are allowed, we will send only one key at a time
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  // VALGET uses different memory layer definitions to VALSET
  // because it can only return the value for one layer.
  // So we need to fiddle the layer here.
  // And just to complicate things further, the ZED-F9P only responds
  // correctly to layer 0 (RAM) and layer 7 (Default)!
  uint8_t getLayer = VAL_LAYER_DEFAULT; // 7 is the "Default Layer"
  if (layer == VAL_LAYER_RAM)           // Did the user request the RAM layer?
  {
    getLayer = 0; // Layer 0 is RAM
  }
  else if (layer == VAL_LAYER_BBR) // Did the user request the BBR layer?
  {
    getLayer = 1; // Layer 1 is BBR
  }
  else if (layer == VAL_LAYER_FLASH) // Did the user request the Flash layer?
  {
    getLayer = 2; // Layer 2 is Flash
  }

  payloadCfg[0] = 0;        // Message Version - set to 0
  payloadCfg[1] = getLayer; // Layer

  // Load key into outgoing payload
  key &= ~UBX_CFG_SIZE_MASK;    // Mask off the size identifer bits
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("getVal key: 0x"));
    _debugSerial.print(key, HEX);
    _debugSerial.println();
  }
#endif

  // Send VALGET command with this key

  sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("getVal: sendCommand returned: "));
    _debugSerial.println(statusString(retVal));
  }
#endif

  // Verify the response is the correct length as compared to what the user called (did the module respond with 8-bits but the user called getVal32?)
  // Response is 8 bytes plus cfg data
  // if(packet->len > 8+1)

  // The response is now sitting in payload, ready for extraction
  return (retVal);
}

// Given a key, return its value
// This function takes a full 32-bit key
// Default layer is RAM
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::getVal8(uint32_t key, uint8_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractByte(&packetCfg, 8);
  return result;
}
uint8_t DevUBLOXGNSS::getVal8(uint32_t key, uint8_t layer, uint16_t maxWait) // Unsafe overload
{
  uint8_t result = 0;
  getVal8(key, &result, layer, maxWait);
  return result;
}
bool DevUBLOXGNSS::getValSigned8(uint32_t key, int8_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractSignedChar(&packetCfg, 8);
  return result;
}

bool DevUBLOXGNSS::getVal16(uint32_t key, uint16_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractInt(&packetCfg, 8);
  return result;
}
uint16_t DevUBLOXGNSS::getVal16(uint32_t key, uint8_t layer, uint16_t maxWait) // Unsafe overload
{
  uint16_t result = 0;
  getVal16(key, &result, layer, maxWait);
  return result;
}
bool DevUBLOXGNSS::getValSigned16(uint32_t key, int16_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractSignedInt(&packetCfg, 8);
  return result;
}

bool DevUBLOXGNSS::getVal32(uint32_t key, uint32_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractLong(&packetCfg, 8);
  return result;
}
uint32_t DevUBLOXGNSS::getVal32(uint32_t key, uint8_t layer, uint16_t maxWait) // Unsafe overload
{
  uint32_t result = 0;
  getVal32(key, &result, layer, maxWait);
  return result;
}
bool DevUBLOXGNSS::getValSigned32(uint32_t key, int32_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractSignedLong(&packetCfg, 8);
  return result;
}

bool DevUBLOXGNSS::getVal64(uint32_t key, uint64_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractLongLong(&packetCfg, 8);
  return result;
}
uint64_t DevUBLOXGNSS::getVal64(uint32_t key, uint8_t layer, uint16_t maxWait) // Unsafe overload
{
  uint64_t result = 0;
  getVal64(key, &result, layer, maxWait);
  return result;
}
bool DevUBLOXGNSS::getValSigned64(uint32_t key, int64_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractSignedLongLong(&packetCfg, 8);
  return result;
}

bool DevUBLOXGNSS::getValFloat(uint32_t key, float *val, uint8_t layer, uint16_t maxWait)
{
  if (sizeof(float) != 4)
    return false;

  bool result = getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractFloat(&packetCfg, 8);
  return result;
}

bool DevUBLOXGNSS::getValDouble(uint32_t key, double *val, uint8_t layer, uint16_t maxWait)
{
  if (sizeof(double) != 8)
    return false;

  bool result = getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractDouble(&packetCfg, 8);
  return result;
}

// Given a key, set a N-byte value
// This function takes a full 32-bit key
// Default layer is RAM+BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::setValN(uint32_t key, uint8_t *value, uint8_t N, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + N; // 4 byte header, 4 byte key ID, N bytes of value
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  key &= ~UBX_CFG_SIZE_MASK; // Mask off the size identifer bits
  for (uint8_t i = 0; i < 4; i++)
    payloadCfg[i + 4] = key >> (8 * i); // Key

  // Load user's value
  for (uint8_t i = 0; i < N; i++)
    payloadCfg[i + 8] = *value++;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Given a key, set an 8-bit value
// This function takes a full 32-bit key
// Default layer is RAM+BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::setVal8(uint32_t key, uint8_t value, uint8_t layer, uint16_t maxWait)
{
  uint8_t val[1] = {value};
  return (setValN(key, val, 1, layer, maxWait));
}
bool DevUBLOXGNSS::setValSigned8(uint32_t key, int8_t value, uint8_t layer, uint16_t maxWait)
{
  unsignedSigned8 converter;
  converter.signed8 = value;
  return (setVal8(key, converter.unsigned8, layer, maxWait));
}

// Given a key, set a 16-bit value
// This function takes a full 32-bit key
// Default layer is RAM+BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::setVal16(uint32_t key, uint16_t value, uint8_t layer, uint16_t maxWait)
{
  uint8_t val[2] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8)};
  return (setValN(key, val, 2, layer, maxWait));
}
bool DevUBLOXGNSS::setValSigned16(uint32_t key, int16_t value, uint8_t layer, uint16_t maxWait)
{
  unsignedSigned16 converter;
  converter.signed16 = value;
  return (setVal16(key, converter.unsigned16, layer, maxWait));
}

// Given a key, set a 32-bit value
// This function takes a full 32-bit key
// Default layer is RAM+BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::setVal32(uint32_t key, uint32_t value, uint8_t layer, uint16_t maxWait)
{
  uint8_t val[4] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
  return (setValN(key, val, 4, layer, maxWait));
}
bool DevUBLOXGNSS::setValSigned32(uint32_t key, int32_t value, uint8_t layer, uint16_t maxWait)
{
  unsignedSigned32 converter;
  converter.signed32 = value;
  return (setVal32(key, converter.unsigned32, layer, maxWait));
}

// Given a key, set a 64-bit value
// This function takes a full 32-bit key
// Default layer is RAM+BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::setVal64(uint32_t key, uint64_t value, uint8_t layer, uint16_t maxWait)
{
  uint8_t val[8];

  // Load user's value
  for (uint8_t i = 0; i < 8; i++)
    val[i] = (uint8_t)(value >> (8 * i)); // Value

  return (setValN(key, val, 8, layer, maxWait));
}
bool DevUBLOXGNSS::setValSigned64(uint32_t key, int64_t value, uint8_t layer, uint16_t maxWait)
{
  unsignedSigned64 converter;
  converter.signed64 = value;
  return (setVal64(key, converter.unsigned64, layer, maxWait));
}

bool DevUBLOXGNSS::setValFloat(uint32_t key, float value, uint8_t layer, uint16_t maxWait)
{
  if (sizeof(float) != 4)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("setValFloat not supported!"));
    }
#endif
    return false;
  }
  unsigned32float converter;
  converter.flt = value;
  return (setVal32(key, converter.unsigned32, layer, maxWait));
}

bool DevUBLOXGNSS::setValDouble(uint32_t key, double value, uint8_t layer, uint16_t maxWait)
{
  if (sizeof(double) != 8)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("setValDouble not supported!"));
    }
#endif
    return false;
  }
  unsigned64double converter;
  converter.dbl = value;
  return (setVal64(key, converter.unsigned64, layer, maxWait));
}

// Start defining a new (empty) UBX-CFG-VALSET ubxPacket
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::newCfgValset(uint8_t layer)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4; // 4 byte header
  packetCfg.startingSpot = 0;

  _numCfgKeys = 0;

  // Clear all of packet payload
  memset(payloadCfg, 0, packetCfgPayloadSize);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // All done
  return (true);
}

// Add another key and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and N-byte value
bool DevUBLOXGNSS::addCfgValsetN(uint32_t key, uint8_t *value, uint8_t N)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("addCfgValsetN: autosend"));
#endif
    if (sendCommand(&packetCfg) != SFE_UBLOX_STATUS_DATA_SENT) // We are only expecting an ACK
      return false;
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeys = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - (4 + N)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("addCfgValsetN: packetCfgPayloadSize reached!"));
#endif
    return false;
  }

  if (_numCfgKeys == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("addCfgValsetN: key limit reached!"));
#endif
    return false;
  }

  // Load key into outgoing payload
  key &= ~UBX_CFG_SIZE_MASK; // Mask off the size identifer bits
  for (uint8_t i = 0; i < 4; i++)
    payloadCfg[packetCfg.len + i] = key >> (8 * i); // Key

  // Load user's value
  for (uint8_t i = 0; i < N; i++)
    payloadCfg[packetCfg.len + i + 4] = *value++; // Value

  // Update packet length: 4 byte key ID, 8 bytes of value
  packetCfg.len = packetCfg.len + 4 + N;

  _numCfgKeys++;

  // All done
  return (true);
}

// Add another key and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 64-bit value
bool DevUBLOXGNSS::addCfgValset64(uint32_t key, uint64_t value)
{
  uint8_t val[8];

  // Load user's value
  for (uint8_t i = 0; i < 8; i++)
    val[i] = (uint8_t)(value >> (8 * i)); // Value

  return (addCfgValsetN(key, val, 8));
}

// Add another key and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 32-bit value
bool DevUBLOXGNSS::addCfgValset32(uint32_t key, uint32_t value)
{
  uint8_t val[4] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
  return (addCfgValsetN(key, val, 4));
}

// Add another key and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 16-bit value
bool DevUBLOXGNSS::addCfgValset16(uint32_t key, uint16_t value)
{
  uint8_t val[2] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8)};
  return (addCfgValsetN(key, val, 2));
}

// Add another key and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 8-bit value
bool DevUBLOXGNSS::addCfgValset8(uint32_t key, uint8_t value)
{
  uint8_t val[1] = {value};
  return (addCfgValsetN(key, val, 1));
}

// Add another key and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 32-bit float (R4) value
bool DevUBLOXGNSS::addCfgValsetFloat(uint32_t key, float value)
{
  if (sizeof(float) != 4)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("addCfgValsetFloat not supported!"));
    }
#endif
    return false;
  }

  // Define a union to convert from float to uint32_t
  unsigned32float convert32;

  convert32.flt = value;

  return (addCfgValset32(key, convert32.unsigned32));
}

// Add another key and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 64-bit double (R8) value
// This won't work on older AVR platforms where double is 32-bit
bool DevUBLOXGNSS::addCfgValsetDouble(uint32_t key, double value)
{
  if (sizeof(double) != 8)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("addCfgValsetDouble not supported!"));
    }
#endif
    return false;
  }

  // Define a union to convert from double to uint64_t
  unsigned64double convert64;

  convert64.dbl = value;

  return (addCfgValset64(key, convert64.unsigned64));
}

// Send the UBX-CFG-VALSET ubxPacket
bool DevUBLOXGNSS::sendCfgValset(uint16_t maxWait)
{
  if (_numCfgKeys == 0)
    return true; // Nothing to send...

  // Send VALSET command with this key and value
  bool success = sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK

  if (success)
    _numCfgKeys = 0;

  return success;
}

// Return the number of keys in the CfgValset
uint8_t DevUBLOXGNSS::getCfgValsetLen()
{
  return _numCfgKeys;
}

// Return the number of free bytes remaining in packetCfgPayload
size_t DevUBLOXGNSS::getCfgValsetSpaceRemaining()
{
  return getPacketCfgSpaceRemaining();
}

// Deprecated - only included for backward-compatibility. Use newCfgValset and sendCfgValset
bool DevUBLOXGNSS::newCfgValset8(uint32_t key, uint8_t value, uint8_t layer)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(key, value);
  return result;
}
bool DevUBLOXGNSS::newCfgValset16(uint32_t key, uint16_t value, uint8_t layer)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset16(key, value);
  return result;
}
bool DevUBLOXGNSS::newCfgValset32(uint32_t key, uint32_t value, uint8_t layer)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset32(key, value);
  return result;
}
bool DevUBLOXGNSS::newCfgValset64(uint32_t key, uint64_t value, uint8_t layer)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset64(key, value);
  return result;
}
bool DevUBLOXGNSS::sendCfgValset8(uint32_t key, uint8_t value, uint16_t maxWait)
{
  bool result = addCfgValset8(key, value);
  result &= sendCfgValset(maxWait);
  return result;
}
bool DevUBLOXGNSS::sendCfgValset16(uint32_t key, uint16_t value, uint16_t maxWait)
{
  bool result = addCfgValset16(key, value);
  result &= sendCfgValset(maxWait);
  return result;
}
bool DevUBLOXGNSS::sendCfgValset32(uint32_t key, uint32_t value, uint16_t maxWait)
{
  bool result = addCfgValset32(key, value);
  result &= sendCfgValset(maxWait);
  return result;
}
bool DevUBLOXGNSS::sendCfgValset64(uint32_t key, uint64_t value, uint16_t maxWait)
{
  bool result = addCfgValset64(key, value);
  result &= sendCfgValset(maxWait);
  return result;
}

bool DevUBLOXGNSS::newCfgValget(uint8_t layer) // Create a new, empty UBX-CFG-VALGET. Add entries with addCfgValget8/16/32/64
{
  return (newCfgValget(&packetCfg, packetCfgPayloadSize, layer));
}

bool DevUBLOXGNSS::newCfgValget(ubxPacket *pkt, uint16_t maxPayload, uint8_t layer) // Create a new, empty UBX-CFG-VALGET. Add entries with addCfgValget8/16/32/64
{
  if (cfgValgetValueSizes == nullptr) // Check if RAM has been allocated for cfgValgetValueSizes
  {
    cfgValgetValueSizes = new uint8_t[CFG_VALSET_MAX_KEYS];
  }

  _cfgValgetMaxPayload = maxPayload;

  pkt->cls = UBX_CLASS_CFG;
  pkt->id = UBX_CFG_VALGET;
  pkt->len = 4; // 4 byte header
  pkt->startingSpot = 0;

  _numGetCfgKeys = 0;
  _lenCfgValGetResponse = 0;

  // Clear all of packet payload
  if (pkt == &packetCfg)
  {
    memset(payloadCfg, 0, packetCfgPayloadSize);
  }
  else
  {
    // Custom packet: we don't know how large payload is, so only clear the two skip keys bytes
    pkt->payload[2] = 0; // Set the skip keys bytes to zero
    pkt->payload[3] = 0;
  }

  // VALGET uses different memory layer definitions to VALSET
  // because it can only return the value for one layer.
  // So we need to fiddle the layer here.
  // And just to complicate things further, the ZED-F9P only responds
  // correctly to layer 0 (RAM) and layer 7 (Default)!
  uint8_t getLayer = VAL_LAYER_DEFAULT; // 7 is the "Default Layer"
  if (layer == VAL_LAYER_RAM)           // Did the user request the RAM layer?
  {
    getLayer = 0; // Layer 0 is RAM
  }
  else if (layer == VAL_LAYER_BBR) // Did the user request the BBR layer?
  {
    getLayer = 1; // Layer 1 is BBR
  }
  else if (layer == VAL_LAYER_FLASH) // Did the user request the Flash layer?
  {
    getLayer = 2; // Layer 2 is Flash
  }

  pkt->payload[0] = 0;        // Message Version - set to 0
  pkt->payload[1] = getLayer; // Layer

  if (maxPayload < 9) // Sanity check - make sure there's room for a single L/U1 response
    return false;

  // All done
  return (true);
}

bool DevUBLOXGNSS::addCfgValget(uint32_t key) // Add a new key to an existing UBX-CFG-VALGET ubxPacket
{
  return (addCfgValget(&packetCfg, key));
}

bool DevUBLOXGNSS::addCfgValget(ubxPacket *pkt, uint32_t key) // Add a new key to an existing UBX-CFG-VALGET ubxPacket
{
  // Extract the value size
  uint8_t valueSizeBytes = getCfgValueSizeBytes(key);

  if (_lenCfgValGetResponse >= (_cfgValgetMaxPayload - (4 + (valueSizeBytes))))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("addCfgValget: packetCfgPayloadSize reached!"));
    }
#endif
    return false;
  }

  if (_numGetCfgKeys == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial.println(F("addCfgValget: key limit reached!"));
    }
#endif
    return false;
  }

  // Store the value size in cfgValgetValueSizes
  if (cfgValgetValueSizes != nullptr)
  {
    cfgValgetValueSizes[_numGetCfgKeys] = valueSizeBytes;
  }

  // Load key into outgoing payload
  uint8_t *ptr;
  ptr = pkt->payload;
  ptr += pkt->len;
  key &= ~UBX_CFG_SIZE_MASK; // Mask off the size identifer bits
  for (uint8_t i = 0; i < 4; i++)
  {
    *ptr = key >> (8 * i); // Key
    ptr++;
  }

  // Update packet length: 4 byte key ID
  pkt->len += 4;

  _numGetCfgKeys++;
  _lenCfgValGetResponse += 4 + (valueSizeBytes); // 4 byte key ID, N byte value

  // All done
  return (true);
}

bool DevUBLOXGNSS::sendCfgValget(uint16_t maxWait) // Send the CfgValget (UBX-CFG-VALGET) construct
{
  return (sendCfgValget(&packetCfg, maxWait));
}

bool DevUBLOXGNSS::sendCfgValget(ubxPacket *pkt, uint16_t maxWait) // Send the CfgValget (UBX-CFG-VALGET) construct
{
  if (_numGetCfgKeys == 0)
    return true; // Nothing to send...

  // Send VALSET command with this key and value
  bool success = sendCommand(pkt, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED; // We are expecting data and an ACK

  if (success)
    _numGetCfgKeys = 0;

  return success;
}

uint8_t DevUBLOXGNSS::getCfgValueSizeBytes(const uint32_t key)
{
  switch (key & UBX_CFG_SIZE_MASK)
  {
  case UBX_CFG_L:
  case UBX_CFG_U1:
  case UBX_CFG_I1:
  case UBX_CFG_E1:
  case UBX_CFG_X1:
    return 1;
    break;
  case UBX_CFG_U2:
  case UBX_CFG_I2:
  case UBX_CFG_E2:
  case UBX_CFG_X2:
    return 2;
    break;
  case UBX_CFG_U4:
  case UBX_CFG_I4:
  case UBX_CFG_E4:
  case UBX_CFG_X4:
  case UBX_CFG_R4:
    return 4;
    break;
  case UBX_CFG_U8:
  case UBX_CFG_I8:
  case UBX_CFG_X8:
  case UBX_CFG_R8:
    return 8;
    break;
  default:
    return 0; // Error
    break;
  }
  return 0;
}

//=-=-=-=-=-=-=-= "Automatic" Messages =-=-=-=-=-=-=-==-=-=-=-=-=-=-=
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// ***** NAV POSECEF automatic support

bool DevUBLOXGNSS::getNAVPOSECEF(uint16_t maxWait)
{
  if (packetUBXNAVPOSECEF == nullptr)
    initPacketUBXNAVPOSECEF();        // Check that RAM has been allocated for the POSECEF data
  if (packetUBXNAVPOSECEF == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic && packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVPOSECEF->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic && !packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_POSECEF;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPOSECEF
// works.
bool DevUBLOXGNSS::setAutoNAVPOSECEF(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVPOSECEFrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPOSECEF
// works.
bool DevUBLOXGNSS::setAutoNAVPOSECEF(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVPOSECEFrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPOSECEF
// works.
bool DevUBLOXGNSS::setAutoNAVPOSECEFrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVPOSECEF == nullptr)
    initPacketUBXNAVPOSECEF();        // Check that RAM has been allocated for the data
  if (packetUBXNAVPOSECEF == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVPOSECEF->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVPOSECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_POSECEF_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVPOSECEF(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVPOSECEF->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPOSECEF->callbackData = new UBX_NAV_POSECEF_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPOSECEF->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVPOSECEFcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVPOSECEF->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and POSECEF is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVPOSECEF(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVPOSECEF == nullptr)
    initPacketUBXNAVPOSECEF();        // Check that RAM has been allocated for the data
  if (packetUBXNAVPOSECEF == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVPOSECEF and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVPOSECEF()
{
  packetUBXNAVPOSECEF = new UBX_NAV_POSECEF_t; // Allocate RAM for the main struct
  if (packetUBXNAVPOSECEF == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVPOSECEF: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVPOSECEF->automaticFlags.flags.all = 0;
  packetUBXNAVPOSECEF->callbackPointerPtr = nullptr;
  packetUBXNAVPOSECEF->callbackData = nullptr;
  packetUBXNAVPOSECEF->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale. This is handy to get data alignment after CRC failure
// or if there are no helper functions and the user wants to request fresh data
void DevUBLOXGNSS::flushNAVPOSECEF()
{
  if (packetUBXNAVPOSECEF == nullptr)
    return;                                                 // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPOSECEF->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVPOSECEF(bool enabled)
{
  if (packetUBXNAVPOSECEF == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPOSECEF->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV STATUS automatic support

bool DevUBLOXGNSS::getNAVSTATUS(uint16_t maxWait)
{
  if (packetUBXNAVSTATUS == nullptr)
    initPacketUBXNAVSTATUS();        // Check that RAM has been allocated for the STATUS data
  if (packetUBXNAVSTATUS == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic && packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVSTATUS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic && !packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_STATUS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getNAVSTATUS
// works.
bool DevUBLOXGNSS::setAutoNAVSTATUS(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVSTATUSrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getNAVSTATUS
// works.
bool DevUBLOXGNSS::setAutoNAVSTATUS(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVSTATUSrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getNAVSTATUS
// works.
bool DevUBLOXGNSS::setAutoNAVSTATUSrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVSTATUS == nullptr)
    initPacketUBXNAVSTATUS();        // Check that RAM has been allocated for the data
  if (packetUBXNAVSTATUS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVSTATUS->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_NAV_STATUS_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVSTATUS(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVSTATUS->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVSTATUS->callbackData = new UBX_NAV_STATUS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVSTATUS->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVSTATUScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVSTATUS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and STATUS is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVSTATUS(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVSTATUS == nullptr)
    initPacketUBXNAVSTATUS();        // Check that RAM has been allocated for the data
  if (packetUBXNAVSTATUS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVSTATUS and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVSTATUS()
{
  packetUBXNAVSTATUS = new UBX_NAV_STATUS_t; // Allocate RAM for the main struct
  if (packetUBXNAVSTATUS == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVSTATUS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVSTATUS->automaticFlags.flags.all = 0;
  packetUBXNAVSTATUS->callbackPointerPtr = nullptr;
  packetUBXNAVSTATUS->callbackData = nullptr;
  packetUBXNAVSTATUS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale. This is handy to get data alignment after CRC failure
// or if there are no helper functions and the user wants to request fresh data
void DevUBLOXGNSS::flushNAVSTATUS()
{
  if (packetUBXNAVSTATUS == nullptr)
    return;                                                // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSTATUS->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVSTATUS(bool enabled)
{
  if (packetUBXNAVSTATUS == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSTATUS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** DOP automatic support

bool DevUBLOXGNSS::getDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVDOP->automaticFlags.flags.bits.automatic && packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVDOP->automaticFlags.flags.bits.automatic && !packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_DOP;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getDOP
// works.
bool DevUBLOXGNSS::setAutoDOP(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoDOPrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getDOP
// works.
bool DevUBLOXGNSS::setAutoDOP(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoDOPrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getDOP
// works.
bool DevUBLOXGNSS::setAutoDOPrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the data
  if (packetUBXNAVDOP == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_DOP_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_DOP_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_DOP_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_DOP_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVDOP->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoDOPcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_DOP_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoDOP(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVDOP->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVDOP->callbackData = new UBX_NAV_DOP_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVDOP->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoDOPcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVDOP->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and DOP is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoDOP(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the data
  if (packetUBXNAVDOP == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVDOP->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVDOP->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVDOP and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVDOP()
{
  packetUBXNAVDOP = new UBX_NAV_DOP_t; // Allocate RAM for the main struct
  if (packetUBXNAVDOP == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVDOP: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVDOP->automaticFlags.flags.all = 0;
  packetUBXNAVDOP->callbackPointerPtr = nullptr;
  packetUBXNAVDOP->callbackData = nullptr;
  packetUBXNAVDOP->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the DOP data as read/stale. This is handy to get data alignment after CRC failure
void DevUBLOXGNSS::flushDOP()
{
  if (packetUBXNAVDOP == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVDOP->moduleQueried.moduleQueried.all = 0; // Mark all DOPs as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVDOP(bool enabled)
{
  if (packetUBXNAVDOP == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVDOP->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** EOE automatic support

bool DevUBLOXGNSS::getNAVEOE(uint16_t maxWait)
{
  if (packetUBXNAVEOE == nullptr)
    initPacketUBXNAVEOE();        // Check that RAM has been allocated for the EOE data
  if (packetUBXNAVEOE == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVEOE->automaticFlags.flags.bits.automatic && packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVEOE->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVEOE->automaticFlags.flags.bits.automatic && !packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // Note to self: NAV-EOE is "Periodic" (only). Not sure if it can be polled?

    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_EOE;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getEOE
// works.
bool DevUBLOXGNSS::setAutoNAVEOE(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVEOErate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getEOE
// works.
bool DevUBLOXGNSS::setAutoNAVEOE(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVEOErate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getEOE
// works.
bool DevUBLOXGNSS::setAutoNAVEOErate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVEOE == nullptr)
    initPacketUBXNAVEOE();        // Check that RAM has been allocated for the data
  if (packetUBXNAVEOE == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_EOE_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_EOE_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_EOE_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_EOE_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVEOE->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVEOE->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVEOEcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_EOE_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVEOE(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVEOE->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVEOE->callbackData = new UBX_NAV_EOE_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVEOE->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVEOEcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVEOE->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and EOE is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVEOE(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVEOE == nullptr)
    initPacketUBXNAVEOE();        // Check that RAM has been allocated for the data
  if (packetUBXNAVEOE == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVEOE->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVEOE->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVEOE and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVEOE()
{
  packetUBXNAVEOE = new UBX_NAV_EOE_t; // Allocate RAM for the main struct
  if (packetUBXNAVEOE == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVEOE: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVEOE->automaticFlags.flags.all = 0;
  packetUBXNAVEOE->callbackPointerPtr = nullptr;
  packetUBXNAVEOE->callbackData = nullptr;
  packetUBXNAVEOE->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the EOE data as read/stale. This is handy to get data alignment after CRC failure
void DevUBLOXGNSS::flushNAVEOE()
{
  if (packetUBXNAVEOE == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVEOE->moduleQueried.moduleQueried.all = 0; // Mark all EOEs as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVEOE(bool enabled)
{
  if (packetUBXNAVEOE == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVEOE->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** VEH ATT automatic support

bool DevUBLOXGNSS::getVehAtt(uint16_t maxWait)
{
  return (getNAVATT(maxWait));
}

bool DevUBLOXGNSS::getNAVATT(uint16_t maxWait)
{
  if (packetUBXNAVATT == nullptr)
    initPacketUBXNAVATT();        // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXNAVATT->automaticFlags.flags.bits.automatic && packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVATT->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVATT->automaticFlags.flags.bits.automatic && !packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_ATT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic NAV ATT message generation by the GNSS. This changes the way getVehAtt
// works.
bool DevUBLOXGNSS::setAutoNAVATT(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVATTrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic NAV ATT message generation by the GNSS. This changes the way getVehAtt
// works.
bool DevUBLOXGNSS::setAutoNAVATT(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVATTrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic NAV ATT attitude message generation by the GNSS. This changes the way getVehAtt
// works.
bool DevUBLOXGNSS::setAutoNAVATTrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVATT == nullptr)
    initPacketUBXNAVATT();        // Check that RAM has been allocated for the data
  if (packetUBXNAVATT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_ATT_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_ATT_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_ATT_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_ATT_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVATT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVATTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_ATT_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVATT(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVATT->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVATT->callbackData = new UBX_NAV_ATT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVATT->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVATTcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVATT->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and NAV ATT attitude is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVATT(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVATT == nullptr)
    initPacketUBXNAVATT();        // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVATT->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVATT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVATT and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVATT()
{
  packetUBXNAVATT = new UBX_NAV_ATT_t; // Allocate RAM for the main struct
  if (packetUBXNAVATT == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVATT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVATT->automaticFlags.flags.all = 0;
  packetUBXNAVATT->callbackPointerPtr = nullptr;
  packetUBXNAVATT->callbackData = nullptr;
  packetUBXNAVATT->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the ATT data as read/stale. This is handy to get data alignment after CRC failure
void DevUBLOXGNSS::flushNAVATT()
{
  if (packetUBXNAVATT == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVATT->moduleQueried.moduleQueried.all = 0; // Mark all ATT data as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVATT(bool enabled)
{
  if (packetUBXNAVATT == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVATT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** PVT automatic support

// Get the latest Position/Velocity/Time solution and fill all global variables
bool DevUBLOXGNSS::getPVT(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->automaticFlags.flags.bits.automatic && packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all;
  }
  else if (packetUBXNAVPVT->automaticFlags.flags.bits.automatic && !packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_PVT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;
    // packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can record up to 20+packetCfgPayloadSize = 84 bytes Note:now hard-coded in processUBX

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
      return (true);

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVT
// works.
bool DevUBLOXGNSS::setAutoPVT(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoPVTrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVT
// works.
bool DevUBLOXGNSS::setAutoPVT(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoPVTrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVT
// works.
bool DevUBLOXGNSS::setAutoPVTrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_PVT_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_PVT_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVPVT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS. This changes the way getPVT works.
bool DevUBLOXGNSS::setAutoPVTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVT_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoPVT(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAutoPVT failed

  if (packetUBXNAVPVT->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPVT->callbackData = new UBX_NAV_PVT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPVT->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoPVTcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVPVT->callbackPointerPtr = callbackPointerPtr; // RAM has been allocated so now update the pointer

  return (true);
}

// In case no config access to the GNSS is possible and PVT is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoPVT(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVPVT->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVPVT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVPVT and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVPVT()
{
  packetUBXNAVPVT = new UBX_NAV_PVT_t; // Allocate RAM for the main struct
  if (packetUBXNAVPVT == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVPVT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVPVT->automaticFlags.flags.all = 0;
  packetUBXNAVPVT->callbackPointerPtr = nullptr;
  packetUBXNAVPVT->callbackData = nullptr;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0;
  packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0;
  return (true);
}

// Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure
void DevUBLOXGNSS::flushPVT()
{
  if (packetUBXNAVPVT == nullptr)
    return;                                              // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0; // Mark all datums as stale (read before)
  packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0;
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVPVT(bool enabled)
{
  if (packetUBXNAVPVT == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPVT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV ODO automatic support

bool DevUBLOXGNSS::getNAVODO(uint16_t maxWait)
{
  if (packetUBXNAVODO == nullptr)
    initPacketUBXNAVODO();        // Check that RAM has been allocated for the ODO data
  if (packetUBXNAVODO == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVODO->automaticFlags.flags.bits.automatic && packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVODO->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVODO->automaticFlags.flags.bits.automatic && !packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_ODO;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getODO
// works.
bool DevUBLOXGNSS::setAutoNAVODO(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVODOrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getODO
// works.
bool DevUBLOXGNSS::setAutoNAVODO(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVODOrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getODO
// works.
bool DevUBLOXGNSS::setAutoNAVODOrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVODO == nullptr)
    initPacketUBXNAVODO();        // Check that RAM has been allocated for the data
  if (packetUBXNAVODO == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_ODO_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_ODO_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_ODO_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_ODO_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVODO->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVODO->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVODOcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_ODO_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVODO(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVODO->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVODO->callbackData = new UBX_NAV_ODO_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVODO->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVODOcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVODO->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ODO is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVODO(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVODO == nullptr)
    initPacketUBXNAVODO();        // Check that RAM has been allocated for the data
  if (packetUBXNAVODO == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVODO->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVODO->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVODO and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVODO()
{
  packetUBXNAVODO = new UBX_NAV_ODO_t; // Allocate RAM for the main struct
  if (packetUBXNAVODO == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVODO: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVODO->automaticFlags.flags.all = 0;
  packetUBXNAVODO->callbackPointerPtr = nullptr;
  packetUBXNAVODO->callbackData = nullptr;
  packetUBXNAVODO->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVODO()
{
  if (packetUBXNAVODO == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVODO->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVODO(bool enabled)
{
  if (packetUBXNAVODO == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVODO->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV VELECEF automatic support

bool DevUBLOXGNSS::getNAVVELECEF(uint16_t maxWait)
{
  if (packetUBXNAVVELECEF == nullptr)
    initPacketUBXNAVVELECEF();        // Check that RAM has been allocated for the VELECEF data
  if (packetUBXNAVVELECEF == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic && packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVVELECEF->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic && !packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_VELECEF;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELECEF
// works.
bool DevUBLOXGNSS::setAutoNAVVELECEF(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVVELECEFrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELECEF
// works.
bool DevUBLOXGNSS::setAutoNAVVELECEF(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVVELECEFrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELECEF
// works.
bool DevUBLOXGNSS::setAutoNAVVELECEFrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVVELECEF == nullptr)
    initPacketUBXNAVVELECEF();        // Check that RAM has been allocated for the data
  if (packetUBXNAVVELECEF == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVVELECEF->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVVELECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_VELECEF_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVVELECEF(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVVELECEF->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVVELECEF->callbackData = new UBX_NAV_VELECEF_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVVELECEF->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVVELECEFcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVVELECEF->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and VELECEF is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVVELECEF(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVVELECEF == nullptr)
    initPacketUBXNAVVELECEF();        // Check that RAM has been allocated for the data
  if (packetUBXNAVVELECEF == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVVELECEF and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVVELECEF()
{
  packetUBXNAVVELECEF = new UBX_NAV_VELECEF_t; // Allocate RAM for the main struct
  if (packetUBXNAVVELECEF == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVVELECEF: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVVELECEF->automaticFlags.flags.all = 0;
  packetUBXNAVVELECEF->callbackPointerPtr = nullptr;
  packetUBXNAVVELECEF->callbackData = nullptr;
  packetUBXNAVVELECEF->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVVELECEF()
{
  if (packetUBXNAVVELECEF == nullptr)
    return;                                                 // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVVELECEF->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVVELECEF(bool enabled)
{
  if (packetUBXNAVVELECEF == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVVELECEF->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV VELNED automatic support

bool DevUBLOXGNSS::getNAVVELNED(uint16_t maxWait)
{
  if (packetUBXNAVVELNED == nullptr)
    initPacketUBXNAVVELNED();        // Check that RAM has been allocated for the VELNED data
  if (packetUBXNAVVELNED == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVVELNED->automaticFlags.flags.bits.automatic && packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVVELNED->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVVELNED->automaticFlags.flags.bits.automatic && !packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_VELNED;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELNED
// works.
bool DevUBLOXGNSS::setAutoNAVVELNED(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVVELNEDrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELNED
// works.
bool DevUBLOXGNSS::setAutoNAVVELNED(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVVELNEDrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELNED
// works.
bool DevUBLOXGNSS::setAutoNAVVELNEDrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVVELNED == nullptr)
    initPacketUBXNAVVELNED();        // Check that RAM has been allocated for the data
  if (packetUBXNAVVELNED == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVVELNED->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVVELNED->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVVELNEDcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_VELNED_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVVELNED(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVVELNED->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVVELNED->callbackData = new UBX_NAV_VELNED_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVVELNED->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVVELNEDcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVVELNED->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and VELNED is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVVELNED(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVVELNED == nullptr)
    initPacketUBXNAVVELNED();        // Check that RAM has been allocated for the data
  if (packetUBXNAVVELNED == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVVELNED->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVVELNED->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVVELNED and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVVELNED()
{
  packetUBXNAVVELNED = new UBX_NAV_VELNED_t; // Allocate RAM for the main struct
  if (packetUBXNAVVELNED == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVVELNED: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVVELNED->automaticFlags.flags.all = 0;
  packetUBXNAVVELNED->callbackPointerPtr = nullptr;
  packetUBXNAVVELNED->callbackData = nullptr;
  packetUBXNAVVELNED->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVVELNED()
{
  if (packetUBXNAVVELNED == nullptr)
    return;                                                // Bail if RAM has not been allocated (otherwise we could be writing anywhere!
  packetUBXNAVVELNED->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVVELNED(bool enabled)
{
  if (packetUBXNAVVELNED == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVVELNED->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV HPPOSECEF automatic support

bool DevUBLOXGNSS::getNAVHPPOSECEF(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic && packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic && !packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_HPPOSECEF;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSECEF
// works.
bool DevUBLOXGNSS::setAutoNAVHPPOSECEF(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVHPPOSECEFrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSECEF
// works.
bool DevUBLOXGNSS::setAutoNAVHPPOSECEF(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVHPPOSECEFrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSECEF
// works.
bool DevUBLOXGNSS::setAutoNAVHPPOSECEFrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the data
  if (packetUBXNAVHPPOSECEF == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVHPPOSECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_HPPOSECEF_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVHPPOSECEF(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVHPPOSECEF->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVHPPOSECEF->callbackData = new UBX_NAV_HPPOSECEF_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVHPPOSECEF->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVHPPOSECEFcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVHPPOSECEF->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HPPOSECEF is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVHPPOSECEF(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the data
  if (packetUBXNAVHPPOSECEF == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVHPPOSECEF and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVHPPOSECEF()
{
  packetUBXNAVHPPOSECEF = new UBX_NAV_HPPOSECEF_t; // Allocate RAM for the main struct
  if (packetUBXNAVHPPOSECEF == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVHPPOSECEF: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVHPPOSECEF->automaticFlags.flags.all = 0;
  packetUBXNAVHPPOSECEF->callbackPointerPtr = nullptr;
  packetUBXNAVHPPOSECEF->callbackData = nullptr;
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVHPPOSECEF()
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    return;                                                   // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVHPPOSECEF(bool enabled)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV HPPOSLLH automatic support

bool DevUBLOXGNSS::getHPPOSLLH(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic && packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic && !packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_HPPOSLLH;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSLLH
// works.
bool DevUBLOXGNSS::setAutoHPPOSLLH(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoHPPOSLLHrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSLLH
// works.
bool DevUBLOXGNSS::setAutoHPPOSLLH(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoHPPOSLLHrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSLLH
// works.
bool DevUBLOXGNSS::setAutoHPPOSLLHrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the data
  if (packetUBXNAVHPPOSLLH == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoHPPOSLLHcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_HPPOSLLH_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHPPOSLLH(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVHPPOSLLH->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVHPPOSLLH->callbackData = new UBX_NAV_HPPOSLLH_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVHPPOSLLH->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoHPPOSLLHcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVHPPOSLLH->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HPPOSLLH is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoHPPOSLLH(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the data
  if (packetUBXNAVHPPOSLLH == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVHPPOSLLH and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVHPPOSLLH()
{
  packetUBXNAVHPPOSLLH = new UBX_NAV_HPPOSLLH_t; // Allocate RAM for the main struct
  if (packetUBXNAVHPPOSLLH == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVHPPOSLLH: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVHPPOSLLH->automaticFlags.flags.all = 0;
  packetUBXNAVHPPOSLLH->callbackPointerPtr = nullptr;
  packetUBXNAVHPPOSLLH->callbackData = nullptr;
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the HPPOSLLH data as read/stale. This is handy to get data alignment after CRC failure
void DevUBLOXGNSS::flushHPPOSLLH()
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    return;                                                  // Bail if RAM has not been allocated (otherwise we could be writing anywhere!
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVHPPOSLLH(bool enabled)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** PVAT automatic support

// Get the latest Position/Velocity/Time solution and fill all global variables
bool DevUBLOXGNSS::getNAVPVAT(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT();        // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVAT->automaticFlags.flags.bits.automatic && packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate)
  {
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all;
  }
  else if (packetUBXNAVPVAT->automaticFlags.flags.bits.automatic && !packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_PVAT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVAT
// works.
bool DevUBLOXGNSS::setAutoNAVPVAT(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVPVATrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVAT
// works.
bool DevUBLOXGNSS::setAutoNAVPVAT(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVPVATrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVAT
// works.
bool DevUBLOXGNSS::setAutoNAVPVATrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT();        // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVPVAT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS. This changes the way getPVAT works.
bool DevUBLOXGNSS::setAutoNAVPVATcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVAT_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVPVAT(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAutoPVAT failed

  if (packetUBXNAVPVAT->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPVAT->callbackData = new UBX_NAV_PVAT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPVAT->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVPVATcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVPVAT->callbackPointerPtr = callbackPointerPtr; // RAM has been allocated so now update the pointer

  return (true);
}

// In case no config access to the GNSS is possible and PVAT is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVPVAT(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT();        // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVPVAT->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVPVAT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVPVAT and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVPVAT()
{
  packetUBXNAVPVAT = new UBX_NAV_PVAT_t; // Allocate RAM for the main struct
  if (packetUBXNAVPVAT == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVPVAT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVPVAT->automaticFlags.flags.all = 0;
  packetUBXNAVPVAT->callbackPointerPtr = nullptr;
  packetUBXNAVPVAT->callbackData = nullptr;
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.all = 0;
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.all = 0;
  return (true);
}

// Mark all the PVAT data as read/stale. This is handy to get data alignment after CRC failure
void DevUBLOXGNSS::flushNAVPVAT()
{
  if (packetUBXNAVPVAT == nullptr)
    return;                                               // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.all = 0; // Mark all datums as stale (read before)
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.all = 0;
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVPVAT(bool enabled)
{
  if (packetUBXNAVPVAT == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPVAT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV TIMEUTC automatic support

bool DevUBLOXGNSS::getNAVTIMEUTC(uint16_t maxWait)
{
  if (packetUBXNAVTIMEUTC == nullptr)
    initPacketUBXNAVTIMEUTC();        // Check that RAM has been allocated for the TIMEUTC data
  if (packetUBXNAVTIMEUTC == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic && packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic && !packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_TIMEUTC;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMEUTC
// works.
bool DevUBLOXGNSS::setAutoNAVTIMEUTC(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVTIMEUTCrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMEUTC
// works.
bool DevUBLOXGNSS::setAutoNAVTIMEUTC(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVTIMEUTCrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMEUTC
// works.
bool DevUBLOXGNSS::setAutoNAVTIMEUTCrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVTIMEUTC == nullptr)
    initPacketUBXNAVTIMEUTC();        // Check that RAM has been allocated for the data
  if (packetUBXNAVTIMEUTC == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVTIMEUTCcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_TIMEUTC_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVTIMEUTC(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVTIMEUTC->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVTIMEUTC->callbackData = new UBX_NAV_TIMEUTC_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVTIMEUTC->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVTIMEUTCcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVTIMEUTC->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and TIMEUTC is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVTIMEUTC(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVTIMEUTC == nullptr)
    initPacketUBXNAVTIMEUTC();        // Check that RAM has been allocated for the data
  if (packetUBXNAVTIMEUTC == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVTIMEUTC and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVTIMEUTC()
{
  packetUBXNAVTIMEUTC = new UBX_NAV_TIMEUTC_t; // Allocate RAM for the main struct
  if (packetUBXNAVTIMEUTC == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVTIMEUTC: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVTIMEUTC->automaticFlags.flags.all = 0;
  packetUBXNAVTIMEUTC->callbackPointerPtr = nullptr;
  packetUBXNAVTIMEUTC->callbackData = nullptr;
  packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVTIMEUTC()
{
  if (packetUBXNAVTIMEUTC == nullptr)
    return;                                                 // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVTIMEUTC(bool enabled)
{
  if (packetUBXNAVTIMEUTC == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVTIMEUTC->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV CLOCK automatic support

bool DevUBLOXGNSS::getNAVCLOCK(uint16_t maxWait)
{
  if (packetUBXNAVCLOCK == nullptr)
    initPacketUBXNAVCLOCK();        // Check that RAM has been allocated for the CLOCK data
  if (packetUBXNAVCLOCK == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic && packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVCLOCK->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic && !packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting CLOCK so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_CLOCK;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic CLOCK message generation by the GNSS. This changes the way getNAVCLOCK
// works.
bool DevUBLOXGNSS::setAutoNAVCLOCK(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVCLOCKrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic CLOCK message generation by the GNSS. This changes the way getNAVCLOCK
// works.
bool DevUBLOXGNSS::setAutoNAVCLOCK(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVCLOCKrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic CLOCK message generation by the GNSS. This changes the way getNAVCLOCK
// works.
bool DevUBLOXGNSS::setAutoNAVCLOCKrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVCLOCK == nullptr)
    initPacketUBXNAVCLOCK();        // Check that RAM has been allocated for the data
  if (packetUBXNAVCLOCK == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVCLOCK->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVCLOCKcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_CLOCK_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVCLOCK(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVCLOCK->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVCLOCK->callbackData = new UBX_NAV_CLOCK_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVCLOCK->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVCLOCKcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVCLOCK->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and NAV CLOCK is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVCLOCK(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVCLOCK == nullptr)
    initPacketUBXNAVCLOCK();        // Check that RAM has been allocated for the CLOCK data
  if (packetUBXNAVCLOCK == nullptr) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVCLOCK and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVCLOCK()
{
  packetUBXNAVCLOCK = new UBX_NAV_CLOCK_t; // Allocate RAM for the main struct
  if (packetUBXNAVCLOCK == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVCLOCK: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVCLOCK->automaticFlags.flags.all = 0;
  packetUBXNAVCLOCK->callbackPointerPtr = nullptr;
  packetUBXNAVCLOCK->callbackData = nullptr;
  packetUBXNAVCLOCK->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVCLOCK()
{
  if (packetUBXNAVCLOCK == nullptr)
    return;                                               // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVCLOCK->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVCLOCK(bool enabled)
{
  if (packetUBXNAVCLOCK == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVCLOCK->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV TIMELS automatic support

// Reads leap second event information and sets the global variables
// for future leap second change and number of leap seconds since GPS epoch
// Returns true if commands was successful
bool DevUBLOXGNSS::getLeapSecondEvent(uint16_t maxWait)
{
  if (packetUBXNAVTIMELS == nullptr)
    initPacketUBXNAVTIMELS();        // Check that RAM has been allocated for the TIMELS data
  if (packetUBXNAVTIMELS == nullptr) // Abort if the RAM allocation failed
    return (false);

  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_TIMELS;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // The data is parsed as part of processing the response
  sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

  if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (true);

  if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
  {
    return (true);
  }

  return (false);
}

// PRIVATE: Allocate RAM for packetUBXNAVTIMELS and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVTIMELS()
{
  packetUBXNAVTIMELS = new UBX_NAV_TIMELS_t; // Allocate RAM for the main struct
  if (packetUBXNAVTIMELS == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVTIMELS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVTIMELS->automaticFlags.flags.all = 0;
  packetUBXNAVTIMELS->callbackPointerPtr = nullptr;
  packetUBXNAVTIMELS->callbackData = nullptr;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// ***** NAV SVIN automatic support

// Reads survey in status and sets the global variables
// for status, position valid, observation time, and mean 3D StdDev
// Returns true if commands was successful
bool DevUBLOXGNSS::getSurveyStatus(uint16_t maxWait)
{
  if (packetUBXNAVSVIN == nullptr)
    initPacketUBXNAVSVIN();        // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == nullptr) // Abort if the RAM allocation failed
    return (false);

  if (packetUBXNAVSVIN->automaticFlags.flags.bits.automatic && packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVSVIN->automaticFlags.flags.bits.automatic && !packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting SVIN so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_SVIN;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic SVIN message generation by the GNSS. This changes the way getSurveyStatus
// works.
bool DevUBLOXGNSS::setAutoNAVSVIN(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVSVINrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic SVIN message generation by the GNSS. This changes the way getSurveyStatus
// works.
bool DevUBLOXGNSS::setAutoNAVSVIN(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVSVINrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic SVIN message generation by the GNSS. This changes the way getSurveyStatus
// works.
bool DevUBLOXGNSS::setAutoNAVSVINrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVSVIN == nullptr)
    initPacketUBXNAVSVIN();        // Check that RAM has been allocated for the data
  if (packetUBXNAVSVIN == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVSVIN->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVSVINcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SVIN_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVSVIN(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVSVIN->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVSVIN->callbackData = new UBX_NAV_SVIN_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVSVIN->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVSVINcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVSVIN->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and SVIN is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVSVIN(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVSVIN == nullptr)
    initPacketUBXNAVSVIN();        // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == nullptr) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXNAVSVIN->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVSVIN->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVSVIN and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVSVIN()
{
  packetUBXNAVSVIN = new UBX_NAV_SVIN_t; // Allocate RAM for the main struct
  if (packetUBXNAVSVIN == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVSVIN: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVSVIN->automaticFlags.flags.all = 0;
  packetUBXNAVSVIN->callbackPointerPtr = nullptr;
  packetUBXNAVSVIN->callbackData = nullptr;
  packetUBXNAVSVIN->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVSVIN()
{
  if (packetUBXNAVSVIN == nullptr)
    return;                                              // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSVIN->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVSVIN(bool enabled)
{
  if (packetUBXNAVSVIN == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSVIN->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
// ***** NAV SAT automatic support

// Signal information
// Returns true if commands was successful
bool DevUBLOXGNSS::getNAVSAT(uint16_t maxWait)
{
  if (packetUBXNAVSAT == nullptr)
    initPacketUBXNAVSAT();        // Check that RAM has been allocated for the NAVSAT data
  if (packetUBXNAVSAT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVSAT->automaticFlags.flags.bits.automatic && packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVSAT->moduleQueried;
  }
  else if (packetUBXNAVSAT->automaticFlags.flags.bits.automatic && !packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting NAVSAT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_SAT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic NAVSAT message generation by the GNSS. This changes the way getNAVSAT
// works.
bool DevUBLOXGNSS::setAutoNAVSAT(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVSATrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic NAVSAT message generation by the GNSS. This changes the way getNAVSAT
// works.
bool DevUBLOXGNSS::setAutoNAVSAT(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVSATrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic NAV SAT message generation by the GNSS. This changes the way getNAVSAT
// works.
bool DevUBLOXGNSS::setAutoNAVSATrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVSAT == nullptr)
    initPacketUBXNAVSAT();        // Check that RAM has been allocated for the data
  if (packetUBXNAVSAT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_SAT_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_SAT_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVSAT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVSAT->moduleQueried = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVSATcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SAT_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVSAT(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVSAT->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVSAT->callbackData = new UBX_NAV_SAT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVSAT->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVSATcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVSAT->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and NAV SAT is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVSAT(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVSAT == nullptr)
    initPacketUBXNAVSAT();        // Check that RAM has been allocated for the NAVSAT data
  if (packetUBXNAVSAT == nullptr) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXNAVSAT->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVSAT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVSAT and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVSAT()
{
  packetUBXNAVSAT = new UBX_NAV_SAT_t; // Allocate RAM for the main struct
  if (packetUBXNAVSAT == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVSAT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVSAT->automaticFlags.flags.all = 0;
  packetUBXNAVSAT->callbackPointerPtr = nullptr;
  packetUBXNAVSAT->callbackData = nullptr;
  packetUBXNAVSAT->moduleQueried = false;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVSAT()
{
  if (packetUBXNAVSAT == nullptr)
    return;                               // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSAT->moduleQueried = false; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVSAT(bool enabled)
{
  if (packetUBXNAVSAT == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSAT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV SIG automatic support

// Signal information
// Returns true if commands was successful
bool DevUBLOXGNSS::getNAVSIG(uint16_t maxWait)
{
  if (packetUBXNAVSIG == nullptr)
    initPacketUBXNAVSIG();        // Check that RAM has been allocated for the NAVSIG data
  if (packetUBXNAVSIG == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVSIG->automaticFlags.flags.bits.automatic && packetUBXNAVSIG->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVSIG->moduleQueried;
  }
  else if (packetUBXNAVSIG->automaticFlags.flags.bits.automatic && !packetUBXNAVSIG->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting NAVSIG so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_SIG;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic NAVSIG message generation by the GNSS. This changes the way getNAVSIG
// works.
bool DevUBLOXGNSS::setAutoNAVSIG(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVSIGrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic NAVSIG message generation by the GNSS. This changes the way getNAVSIG
// works.
bool DevUBLOXGNSS::setAutoNAVSIG(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoNAVSIGrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic NAV SAT message generation by the GNSS. This changes the way getNAVSIG
// works.
bool DevUBLOXGNSS::setAutoNAVSIGrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVSIG == nullptr)
    initPacketUBXNAVSIG();        // Check that RAM has been allocated for the data
  if (packetUBXNAVSIG == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_SIG_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_SIG_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_SIG_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_SIG_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVSIG->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVSIG->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVSIG->moduleQueried = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoNAVSIGcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SIG_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVSIG(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVSIG->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVSIG->callbackData = new UBX_NAV_SIG_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVSIG->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoNAVSIGcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVSIG->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and NAV SAT is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoNAVSIG(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVSIG == nullptr)
    initPacketUBXNAVSIG();        // Check that RAM has been allocated for the NAVSIG data
  if (packetUBXNAVSIG == nullptr) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXNAVSIG->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVSIG->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVSIG->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVSIG->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVSIG and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVSIG()
{
  packetUBXNAVSIG = new UBX_NAV_SIG_t; // Allocate RAM for the main struct
  if (packetUBXNAVSIG == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVSIG: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVSIG->automaticFlags.flags.all = 0;
  packetUBXNAVSIG->callbackPointerPtr = nullptr;
  packetUBXNAVSIG->callbackData = nullptr;
  packetUBXNAVSIG->moduleQueried = false;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVSIG()
{
  if (packetUBXNAVSIG == nullptr)
    return;                               // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSIG->moduleQueried = false; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVSIG(bool enabled)
{
  if (packetUBXNAVSIG == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSIG->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}
#endif

// ***** NAV RELPOSNED automatic support

// Relative Positioning Information in NED frame
// Returns true if commands was successful
// Note:
//   RELPOSNED on the M8 is only 40 bytes long
//   RELPOSNED on the F9 is 64 bytes long and contains much more information
bool DevUBLOXGNSS::getRELPOSNED(uint16_t maxWait)
{
  if (packetUBXNAVRELPOSNED == nullptr)
    initPacketUBXNAVRELPOSNED();        // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic && packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic && !packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting RELPOSNED so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_RELPOSNED;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic RELPOSNED message generation by the GNSS. This changes the way getRELPOSNED
// works.
bool DevUBLOXGNSS::setAutoRELPOSNED(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoRELPOSNEDrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic RELPOSNED message generation by the GNSS. This changes the way getRELPOSNED
// works.
bool DevUBLOXGNSS::setAutoRELPOSNED(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoRELPOSNEDrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic RELPOSNED message generation by the GNSS. This changes the way getRELPOSNED
// works.
bool DevUBLOXGNSS::setAutoRELPOSNEDrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVRELPOSNED == nullptr)
    initPacketUBXNAVRELPOSNED();        // Check that RAM has been allocated for the data
  if (packetUBXNAVRELPOSNED == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoRELPOSNEDcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_RELPOSNED_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRELPOSNED(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVRELPOSNED->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVRELPOSNED->callbackData = new UBX_NAV_RELPOSNED_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVRELPOSNED->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoRELPOSNEDcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVRELPOSNED->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and RELPOSNED is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoRELPOSNED(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVRELPOSNED == nullptr)
    initPacketUBXNAVRELPOSNED();        // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVRELPOSNED and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVRELPOSNED()
{
  packetUBXNAVRELPOSNED = new UBX_NAV_RELPOSNED_t; // Allocate RAM for the main struct
  if (packetUBXNAVRELPOSNED == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVRELPOSNED: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVRELPOSNED->automaticFlags.flags.all = 0;
  packetUBXNAVRELPOSNED->callbackPointerPtr = nullptr;
  packetUBXNAVRELPOSNED->callbackData = nullptr;
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushNAVRELPOSNED()
{
  if (packetUBXNAVRELPOSNED == nullptr)
    return;                                                   // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logNAVRELPOSNED(bool enabled)
{
  if (packetUBXNAVRELPOSNED == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVRELPOSNED->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** AOPSTATUS automatic support

bool DevUBLOXGNSS::getAOPSTATUS(uint16_t maxWait)
{
  if (packetUBXNAVAOPSTATUS == nullptr)
    initPacketUBXNAVAOPSTATUS();        // Check that RAM has been allocated for the AOPSTATUS data
  if (packetUBXNAVAOPSTATUS == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic && packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic && !packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_AOPSTATUS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getAOPSTATUS
// works.
bool DevUBLOXGNSS::setAutoAOPSTATUS(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoAOPSTATUSrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getAOPSTATUS
// works.
bool DevUBLOXGNSS::setAutoAOPSTATUS(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoAOPSTATUSrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getAOPSTATUS
// works.
bool DevUBLOXGNSS::setAutoAOPSTATUSrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVAOPSTATUS == nullptr)
    initPacketUBXNAVAOPSTATUS();        // Check that RAM has been allocated for the data
  if (packetUBXNAVAOPSTATUS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_AOPSTATUS_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_AOPSTATUS_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
    key = UBLOX_CFG_MSGOUT_UBX_NAV_AOPSTATUS_UART1; // Only supported on the M10 - no UART2

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoAOPSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_NAV_AOPSTATUS_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoAOPSTATUS(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVAOPSTATUS->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVAOPSTATUS->callbackData = new UBX_NAV_AOPSTATUS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVAOPSTATUS->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoAOPSTATUScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVAOPSTATUS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and AOPSTATUS is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoAOPSTATUS(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVAOPSTATUS == nullptr)
    initPacketUBXNAVAOPSTATUS();        // Check that RAM has been allocated for the data
  if (packetUBXNAVAOPSTATUS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVAOPSTATUS and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVAOPSTATUS()
{
  packetUBXNAVAOPSTATUS = new UBX_NAV_AOPSTATUS_t; // Allocate RAM for the main struct
  if (packetUBXNAVAOPSTATUS == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVAOPSTATUS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVAOPSTATUS->automaticFlags.flags.all = 0;
  packetUBXNAVAOPSTATUS->callbackPointerPtr = nullptr;
  packetUBXNAVAOPSTATUS->callbackData = nullptr;
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the AOPSTATUS data as read/stale. This is handy to get data alignment after CRC failure
void DevUBLOXGNSS::flushAOPSTATUS()
{
  if (packetUBXNAVAOPSTATUS == nullptr)
    return;                                                   // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.all = 0; // Mark all AOPSTATUSs as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logAOPSTATUS(bool enabled)
{
  if (packetUBXNAVAOPSTATUS == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
// ***** RXM PMP automatic support

// Callback receives a pointer to the data, instead of _all_ the data. Much kinder on the stack!
bool DevUBLOXGNSS::setRXMPMPcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_PMP_data_t *))
{
  if (packetUBXRXMPMP == nullptr)
    initPacketUBXRXMPMP();        // Check that RAM has been allocated for the data
  if (packetUBXRXMPMP == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMPMP->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMPMP->callbackData = new UBX_RXM_PMP_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMPMP->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoRXMPMPcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMPMP->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// PRIVATE: Allocate RAM for packetUBXRXMPMP and initialize it
bool DevUBLOXGNSS::initPacketUBXRXMPMP()
{
  packetUBXRXMPMP = new UBX_RXM_PMP_t; // Allocate RAM for the main struct
  if (packetUBXRXMPMP == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXRXMPMP: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMPMP->automaticFlags.flags.all = 0;
  packetUBXRXMPMP->callbackPointerPtr = nullptr;
  packetUBXRXMPMP->callbackData = nullptr;
  return (true);
}

// Callback receives a pointer to the data, instead of _all_ the data. Much kinder on the stack!
bool DevUBLOXGNSS::setRXMPMPmessageCallbackPtr(void (*callbackPointerPtr)(UBX_RXM_PMP_message_data_t *))
{
  if (packetUBXRXMPMPmessage == nullptr)
    initPacketUBXRXMPMPmessage();        // Check that RAM has been allocated for the data
  if (packetUBXRXMPMPmessage == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMPMPmessage->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMPMPmessage->callbackData = new UBX_RXM_PMP_message_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMPMPmessage->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoRXMPMPmessagecallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMPMPmessage->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// PRIVATE: Allocate RAM for packetUBXRXMPMPmessage and initialize it
bool DevUBLOXGNSS::initPacketUBXRXMPMPmessage()
{
  packetUBXRXMPMPmessage = new UBX_RXM_PMP_message_t; // Allocate RAM for the main struct
  if (packetUBXRXMPMPmessage == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXRXMPMPmessage: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMPMPmessage->automaticFlags.flags.all = 0;
  packetUBXRXMPMPmessage->callbackPointerPtr = nullptr;
  packetUBXRXMPMPmessage->callbackData = nullptr;
  return (true);
}

// ***** RXM QZSSL6 automatic support

// Callback receives a pointer to the data, instead of _all_ the data. Much kinder on the stack!
bool DevUBLOXGNSS::setRXMQZSSL6messageCallbackPtr(void (*callbackPointerPtr)(UBX_RXM_QZSSL6_message_data_t *))
{
  if (packetUBXRXMQZSSL6message == nullptr)
    initPacketUBXRXMQZSSL6message();        // Check that RAM has been allocated for the data
  if (packetUBXRXMQZSSL6message == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMQZSSL6message->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMQZSSL6message->callbackData = new UBX_RXM_QZSSL6_message_data_t[UBX_RXM_QZSSL6_NUM_CHANNELS]; // Allocate RAM for the main struct
  }

  if (packetUBXRXMQZSSL6message->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoRXMQZSSL6messagecallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMQZSSL6message->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// PRIVATE: Allocate RAM for packetUBXRXMQZSSL6message and initialize it
bool DevUBLOXGNSS::initPacketUBXRXMQZSSL6message()
{
  packetUBXRXMQZSSL6message = new UBX_RXM_QZSSL6_message_t; // Allocate RAM for the main struct
  if (packetUBXRXMQZSSL6message == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXRXMQZSSL6message: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMQZSSL6message->automaticFlags.flags.all = 0;
  packetUBXRXMQZSSL6message->callbackPointerPtr = nullptr;
  packetUBXRXMQZSSL6message->callbackData = nullptr;
  return (true);
}

bool DevUBLOXGNSS::setRXMCORcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_COR_data_t *))
{
  if (packetUBXRXMCOR == nullptr)
    initPacketUBXRXMCOR();        // Check that RAM has been allocated for the data
  if (packetUBXRXMCOR == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMCOR->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMCOR->callbackData = new UBX_RXM_COR_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMCOR->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoRXMCORcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMCOR->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// PRIVATE: Allocate RAM for packetUBXRXMCOR and initialize it
bool DevUBLOXGNSS::initPacketUBXRXMCOR()
{
  packetUBXRXMCOR = new UBX_RXM_COR_t; // Allocate RAM for the main struct
  if (packetUBXRXMCOR == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXRXMCOR: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMCOR->automaticFlags.flags.all = 0;
  packetUBXRXMCOR->callbackPointerPtr = nullptr;
  packetUBXRXMCOR->callbackData = nullptr;
  return (true);
}

// ***** RXM SFRBX automatic support

bool DevUBLOXGNSS::getRXMSFRBX(uint16_t maxWait)
{
  if (packetUBXRXMSFRBX == nullptr)
    initPacketUBXRXMSFRBX();        // Check that RAM has been allocated for the SFRBX data
  if (packetUBXRXMSFRBX == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic && packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXRXMSFRBX->moduleQueried;
  }
  else if (packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic && !packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // SFRBX is output-only. It cannot be polled...
    // Strictly, getRXMSFRBX should be deprecated. But, to keep the library backward compatible, return(false) here.
    // See issue #167 for details
    (void)maxWait;
    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMSFRBX
// works.
bool DevUBLOXGNSS::setAutoRXMSFRBX(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoRXMSFRBXrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMSFRBX
// works.
bool DevUBLOXGNSS::setAutoRXMSFRBX(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoRXMSFRBXrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMSFRBX
// works.
bool DevUBLOXGNSS::setAutoRXMSFRBXrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXRXMSFRBX == nullptr)
    initPacketUBXRXMSFRBX();        // Check that RAM has been allocated for the data
  if (packetUBXRXMSFRBX == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXRXMSFRBX->moduleQueried = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoRXMSFRBXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_SFRBX_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRXMSFRBX(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXRXMSFRBX->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMSFRBX->callbackData = new UBX_RXM_SFRBX_data_t[UBX_RXM_SFRBX_CALLBACK_BUFFERS]; // Allocate RAM for the main struct
  }

  if (packetUBXRXMSFRBX->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoRXMSFRBXcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMSFRBX->callbackPointerPtr = callbackPointerPtr;
  return (true);
}
// Use this if you want all of the SFRBX message (including sync chars, checksum, etc.) to push to the PointPerfect Library
bool DevUBLOXGNSS::setAutoRXMSFRBXmessageCallbackPtr(void (*callbackMessagePointerPtr)(UBX_RXM_SFRBX_message_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRXMSFRBX(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXRXMSFRBX->callbackMessageData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMSFRBX->callbackMessageData = new UBX_RXM_SFRBX_message_data_t[UBX_RXM_SFRBX_CALLBACK_BUFFERS]; // Allocate RAM for the main struct
  }

  if (packetUBXRXMSFRBX->callbackMessageData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoRXMSFRBXmessageCallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMSFRBX->callbackMessagePointerPtr = callbackMessagePointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and SFRBX is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoRXMSFRBX(bool enabled, bool implicitUpdate)
{
  if (packetUBXRXMSFRBX == nullptr)
    initPacketUBXRXMSFRBX();        // Check that RAM has been allocated for the data
  if (packetUBXRXMSFRBX == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic != enabled || packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic = enabled;
    packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXRXMSFRBX and initialize it
bool DevUBLOXGNSS::initPacketUBXRXMSFRBX()
{
  packetUBXRXMSFRBX = new UBX_RXM_SFRBX_t; // Allocate RAM for the main struct
  if (packetUBXRXMSFRBX == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXRXMSFRBX: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMSFRBX->automaticFlags.flags.all = 0;
  packetUBXRXMSFRBX->callbackPointerPtr = nullptr;
  packetUBXRXMSFRBX->callbackData = nullptr;
  packetUBXRXMSFRBX->callbackMessagePointerPtr = nullptr;
  packetUBXRXMSFRBX->callbackMessageData = nullptr;
  packetUBXRXMSFRBX->moduleQueried = false;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushRXMSFRBX()
{
  if (packetUBXRXMSFRBX == nullptr)
    return;                                 // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMSFRBX->moduleQueried = false; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logRXMSFRBX(bool enabled)
{
  if (packetUBXRXMSFRBX == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMSFRBX->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** RXM RAWX automatic support

bool DevUBLOXGNSS::getRXMRAWX(uint16_t maxWait)
{
  if (packetUBXRXMRAWX == nullptr)
    initPacketUBXRXMRAWX();        // Check that RAM has been allocated for the RAWX data
  if (packetUBXRXMRAWX == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXRXMRAWX->automaticFlags.flags.bits.automatic && packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXRXMRAWX->moduleQueried;
  }
  else if (packetUBXRXMRAWX->automaticFlags.flags.bits.automatic && !packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_RXM;
    packetCfg.id = UBX_RXM_RAWX;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMRAWX
// works.
bool DevUBLOXGNSS::setAutoRXMRAWX(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoRXMRAWXrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMRAWX
// works.
bool DevUBLOXGNSS::setAutoRXMRAWX(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoRXMRAWXrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMRAWX
// works.
bool DevUBLOXGNSS::setAutoRXMRAWXrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXRXMRAWX == nullptr)
    initPacketUBXRXMRAWX();        // Check that RAM has been allocated for the data
  if (packetUBXRXMRAWX == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXRXMRAWX->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXRXMRAWX->moduleQueried = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoRXMRAWXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_RAWX_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRXMRAWX(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXRXMRAWX->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMRAWX->callbackData = new UBX_RXM_RAWX_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMRAWX->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoRXMRAWXcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMRAWX->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and VELNED is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoRXMRAWX(bool enabled, bool implicitUpdate)
{
  if (packetUBXRXMRAWX == nullptr)
    initPacketUBXRXMRAWX();        // Check that RAM has been allocated for the data
  if (packetUBXRXMRAWX == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXRXMRAWX->automaticFlags.flags.bits.automatic != enabled || packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXRXMRAWX->automaticFlags.flags.bits.automatic = enabled;
    packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXRXMRAWX and initialize it
bool DevUBLOXGNSS::initPacketUBXRXMRAWX()
{
  packetUBXRXMRAWX = new UBX_RXM_RAWX_t; // Allocate RAM for the main struct
  if (packetUBXRXMRAWX == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXRXMRAWX: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMRAWX->automaticFlags.flags.all = 0;
  packetUBXRXMRAWX->callbackPointerPtr = nullptr;
  packetUBXRXMRAWX->callbackData = nullptr;
  packetUBXRXMRAWX->moduleQueried = false;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushRXMRAWX()
{
  if (packetUBXRXMRAWX == nullptr)
    return;                                // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMRAWX->moduleQueried = false; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logRXMRAWX(bool enabled)
{
  if (packetUBXRXMRAWX == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMRAWX->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** RXM MEASX automatic support

bool DevUBLOXGNSS::getRXMMEASX(uint16_t maxWait)
{
  if (packetUBXRXMMEASX == nullptr)
    initPacketUBXRXMMEASX();        // Check that RAM has been allocated for the MEASX data
  if (packetUBXRXMMEASX == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXRXMMEASX->automaticFlags.flags.bits.automatic && packetUBXRXMMEASX->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXRXMMEASX->moduleQueried;
  }
  else if (packetUBXRXMMEASX->automaticFlags.flags.bits.automatic && !packetUBXRXMMEASX->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_RXM;
    packetCfg.id = UBX_RXM_MEASX;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMMEASX
// works.
bool DevUBLOXGNSS::setAutoRXMMEASX(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoRXMMEASXrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMMEASX
// works.
bool DevUBLOXGNSS::setAutoRXMMEASX(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoRXMMEASXrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMMEASX
// works.
bool DevUBLOXGNSS::setAutoRXMMEASXrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXRXMMEASX == nullptr)
    initPacketUBXRXMMEASX();        // Check that RAM has been allocated for the data
  if (packetUBXRXMMEASX == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXRXMMEASX->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXRXMMEASX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXRXMMEASX->moduleQueried = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoRXMMEASXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_MEASX_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRXMMEASX(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXRXMMEASX->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMMEASX->callbackData = new UBX_RXM_MEASX_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMMEASX->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoRXMMEASXcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMMEASX->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and VELNED is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoRXMMEASX(bool enabled, bool implicitUpdate)
{
  if (packetUBXRXMMEASX == nullptr)
    initPacketUBXRXMMEASX();        // Check that RAM has been allocated for the data
  if (packetUBXRXMMEASX == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXRXMMEASX->automaticFlags.flags.bits.automatic != enabled || packetUBXRXMMEASX->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXRXMMEASX->automaticFlags.flags.bits.automatic = enabled;
    packetUBXRXMMEASX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXRXMMEASX and initialize it
bool DevUBLOXGNSS::initPacketUBXRXMMEASX()
{
  packetUBXRXMMEASX = new UBX_RXM_MEASX_t; // Allocate RAM for the main struct
  if (packetUBXRXMMEASX == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXRXMMEASX: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMMEASX->automaticFlags.flags.all = 0;
  packetUBXRXMMEASX->callbackPointerPtr = nullptr;
  packetUBXRXMMEASX->callbackData = nullptr;
  packetUBXRXMMEASX->moduleQueried = false;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushRXMMEASX()
{
  if (packetUBXRXMMEASX == nullptr)
    return;                                 // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMMEASX->moduleQueried = false; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logRXMMEASX(bool enabled)
{
  if (packetUBXRXMMEASX == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMMEASX->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}
#endif

// ***** TIM TM2 automatic support

bool DevUBLOXGNSS::getTIMTM2(uint16_t maxWait)
{
  if (packetUBXTIMTM2 == nullptr)
    initPacketUBXTIMTM2();        // Check that RAM has been allocated for the TM2 data
  if (packetUBXTIMTM2 == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXTIMTM2->automaticFlags.flags.bits.automatic && packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXTIMTM2->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXTIMTM2->automaticFlags.flags.bits.automatic && !packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_TIM;
    packetCfg.id = UBX_TIM_TM2;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMTM2
// works.
bool DevUBLOXGNSS::setAutoTIMTM2(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoTIMTM2rate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMTM2
// works.
bool DevUBLOXGNSS::setAutoTIMTM2(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoTIMTM2rate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMTM2
// works.
bool DevUBLOXGNSS::setAutoTIMTM2rate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXTIMTM2 == nullptr)
    initPacketUBXTIMTM2();        // Check that RAM has been allocated for the data
  if (packetUBXTIMTM2 == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_TIM_TM2_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_TIM_TM2_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_TIM_TM2_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_TIM_TM2_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXTIMTM2->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXTIMTM2->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoTIMTM2callbackPtr(void (*callbackPointerPtr)(UBX_TIM_TM2_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoTIMTM2(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXTIMTM2->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXTIMTM2->callbackData = new UBX_TIM_TM2_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXTIMTM2->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoTIMTM2callbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXTIMTM2->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and VELNED is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoTIMTM2(bool enabled, bool implicitUpdate)
{
  if (packetUBXTIMTM2 == nullptr)
    initPacketUBXTIMTM2();        // Check that RAM has been allocated for the data
  if (packetUBXTIMTM2 == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXTIMTM2->automaticFlags.flags.bits.automatic != enabled || packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXTIMTM2->automaticFlags.flags.bits.automatic = enabled;
    packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXTIMTM2 and initialize it
bool DevUBLOXGNSS::initPacketUBXTIMTM2()
{
  packetUBXTIMTM2 = new UBX_TIM_TM2_t; // Allocate RAM for the main struct
  if (packetUBXTIMTM2 == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXTIMTM2: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXTIMTM2->automaticFlags.flags.all = 0;
  packetUBXTIMTM2->callbackPointerPtr = nullptr;
  packetUBXTIMTM2->callbackData = nullptr;
  packetUBXTIMTM2->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushTIMTM2()
{
  if (packetUBXTIMTM2 == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXTIMTM2->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logTIMTM2(bool enabled)
{
  if (packetUBXTIMTM2 == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXTIMTM2->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** TIM TP automatic support

bool DevUBLOXGNSS::getTIMTP(uint16_t maxWait)
{
  if (packetUBXTIMTP == nullptr)
    initPacketUBXTIMTP();        // Check that RAM has been allocated for the TP data
  if (packetUBXTIMTP == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXTIMTP->automaticFlags.flags.bits.automatic && packetUBXTIMTP->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXTIMTP->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXTIMTP->automaticFlags.flags.bits.automatic && !packetUBXTIMTP->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_TIM;
    packetCfg.id = UBX_TIM_TP;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic message generation by the GNSS. This changes the way getTIMTP works.
bool DevUBLOXGNSS::setAutoTIMTP(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoTIMTPrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic message generation by the GNSS. This changes the way getTIMTP works.
bool DevUBLOXGNSS::setAutoTIMTP(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoTIMTPrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic message generation by the GNSS. This changes the way getTIMTP works.
bool DevUBLOXGNSS::setAutoTIMTPrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXTIMTP == nullptr)
    initPacketUBXTIMTP();        // Check that RAM has been allocated for the data
  if (packetUBXTIMTP == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_TIM_TP_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_TIM_TP_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_TIM_TP_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_TIM_TP_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXTIMTP->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXTIMTP->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic message generation by the GNSS.
bool DevUBLOXGNSS::setAutoTIMTPcallbackPtr(void (*callbackPointerPtr)(UBX_TIM_TP_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoTIMTP(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXTIMTP->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXTIMTP->callbackData = new UBX_TIM_TP_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXTIMTP->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoTIMTPcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXTIMTP->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and TIM TP is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoTIMTP(bool enabled, bool implicitUpdate)
{
  if (packetUBXTIMTP == nullptr)
    initPacketUBXTIMTP();        // Check that RAM has been allocated for the data
  if (packetUBXTIMTP == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXTIMTP->automaticFlags.flags.bits.automatic != enabled || packetUBXTIMTP->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXTIMTP->automaticFlags.flags.bits.automatic = enabled;
    packetUBXTIMTP->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXTIMTP and initialize it
bool DevUBLOXGNSS::initPacketUBXTIMTP()
{
  packetUBXTIMTP = new UBX_TIM_TP_t; // Allocate RAM for the main struct
  if (packetUBXTIMTP == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXTIMTP: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXTIMTP->automaticFlags.flags.all = 0;
  packetUBXTIMTP->callbackPointerPtr = nullptr;
  packetUBXTIMTP->callbackData = nullptr;
  packetUBXTIMTP->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushTIMTP()
{
  if (packetUBXTIMTP == nullptr)
    return;                                            // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXTIMTP->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logTIMTP(bool enabled)
{
  if (packetUBXTIMTP == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXTIMTP->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** MON HW automatic support

bool DevUBLOXGNSS::getMONHW(uint16_t maxWait)
{
  if (packetUBXMONHW == nullptr)
    initPacketUBXMONHW();        // Check that RAM has been allocated for the TP data
  if (packetUBXMONHW == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXMONHW->automaticFlags.flags.bits.automatic && packetUBXMONHW->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXMONHW->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXMONHW->automaticFlags.flags.bits.automatic && !packetUBXMONHW->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_MON;
    packetCfg.id = UBX_MON_HW;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic message generation by the GNSS. This changes the way getMONHW works.
bool DevUBLOXGNSS::setAutoMONHW(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoMONHWrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic message generation by the GNSS. This changes the way getMONHW works.
bool DevUBLOXGNSS::setAutoMONHW(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoMONHWrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic message generation by the GNSS. This changes the way getMONHW works.
bool DevUBLOXGNSS::setAutoMONHWrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXMONHW == nullptr)
    initPacketUBXMONHW();        // Check that RAM has been allocated for the data
  if (packetUBXMONHW == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_MON_HW_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_MON_HW_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_MON_HW_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_MON_HW_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXMONHW->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXMONHW->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXMONHW->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic message generation by the GNSS.
bool DevUBLOXGNSS::setAutoMONHWcallbackPtr(void (*callbackPointerPtr)(UBX_MON_HW_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoMONHW(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXMONHW->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXMONHW->callbackData = new UBX_MON_HW_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXMONHW->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoMONHWcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXMONHW->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and TIM TP is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoMONHW(bool enabled, bool implicitUpdate)
{
  if (packetUBXMONHW == nullptr)
    initPacketUBXMONHW();        // Check that RAM has been allocated for the data
  if (packetUBXMONHW == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXMONHW->automaticFlags.flags.bits.automatic != enabled || packetUBXMONHW->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXMONHW->automaticFlags.flags.bits.automatic = enabled;
    packetUBXMONHW->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXMONHW and initialize it
bool DevUBLOXGNSS::initPacketUBXMONHW()
{
  packetUBXMONHW = new UBX_MON_HW_t; // Allocate RAM for the main struct
  if (packetUBXMONHW == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXMONHW: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXMONHW->automaticFlags.flags.all = 0;
  packetUBXMONHW->callbackPointerPtr = nullptr;
  packetUBXMONHW->callbackData = nullptr;
  packetUBXMONHW->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushMONHW()
{
  if (packetUBXMONHW == nullptr)
    return;                                            // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXMONHW->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logMONHW(bool enabled)
{
  if (packetUBXMONHW == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXMONHW->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

#ifndef SFE_UBLOX_DISABLE_ESF
// ***** ESF ALG automatic support

bool DevUBLOXGNSS::getEsfAlignment(uint16_t maxWait)
{
  return (getESFALG(maxWait));
}

bool DevUBLOXGNSS::getESFALG(uint16_t maxWait)
{
  if (packetUBXESFALG == nullptr)
    initPacketUBXESFALG();        // Check that RAM has been allocated for the ESF alignment data
  if (packetUBXESFALG == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXESFALG->automaticFlags.flags.bits.automatic && packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXESFALG->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXESFALG->automaticFlags.flags.bits.automatic && !packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_ALG;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic ESF ALG message generation by the GNSS. This changes the way getEsfAlignment
// works.
bool DevUBLOXGNSS::setAutoESFALG(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFALGrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic ESF ALG message generation by the GNSS. This changes the way getEsfAlignment
// works.
bool DevUBLOXGNSS::setAutoESFALG(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFALGrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic ESF ALG message generation by the GNSS. This changes the way getEsfAlignment
// works.
bool DevUBLOXGNSS::setAutoESFALGrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXESFALG == nullptr)
    initPacketUBXESFALG();        // Check that RAM has been allocated for the data
  if (packetUBXESFALG == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_ESF_ALG_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_ESF_ALG_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_ESF_ALG_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_ESF_ALG_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXESFALG->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoESFALGcallbackPtr(void (*callbackPointerPtr)(UBX_ESF_ALG_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFALG(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFALG->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFALG->callbackData = new UBX_ESF_ALG_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFALG->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoESFALGcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFALG->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF ALG is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoESFALG(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFALG == nullptr)
    initPacketUBXESFALG();        // Check that RAM has been allocated for the ESF alignment data
  if (packetUBXESFALG == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFALG->automaticFlags.flags.bits.automatic != enabled || packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFALG->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFALG and initialize it
bool DevUBLOXGNSS::initPacketUBXESFALG()
{
  packetUBXESFALG = new UBX_ESF_ALG_t; // Allocate RAM for the main struct
  if (packetUBXESFALG == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXESFALG: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFALG->automaticFlags.flags.all = 0;
  packetUBXESFALG->callbackPointerPtr = nullptr;
  packetUBXESFALG->callbackData = nullptr;
  packetUBXESFALG->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushESFALG()
{
  if (packetUBXESFALG == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFALG->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logESFALG(bool enabled)
{
  if (packetUBXESFALG == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFALG->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** ESF STATUS automatic support

bool DevUBLOXGNSS::getEsfInfo(uint16_t maxWait)
{
  return (getESFSTATUS(maxWait));
}

bool DevUBLOXGNSS::getESFSTATUS(uint16_t maxWait)
{
  if (packetUBXESFSTATUS == nullptr)
    initPacketUBXESFSTATUS();        // Check that RAM has been allocated for the ESF status data
  if (packetUBXESFSTATUS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXESFSTATUS->automaticFlags.flags.bits.automatic && packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXESFSTATUS->automaticFlags.flags.bits.automatic && !packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_STATUS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic ESF STATUS message generation by the GNSS. This changes the way getESFInfo
// works.
bool DevUBLOXGNSS::setAutoESFSTATUS(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFSTATUSrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic ESF STATUS message generation by the GNSS. This changes the way getESFInfo
// works.
bool DevUBLOXGNSS::setAutoESFSTATUS(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFSTATUSrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic ESF STATUS message generation by the GNSS. This changes the way getESFInfo
// works.
bool DevUBLOXGNSS::setAutoESFSTATUSrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXESFSTATUS == nullptr)
    initPacketUBXESFSTATUS();        // Check that RAM has been allocated for the data
  if (packetUBXESFSTATUS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_ESF_STATUS_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_ESF_STATUS_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_ESF_STATUS_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_ESF_STATUS_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXESFSTATUS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoESFSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_STATUS_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFSTATUS(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFSTATUS->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFSTATUS->callbackData = new UBX_ESF_STATUS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFSTATUS->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoESFSTATUScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFSTATUS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF STATUS is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoESFSTATUS(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFSTATUS == nullptr)
    initPacketUBXESFSTATUS();        // Check that RAM has been allocated for the ESF status data
  if (packetUBXESFSTATUS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFSTATUS->automaticFlags.flags.bits.automatic != enabled || packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFSTATUS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFSTATUS and initialize it
bool DevUBLOXGNSS::initPacketUBXESFSTATUS()
{
  packetUBXESFSTATUS = new UBX_ESF_STATUS_t; // Allocate RAM for the main struct

  if (packetUBXESFSTATUS == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXESFSTATUS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFSTATUS->automaticFlags.flags.all = 0;
  packetUBXESFSTATUS->callbackPointerPtr = nullptr;
  packetUBXESFSTATUS->callbackData = nullptr;
  packetUBXESFSTATUS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushESFSTATUS()
{
  if (packetUBXESFSTATUS == nullptr)
    return;                                                // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFSTATUS->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logESFSTATUS(bool enabled)
{
  if (packetUBXESFSTATUS == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFSTATUS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** ESF INS automatic support

bool DevUBLOXGNSS::getEsfIns(uint16_t maxWait)
{
  return (getESFINS(maxWait));
}

bool DevUBLOXGNSS::getESFINS(uint16_t maxWait)
{
  if (packetUBXESFINS == nullptr)
    initPacketUBXESFINS();        // Check that RAM has been allocated for the ESF INS data
  if (packetUBXESFINS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXESFINS->automaticFlags.flags.bits.automatic && packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXESFINS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXESFINS->automaticFlags.flags.bits.automatic && !packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_INS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic ESF INS message generation by the GNSS. This changes the way getESFIns
// works.
bool DevUBLOXGNSS::setAutoESFINS(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFINSrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic ESF INS message generation by the GNSS. This changes the way getESFIns
// works.
bool DevUBLOXGNSS::setAutoESFINS(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFINSrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic ESF INS message generation by the GNSS. This changes the way getESFIns
// works.
bool DevUBLOXGNSS::setAutoESFINSrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXESFINS == nullptr)
    initPacketUBXESFINS();        // Check that RAM has been allocated for the data
  if (packetUBXESFINS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_ESF_INS_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_ESF_INS_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_ESF_INS_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_ESF_INS_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXESFINS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXESFINS->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoESFINScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_INS_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFINS(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFINS->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFINS->callbackData = new UBX_ESF_INS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFINS->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoESFINScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFINS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF INS is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoESFINS(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFINS == nullptr)
    initPacketUBXESFINS();        // Check that RAM has been allocated for the ESF INS data
  if (packetUBXESFINS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFINS->automaticFlags.flags.bits.automatic != enabled || packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFINS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFINS and initialize it
bool DevUBLOXGNSS::initPacketUBXESFINS()
{
  packetUBXESFINS = new UBX_ESF_INS_t; // Allocate RAM for the main struct
  if (packetUBXESFINS == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXESFINS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFINS->automaticFlags.flags.all = 0;
  packetUBXESFINS->callbackPointerPtr = nullptr;
  packetUBXESFINS->callbackData = nullptr;
  packetUBXESFINS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushESFINS()
{
  if (packetUBXESFINS == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFINS->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logESFINS(bool enabled)
{
  if (packetUBXESFINS == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFINS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** ESF MEAS automatic support

// Enable or disable automatic ESF MEAS message generation by the GNSS
bool DevUBLOXGNSS::setAutoESFMEAS(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFMEASrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic ESF MEAS message generation by the GNSS
bool DevUBLOXGNSS::setAutoESFMEAS(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFMEASrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic ESF MEAS message generation by the GNSS
bool DevUBLOXGNSS::setAutoESFMEASrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXESFMEAS == nullptr)
    initPacketUBXESFMEAS();        // Check that RAM has been allocated for the data
  if (packetUBXESFMEAS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_ESF_MEAS_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_ESF_MEAS_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_ESF_MEAS_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_ESF_MEAS_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXESFMEAS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFMEAS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoESFMEAScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_MEAS_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFMEAS(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFMEAS->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFMEAS->callbackData = new UBX_ESF_MEAS_data_t[UBX_ESF_MEAS_CALLBACK_BUFFERS]; // Allocate RAM for the main struct
  }

  if (packetUBXESFMEAS->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoESFMEAScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFMEAS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF MEAS is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoESFMEAS(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFMEAS == nullptr)
    initPacketUBXESFMEAS();        // Check that RAM has been allocated for the ESF MEAS data
  if (packetUBXESFMEAS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFMEAS->automaticFlags.flags.bits.automatic != enabled || packetUBXESFMEAS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFMEAS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFMEAS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFMEAS and initialize it
bool DevUBLOXGNSS::initPacketUBXESFMEAS()
{
  packetUBXESFMEAS = new UBX_ESF_MEAS_t; // Allocate RAM for the main struct
  if (packetUBXESFMEAS == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXESFMEAS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFMEAS->automaticFlags.flags.all = 0;
  packetUBXESFMEAS->callbackPointerPtr = nullptr;
  packetUBXESFMEAS->callbackData = nullptr;
  return (true);
}

// Log this data in file buffer
void DevUBLOXGNSS::logESFMEAS(bool enabled)
{
  if (packetUBXESFMEAS == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFMEAS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** ESF RAW automatic support

// ESF RAW messages are output only. They cannot be polled.

// Enable or disable automatic ESF RAW message generation by the GNSS.
bool DevUBLOXGNSS::setAutoESFRAW(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFRAWrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic ESF RAW message generation by the GNSS.
bool DevUBLOXGNSS::setAutoESFRAW(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoESFRAWrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic ESF RAW message generation by the GNSS.
// Note: this function can only be used to enable or disable the messages. A rate of zero disables the messages.
// A rate of 1 or more causes the messages to be generated at the full 100Hz.
bool DevUBLOXGNSS::setAutoESFRAWrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXESFRAW == nullptr)
    initPacketUBXESFRAW();        // Check that RAM has been allocated for the data
  if (packetUBXESFRAW == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_ESF_RAW_I2C;
  if (_commType == COMM_TYPE_SPI)
    key = UBLOX_CFG_MSGOUT_UBX_ESF_RAW_SPI;
  else if (_commType == COMM_TYPE_SERIAL)
  {
    if (!_UART2)
      key = UBLOX_CFG_MSGOUT_UBX_ESF_RAW_UART1;
    else
      key = UBLOX_CFG_MSGOUT_UBX_ESF_RAW_UART2;
  }

  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXESFRAW->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFRAW->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return ok;
}

// Enable automatic message generation by the GNSS.
bool DevUBLOXGNSS::setAutoESFRAWcallbackPtr(void (*callbackPointerPtr)(UBX_ESF_RAW_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFRAW(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFRAW->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFRAW->callbackData = new UBX_ESF_RAW_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFRAW->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoESFRAWcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFRAW->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF RAW is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoESFRAW(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFRAW == nullptr)
    initPacketUBXESFRAW();        // Check that RAM has been allocated for the ESF RAW data
  if (packetUBXESFRAW == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFRAW->automaticFlags.flags.bits.automatic != enabled || packetUBXESFRAW->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFRAW->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFRAW->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFRAW and initialize it
bool DevUBLOXGNSS::initPacketUBXESFRAW()
{
  packetUBXESFRAW = new UBX_ESF_RAW_t; // Allocate RAM for the main struct
  if (packetUBXESFRAW == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXESFRAW: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFRAW->automaticFlags.flags.all = 0;
  packetUBXESFRAW->callbackPointerPtr = nullptr;
  packetUBXESFRAW->callbackData = nullptr;
  return (true);
}

// Log this data in file buffer
void DevUBLOXGNSS::logESFRAW(bool enabled)
{
  if (packetUBXESFRAW == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFRAW->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}
#endif

#ifndef SFE_UBLOX_DISABLE_HNR
// ***** HNR ATT automatic support

bool DevUBLOXGNSS::getHNRAtt(uint16_t maxWait)
{
  return (getHNRATT(maxWait));
}

// Get the HNR Attitude data
//  Returns true if the get HNR attitude is successful. Data is returned in hnrAtt
//  Note: if hnrAttQueried is true, it gets set to false by this function since we assume
//        that the user will read hnrAtt immediately after this. I.e. this function will
//        only return true _once_ after each auto HNR Att is processed
bool DevUBLOXGNSS::getHNRATT(uint16_t maxWait)
{
  if (packetUBXHNRATT == nullptr)
    initPacketUBXHNRATT();        // Check that RAM has been allocated for the data
  if (packetUBXHNRATT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXHNRATT->automaticFlags.flags.bits.automatic && packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXHNRATT->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXHNRATT->automaticFlags.flags.bits.automatic && !packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting HNR attitude so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_HNR;
    packetCfg.id = UBX_HNR_ATT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic HNR attitude message generation by the GNSS. This changes the way getHNRAtt
// works.
bool DevUBLOXGNSS::setAutoHNRATT(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoHNRATTrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic HNR attitude message generation by the GNSS. This changes the way getHNRAtt
// works.
bool DevUBLOXGNSS::setAutoHNRATT(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoHNRATTrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic HNR attitude message generation by the GNSS. This changes the way getHNRAtt
// works.
bool DevUBLOXGNSS::setAutoHNRATTrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXHNRATT == nullptr)
    initPacketUBXHNRATT();        // Check that RAM has been allocated for the data
  if (packetUBXHNRATT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  // Note: HNR is only supported on the NEO-M8U - which does not currently support the configuration interface
  (void)layer; // Placeholder - until the configuration interface is implemented on the NEO-M8U

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_HNR;
  payloadCfg[1] = UBX_HNR_ATT;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXHNRATT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoHNRATTcallbackPtr(void (*callbackPointerPtr)(UBX_HNR_ATT_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHNRATT(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXHNRATT->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXHNRATT->callbackData = new UBX_HNR_ATT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXHNRATT->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoHNRAttcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXHNRATT->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HNR attitude is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoHNRATT(bool enabled, bool implicitUpdate)
{
  if (packetUBXHNRATT == nullptr)
    initPacketUBXHNRATT();        // Check that RAM has been allocated for the data
  if (packetUBXHNRATT == nullptr) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXHNRATT->automaticFlags.flags.bits.automatic != enabled || packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXHNRATT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXHNRATT and initialize it
bool DevUBLOXGNSS::initPacketUBXHNRATT()
{
  packetUBXHNRATT = new UBX_HNR_ATT_t; // Allocate RAM for the main struct
  if (packetUBXHNRATT == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXHNRATT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXHNRATT->automaticFlags.flags.all = 0;
  packetUBXHNRATT->callbackPointerPtr = nullptr;
  packetUBXHNRATT->callbackData = nullptr;
  packetUBXHNRATT->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushHNRATT()
{
  if (packetUBXHNRATT == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRATT->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logHNRATT(bool enabled)
{
  if (packetUBXHNRATT == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRATT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** HNR DYN automatic support

bool DevUBLOXGNSS::getHNRDyn(uint16_t maxWait)
{
  return (getHNRINS(maxWait));
}

// Get the HNR vehicle dynamics data
//  Returns true if the get HNR vehicle dynamics is successful. Data is returned in hnrVehDyn
//  Note: if hnrDynQueried is true, it gets set to false by this function since we assume
//        that the user will read hnrVehDyn immediately after this. I.e. this function will
//        only return true _once_ after each auto HNR Dyn is processed
bool DevUBLOXGNSS::getHNRINS(uint16_t maxWait)
{
  if (packetUBXHNRINS == nullptr)
    initPacketUBXHNRINS();        // Check that RAM has been allocated for the data
  if (packetUBXHNRINS == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXHNRINS->automaticFlags.flags.bits.automatic && packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXHNRINS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXHNRINS->automaticFlags.flags.bits.automatic && !packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting HNR vehicle dynamics so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_HNR;
    packetCfg.id = UBX_HNR_INS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic HNR vehicle dynamics message generation by the GNSS. This changes the way getHNRINS
// works.
bool DevUBLOXGNSS::setAutoHNRINS(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoHNRINSrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic HNR vehicle dynamics message generation by the GNSS. This changes the way getHNRINS
// works.
bool DevUBLOXGNSS::setAutoHNRINS(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoHNRINSrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic HNR vehicle dynamics message generation by the GNSS. This changes the way getHNRINS
// works.
bool DevUBLOXGNSS::setAutoHNRINSrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXHNRINS == nullptr)
    initPacketUBXHNRINS();        // Check that RAM has been allocated for the data
  if (packetUBXHNRINS == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  // Note: HNR is only supported on the NEO-M8U - which does not currently support the configuration interface
  (void)layer; // Placeholder - until the configuration interface is implemented on the NEO-M8U

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_HNR;
  payloadCfg[1] = UBX_HNR_INS;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXHNRINS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXHNRINS->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoHNRINScallbackPtr(void (*callbackPointerPtr)(UBX_HNR_INS_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHNRINS(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXHNRINS->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXHNRINS->callbackData = new UBX_HNR_INS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXHNRINS->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoHNRINScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXHNRINS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HNR vehicle dynamics is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoHNRINS(bool enabled, bool implicitUpdate)
{
  if (packetUBXHNRINS == nullptr)
    initPacketUBXHNRINS();        // Check that RAM has been allocated for the data
  if (packetUBXHNRINS == nullptr) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXHNRINS->automaticFlags.flags.bits.automatic != enabled || packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXHNRINS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXHNRINS and initialize it
bool DevUBLOXGNSS::initPacketUBXHNRINS()
{
  packetUBXHNRINS = new UBX_HNR_INS_t; // Allocate RAM for the main struct
  if (packetUBXHNRINS == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXHNRINS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXHNRINS->automaticFlags.flags.all = 0;
  packetUBXHNRINS->callbackPointerPtr = nullptr;
  packetUBXHNRINS->callbackData = nullptr;
  packetUBXHNRINS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushHNRINS()
{
  if (packetUBXHNRINS == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRINS->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logHNRINS(bool enabled)
{
  if (packetUBXHNRINS == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRINS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** HNR PVT automatic support

// Get the HNR PVT data
//  Returns true if the get HNR PVT is successful. Data is returned in hnrPVT
//  Note: if hnrPVTQueried is true, it gets set to false by this function since we assume
//        that the user will read hnrPVT immediately after this. I.e. this function will
//        only return true _once_ after each auto HNR PVT is processed
bool DevUBLOXGNSS::getHNRPVT(uint16_t maxWait)
{
  if (packetUBXHNRPVT == nullptr)
    initPacketUBXHNRPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXHNRPVT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXHNRPVT->automaticFlags.flags.bits.automatic && packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXHNRPVT->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXHNRPVT->automaticFlags.flags.bits.automatic && !packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_HNR;
    packetCfg.id = UBX_HNR_PVT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic HNR PVT message generation by the GNSS. This changes the way getHNRPVT
// works.
bool DevUBLOXGNSS::setAutoHNRPVT(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoHNRPVTrate(enable ? 1 : 0, true, layer, maxWait);
}

// Enable or disable automatic HNR PVT message generation by the GNSS. This changes the way getHNRPVT
// works.
bool DevUBLOXGNSS::setAutoHNRPVT(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoHNRPVTrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}

// Enable or disable automatic HNR PVT message generation by the GNSS. This changes the way getHNRPVT
// works.
bool DevUBLOXGNSS::setAutoHNRPVTrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXHNRPVT == nullptr)
    initPacketUBXHNRPVT();        // Check that RAM has been allocated for the data
  if (packetUBXHNRPVT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  // Note: HNR is only supported on the NEO-M8U - which does not currently support the configuration interface
  (void)layer; // Placeholder - until the configuration interface is implemented on the NEO-M8U

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_HNR;
  payloadCfg[1] = UBX_HNR_PVT;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXHNRPVT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXHNRPVT->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool DevUBLOXGNSS::setAutoHNRPVTcallbackPtr(void (*callbackPointerPtr)(UBX_HNR_PVT_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHNRPVT(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXHNRPVT->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXHNRPVT->callbackData = new UBX_HNR_PVT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXHNRPVT->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setAutoHNRPVTcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXHNRPVT->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HNR PVT is send cyclically already
// set config to suitable parameters
bool DevUBLOXGNSS::assumeAutoHNRPVT(bool enabled, bool implicitUpdate)
{
  if (packetUBXHNRPVT == nullptr)
    initPacketUBXHNRPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXHNRPVT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXHNRPVT->automaticFlags.flags.bits.automatic != enabled || packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXHNRPVT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXHNRPVT and initialize it
bool DevUBLOXGNSS::initPacketUBXHNRPVT()
{
  packetUBXHNRPVT = new UBX_HNR_PVT_t; // Allocate RAM for the main struct
  if (packetUBXHNRPVT == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXHNRPVT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXHNRPVT->automaticFlags.flags.all = 0;
  packetUBXHNRPVT->callbackPointerPtr = nullptr;
  packetUBXHNRPVT->callbackData = nullptr;
  packetUBXHNRPVT->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void DevUBLOXGNSS::flushHNRPVT()
{
  if (packetUBXHNRPVT == nullptr)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRPVT->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void DevUBLOXGNSS::logHNRPVT(bool enabled)
{
  if (packetUBXHNRPVT == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRPVT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}
#endif

// ***** Helper Functions for NMEA Logging / Processing

// Set the mainTalkerId used by NMEA messages - allows all NMEA messages except GSV to be prefixed with GP instead of GN
bool DevUBLOXGNSS::setMainTalkerID(sfe_ublox_talker_ids_e id, uint8_t layer, uint16_t maxWait)
{
  return (setVal8(UBLOX_CFG_NMEA_MAINTALKERID, (uint8_t)id, layer, maxWait));
}

// Enable/Disable NMEA High Precision Mode - include extra decimal places in the Lat and Lon
bool DevUBLOXGNSS::setHighPrecisionMode(bool enable, uint8_t layer, uint16_t maxWait)
{
  return (setVal8(UBLOX_CFG_NMEA_HIGHPREC, (uint8_t)enable, layer, maxWait));
}

// Log selected NMEA messages to file buffer - if the messages are enabled and if the file buffer exists
// User needs to call setFileBufferSize before .begin
void DevUBLOXGNSS::setNMEALoggingMask(uint32_t messages)
{
  _logNMEA.all = messages;
}
uint32_t DevUBLOXGNSS::getNMEALoggingMask()
{
  return (_logNMEA.all);
}

// Pass selected NMEA messages to processNMEA
void DevUBLOXGNSS::setProcessNMEAMask(uint32_t messages)
{
  _processNMEA.all = messages;
}
uint32_t DevUBLOXGNSS::getProcessNMEAMask()
{
  return (_processNMEA.all);
}

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
// Initiate automatic storage of NMEA GPGGA messages

// Get the most recent GPGGA message
// Return 0 if the message has not been received from the module
// Return 1 if the data is valid but has been read before
// Return 2 if the data is valid and is fresh/unread
uint8_t DevUBLOXGNSS::getLatestNMEAGPGGA(NMEA_GGA_data_t *data)
{
  if (storageNMEAGPGGA == nullptr)
    initStorageNMEAGPGGA();        // Check that RAM has been allocated for the message
  if (storageNMEAGPGGA == nullptr) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID

  memcpy(data, &storageNMEAGPGGA->completeCopy, sizeof(NMEA_GGA_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGPGGA->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGPGGA->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGPGGA->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

// Enable a callback on the arrival of a GPGGA message
bool DevUBLOXGNSS::setNMEAGPGGAcallbackPtr(void (*callbackPointerPtr)(NMEA_GGA_data_t *))
{
  if (storageNMEAGPGGA == nullptr)
    initStorageNMEAGPGGA();        // Check that RAM has been allocated for the message
  if (storageNMEAGPGGA == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPGGA->callbackCopy == nullptr) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPGGA->callbackCopy = new NMEA_GGA_data_t;
  }

  if (storageNMEAGPGGA->callbackCopy == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setNMEAGPGGAcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPGGA->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GPGGA messages and initialize it
bool DevUBLOXGNSS::initStorageNMEAGPGGA()
{
  storageNMEAGPGGA = new NMEA_GPGGA_t; // Allocate RAM for the main struct
  if (storageNMEAGPGGA == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEAGPGGA: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPGGA->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGPGGA->workingCopy.nmea, 0, NMEA_GGA_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGPGGA->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGPGGA->completeCopy.nmea, 0, NMEA_GGA_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGPGGA->callbackPointerPtr = nullptr; // Clear the callback pointers
  storageNMEAGPGGA->callbackCopy = nullptr;

  storageNMEAGPGGA->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

uint8_t DevUBLOXGNSS::getLatestNMEAGNGGA(NMEA_GGA_data_t *data)
{
  if (storageNMEAGNGGA == nullptr)
    initStorageNMEAGNGGA();        // Check that RAM has been allocated for the message
  if (storageNMEAGNGGA == nullptr) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID

  memcpy(data, &storageNMEAGNGGA->completeCopy, sizeof(NMEA_GGA_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGNGGA->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGNGGA->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGNGGA->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

bool DevUBLOXGNSS::setNMEAGNGGAcallbackPtr(void (*callbackPointerPtr)(NMEA_GGA_data_t *))
{
  if (storageNMEAGNGGA == nullptr)
    initStorageNMEAGNGGA();        // Check that RAM has been allocated for the message
  if (storageNMEAGNGGA == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNGGA->callbackCopy == nullptr) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNGGA->callbackCopy = new NMEA_GGA_data_t;
  }

  if (storageNMEAGNGGA->callbackCopy == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setNMEAGNGGAcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNGGA->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GNGGA messages and initialize it
bool DevUBLOXGNSS::initStorageNMEAGNGGA()
{
  storageNMEAGNGGA = new NMEA_GNGGA_t; // Allocate RAM for the main struct
  if (storageNMEAGNGGA == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEAGNGGA: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNGGA->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGNGGA->workingCopy.nmea, 0, NMEA_GGA_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGNGGA->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGNGGA->completeCopy.nmea, 0, NMEA_GGA_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGNGGA->callbackPointerPtr = nullptr; // Clear the callback pointers
  storageNMEAGNGGA->callbackCopy = nullptr;

  storageNMEAGNGGA->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

// Initiate automatic storage of NMEA GPVTG messages

// Get the most recent GPVTG message
// Return 0 if the message has not been received from the module
// Return 1 if the data is valid but has been read before
// Return 2 if the data is valid and is fresh/unread
uint8_t DevUBLOXGNSS::getLatestNMEAGPVTG(NMEA_VTG_data_t *data)
{
  if (storageNMEAGPVTG == nullptr)
    initStorageNMEAGPVTG();        // Check that RAM has been allocated for the message
  if (storageNMEAGPVTG == nullptr) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID

  memcpy(data, &storageNMEAGPVTG->completeCopy, sizeof(NMEA_VTG_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGPVTG->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGPVTG->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGPVTG->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

// Enable a callback on the arrival of a GPVTG message
bool DevUBLOXGNSS::setNMEAGPVTGcallbackPtr(void (*callbackPointerPtr)(NMEA_VTG_data_t *))
{
  if (storageNMEAGPVTG == nullptr)
    initStorageNMEAGPVTG();        // Check that RAM has been allocated for the message
  if (storageNMEAGPVTG == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPVTG->callbackCopy == nullptr) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPVTG->callbackCopy = new NMEA_VTG_data_t;
  }

  if (storageNMEAGPVTG->callbackCopy == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setNMEAGPVTGcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPVTG->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GPVTG messages and initialize it
bool DevUBLOXGNSS::initStorageNMEAGPVTG()
{
  storageNMEAGPVTG = new NMEA_GPVTG_t; // Allocate RAM for the main struct
  if (storageNMEAGPVTG == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEAGPVTG: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPVTG->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGPVTG->workingCopy.nmea, 0, NMEA_VTG_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGPVTG->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGPVTG->completeCopy.nmea, 0, NMEA_VTG_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGPVTG->callbackPointerPtr = nullptr; // Clear the callback pointers
  storageNMEAGPVTG->callbackCopy = nullptr;

  storageNMEAGPVTG->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

uint8_t DevUBLOXGNSS::getLatestNMEAGNVTG(NMEA_VTG_data_t *data)
{
  if (storageNMEAGNVTG == nullptr)
    initStorageNMEAGNVTG();        // Check that RAM has been allocated for the message
  if (storageNMEAGNVTG == nullptr) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID

  memcpy(data, &storageNMEAGNVTG->completeCopy, sizeof(NMEA_VTG_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGNVTG->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGNVTG->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGNVTG->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

bool DevUBLOXGNSS::setNMEAGNVTGcallbackPtr(void (*callbackPointerPtr)(NMEA_VTG_data_t *))
{
  if (storageNMEAGNVTG == nullptr)
    initStorageNMEAGNVTG();        // Check that RAM has been allocated for the message
  if (storageNMEAGNVTG == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNVTG->callbackCopy == nullptr) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNVTG->callbackCopy = new NMEA_VTG_data_t;
  }

  if (storageNMEAGNVTG->callbackCopy == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setNMEAGNVTGcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNVTG->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GNVTG messages and initialize it
bool DevUBLOXGNSS::initStorageNMEAGNVTG()
{
  storageNMEAGNVTG = new NMEA_GNVTG_t; // Allocate RAM for the main struct
  if (storageNMEAGNVTG == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEAGNVTG: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNVTG->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGNVTG->workingCopy.nmea, 0, NMEA_VTG_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGNVTG->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGNVTG->completeCopy.nmea, 0, NMEA_VTG_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGNVTG->callbackPointerPtr = nullptr; // Clear the callback pointers
  storageNMEAGNVTG->callbackCopy = nullptr;

  storageNMEAGNVTG->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

// Initiate automatic storage of NMEA GPRMC messages

// Get the most recent GPRMC message
// Return 0 if the message has not been received from the module
// Return 1 if the data is valid but has been read before
// Return 2 if the data is valid and is fresh/unread
uint8_t DevUBLOXGNSS::getLatestNMEAGPRMC(NMEA_RMC_data_t *data)
{
  if (storageNMEAGPRMC == nullptr)
    initStorageNMEAGPRMC();        // Check that RAM has been allocated for the message
  if (storageNMEAGPRMC == nullptr) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID

  memcpy(data, &storageNMEAGPRMC->completeCopy, sizeof(NMEA_RMC_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGPRMC->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGPRMC->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGPRMC->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

// Enable a callback on the arrival of a GPRMC message
bool DevUBLOXGNSS::setNMEAGPRMCcallbackPtr(void (*callbackPointerPtr)(NMEA_RMC_data_t *))
{
  if (storageNMEAGPRMC == nullptr)
    initStorageNMEAGPRMC();        // Check that RAM has been allocated for the message
  if (storageNMEAGPRMC == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPRMC->callbackCopy == nullptr) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPRMC->callbackCopy = new NMEA_RMC_data_t;
  }

  if (storageNMEAGPRMC->callbackCopy == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setNMEAGPRMCcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPRMC->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GPRMC messages and initialize it
bool DevUBLOXGNSS::initStorageNMEAGPRMC()
{
  storageNMEAGPRMC = new NMEA_GPRMC_t; // Allocate RAM for the main struct
  if (storageNMEAGPRMC == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEAGPRMC: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPRMC->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGPRMC->workingCopy.nmea, 0, NMEA_RMC_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGPRMC->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGPRMC->completeCopy.nmea, 0, NMEA_RMC_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGPRMC->callbackPointerPtr = nullptr; // Clear the callback pointers
  storageNMEAGPRMC->callbackCopy = nullptr;

  storageNMEAGPRMC->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

uint8_t DevUBLOXGNSS::getLatestNMEAGNRMC(NMEA_RMC_data_t *data)
{
  if (storageNMEAGNRMC == nullptr)
    initStorageNMEAGNRMC();        // Check that RAM has been allocated for the message
  if (storageNMEAGNRMC == nullptr) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID

  memcpy(data, &storageNMEAGNRMC->completeCopy, sizeof(NMEA_RMC_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGNRMC->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGNRMC->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGNRMC->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

bool DevUBLOXGNSS::setNMEAGNRMCcallbackPtr(void (*callbackPointerPtr)(NMEA_RMC_data_t *))
{
  if (storageNMEAGNRMC == nullptr)
    initStorageNMEAGNRMC();        // Check that RAM has been allocated for the message
  if (storageNMEAGNRMC == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNRMC->callbackCopy == nullptr) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNRMC->callbackCopy = new NMEA_RMC_data_t;
  }

  if (storageNMEAGNRMC->callbackCopy == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setNMEAGNRMCcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNRMC->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GNRMC messages and initialize it
bool DevUBLOXGNSS::initStorageNMEAGNRMC()
{
  storageNMEAGNRMC = new NMEA_GNRMC_t; // Allocate RAM for the main struct
  if (storageNMEAGNRMC == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEAGNRMC: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNRMC->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGNRMC->workingCopy.nmea, 0, NMEA_RMC_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGNRMC->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGNRMC->completeCopy.nmea, 0, NMEA_RMC_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGNRMC->callbackPointerPtr = nullptr; // Clear the callback pointers
  storageNMEAGNRMC->callbackCopy = nullptr;

  storageNMEAGNRMC->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

// Initiate automatic storage of NMEA GPZDA messages

// Get the most recent GPZDA message
// Return 0 if the message has not been received from the module
// Return 1 if the data is valid but has been read before
// Return 2 if the data is valid and is fresh/unread
uint8_t DevUBLOXGNSS::getLatestNMEAGPZDA(NMEA_ZDA_data_t *data)
{
  if (storageNMEAGPZDA == nullptr)
    initStorageNMEAGPZDA();        // Check that RAM has been allocated for the message
  if (storageNMEAGPZDA == nullptr) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID

  memcpy(data, &storageNMEAGPZDA->completeCopy, sizeof(NMEA_ZDA_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGPZDA->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGPZDA->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGPZDA->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

// Enable a callback on the arrival of a GPZDA message
bool DevUBLOXGNSS::setNMEAGPZDAcallbackPtr(void (*callbackPointerPtr)(NMEA_ZDA_data_t *))
{
  if (storageNMEAGPZDA == nullptr)
    initStorageNMEAGPZDA();        // Check that RAM has been allocated for the message
  if (storageNMEAGPZDA == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPZDA->callbackCopy == nullptr) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPZDA->callbackCopy = new NMEA_ZDA_data_t;
  }

  if (storageNMEAGPZDA->callbackCopy == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setNMEAGPZDAcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPZDA->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GPZDA messages and initialize it
bool DevUBLOXGNSS::initStorageNMEAGPZDA()
{
  storageNMEAGPZDA = new NMEA_GPZDA_t; // Allocate RAM for the main struct
  if (storageNMEAGPZDA == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEAGPZDA: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPZDA->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGPZDA->workingCopy.nmea, 0, NMEA_ZDA_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGPZDA->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGPZDA->completeCopy.nmea, 0, NMEA_ZDA_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGPZDA->callbackPointerPtr = nullptr; // Clear the callback pointers
  storageNMEAGPZDA->callbackCopy = nullptr;

  storageNMEAGPZDA->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

uint8_t DevUBLOXGNSS::getLatestNMEAGNZDA(NMEA_ZDA_data_t *data)
{
  if (storageNMEAGNZDA == nullptr)
    initStorageNMEAGNZDA();        // Check that RAM has been allocated for the message
  if (storageNMEAGNZDA == nullptr) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID

  memcpy(data, &storageNMEAGNZDA->completeCopy, sizeof(NMEA_ZDA_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGNZDA->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGNZDA->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGNZDA->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

bool DevUBLOXGNSS::setNMEAGNZDAcallbackPtr(void (*callbackPointerPtr)(NMEA_ZDA_data_t *))
{
  if (storageNMEAGNZDA == nullptr)
    initStorageNMEAGNZDA();        // Check that RAM has been allocated for the message
  if (storageNMEAGNZDA == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNZDA->callbackCopy == nullptr) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNZDA->callbackCopy = new NMEA_ZDA_data_t;
  }

  if (storageNMEAGNZDA->callbackCopy == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setNMEAGNZDAcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNZDA->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GNZDA messages and initialize it
bool DevUBLOXGNSS::initStorageNMEAGNZDA()
{
  storageNMEAGNZDA = new NMEA_GNZDA_t; // Allocate RAM for the main struct
  if (storageNMEAGNZDA == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEAGNZDA: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNZDA->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGNZDA->workingCopy.nmea, 0, NMEA_ZDA_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGNZDA->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGNZDA->completeCopy.nmea, 0, NMEA_ZDA_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGNZDA->callbackPointerPtr = nullptr; // Clear the callback pointers
  storageNMEAGNZDA->callbackCopy = nullptr;

  storageNMEAGNZDA->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}
#endif

// Private: allocate RAM for incoming non-Auto NMEA messages and initialize it
bool DevUBLOXGNSS::initStorageNMEA()
{
  if (_storageNMEA != nullptr) // Check if storage already exists
    return true;

  _storageNMEA = new NMEA_STORAGE_t; // Allocate RAM for the main struct
  if (_storageNMEA == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEA: RAM alloc failed!"));
#endif
    return (false);
  }
  _storageNMEA->data = nullptr;

  _storageNMEA->data = new uint8_t[maxNMEAByteCount];
  if (_storageNMEA->data == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageNMEA: RAM alloc failed!"));
#endif
    return (false);
  }

  return (true);
}

// ***** RTCM Auto Support

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING

// Log selected RTCM messages to file buffer - if the messages are enabled and if the file buffer exists
// User needs to call setFileBufferSize before .begin
bool DevUBLOXGNSS::setRTCMLoggingMask(uint32_t messages)
{
  _logRTCM.all = messages;
  return (initStorageRTCM());
}
uint32_t DevUBLOXGNSS::getRTCMLoggingMask()
{
  return (_logRTCM.all);
}

// Private: allocate RAM for incoming RTCM messages and initialize it
bool DevUBLOXGNSS::initStorageRTCM()
{
  if (_storageRTCM != nullptr) // Check if storage already exists
    return true;

  _storageRTCM = new RTCM_FRAME_t; // Allocate RAM for the main struct
  if (_storageRTCM == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageRTCM: RAM alloc failed!"));
#endif
    return (false);
  }

  return (true);
}

void DevUBLOXGNSS::crc24q(uint8_t incoming, uint32_t *checksum)
{
  uint32_t crc = *checksum; // Seed is 0

  crc ^= ((uint32_t)incoming) << 16; // XOR-in incoming

  for (uint8_t i = 0; i < 8; i++)
  {
    crc <<= 1;
    if (crc & 0x1000000)
      // CRC-24Q Polynomial:
      // gi = 1 for i = 0, 1, 3, 4, 5, 6, 7, 10, 11, 14, 17, 18, 23, 24
      // 0b 1 1000 0110 0100 1100 1111 1011
      crc ^= 0x1864CFB; // CRC-24Q
  }

  *checksum = crc & 0xFFFFFF;
}

// Return the most recent RTCM 1005: 0 = no data, 1 = stale data, 2 = fresh data
uint8_t DevUBLOXGNSS::getLatestRTCM1005(RTCM_1005_data_t *data)
{
  if (!initStorageRTCM())
    return 0;
  if (!initStorageRTCM1005())
    return 0;

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID

  memcpy(data, &storageRTCM1005->data, sizeof(RTCM_1005_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageRTCM1005->automaticFlags.flags.bits.dataValid == 1) // Is the copy valid?
  {
    result = 1;
    if (storageRTCM1005->automaticFlags.flags.bits.dataRead == 0) // Has the data already been read?
    {
      result = 2;
      storageRTCM1005->automaticFlags.flags.bits.dataRead = 1; // Mark the data as read
    }
  }

  return (result);
}

bool DevUBLOXGNSS::setRTCM1005callbackPtr(void (*callbackPointerPtr)(RTCM_1005_data_t *))
{
  if (!initStorageRTCM())
    return false;
  if (!initStorageRTCM1005())
    return false;

  if (storageRTCM1005->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    storageRTCM1005->callbackData = new RTCM_1005_data_t;
  }

  if (storageRTCM1005->callbackData == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("setRTCM1005callbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageRTCM1005->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming RTCM 1005 messages and initialize it
bool DevUBLOXGNSS::initStorageRTCM1005()
{
  if (storageRTCM1005 != nullptr) // Check if storage already exists
    return true;

  storageRTCM1005 = new RTCM_1005_t; // Allocate RAM for the main struct
  if (storageRTCM1005 == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initStorageRTCM1005: RAM alloc failed!"));
#endif
    return (false);
  }

  storageRTCM1005->callbackPointerPtr = nullptr; // Clear the callback pointers
  storageRTCM1005->callbackData = nullptr;

  storageRTCM1005->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

// Return the most recent RTCM 1005 Input - from pushRawData: 0 = no data, 1 = stale data, 2 = fresh data
uint8_t DevUBLOXGNSS::getLatestRTCM1005Input(RTCM_1005_data_t *data)
{
  memcpy(data, &rtcmInputStorage.rtcm1005, sizeof(RTCM_1005_data_t)); // Copy the complete message

  uint8_t result = 0;
  if (rtcmInputStorage.flags.bits.dataValid1005 == 1) // Is the data valid?
  {
    result = 1;
    if (rtcmInputStorage.flags.bits.dataRead1005 == 0) // Has the data already been read?
    {
      result = 2;
      rtcmInputStorage.flags.bits.dataRead1005 = 1; // Mark the data as read
    }
  }

  return result;
}

// Return the most recent RTCM 1006 Input - from pushRawData: 0 = no data, 1 = stale data, 2 = fresh data
uint8_t DevUBLOXGNSS::getLatestRTCM1006Input(RTCM_1006_data_t *data)
{
  memcpy(data, &rtcmInputStorage.rtcm1006, sizeof(RTCM_1006_data_t)); // Copy the complete message

  uint8_t result = 0;
  if (rtcmInputStorage.flags.bits.dataValid1006 == 1) // Is the data valid?
  {
    result = 1;
    if (rtcmInputStorage.flags.bits.dataRead1006 == 0) // Has the data already been read?
    {
      result = 2;
      rtcmInputStorage.flags.bits.dataRead1006 = 1; // Mark the data as read
    }
  }

  return result;
}

// Configure a callback for RTCM 1005 Input - from pushRawData
void DevUBLOXGNSS::setRTCM1005InputcallbackPtr(void (*rtcm1005CallbackPointer)(RTCM_1005_data_t *))
{
  rtcmInputStorage.rtcm1005CallbackPointer = rtcm1005CallbackPointer;
}

// Configure a callback for RTCM 1006 Input - from pushRawData
void DevUBLOXGNSS::setRTCM1006InputcallbackPtr(void (*rtcm1006CallbackPointer)(RTCM_1006_data_t *))
{
  rtcmInputStorage.rtcm1006CallbackPointer = rtcm1006CallbackPointer;
}

#endif

// ***** CFG RATE Helper Functions

// Set the rate at which the module will give us an updated navigation solution
// Expects a number that is the updates per second. For example 1 = 1Hz, 2 = 2Hz, etc.
// Max is 40Hz(?!)
bool DevUBLOXGNSS::setNavigationFrequency(uint8_t navFreq, uint8_t layer, uint16_t maxWait)
{
  if (navFreq == 0) // Return now if navFreq is zero
    return (false);

  if (navFreq > 40)
    navFreq = 40; // Limit navFreq to 40Hz so i2cPollingWait is set correctly

  // Adjust the I2C polling timeout based on update rate
  // Do this even if the sendCommand fails
  i2cPollingWaitNAV = 1000 / (((int)navFreq) * 4);                                                // This is the number of ms to wait between checks for new I2C data. Max is 250. Min is 6.
  i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; // Set i2cPollingWait to the lower of NAV and HNR

  uint16_t measurementRate = 1000 / navFreq;

  return setVal16(UBLOX_CFG_RATE_MEAS, measurementRate, layer, maxWait);
}

// Get the rate at which the module is outputting nav solutions
// Note: if the measurementRate (which is actually a period) is less than 1000ms, this will return zero
bool DevUBLOXGNSS::getNavigationFrequency(uint8_t *navFreq, uint8_t layer, uint16_t maxWait)
{
  uint16_t measurementRate = 0;

  bool result = getVal16(UBLOX_CFG_RATE_MEAS, &measurementRate, layer, maxWait);

  if ((result) && (measurementRate > 0))
    *navFreq = 1000 / measurementRate; // This may return an int when it's a float, but I'd rather not return 4 bytes

  return result;
}
uint8_t DevUBLOXGNSS::getNavigationFrequency(uint8_t layer, uint16_t maxWait) // Unsafe overload...
{
  uint8_t navFreq = 0;

  getNavigationFrequency(&navFreq, layer, maxWait);

  return navFreq;
}

// Set the elapsed time between GNSS measurements in milliseconds, which defines the rate
bool DevUBLOXGNSS::setMeasurementRate(uint16_t rate, uint8_t layer, uint16_t maxWait)
{
  if (rate < 25) // "Measurement rate should be greater than or equal to 25 ms."
    rate = 25;

  // Adjust the I2C polling timeout based on update rate
  if (rate >= 1000)
    i2cPollingWaitNAV = 250;
  else
    i2cPollingWaitNAV = rate / 4;                                                                 // This is the number of ms to wait between checks for new I2C data
  i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; // Set i2cPollingWait to the lower of NAV and HNR

  return setVal16(UBLOX_CFG_RATE_MEAS, rate, layer, maxWait);
}

// Return the elapsed time between GNSS measurements in milliseconds, which defines the rate
bool DevUBLOXGNSS::getMeasurementRate(uint16_t *measRate, uint8_t layer, uint16_t maxWait)
{
  uint16_t measurementRate;

  bool result = getVal16(UBLOX_CFG_RATE_MEAS, &measurementRate, layer, maxWait);

  if (result)
    *measRate = measurementRate;

  return result;
}
uint16_t DevUBLOXGNSS::getMeasurementRate(uint8_t layer, uint16_t maxWait) // Unsafe overload...
{
  uint16_t measurementRate;

  getMeasurementRate(&measurementRate, layer, maxWait);

  return measurementRate;
}

// Set the ratio between the number of measurements and the number of navigation solutions. Unit is cycles. Max is 127.
bool DevUBLOXGNSS::setNavigationRate(uint16_t rate, uint8_t layer, uint16_t maxWait)
{
  return setVal16(UBLOX_CFG_RATE_NAV, rate, layer, maxWait);
}

// Return the ratio between the number of measurements and the number of navigation solutions. Unit is cycles
bool DevUBLOXGNSS::getNavigationRate(uint16_t *navRate, uint8_t layer, uint16_t maxWait)
{
  uint16_t navigationRate;

  bool result = getVal16(UBLOX_CFG_RATE_NAV, &navigationRate, layer, maxWait);

  if (result)
    *navRate = navigationRate;

  return result;
}
uint16_t DevUBLOXGNSS::getNavigationRate(uint8_t layer, uint16_t maxWait) // Unsafe overload...
{
  uint16_t navigationRate;

  getNavigationRate(&navigationRate, layer, maxWait);

  return navigationRate;
}

// ***** DOP Helper Functions

uint16_t DevUBLOXGNSS::getGeometricDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.gDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.gDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.gDOP);
}

uint16_t DevUBLOXGNSS::getPositionDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.pDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.pDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.pDOP);
}

uint16_t DevUBLOXGNSS::getTimeDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.tDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.tDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.tDOP);
}

uint16_t DevUBLOXGNSS::getVerticalDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.vDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.vDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.vDOP);
}

uint16_t DevUBLOXGNSS::getHorizontalDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.hDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.hDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.hDOP);
}

uint16_t DevUBLOXGNSS::getNorthingDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.nDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.nDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.nDOP);
}

uint16_t DevUBLOXGNSS::getEastingDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == nullptr)
    initPacketUBXNAVDOP();        // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.eDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.eDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.eDOP);
}

// ***** ATT Helper Functions

float DevUBLOXGNSS::getATTroll(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXNAVATT == nullptr)
    initPacketUBXNAVATT();        // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == nullptr) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXNAVATT->moduleQueried.moduleQueried.bits.roll == false)
    getNAVATT(maxWait);
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.roll = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVATT->data.roll) / 100000.0); // Convert to degrees
}

float DevUBLOXGNSS::getATTpitch(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXNAVATT == nullptr)
    initPacketUBXNAVATT();        // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == nullptr) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXNAVATT->moduleQueried.moduleQueried.bits.pitch == false)
    getNAVATT(maxWait);
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.pitch = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVATT->data.pitch) / 100000.0); // Convert to degrees
}

float DevUBLOXGNSS::getATTheading(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXNAVATT == nullptr)
    initPacketUBXNAVATT();        // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == nullptr) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXNAVATT->moduleQueried.moduleQueried.bits.heading == false)
    getNAVATT(maxWait);
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.heading = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVATT->data.heading) / 100000.0); // Convert to degrees
}

// ***** PVT Helper Functions

uint32_t DevUBLOXGNSS::getTimeOfWeek(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.iTOW);
}

// Get the current year
uint16_t DevUBLOXGNSS::getYear(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.year);
}

// Get the current month
uint8_t DevUBLOXGNSS::getMonth(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.month);
}

// Get the current day
uint8_t DevUBLOXGNSS::getDay(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.day);
}

// Get the current hour
uint8_t DevUBLOXGNSS::getHour(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.hour);
}

// Get the current minute
uint8_t DevUBLOXGNSS::getMinute(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.min);
}

// Get the current second
uint8_t DevUBLOXGNSS::getSecond(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.sec);
}

// Get the current millisecond
uint16_t DevUBLOXGNSS::getMillisecond(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.iTOW % 1000);
}

// Get the current nanoseconds - includes milliseconds
int32_t DevUBLOXGNSS::getNanosecond(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.nano);
}

// Get the current Unix epoch time rounded to the nearest second
uint32_t DevUBLOXGNSS::getUnixEpoch(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  uint32_t t = SFE_UBLOX_DAYS_FROM_1970_TO_2020;                                                                           // Jan 1st 2020 as days from Jan 1st 1970
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_2020[packetUBXNAVPVT->data.year - 2020];                                             // Add on the number of days since 2020
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_MONTH[packetUBXNAVPVT->data.year % 4 == 0 ? 0 : 1][packetUBXNAVPVT->data.month - 1]; // Add on the number of days since Jan 1st
  t += (uint32_t)packetUBXNAVPVT->data.day - 1;                                                                            // Add on the number of days since the 1st of the month
  t *= 24;                                                                                                                 // Convert to hours
  t += (uint32_t)packetUBXNAVPVT->data.hour;                                                                               // Add on the hour
  t *= 60;                                                                                                                 // Convert to minutes
  t += (uint32_t)packetUBXNAVPVT->data.min;                                                                                // Add on the minute
  t *= 60;                                                                                                                 // Convert to seconds
  t += (uint32_t)packetUBXNAVPVT->data.sec;                                                                                // Add on the second
  return t;
}

// Get the current Unix epoch including microseconds
uint32_t DevUBLOXGNSS::getUnixEpoch(uint32_t &microsecond, uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  uint32_t t = SFE_UBLOX_DAYS_FROM_1970_TO_2020;                                                                           // Jan 1st 2020 as days from Jan 1st 1970
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_2020[packetUBXNAVPVT->data.year - 2020];                                             // Add on the number of days since 2020
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_MONTH[packetUBXNAVPVT->data.year % 4 == 0 ? 0 : 1][packetUBXNAVPVT->data.month - 1]; // Add on the number of days since Jan 1st
  t += (uint32_t)packetUBXNAVPVT->data.day - 1;                                                                            // Add on the number of days since the 1st of the month
  t *= 24;                                                                                                                 // Convert to hours
  t += (uint32_t)packetUBXNAVPVT->data.hour;                                                                               // Add on the hour
  t *= 60;                                                                                                                 // Convert to minutes
  t += (uint32_t)packetUBXNAVPVT->data.min;                                                                                // Add on the minute
  t *= 60;                                                                                                                 // Convert to seconds
  t += (uint32_t)packetUBXNAVPVT->data.sec;                                                                                // Add on the second
  int32_t us = packetUBXNAVPVT->data.nano / 1000;                                                                          // Convert nanos to micros
  microsecond = (uint32_t)us;                                                                                              // Could be -ve!
  // Adjust t if nano is negative
  if (us < 0)
  {
    microsecond = (uint32_t)(us + 1000000); // Make nano +ve
    t--;                                    // Decrement t by 1 second
  }
  return t;
}

// Get the current date validity
bool DevUBLOXGNSS::getDateValid(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validDate == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validDate = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.valid.bits.validDate);
}

// Get the current time validity
bool DevUBLOXGNSS::getTimeValid(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validTime == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validTime = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.valid.bits.validTime);
}

// Check to see if the UTC time has been fully resolved
bool DevUBLOXGNSS::getTimeFullyResolved(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fullyResolved == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fullyResolved = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.valid.bits.fullyResolved);
}

// Get the confirmed date validity
bool DevUBLOXGNSS::getConfirmedDate(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedDate == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedDate = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.flags2.bits.confirmedDate);
}

// Get the confirmed time validity
bool DevUBLOXGNSS::getConfirmedTime(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedTime == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedTime = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.flags2.bits.confirmedTime);
}

// Get the current fix type
// 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
uint8_t DevUBLOXGNSS::getFixType(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fixType == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fixType = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.fixType);
}

// Get whether we have a valid fix (i.e within DOP & accuracy masks)
bool DevUBLOXGNSS::getGnssFixOk(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.gnssFixOK == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.gnssFixOK = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.flags.bits.gnssFixOK);
}

// Get whether differential corrections were applied
bool DevUBLOXGNSS::getDiffSoln(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.diffSoln == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.diffSoln = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.flags.bits.diffSoln);
}

// Get whether head vehicle valid or not
bool DevUBLOXGNSS::getHeadVehValid(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.headVehValid == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.headVehValid = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.flags.bits.headVehValid);
}

// Get the carrier phase range solution status
// Useful when querying module to see if it has high-precision RTK fix
// 0=No solution, 1=Float solution, 2=Fixed solution
uint8_t DevUBLOXGNSS::getCarrierSolutionType(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.carrSoln == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.carrSoln = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.flags.bits.carrSoln);
}

// Get the number of satellites used in fix
uint8_t DevUBLOXGNSS::getSIV(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.numSV == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.numSV = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.numSV);
}

// Get the current longitude in degrees
// Returns a long representing the number of degrees *10^-7
int32_t DevUBLOXGNSS::getLongitude(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lon == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lon = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.lon);
}

// Get the current latitude in degrees
// Returns a long representing the number of degrees *10^-7
int32_t DevUBLOXGNSS::getLatitude(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lat == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lat = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.lat);
}

// Get the current altitude in mm according to ellipsoid model
int32_t DevUBLOXGNSS::getAltitude(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.height == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.height = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.height);
}

// Get the current altitude in mm according to mean sea level
// Ellipsoid model: https://www.esri.com/news/arcuser/0703/geoid1of3.html
// Difference between Ellipsoid Model and Mean Sea Level: https://eos-gnss.com/elevation-for-beginners/
// Also see: https://portal.u-blox.com/s/question/0D52p00008HKDSkCAP/what-geoid-model-is-used-and-where-is-this-calculated
// and: https://cddis.nasa.gov/926/egm96/egm96.html on 10x10 degree grid
int32_t DevUBLOXGNSS::getAltitudeMSL(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hMSL == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hMSL = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.hMSL);
}

int32_t DevUBLOXGNSS::getHorizontalAccEst(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.hAcc);
}

int32_t DevUBLOXGNSS::getVerticalAccEst(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.vAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.vAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.vAcc);
}

int32_t DevUBLOXGNSS::getNedNorthVel(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velN == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velN = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.velN);
}

int32_t DevUBLOXGNSS::getNedEastVel(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velE == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velE = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.velE);
}

int32_t DevUBLOXGNSS::getNedDownVel(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.velD == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.velD = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.velD);
}

// Get the ground speed in mm/s
int32_t DevUBLOXGNSS::getGroundSpeed(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.gSpeed == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.gSpeed = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.gSpeed);
}

// Get the heading of motion (as opposed to heading of car) in degrees * 10^-5
int32_t DevUBLOXGNSS::getHeading(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headMot == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headMot = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.headMot);
}

uint32_t DevUBLOXGNSS::getSpeedAccEst(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.sAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.sAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.sAcc);
}

uint32_t DevUBLOXGNSS::getHeadingAccEst(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.headAcc);
}

// Get the positional dillution of precision * 10^-2 (dimensionless)
uint16_t DevUBLOXGNSS::getPDOP(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.pDOP == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.pDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.pDOP);
}

bool DevUBLOXGNSS::getInvalidLlh(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.invalidLlh == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.invalidLlh = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.flags3.bits.invalidLlh);
}

int32_t DevUBLOXGNSS::getHeadVeh(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headVeh == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headVeh = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.headVeh);
}

int16_t DevUBLOXGNSS::getMagDec(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magDec == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magDec = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.magDec);
}

uint16_t DevUBLOXGNSS::getMagAcc(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.magAcc);
}

// getGeoidSeparation is currently redundant. The geoid separation seems to only be provided in NMEA GGA and GNS messages.
int32_t DevUBLOXGNSS::getGeoidSeparation(uint16_t maxWait)
{
  (void)maxWait; // Do something with maxWait just to get rid of the pesky compiler warning

  return (0);
}

// ***** HPPOSECEF Helper Functions

// Get the current 3D high precision positional accuracy - a fun thing to watch
// Returns a long representing the 3D accuracy in millimeters
uint32_t DevUBLOXGNSS::getPositionAccuracy(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.pAcc == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.pAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  uint32_t tempAccuracy = packetUBXNAVHPPOSECEF->data.pAcc;

  if ((tempAccuracy % 10) >= 5)
    tempAccuracy += 5; // Round fraction of mm up to next mm if .5 or above
  tempAccuracy /= 10;  // Convert 0.1mm units to mm

  return (tempAccuracy);
}

// Get the current 3D high precision X coordinate
// Returns a long representing the coordinate in cm
int32_t DevUBLOXGNSS::getHighResECEFX(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefX == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefX = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefX);
}

// Get the current 3D high precision Y coordinate
// Returns a long representing the coordinate in cm
int32_t DevUBLOXGNSS::getHighResECEFY(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefY == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefY = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefY);
}

// Get the current 3D high precision Z coordinate
// Returns a long representing the coordinate in cm
int32_t DevUBLOXGNSS::getHighResECEFZ(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZ == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZ = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefZ);
}

// Get the high precision component of the ECEF X coordinate
// Returns a signed byte representing the component as 0.1*mm
int8_t DevUBLOXGNSS::getHighResECEFXHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefXHp == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefXHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefXHp);
}

// Get the high precision component of the ECEF Y coordinate
// Returns a signed byte representing the component as 0.1*mm
int8_t DevUBLOXGNSS::getHighResECEFYHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefYHp == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefYHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefYHp);
}

// Get the high precision component of the ECEF Z coordinate
// Returns a signed byte representing the component as 0.1*mm
int8_t DevUBLOXGNSS::getHighResECEFZHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == nullptr)
    initPacketUBXNAVHPPOSECEF();        // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZHp == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefZHp);
}

// ***** HPPOSLLH Helper Functions

uint32_t DevUBLOXGNSS::getTimeOfWeekFromHPPOSLLH(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.iTOW == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.iTOW = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.iTOW);
}

int32_t DevUBLOXGNSS::getHighResLongitude(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lon == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lon = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.lon);
}

int32_t DevUBLOXGNSS::getHighResLatitude(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lat == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lat = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.lat);
}

int32_t DevUBLOXGNSS::getElipsoid(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.height == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.height = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.height);
}

int32_t DevUBLOXGNSS::getMeanSeaLevel(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSL == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSL = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.hMSL);
}

int8_t DevUBLOXGNSS::getHighResLongitudeHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lonHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lonHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.lonHp);
}

int8_t DevUBLOXGNSS::getHighResLatitudeHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.latHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.latHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.latHp);
}

int8_t DevUBLOXGNSS::getElipsoidHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.heightHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.heightHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.heightHp);
}

int8_t DevUBLOXGNSS::getMeanSeaLevelHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSLHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSLHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.hMSLHp);
}

uint32_t DevUBLOXGNSS::getHorizontalAccuracy(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hAcc == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.hAcc);
}

uint32_t DevUBLOXGNSS::getVerticalAccuracy(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.vAcc == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.vAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.vAcc);
}

// ***** PVAT Helper Functions

int32_t DevUBLOXGNSS::getVehicleRoll(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT();        // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehRoll == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehRoll = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.vehRoll);
}

int32_t DevUBLOXGNSS::getVehiclePitch(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT();        // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehPitch == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehPitch = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.vehPitch);
}

int32_t DevUBLOXGNSS::getVehicleHeading(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT();        // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehHeading == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehHeading = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.vehHeading);
}

int32_t DevUBLOXGNSS::getMotionHeading(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT();        // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.motHeading == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.motHeading = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.motHeading);
}

// ***** SVIN Helper Functions

bool DevUBLOXGNSS::getSurveyInActive(uint16_t maxWait)
{
  if (packetUBXNAVSVIN == nullptr)
    initPacketUBXNAVSVIN();        // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == nullptr) // Bail if the RAM allocation failed
    return false;

  if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.active == false)
    getSurveyStatus(maxWait);
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.active = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;
  return ((bool)packetUBXNAVSVIN->data.active);
}

bool DevUBLOXGNSS::getSurveyInValid(uint16_t maxWait)
{
  if (packetUBXNAVSVIN == nullptr)
    initPacketUBXNAVSVIN();        // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == nullptr) // Bail if the RAM allocation failed
    return false;

  if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.valid == false)
    getSurveyStatus(maxWait);
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.valid = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;
  return ((bool)packetUBXNAVSVIN->data.valid);
}

uint32_t DevUBLOXGNSS::getSurveyInObservationTimeFull(uint16_t maxWait) // Return the full uint32_t
{
  if (packetUBXNAVSVIN == nullptr)
    initPacketUBXNAVSVIN();        // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.dur == false)
    getSurveyStatus(maxWait);
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.dur = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVSVIN->data.dur);
}

uint16_t DevUBLOXGNSS::getSurveyInObservationTime(uint16_t maxWait) // Truncated to 65535 seconds
{
  // dur (Passed survey-in observation time) is U4 (uint32_t) seconds. Here we truncate to 16 bits
  uint32_t tmpObsTime = getSurveyInObservationTimeFull(maxWait);
  if (tmpObsTime <= 0xFFFF)
  {
    return ((uint16_t)tmpObsTime);
  }
  else
  {
    return (0xFFFF);
  }
}

float DevUBLOXGNSS::getSurveyInMeanAccuracy(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVSVIN == nullptr)
    initPacketUBXNAVSVIN();        // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.meanAcc == false)
    getSurveyStatus(maxWait);
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.meanAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;

  // meanAcc is U4 (uint32_t) in 0.1mm. We convert this to float.
  uint32_t tempFloat = packetUBXNAVSVIN->data.meanAcc;
  return (((float)tempFloat) / 10000.0); // Convert 0.1mm to m
}

// ***** TIMELS Helper Functions

uint8_t DevUBLOXGNSS::getLeapIndicator(int32_t &timeToLsEvent, uint16_t maxWait)
{
  if (packetUBXNAVTIMELS == nullptr)
    initPacketUBXNAVTIMELS();        // Check that RAM has been allocated for the TIMELS data
  if (packetUBXNAVTIMELS == nullptr) // Bail if the RAM allocation failed
    return 3;

  if (packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validTimeToLsEvent == false)
    getLeapSecondEvent(maxWait);
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validTimeToLsEvent = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.lsChange = false;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.timeToLsEvent = false;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.all = false;
  timeToLsEvent = packetUBXNAVTIMELS->data.timeToLsEvent;
  // returns NTP Leap Indicator
  // 0 -no warning
  // 1 -last minute of the day has 61 seconds
  // 2 -last minute of the day has 59 seconds
  // 3 -unknown (clock unsynchronized)
  return ((bool)packetUBXNAVTIMELS->data.valid.bits.validTimeToLsEvent ? (uint8_t)(packetUBXNAVTIMELS->data.lsChange == -1 ? 2 : packetUBXNAVTIMELS->data.lsChange) : 3);
}

int8_t DevUBLOXGNSS::getCurrentLeapSeconds(sfe_ublox_ls_src_e &source, uint16_t maxWait)
{
  if (packetUBXNAVTIMELS == nullptr)
    initPacketUBXNAVTIMELS();        // Check that RAM has been allocated for the TIMELS data
  if (packetUBXNAVTIMELS == nullptr) // Bail if the RAM allocation failed
    return false;

  if (packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validCurrLs == false)
    getLeapSecondEvent(maxWait);
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validCurrLs = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.srcOfCurrLs = false;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.currLs = false;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.all = false;
  source = ((sfe_ublox_ls_src_e)packetUBXNAVTIMELS->data.srcOfCurrLs);
  return ((int8_t)packetUBXNAVTIMELS->data.currLs);
}

// ***** RELPOSNED Helper Functions and automatic support

float DevUBLOXGNSS::getRelPosN(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == nullptr)
    initPacketUBXNAVRELPOSNED();        // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosN == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosN = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.relPosN) / 100.0); // Convert to m
}

float DevUBLOXGNSS::getRelPosE(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == nullptr)
    initPacketUBXNAVRELPOSNED();        // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosE == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosE = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.relPosE) / 100.0); // Convert to m
}

float DevUBLOXGNSS::getRelPosD(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == nullptr)
    initPacketUBXNAVRELPOSNED();        // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosD == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosD = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.relPosD) / 100.0); // Convert to m
}

float DevUBLOXGNSS::getRelPosAccN(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == nullptr)
    initPacketUBXNAVRELPOSNED();        // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accN == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accN = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.accN) / 10000.0); // Convert to m
}

float DevUBLOXGNSS::getRelPosAccE(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == nullptr)
    initPacketUBXNAVRELPOSNED();        // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accE == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accE = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.accE) / 10000.0); // Convert to m
}

float DevUBLOXGNSS::getRelPosAccD(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == nullptr)
    initPacketUBXNAVRELPOSNED();        // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accD == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accD = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.accD) / 10000.0); // Convert to m
}

// ***** AOPSTATUS Helper Functions

uint8_t DevUBLOXGNSS::getAOPSTATUSuseAOP(uint16_t maxWait)
{
  if (packetUBXNAVAOPSTATUS == nullptr)
    initPacketUBXNAVAOPSTATUS();        // Check that RAM has been allocated for the AOPSTATUS data
  if (packetUBXNAVAOPSTATUS == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.useAOP == false)
    getAOPSTATUS(maxWait);
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.useAOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVAOPSTATUS->data.aopCfg.bits.useAOP);
}

uint8_t DevUBLOXGNSS::getAOPSTATUSstatus(uint16_t maxWait)
{
  if (packetUBXNAVAOPSTATUS == nullptr)
    initPacketUBXNAVAOPSTATUS();        // Check that RAM has been allocated for the AOPSTATUS data
  if (packetUBXNAVAOPSTATUS == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.status == false)
    getAOPSTATUS(maxWait);
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.status = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVAOPSTATUS->data.status);
}

// ***** TIM TP Helper Functions

uint32_t DevUBLOXGNSS::getTIMTPtowMS(uint16_t maxWait)
{
  if (packetUBXTIMTP == nullptr)
    initPacketUBXTIMTP();        // Check that RAM has been allocated for the TP data
  if (packetUBXTIMTP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXTIMTP->moduleQueried.moduleQueried.bits.towMS == false)
    getTIMTP(maxWait);
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.towMS = false; // Since we are about to give this to user, mark this data as stale
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXTIMTP->data.towMS);
}

uint32_t DevUBLOXGNSS::getTIMTPtowSubMS(uint16_t maxWait)
{
  if (packetUBXTIMTP == nullptr)
    initPacketUBXTIMTP();        // Check that RAM has been allocated for the TP data
  if (packetUBXTIMTP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXTIMTP->moduleQueried.moduleQueried.bits.towSubMS == false)
    getTIMTP(maxWait);
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.towSubMS = false; // Since we are about to give this to user, mark this data as stale
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXTIMTP->data.towSubMS);
}

uint16_t DevUBLOXGNSS::getTIMTPweek(uint16_t maxWait)
{
  if (packetUBXTIMTP == nullptr)
    initPacketUBXTIMTP();        // Check that RAM has been allocated for the TP data
  if (packetUBXTIMTP == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXTIMTP->moduleQueried.moduleQueried.bits.week == false)
    getTIMTP(maxWait);
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.week = false; // Since we are about to give this to user, mark this data as stale
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXTIMTP->data.week);
}

// Convert TIM TP to Unix epoch including microseconds
// CAUTION! Assumes the time base is UTC and the week number is GPS
uint32_t DevUBLOXGNSS::getTIMTPAsEpoch(uint32_t &microsecond, uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXTIMTP->moduleQueried.moduleQueried.bits.week == false)
    getTIMTP(maxWait);
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.week = false; // Since we are about to give this to user, mark this data as stale
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.towMS = false;
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.towSubMS = false;
  packetUBXTIMTP->moduleQueried.moduleQueried.bits.all = false;

  uint32_t tow = packetUBXTIMTP->data.week - SFE_UBLOX_JAN_1ST_2020_WEEK; // Calculate the number of weeks since Jan 1st 2020
  tow *= SFE_UBLOX_SECS_PER_WEEK;                                         // Convert weeks to seconds
  tow += SFE_UBLOX_EPOCH_WEEK_2086;                                       // Add the TOW for Jan 1st 2020
  tow += packetUBXTIMTP->data.towMS / 1000;                               // Add the TOW for the next TP

  uint32_t us = packetUBXTIMTP->data.towMS % 1000; // Extract the milliseconds
  us *= 1000;                                      // Convert to microseconds

  double subMS = packetUBXTIMTP->data.towSubMS; // Get towSubMS (ms * 2^-32)
  subMS *= pow(2.0, -32.0);                     // Convert to milliseconds
  subMS *= 1000;                                // Convert to microseconds

  us += (uint32_t)subMS; // Add subMS

  microsecond = us;
  return tow;
}

// ***** MON HW Helper Functions

// Get the hardware status (including jamming) using UBX_MON_HW
bool DevUBLOXGNSS::getHWstatus(UBX_MON_HW_data_t *data, uint16_t maxWait)
{
  if (data == nullptr) // Check if the user forgot to include the data pointer
    return (false);    // Bail

  if (!getMONHW(maxWait))
    return (false);

  memcpy(data, &packetUBXMONHW->data, sizeof(UBX_MON_HW_data_t));

  return (true);
}

// Return the aStatus: 0=INIT, 1=DONTKNOW, 2=OK, 3=SHORT, 4=OPEN
sfe_ublox_antenna_status_e DevUBLOXGNSS::getAntennaStatus(uint16_t maxWait)
{
  if (packetUBXMONHW == nullptr)
    initPacketUBXMONHW();        // Check that RAM has been allocated for the TP data
  if (packetUBXMONHW == nullptr) // Bail if the RAM allocation failed
    return SFE_UBLOX_ANTENNA_STATUS_INIT;

  if (packetUBXMONHW->moduleQueried.moduleQueried.bits.aStatus == false)
    getMONHW(maxWait);
  packetUBXMONHW->moduleQueried.moduleQueried.bits.aStatus = false; // Since we are about to give this to user, mark this data as stale
  packetUBXMONHW->moduleQueried.moduleQueried.bits.all = false;
  return ((sfe_ublox_antenna_status_e)packetUBXMONHW->data.aStatus);
}

// ***** Helper functions for the NEO-F10N
bool DevUBLOXGNSS::getLNAMode(sfe_ublox_lna_mode_e *mode, uint8_t layer, uint16_t maxWait)
{
  return getVal8(UBLOX_CFG_HW_RF_LNA_MODE, (uint8_t *)mode, layer, maxWait); // Get the LNA mode
}
bool DevUBLOXGNSS::setLNAMode(sfe_ublox_lna_mode_e mode, uint8_t layer, uint16_t maxWait)
{
  return setVal8(UBLOX_CFG_HW_RF_LNA_MODE, (uint8_t)mode, layer, maxWait); // Set the LNA mode
}
bool DevUBLOXGNSS::getGPSL5HealthOverride(bool *override, uint8_t layer, uint16_t maxWait)
{
  return getVal8(UBLOX_CFG_SIGNAL_GPS_L5_HEALTH_OVERRIDE, (uint8_t *) override, layer, maxWait); // Get the GPS L5 health override status
}
bool DevUBLOXGNSS::setGPSL5HealthOverride(bool override, uint8_t layer, uint16_t maxWait)
{
  return setVal8(UBLOX_CFG_SIGNAL_GPS_L5_HEALTH_OVERRIDE, (uint8_t) override, layer, maxWait); // Set the GPS L5 health override status
}

#ifndef SFE_UBLOX_DISABLE_ESF
// ***** ESF Helper Functions

float DevUBLOXGNSS::getESFroll(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXESFALG == nullptr)
    initPacketUBXESFALG();        // Check that RAM has been allocated for the ESF ALG data
  if (packetUBXESFALG == nullptr) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXESFALG->moduleQueried.moduleQueried.bits.roll == false)
    getESFALG(maxWait);
  packetUBXESFALG->moduleQueried.moduleQueried.bits.roll = false; // Since we are about to give this to user, mark this data as stale
  packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXESFALG->data.roll) / 100.0); // Convert to degrees
}

float DevUBLOXGNSS::getESFpitch(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXESFALG == nullptr)
    initPacketUBXESFALG();        // Check that RAM has been allocated for the ESF ALG data
  if (packetUBXESFALG == nullptr) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXESFALG->moduleQueried.moduleQueried.bits.pitch == false)
    getESFALG(maxWait);
  packetUBXESFALG->moduleQueried.moduleQueried.bits.pitch = false; // Since we are about to give this to user, mark this data as stale
  packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXESFALG->data.pitch) / 100.0); // Convert to degrees
}

float DevUBLOXGNSS::getESFyaw(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXESFALG == nullptr)
    initPacketUBXESFALG();        // Check that RAM has been allocated for the ESF ALG data
  if (packetUBXESFALG == nullptr) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXESFALG->moduleQueried.moduleQueried.bits.yaw == false)
    getESFALG(maxWait);
  packetUBXESFALG->moduleQueried.moduleQueried.bits.yaw = false; // Since we are about to give this to user, mark this data as stale
  packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXESFALG->data.yaw) / 100.0); // Convert to degrees
}

bool DevUBLOXGNSS::getSensorFusionMeasurement(UBX_ESF_MEAS_sensorData_t *sensorData, UBX_ESF_MEAS_data_t ubxDataStruct, uint8_t sensor)
{
  sensorData->data.all = ubxDataStruct.data[sensor].data.all;
  return (true);
}

bool DevUBLOXGNSS::getRawSensorMeasurement(UBX_ESF_RAW_sensorData_t *sensorData, UBX_ESF_RAW_data_t ubxDataStruct, uint8_t sensor)
{
  sensorData->data.all = ubxDataStruct.data[sensor].data.all;
  sensorData->sTag = ubxDataStruct.data[sensor].sTag;
  return (true);
}

bool DevUBLOXGNSS::getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, uint8_t sensor, uint16_t maxWait)
{
  if (packetUBXESFSTATUS == nullptr)
    initPacketUBXESFSTATUS();        // Check that RAM has been allocated for the ESF STATUS data
  if (packetUBXESFSTATUS == nullptr) // Bail if the RAM allocation failed
    return (false);

  if ((packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.status & (1 << sensor)) == 0)
    getESFSTATUS(maxWait);
  packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.status &= ~(1 << sensor); // Since we are about to give this to user, mark this data as stale
  packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.all = false;
  sensorStatus->sensStatus1.all = packetUBXESFSTATUS->data.status[sensor].sensStatus1.all;
  sensorStatus->sensStatus2.all = packetUBXESFSTATUS->data.status[sensor].sensStatus2.all;
  sensorStatus->freq = packetUBXESFSTATUS->data.status[sensor].freq;
  sensorStatus->faults.all = packetUBXESFSTATUS->data.status[sensor].faults.all;
  return (true);
}

bool DevUBLOXGNSS::getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, UBX_ESF_STATUS_data_t ubxDataStruct, uint8_t sensor)
{
  sensorStatus->sensStatus1.all = ubxDataStruct.status[sensor].sensStatus1.all;
  sensorStatus->sensStatus2.all = ubxDataStruct.status[sensor].sensStatus2.all;
  sensorStatus->freq = ubxDataStruct.status[sensor].freq;
  sensorStatus->faults.all = ubxDataStruct.status[sensor].faults.all;
  return (true);
}
#endif

#ifndef SFE_UBLOX_DISABLE_HNR
// ***** HNR Helper Functions

// Set the High Navigation Rate
// Returns true if the setHNRNavigationRate is successful
bool DevUBLOXGNSS::setHNRNavigationRate(uint8_t rate, uint8_t layer, uint16_t maxWait)
{
  if (rate == 0) // Return now if rate is zero
    return (false);

  if (rate > 40)
    rate = 40; // Limit rate to 40Hz so i2cPollingWait is set correctly

  // Placeholder for when HNR switches to the configuration interface
  (void)layer;

  // Adjust the I2C polling timeout based on update rate
  // Do this even if the sendCommand is not ACK'd
  i2cPollingWaitHNR = 1000 / (((int)rate) * 4);                                                   // This is the number of ms to wait between checks for new I2C data. Max 250. Min 6.
  i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; // Set i2cPollingWait to the lower of NAV and HNR

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_HNR;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current HNR settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (false);

  // Load the new navigation rate into payloadCfg
  payloadCfg[0] = rate;

  // Update the navigation rate
  sfe_ublox_status_e result = sendCommand(&packetCfg, maxWait); // We are only expecting an ACK

  return (result == SFE_UBLOX_STATUS_DATA_SENT);
}

// Get the High Navigation Rate
// Returns 0 if the getHNRNavigationRate fails
uint8_t DevUBLOXGNSS::getHNRNavigationRate(uint8_t layer, uint16_t maxWait)
{
  // Placeholder for when HNR switches to the configuration interface
  (void)layer;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_HNR;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current HNR settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (0);

  // Return the navigation rate
  return (payloadCfg[0]);
}

float DevUBLOXGNSS::getHNRroll(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXHNRATT == nullptr)
    initPacketUBXHNRATT();        // Check that RAM has been allocated for the HNR ATT data
  if (packetUBXHNRATT == nullptr) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXHNRATT->moduleQueried.moduleQueried.bits.roll == false)
    getHNRATT(maxWait);
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.roll = false; // Since we are about to give this to user, mark this data as stale
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXHNRATT->data.roll) / 100000.0); // Convert to degrees
}

float DevUBLOXGNSS::getHNRpitch(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXHNRATT == nullptr)
    initPacketUBXHNRATT();        // Check that RAM has been allocated for the HNR ATT data
  if (packetUBXHNRATT == nullptr) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXHNRATT->moduleQueried.moduleQueried.bits.pitch == false)
    getHNRATT(maxWait);
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.pitch = false; // Since we are about to give this to user, mark this data as stale
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXHNRATT->data.pitch) / 100000.0); // Convert to degrees
}

float DevUBLOXGNSS::getHNRheading(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXHNRATT == nullptr)
    initPacketUBXHNRATT();        // Check that RAM has been allocated for the HNR ATT data
  if (packetUBXHNRATT == nullptr) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXHNRATT->moduleQueried.moduleQueried.bits.heading == false)
    getHNRATT(maxWait);
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.heading = false; // Since we are about to give this to user, mark this data as stale
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXHNRATT->data.heading) / 100000.0); // Convert to degrees
}
#endif

// Functions to extract signed and unsigned 8/16/32-bit data from a ubxPacket
// From v2.0: These are public. The user can call these to extract data from custom packets

// Given a spot in the payload array, extract eight bytes and build a uint64_t
uint64_t DevUBLOXGNSS::extractLongLong(ubxPacket *msg, uint16_t spotToStart)
{
  uint64_t val = 0;
  for (uint8_t i = 0; i < 8; i++)
    val |= (uint64_t)msg->payload[spotToStart + i] << (8 * i);
  return (val);
}

// Given a spot in the payload array, extract eight bytes and build a int64_t
int64_t DevUBLOXGNSS::extractSignedLongLong(ubxPacket *msg, uint16_t spotToStart)
{
  unsignedSigned64 converter64;

  converter64.unsigned64 = extractLongLong(msg, spotToStart);
  return (converter64.signed64);
}

// Given a spot in the payload array, extract four bytes and build a long
uint32_t DevUBLOXGNSS::extractLong(ubxPacket *msg, uint16_t spotToStart)
{
  uint32_t val = 0;
  for (uint8_t i = 0; i < 4; i++)
    val |= (uint32_t)msg->payload[spotToStart + i] << (8 * i);
  return (val);
}

// Just so there is no ambiguity about whether a uint32_t will cast to a int32_t correctly...
int32_t DevUBLOXGNSS::extractSignedLong(ubxPacket *msg, uint16_t spotToStart)
{
  unsignedSigned32 converter;
  converter.unsigned32 = extractLong(msg, spotToStart);
  return (converter.signed32);
}

// Given a spot in the payload array, extract two bytes and build an int
uint16_t DevUBLOXGNSS::extractInt(ubxPacket *msg, uint16_t spotToStart)
{
  uint16_t val = (uint16_t)msg->payload[spotToStart + 0] << 8 * 0;
  val |= (uint16_t)msg->payload[spotToStart + 1] << 8 * 1;
  return (val);
}

// Just so there is no ambiguity about whether a uint16_t will cast to a int16_t correctly...
int16_t DevUBLOXGNSS::extractSignedInt(ubxPacket *msg, uint16_t spotToStart)
{
  unsignedSigned16 converter;
  converter.unsigned16 = extractInt(msg, spotToStart);
  return (converter.signed16);
}

// Given a spot, extract a byte from the payload
uint8_t DevUBLOXGNSS::extractByte(ubxPacket *msg, uint16_t spotToStart)
{
  return (msg->payload[spotToStart]);
}

// Given a spot, extract a signed 8-bit value from the payload
int8_t DevUBLOXGNSS::extractSignedChar(ubxPacket *msg, uint16_t spotToStart)
{
  unsignedSigned8 converter;
  converter.unsigned8 = extractByte(msg, spotToStart);
  return (converter.signed8);
}

// Given a spot, extract a signed 32-bit float from the payload
float DevUBLOXGNSS::extractFloat(ubxPacket *msg, uint16_t spotToStart)
{
  unsigned32float converter;
  converter.unsigned32 = extractLong(msg, spotToStart);
  return (converter.flt);
}

// Given a spot, extract a signed 64-bit double from the payload
double DevUBLOXGNSS::extractDouble(ubxPacket *msg, uint16_t spotToStart)
{
  unsigned64double converter;
  converter.unsigned64 = extractLongLong(msg, spotToStart);
  return (converter.dbl);
}

// Given a pointer, extract an unsigned integer with width bits, starting at bit start
uint64_t DevUBLOXGNSS::extractUnsignedBits(uint8_t *ptr, uint16_t start, uint16_t width)
{
  uint64_t result = 0;
  uint16_t count = 0;
  uint8_t bitMask = 0x80;

  // Skip whole bytes (8 bits)
  ptr += start / 8;
  count += (start / 8) * 8;

  // Loop until we reach the start bit
  while (count < start)
  {
    bitMask >>= 1; // Shift the bit mask
    count++;       // Increment the count

    if (bitMask == 0) // Have we counted 8 bits?
    {
      ptr++;          // Point to the next byte
      bitMask = 0x80; // Reset the bit mask
    }
  }

  // We have reached the start bit and ptr is pointing at the correct byte
  // Now extract width bits, incrementing ptr and shifting bitMask as we go
  while (count < (start + width))
  {
    if (*ptr & bitMask) // Is the bit set?
      result |= 1;      // Set the corresponding bit in result

    bitMask >>= 1; // Shift the bit mask
    count++;       // Increment the count

    if (bitMask == 0) // Have we counted 8 bits?
    {
      ptr++;          // Point to the next byte
      bitMask = 0x80; // Reset the bit mask
    }

    if (count < (start + width)) // Do we need to shift result?
      result <<= 1;              // Shift the result
  }

  return result;
}

// Given a pointer, extract an signed integer with width bits, starting at bit start
int64_t DevUBLOXGNSS::extractSignedBits(uint8_t *ptr, uint16_t start, uint16_t width)
{

  unsignedSigned64 result;
  result.unsigned64 = 0;

  uint64_t twosComplement = 0xFFFFFFFFFFFFFFFF;

  bool isNegative;

  uint16_t count = 0;
  uint8_t bitMask = 0x80;

  // Skip whole bytes (8 bits)
  ptr += start / 8;
  count += (start / 8) * 8;

  // Loop until we reach the start bit
  while (count < start)
  {
    bitMask >>= 1; // Shift the bit mask
    count++;       // Increment the count

    if (bitMask == 0) // Have we counted 8 bits?
    {
      ptr++;          // Point to the next byte
      bitMask = 0x80; // Reset the bit mask
    }
  }

  isNegative = *ptr & bitMask; // Record the first bit - indicates in the number is negative

  // We have reached the start bit and ptr is pointing at the correct byte
  // Now extract width bits, incrementing ptr and shifting bitMask as we go
  while (count < (start + width))
  {
    if (*ptr & bitMask)       // Is the bit set?
      result.unsigned64 |= 1; // Set the corresponding bit in result

    bitMask >>= 1;        // Shift the bit mask
    count++;              // Increment the count
    twosComplement <<= 1; // Shift the two's complement mask (clear LSB)

    if (bitMask == 0) // Have we counted 8 bits?
    {
      ptr++;          // Point to the next byte
      bitMask = 0x80; // Reset the bit mask
    }

    if (count < (start + width)) // Do we need to shift result?
      result.unsigned64 <<= 1;   // Shift the result
  }

  // Handle negative number
  if (isNegative)
    result.unsigned64 |= twosComplement; // OR in the two's complement mask

  return result.signed64;
}
