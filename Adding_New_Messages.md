## How to add new messages to the SparkFun u-blox GNSS Arduino Library - v3

Here is a quick summary of how to add suport for a new UBX message. It is more of a checklist really. Sorry about that!

### Step 1: Add the new message struct to u-blox_structs.h

Add the new message struct to [u-blox_structs.h](u-blox_structs.h) using one of the existing messages as a template.

Important: use the exact field names as defined in the u-blox interface definition.

```
// UBX-NAV-PVAT (0x01 0x17): Navigation position velocity attitude time solution
const uint16_t UBX_NAV_PVAT_LEN = 116;

typedef struct
{
  uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
  uint8_t version; // Message version (0x00 for this version)

...

  uint8_t reserved2[4];
  uint8_t reserved3[4];
} UBX_NAV_PVAT_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t version : 1;

...

      uint32_t errEllipseMajor : 1;
      uint32_t errEllipseMinor : 1;
    } bits;
  } moduleQueried2;
} UBX_NAV_PVAT_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_PVAT_data_t data;
  UBX_NAV_PVAT_moduleQueried_t moduleQueried;
  void (*callbackPointerPtr)(UBX_NAV_PVAT_data_t *);
  UBX_NAV_PVAT_data_t *callbackData;
} UBX_NAV_PVAT_t;

```

### Step 2: Update u-blox_Class_and_ID.h

If required, add the new message ID to [u-blox_Class_and_ID.h](u-blox_Class_and_ID.h).

```
const uint8_t UBX_NAV_PVAT = 0x17; // Navigation position velocity attitude time solution (ZED-F9R only)
```

### Step 3: Update u-blox_config_keys.h

Add any new configuration interface keys for the new message to [u-blox_config_keys.h](u-blox_config_keys.h).

```
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_I2C = 0x2091062a;     // Output rate of the UBX-NAV-PVAT message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_UART1 = 0x2091062b;   // Output rate of the UBX-NAV-PVAT message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_UART2 = 0x2091062c;   // Output rate of the UBX-NAV-PVAT message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_USB = 0x2091062d;     // Output rate of the UBX-NAV-PVAT message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_SPI = 0x2091062e;     // Output rate of the UBX-NAV-PVAT message on port SPI
```

### Step 4: Update u-blox_GNSS.h

Add the new functions to provide "auto" support for the new message using one of the existing messages as a template:

```
  bool getNAVPVAT(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool setAutoNAVPVAT(bool enabled, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool setAutoNAVPVAT(bool enabled, bool implicitUpdate, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool setAutoNAVPVATrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool setAutoNAVPVATcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVAT_data_t *), uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool assumeAutoNAVPVAT(bool enabled, bool implicitUpdate = true);
  void flushNAVPVAT();
  void logNAVPVAT(bool enabled = true);
```

Add new helper functions to access the most important fields:

```
  int32_t getVehicleRoll(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getVehiclePitch(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getVehicleHeading(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getMotionHeading(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
```

Add the pointer to the struct storage:

```
  UBX_NAV_PVAT_t *packetUBXNAVPVAT = NULL;
```

Add the protected init function:

```
  bool initPacketUBXNAVPVAT();
```

### Step 5: Update u-blox_GNSS.cpp

#### Step 6.1: Update end()

Add the new code which will safely delete the message storgae when ```end()``` is called:

```
  if (packetUBXNAVPVAT != nullptr)
  {
    if (packetUBXNAVPVAT->callbackData != nullptr)
    {
      delete packetUBXNAVPVAT->callbackData;
    }
    delete packetUBXNAVPVAT;
    packetUBXNAVPVAT = nullptr;
  }
```

#### Step 6.2: Update autoLookup()

Add a new `switch` `case` for the message:

```
    else if (ID == UBX_NAV_PVAT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_PVAT_LEN;
      return (packetUBXNAVPVAT != nullptr);
    }
```

#### Step 6.3: Update processUBXpacket()

Add new code to extract the message data in the correct format.

Take time to double-check that you have used the correct data width, signed/unsigned and position for each field.

```
    else if (msg->id == UBX_NAV_PVAT && msg->len == UBX_NAV_PVAT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVAT != nullptr)
      {
        packetUBXNAVPVAT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVAT->data.version = extractByte(msg, 4);
        packetUBXNAVPVAT->data.valid.all = extractByte(msg, 5);

...

        packetUBXNAVPVAT->data.errEllipseOrient = extractInt(msg, 98);
        packetUBXNAVPVAT->data.errEllipseMajor = extractLong(msg, 100);
        packetUBXNAVPVAT->data.errEllipseMinor = extractLong(msg, 104);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVAT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVAT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVAT->callbackData != nullptr) // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVAT->callbackData->iTOW, &packetUBXNAVPVAT->data.iTOW, sizeof(UBX_NAV_PVAT_data_t));
          packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVAT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
```

#### Step 6.4: Update checkCallbacks()

Add the new code to call the callback:

```
  if (packetUBXNAVPVAT != nullptr) // If RAM has been allocated for message storage
    if (packetUBXNAVPVAT->callbackData != nullptr) // If RAM has been allocated for the copy of the data
      if (packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVPVAT->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVPVAT->callbackPointerPtr(packetUBXNAVPVAT->callbackData); // Call the callback
        }
        packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }
```

#### Step 6.5: Add the "auto" functions

```
// ***** PVAT automatic support

// Get the latest Position/Velocity/Time solution and fill all global variables
bool DevUBLOXGNSS::getNAVPVAT(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT();        // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
    return (false);

...

// Log this data in file buffer
void DevUBLOXGNSS::logNAVPVAT(bool enabled)
{
  if (packetUBXNAVPVAT == nullptr)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPVAT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

```

#### Step 6.6: Add the helper functions (if any)

```
// ***** PVAT Helper Functions

int32_t DevUBLOXGNSS::getVehicleRoll(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT(); // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehRoll == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehRoll = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.vehRoll);
}

...

int32_t DevUBLOXGNSS::getMotionHeading(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == nullptr)
    initPacketUBXNAVPVAT(); // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.motHeading == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.motHeading = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.motHeading);
}

```

### Step 7: Add an example

Add a new example to demonstrate the use of the message. This also helps to test your new code.

### Step 8: Update keywords.txt

Add the new message type, "auto" and helper functions to keywords.txt:

```
UBX_NAV_PVAT_data_t	KEYWORD1
```

```
getNAVPVAT	KEYWORD2
setAutoNAVPVAT	KEYWORD2
setAutoNAVPVAT	KEYWORD2
setAutoNAVPVATrate	KEYWORD2
setAutoNAVPVATcallback	KEYWORD2
setAutoNAVPVATcallbackPtr	KEYWORD2
assumeAutoNAVPVAT	KEYWORD2
flushNAVPVAT	KEYWORD2
logNAVPVAT	KEYWORD2
```

```
UBX_NAV_PVAT	LITERAL1
```
