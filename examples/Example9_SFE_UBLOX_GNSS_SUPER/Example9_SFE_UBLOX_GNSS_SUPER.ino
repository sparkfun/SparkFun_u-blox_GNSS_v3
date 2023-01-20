/*
  Using the SFE_UBLOX_GNSS_SUPER class
  By: Paul Clark
  SparkFun Electronics
  Date: January 20th, 2023
  License: MIT. See license file for more information.

  This example shows how to use the SFE_UBLOX_GNSS_SUPER class.
  It allows you to write multi-purpose code that can:
  * Detect what hardware the code is running on
  * Select the correct GNSS interface (I2C/SPI/Serial) for that hardware

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> // Needed for I2C to GNSS
#include <SPI.h> // Needed for SPI to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

// Create an object of the GNSS super-class
SFE_UBLOX_GNSS_SUPER theGNSS;

// The super class supports I2C, SPI and Serial.
// Create a global enum that defines which interface to use
typedef enum {
  GNSS_IS_I2C,
  GNSS_IS_SPI,
  GNSS_IS_SERIAL
} GNSS_INTERFACE;
GNSS_INTERFACE theGNSSinterface;

// Define which Serial port the GNSS is (or could be) connected to:
#define gnssSerial Serial1 // Use hardware Serial1 to connect to the GNSS
uint32_t gnssBaudRate = 38400; // Define what Baud Rate to use. Both F9 and M10 devices default to 38400.

// If the GNSS is (or could be) connected via SPI, we need to define the port and the Chip Select pin:
#define gnssSPI SPI // Use SPI to connect to the GNSS
const int gnssSPICS = 4; // Define the Chip Select pin for the SPI interface - Pin 4 is the "free" pin on Thing Plus C

// Define which Wire (I2C) port the GNSS is (or could be) connected to:
#define gnssWire Wire // Use Wire to connect to the GNSS
const uint8_t gnssI2CAddress = 0x42; // Define the GNSS I2C address. The default is usually 0x42. (NEO-D9S uses 0x43)

void setup()
{
  delay(1000);

  Serial.begin(115200);
  Serial.println("SFE_UBLOX_GNSS_SUPER example");

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Define which interface to test. 
  // The code could auto-detect what board it is running on, e.g. by measuring the
  // voltage on an analog pin (defined by a pair of resistors) or reading the board
  // type from EEPROM, and then set theGNSSinterface to match.
  
  theGNSSinterface = GNSS_IS_I2C; // Select I2C for this test

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Now prepare the correct hardware interface for GNSS communication
  
  if (theGNSSinterface == GNSS_IS_SERIAL)
  {
    gnssSerial.begin(gnssBaudRate); // Begin the Serial port
  }
  else if (theGNSSinterface == GNSS_IS_SPI)
  {
    pinMode(gnssSPICS, OUTPUT); // Pull the chip select pin high
    digitalWrite(gnssSPICS, HIGH);
    SPI.begin();
  }
  else // if (theGNSSinterface == GNSS_IS_I2C) // Catch-all. Default to I2C
  {
    Wire.begin(); // Begin the Wire port
    //Wire.setClock(400000);
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Enable debug messages if required
  
  //theGNSS.enableDebugging(Serial); // Uncomment this line to enable debug messages on Serial

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Now begin the GNSS
  
  bool beginSuccess = false;

  if (theGNSSinterface == GNSS_IS_SERIAL)
  {
    beginSuccess = theGNSS.begin(gnssSerial);
  }
  
  else if (theGNSSinterface == GNSS_IS_SPI)
  {
    beginSuccess = theGNSS.begin(gnssSPI, gnssSPICS); // SPI, default to 4MHz
    
    //beginSuccess = theGNSS.begin(gnssSPI, gnssSPICS, 4000000); // Custom
    
    //SPISettings customSPIsettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
    //beginSuccess = theGNSS.begin(gnssSPI, gnssSPICS, customSPIsettings); // Custom
  }
  
  else // if (theGNSSinterface == GNSS_IS_I2C) // Catch-all. Default to I2C
  {
    beginSuccess = theGNSS.begin(gnssWire, gnssI2CAddress);
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Check begin was successful
 
  if (beginSuccess == false)
  {
    Serial.println(F("u-blox GNSS not detected. Please check wiring. Freezing..."));
    while (1)
    {
      ; // Do nothing more
    }
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Disable the NMEA messages, just to show how to do it on the three interfaces:
  
  if (theGNSSinterface == GNSS_IS_SERIAL)
  {
    // Assume we're connected to UART1. Could be UART2
    theGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise)
  }
  else if (theGNSSinterface == GNSS_IS_SPI)
  {
    theGNSS.setSPIOutput(COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)
  }
  else // if (theGNSSinterface == GNSS_IS_I2C) // Catch-all. Default to I2C
  {
    theGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Read the module info

  if (theGNSS.getModuleInfo()) // This line is optional. getModuleName() etc. will read the info if required
  {
      Serial.print(F("The GNSS module is: "));
      Serial.println(theGNSS.getModuleName());    

      Serial.print(F("The firmware type is: "));
      Serial.println(theGNSS.getFirmwareType());    

      Serial.print(F("The firmware version is: "));
      Serial.print(theGNSS.getFirmwareVersionHigh());
      Serial.print(F("."));
      Serial.println(theGNSS.getFirmwareVersionLow());
      
      Serial.print(F("The protocol version is: "));
      Serial.print(theGNSS.getProtocolVersionHigh());
      Serial.print(F("."));
      Serial.println(theGNSS.getProtocolVersionLow());
  }
}

void loop()
{
  // Request (poll) the position, velocity and time (PVT) information.
  // The module only responds when a new position is available. Default is once per second.
  // getPVT() returns true when new data is received.
  if (theGNSS.getPVT() == true)
  {
    int32_t latitude = theGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    int32_t longitude = theGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    int32_t altitude = theGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.println();
  }
}
