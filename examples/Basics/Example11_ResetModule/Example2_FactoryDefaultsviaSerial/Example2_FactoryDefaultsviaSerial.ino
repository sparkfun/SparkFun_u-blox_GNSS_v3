/*
  Test baud rate changes on serial, factory reset, and hard reset.
  By: Thorsten von Eicken
  Date: January 29rd, 2019
  License: MIT. See license file for more information.

  This example shows how to reset the U-Blox module to factory defaults over serial.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Connect the U-Blox serial port to Serial1
  If you're using a Uno or don't have a 2nd serial port (Serial1), use SoftwareSerial instead (see below)
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SERIAL myGNSS;

#include <SoftwareSerial.h>

//#define mySerial Serial1 // Uncomment this line to connect via Serial1
// - or -
SoftwareSerial mySerial(10, 11); // Uncomment this line to connect via SoftwareSerial(RX, TX). Connect pin 10 to GNSS TX pin.

#define defaultRate 38400 // F9/M10 - which defaults to 38400 Baud on UART1

int state = 0; // steps through auto-baud, reset, etc states

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
}

void loop()
{
    Serial.print("===== STATE ");
    Serial.println(state);
    switch (state) {
    case 0: // auto-baud connection, then switch to 38400 and save config
        do {
            Serial.println("GNSS: trying 38400 baud");
            mySerial.begin(38400);
            if (myGNSS.begin(mySerial)) break;

            delay(100);
            Serial.println("GNSS: trying 9600 baud");
            mySerial.begin(9600);
            if (myGNSS.begin(mySerial)) {
                Serial.println("GNSS: connected at 9600 baud, switching to 38400");
                myGNSS.setSerialRate(38400);
                delay(100);
            } else {
                delay(2000); //Wait a bit before trying again to limit the Serial output flood
            }
        } while(1);
        myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
        myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
        Serial.println("GNSS serial connected, saved config");
        state++;
        break;
    case 1: // hardReset, expect to see GNSS back at 38400 baud
        Serial.println("Issuing hardReset (cold start)");
        myGNSS.hardReset();
        delay(2000);
        mySerial.begin(38400);
        if (myGNSS.begin(mySerial)) {
            Serial.println("Success.");
            state++;
        } else {
            Serial.println("*** GNSS did not respond at 38400 baud, starting over.");
            state = 0;
        }
        break;
    case 2: // factoryDefault, expect to see GNSS back at defaultRate baud
        Serial.println("Issuing factoryDefualt");
        myGNSS.factoryDefault();
        delay(5000); // takes more than one second... a loop to resync would be best
        mySerial.begin(defaultRate);
        if (myGNSS.begin(mySerial)) {
            Serial.println("Success.");
            state++;
        } else {
            Serial.println("*** GNSS did not come back at defaultRate baud, starting over.");
            state = 0;
        }
        break;
    case 3: // print version info
        // Note: this may fail on boards like the UNO (ATmega328P) with modules like the ZED-F9P
        // because getProtocolVersion returns a lot of data - more than the UNO's serial buffer can hold
        Serial.print("GNSS protocol version: ");
        Serial.print(myGNSS.getProtocolVersionHigh());
        Serial.print('.');
        Serial.println(myGNSS.getProtocolVersionLow());
        Serial.println("All finished! Freezing...");
        while(1);
    }
    delay(1000);
}
