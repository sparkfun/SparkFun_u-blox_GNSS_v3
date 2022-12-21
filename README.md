# SparkFun u-blox GNSS Arduino Library - v3

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><a href="https://www.sparkfun.com/products/15136"><img src="https://cdn.sparkfun.com//assets/parts/1/3/5/1/4/15136-SparkFun_GPS-RTK2_Board_-_ZED-F9P__Qwiic_-03.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/16481"><img src="https://cdn.sparkfun.com//assets/parts/1/5/3/5/2/16481-SparkFun_GPS-RTK-SMA_Breakout_-_ZED-F9P__Qwiic_-01a.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/18037"><img src="https://cdn.sparkfun.com//assets/parts/1/7/3/4/1/18037-SparkFun_GNSS_Receiver_Breakout_-_MAX-M10S__Qwiic_-01_Default.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/18719"><img src="https://cdn.sparkfun.com//assets/parts/1/8/2/6/6/18719-SparkFun_GNSS-RTK_Dead_Reckoning_Breakout_-_ZED-F9K__Qwiic_-01.jpg"></a></td>
    <td><a href="https://www.sparkfun.com/products/16344"><img src="https://cdn.sparkfun.com//assets/parts/1/5/0/5/9/16344-SparkFun_GPS-RTK_Dead_Reckoning_Breakout_-_ZED-F9R__Qwiic_-01a.jpg"></a></td>
  </tr>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/15136">SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)</a></td>
    <td><a href="https://www.sparkfun.com/products/16481">SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481)</a></td>
    <td><a href="https://www.sparkfun.com/products/18037">SparkFun MAX-M10S Breakout (GPS-18037)</a></td>
    <td><a href="https://www.sparkfun.com/products/18719">SparkFun ZED-F9K Breakout (GPS-18719)</a></td>
    <td><a href="https://www.sparkfun.com/products/16344">SparkFun ZED-F9R Breakout (GPS-16344)</a></td>
  </tr>
</table>

u-blox makes some incredible GNSS receivers covering everything from low-cost, highly configurable modules such as the MAX-M10S all the way up to the surveyor grade ZED-F9P with precision of the diameter of a dime. This library supports configuration and control of u-blox devices over I<sup>2</sup>C (called DDC by u-blox), Serial and SPI. This version uses the u-blox Configuration Interface to: detect the module; configure message intervals; set the base location; etc.. We wrote it for the most recent u-blox modules which no longer support messages like UBX-CFG-PRT or UBX-CFG-MSG.

If you are using an older module, like the M8, you will need to use [version 2 of this library](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library).

This library can be installed via the Arduino Library manager. Search for **SparkFun u-blox GNSS v3**.

## v3 vs. v2

This library is the new and improved version of the very popular SparkFun u-blox GNSS Arduino Library. v3 contains some big changes and improvements:

* Module detection and configuration is done exclusively using the u-blox Configuration Interface (**UBX-CFG-VALSET** and **UBX-CFG-VALGET**)
  * The library no longer uses **UBX-CFG-PRT** to detect the module and configure the port settings
  * **UBX-CFG-MSG** is no longer used to configure message intervals
  * This allows the library to work with the latest module firmware where those messages have now been deleted
* SPI support has been enhanced:
  * ```pushRawData``` is fully supported on SPI - in addition to I<sup>2</sup>C and Serial (UART)
  * ```pushAssistNowData``` is also fully supported on SPI
* The hardware interface has been abstracted:
  * All hardware communication is performed through ```class GNSSDeviceBus```
  * This makes it _much_ easier to re-use this library on other hardware platforms. Only ```sfe_bus.h``` and ```sfe_bus.cpp``` need to be updated

## Migrating to v3

Migrating to v3 is easy. There are two small changes all users will need to make:

* The name of the library header file has changed:
  * Change: ```#include <SparkFun_u-blox_GNSS_Arduino_Library.h>```
  * to: ```#include <SparkFun_u-blox_GNSS_v3.h>```
* The library now uses three separate hardware classes: ```SFE_UBLOX_GNSS``` , ```SFE_UBLOX_GNSS_SERIAL``` and ```SFE_UBLOX_GNSS_SPI```
  * ```SFE_UBLOX_GNSS``` provides support for I<sup>2</sup>C (Qwiic / Wire) _only_
  * ```SFE_UBLOX_GNSS_SERIAL``` provides support for Serial (UART) _only_
  * ```SFE_UBLOX_GNSS_SPI``` provides support for SPI _only_
  * If you are using Serial or SPI, you will need to change your code to: ```SFE_UBLOX_GNSS_SERIAL myGNSS;``` or ```SFE_UBLOX_GNSS_SPI myGNSS;```

## Compatibility

v3 of the library provides support for generation F9 and M10 u-blox GNSS modules.

For most generation 8 modules, please use [version 2 of this library](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library).

For generation 6 and 7, please see [this example (depricated)](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/tree/master/examples/Series_6_7/Example1_GetPositionAndTime_Series_6_7).

## Contributing

If you would like to contribute to this library: please do, we truly appreciate it, but please follow [these guidelines](./CONTRIBUTING.md). Thanks!

## Repository Contents

* [**/examples**](./examples) - Example sketches for the library (.ino). Run these from the Arduino IDE.
* [**/src**](./src) - Source files for the library (.cpp, .h).
* [**keywords.txt**](./keywords.txt) - Keywords from this library that will be highlighted in the Arduino IDE.
* [**library.properties**](./library.properties) - General library properties for the Arduino package manager.
* [**CONTRIBUTING.md**](./CONTRIBUTING.md) - Guidelines on how to contribute to this library.
* [**/Utils**](./Utils) - contains a Python utility which can check the contents of UBX log files.

## Documentation

* [**Installing an Arduino Library Guide**](https://learn.sparkfun.com/tutorials/installing-an-arduino-library) - Basic information on how to install an Arduino library.

## Products That Use This Library

* [GPS-15136](https://www.sparkfun.com/products/15136) - SparkFun GPS-RTK2 Board - ZED-F9P (Qwiic)
* [GPS-16481](https://www.sparkfun.com/products/16481) - SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic)
* [GPS-16344](https://www.sparkfun.com/products/16344) - SparkFun GPS-RTK Dead Reckoning Breakout - ZED-F9R (Qwiic)
* [GPS-18037](https://www.sparkfun.com/products/18037) - SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic)
* [GPS-18719](https://www.sparkfun.com/products/18719) - SparkFun GNSS-RTK Dead Reckoning Breakout - ZED-F9K (Qwiic)
* [GPS-18774](https://www.sparkfun.com/products/18774) - SparkFun GNSS Timing Breakout - ZED-F9T (Qwiic)
* [GPS-19663](https://www.sparkfun.com/products/19663) - SparkFun MicroMod GNSS Function Board - ZED-F9P
* [GPS-17722](https://www.sparkfun.com/products/17722) - SparkFun MicroMod GNSS Carrier Board (ZED-F9P)

## License Information

This library is _**open source**_!

Please see [LICENSE.md](./LICENSE.md) for full details.

- Your friends at SparkFun.
