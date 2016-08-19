![IMG](http://energia.nu/img/Energia.png)

[![Stories in In Progress](https://badge.waffle.io/robertinant/energiang.png?label=In%20Progress&title=In%20Progress)](https://waffle.io/robertinant/energiang)<br>
[![Stories in Ready to Commit](https://badge.waffle.io/robertinant/energiang.png?label=Ready%20to%20Commit&title=Ready%20to%20Commit)](https://waffle.io/robertinant/energiang)

## What Is Energia?

Energia is a fork/port of Arduino for the Launchpads, or boards with MCUs from Texas Instruments.

## What Are the LaunchPads Supported?

Energia provides native support for the following LaunchPads:

MSP430 LaunchPads:

* MSP-EXP430F5529LP
* MSP-EXP430FR4133
* MSP-EXP430FR5969LP
* MSP-EXP430FR6989
* MSP-EXP430G2553LP
* MSP-EXP430FR5739LP

TivaC LaunchPads:

* EK-TM4C123GXL
* EK-TM4C1294XL
* EK-LM4F120XL

MSP432 LaunchPads:

* MSP-EXP432P401R

CC3200 LaunchPads:

* CC3200-LAUNCHXL
* RedBearLab CC3200
* RedBearLab WiFi Mini
* RedBearLab WiFi Micro

## Looking for Help?

No problem! There are a variety of resources available to get you up and running, and sprinting once you're up and running.

* [GitHub repository](https://github.com/robertinant/EnergiaNG/issues) - This very page!
* [Download](http://energia.nu/download/) - Download a compiled application for Linux, Mac OS X, or Windows.
* [Website](http://energia.nu) - Full documentation including tutorials, reference, pins maps, FAQ and much more!
* [Forum at 43oh](http://forum.43oh.com/forum/28-energia/) - A community around the original LaunchPad  but also the newer ones.
* [Bug report](https://github.com/robertinant/EnergiaNG/issues) - Is something not working as it ought to? Or better yet, is there something we could make better?

## Clone instructions:

To clone Energia:
```
git clone https://github.com/robertinant/EnergiaNG.git
```
## Building Energia
To build Energia you will need a copy of Apache ant and a recent JDK (1.8).
```
cd build
ant -buildfile build-energia.xml
```
### Other build instructions are:
```
ant -buildfile build-energia.xml clean // clean the build
ant -buildfile build-energia.xml build // build
ant -buildfile build-energia.xml dist // build the distribution
```
For other more advanced build options see:
```
build/build-energia.xml
build/build_all_dist.bash
build/build_pull_request.bash
```
Credits
--------
Energia is an open source project, supported by many.

Energia uses

[GCC ARM Embedded toolchain](https://launchpad.net/gcc-arm-embedded),
and code from [Processing](http://www.processing.org)
and [Wiring](http://wiring.org.co), [MSP430 gcc](https://sourceforge.net/projects/mspgcc/) and [mspdebug](http://dlbeer.co.nz/mspdebug/).
