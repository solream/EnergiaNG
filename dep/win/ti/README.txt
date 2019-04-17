BSL Scripter version 3.4.0 Release date: 04/26/2018

===============================================================================
GENERAL USAGE NOTES
===============================================================================
- BSL Scripter is a command-line and script-file interface application
  to perform the MSP430 and MSP432 devices programming 
  
- The package deliverable contains:
  1. bsl-scripter-<osType>.zip
     unzip this folder to obtain the executable file for chosen OS.
	 info: BSL Scripter v3.2.0 is delivered for:
	 - Windows 
	 - Linux 32 bit
	 - Linux 64 bit
	 - Mac OS X
  2. (folder)SourceCode 
     This folder contains the source code of the BSL-Scripter.
	 For Windows environment, the project file for Visual Studio 2013 Express
	 is also provided. For Linux and Mac OS X packages, the Makefile 
	 is provided.
  3. (folder)Example
     This folder contains the script example how to use the BSL-Scripter
	 with for each BSL families. Start from version 3.2.0, the examples
	 of command line interface are also provided.
  3. (folder)Deprecated--only applicable for Windows package
     This folder contains the BSLDEMO2.exe for MSP430F1xx,MSP430F2xx, and
	 MSP430F4xx devices.
  4. BSL-Scripter_3.3.0.manifest.html 
  5. README.txt

- The User's Guide of BSL Scripter is available under:
  http://www.ti.com/lit/slau655 

===============================================================================
BUILDING THE PROJECT FOR CUSTOMIZED BSL-SCRIPTER
===============================================================================
1. Windows
-------------------------------------------------------------------------------
1.1 Development environment
-------------------------------------------------------------------------------
* You will need to have Microsoft Visual Studio 2013 installed.
-------------------------------------------------------------------------------
1.2. Dependencies
-------------------------------------------------------------------------------
In order to compile BSL Scripter with MCVS you will need:
* boost
    -Download and build (http://www.boost.org)
    -Visual Studio users can get pre-built boost binaries from
     http://sourceforge.net/projects/boost/files/boost-binaries/
    -Version used in official build is 1.58
    -Only libboost_system-xxx.lib, libboost_filesystem-xxx.lib  
	 libboost_program_options-xxx.lib are required to be built independently
    -Visual Studio must be configured to find boost headers and libs

* hidapi
    -Download and build (https://github.com/signal11/hidapi/releases)
    -Version used in official build is 0.8.0-rc1
    -hidapi.h must be copied to ThirdParty\include\
    -hidapi.lib must be copied to ThirdParty\lib\

* libusb
    -Download and build (https://github.com/libusb/libusb/releases)
    -Version used in official build is 1.0.21
    -libusb.h must be copied to ThirdParty/include/
    -libusb-1.0.lib must be copied to ThirdParty/lib/
-------------------------------------------------------------------------------
2. Linux
-------------------------------------------------------------------------------
2.1 Development environment
-------------------------------------------------------------------------------
* You will need the GNU tool chain (GCC 4.6.3 or higher) to use the 
  existing Makefile.
-------------------------------------------------------------------------------
2.2. Dependencies
-------------------------------------------------------------------------------
In order to compile the BSL Scripter with GCC you will need:

* boost
    -Download from http://www.boost.org
    -Version used in official build is 1.56
    -Only libboost_system.a/so and libboost_filesystem.a/so
     libboost_program_options-xxx.a/so	are required

* hidapi
    -Download and build (https://github.com/signal11/hidapi/releases)
    -Version used in official build is 0.8.0-rc1
    -The BSL Scripter project assumes hidapi being built against libusb-1.0
     (the default used in the Makefile coming with hidapi 0.8.0-rc1)
    -hidapi.h must be copied to ThirdParty/include/
    -hid-libusb.o must be copied to ThirdParty/lib/

* libusb
    -Download and build (https://github.com/libusb/libusb/releases)
    -Version used in official build is 1.0.21
    -libusb.h must be copied to ThirdParty/include/
    -libusb-1.0.a must be copied to ThirdParty/lib64/ for Ubuntu 64 bit and
	 libusb-1.0.a must be copied to ThirdParty/lib/ for Ubuntu 32 bit
-------------------------------------------------------------------------------
3. OSX
-------------------------------------------------------------------------------
3.1 Development environment
-------------------------------------------------------------------------------
You will need Clang/LLVM to use the existing Makefile.
-------------------------------------------------------------------------------
3.2. Dependencies
-------------------------------------------------------------------------------
In order to compile BSL Scripter with Clang you will need:

* boost
    -Download from http://www.boost.org
    -Version used in official build is 1.56
    -Only libboost_system.a/so and libboost_filesystem.a/so
     libboost_program_options-xxx.a/so	are required

* hidapi
    -Download and build (https://github.com/signal11/hidapi/releases)
    -Version used in official build is 0.8.0-rc1
    -The BSL Scripter project assumes hidapi being built against libusb-1.0
     (the default used in the Makefile coming with hidapi 0.8.0-rc1)
    -hidapi.h must be copied to ThirdParty/include/
    -hid-libusb.o must be copied to ThirdParty/lib/

* libusb
    -Download and build (https://github.com/libusb/libusb/releases)
    -Version used in official build is 1.0.21
    -libusb.h must be copied to ThirdParty/include/
    -libusb-mac-1.0.a must be copied to ThirdParty/lib64/
-------------------------------------------------------------------------------