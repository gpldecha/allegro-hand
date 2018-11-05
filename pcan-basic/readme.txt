===============================================================================
ReadMe.txt

PCAN-Basic Linux V4.2.1
Copyright (c) 2016 PEAK-System Technik GmbH Darmstadt, Germany
All rights reserved.
===============================================================================


Contents:
---------
  * Introduction
  * Rights to use these files
  * System Requirements
  * Contents of this directory
  * Installation of PCAN hardware
  * How to contact PEAK-System Technik GmbH
  * LIFE SUPPORT APPLIANCES


Introduction
------------
The PCAN system of the company PEAK-System Technik GmbH consists of a
collection of Device Drivers. These allow the Real-time connection of
applications to all CAN busses that are physically connected to the PC
via a PCAN hardware.

PCAN-Basic is a simple programming interface to the PCAN system. Via one
Interface DLL it is possible to connect own applications to the Device drivers
and the PCAN hardware, to communicate with the CAN busses.


Rights to use these files
-------------------------
PEAK-System Technik GmbH grants the right to the customer to use the files in
this software package as long as this is done in connection with original
hardware by PEAK-System or OEM hardware coming from PEAK-System. It is NOT
allowed to use any of these files (even not parts) with third-party hardware.

If you are not sure whether you have acquired an appropriate license with the
used hardware, please contact our technical support team (support@peak-system.com).


System Requirements
-------------------
- PCAN Linux Driver: 8.0.17 and above
  (Driver for Linux could be download at www.peak-system.com/linux)


Contents of this directory
--------------------------
readme.txt
    This text file.

release_notes.txt
    Notes about the versions' changes.

\extras
    Additional samples and/or tools.
	
    \pcaninfo
        This tool lists all known PCAN devices and outputs information for each.

\pcanbasic
    The interface DLL.
	
	PCANBasic_enu.chm
		The PCAN-Basic documentation in English (Windows version).

	PCANBasic_deu.chm
		The PCAN-Basic documentation in German (Windows version).

	PCAN-Parameter_Documentation.pdf
		Additional documentation about PCAN-Basic Get/Set parameters in English (Windows version).
		
	PCANBasic_enu_linux_addenda.txt
		List of known differences between Linux and Windows documentation.

    \examples
        Contains example files that demonstrate the use of the PCAN-Basic API in
		different programming languages and development environments
		(will be enhanced in the future).

\pcanjni
    PCAN-Basic Java Native Interface.

    \java
		Contains the Java files to use PCAN-Basic
	\netbeans_project
		Directory of a Java PCAN-Basic sample with NetBeans IDE
	\project-documentation
		Javadoc of the PCAN-Basic Java library

Installation of PCAN-Basic library
----------------------------------
To build PCAN-Basic library:
> cd pcanbasic
> make clean
> make 

To install PCAN-Basic library (inside pcanbasic directory):
> sudo make install
(or as root "make install")

To uninstall PCAN-Basic library (inside pcanbasic directory):
> sudo make uninstall
(or as root "make uninstall")


To build PCAN-Basic JNI, follow the same process:
0. Install PCAN-Basic library first
1. Change the current working directory to pcanjni
2. Build the application with "make"
3. Install the JNI library as root with "make install"
x. To uninstall the JNI library execute "make uninstall" as root


To build or install/uninstall extras and examples, follow the same process:
1. Change the current working directory
2. Build the application/sample with "make"
3. Install the application/sample as root with "make install"
3. Install the application/sample as root with "make uninstall"


How to contact PEAK-System Technik GmbH
---------------------------------------
If you have any questions concerning the installation of PCAN hardware, or
require information about other PEAK CAN products, then please contact us:

PEAK-System Technik GmbH
Otto-Roehm-Str. 69
D-64293 Darmstadt
Germany

Tel. +49 6151 / 8173-20
FAX  +49 6151 / 8173-29

support@peak-system.com
http://www.peak-system.com


LIFE SUPPORT APPLIANCES
-----------------------
These products are not designed for use in life support appliances, devices,
or systems where malfunction of these products can reasonably be expected to
result in personal injury. PEAK-System customers using or selling these
products for use in such applications do so at their own risk and agree to
fully indemnify PEAK-System for any damages resulting from such improper use
or sale.
