# About 

Optiloader is a sketch designed for bulk upgrading of Arduino board
bootloaders using another Arduino as a device programmer.  It stores
multiple copies of the optiboot bootloader in program flash memory.  When
run (ie by hitting reset), it probes the target device, figures out the type
of CPU (ATmega8, ATmega168, ATmega168P, ATmega168PB, ATmega328, ATmega328P,
ATmega328PB) and initiates upload of the bootloader and appropriate fuse
programming.  Since optiLoader runs entirely with the Arduino and there is
no communications with a host PC required, this can procede very rapidly,
and is not subject to interference of (for example) avrdude with auto-reset.

The internal copies of the bootloader are prepared manually from the .hex
files compiled in the bootloader directories.  (There's an easy editor macro
process, but it is manual.)

# Pinout

//
// Designed to connect to a generic programming cable,
// using the following pins:
// 10: slave reset
// 11: MOSI
// 12: MISO
// 13: SCK
//  9: Power to external chip.
//     This is a little questionable, since the power it is legal to draw
//     from a PIC pin is pretty close to the power consumption of an AVR
//     chip being programmed.  But it permits the target to be entirely
//     powered down for safe reconnection of the programmer to additional
//     targets, and it seems to work for most Arduinos.  If the target board
//     contains additional circuitry and is expected to draw more than 40mA,
//     connect the target power to a stronger source of +5V.  Do not use pin
//     9 to power more complex Arduino boards that draw more than 40mA, such
//     as the Arduino Uno Ethernet !
//
// If the aim is to reprogram the bootloader in one Arduino using another
// Arudino as the programmer, you can just use jumpers between the connectors
// on the Arduino board.  In this case, connect:
// Pin 13 to Pin 13
// Pin 12 to Pin 12
// Pin 11 to Pin 11
// Pin 10 (of "programmer") to RESET (of "target" (on the "power" connector))
// +5V to +5V and GND to GND.  Only the "programmer" board should be powered
//     by USB or external power.
//

# Debugging information

While communication with a host PC is not required, the sketch does send
status information to the serial port at 19200bps.  Here's a sample:

OptiLoader Bootstrap programmer.
2011 by Bill Westfield (WestfW)

Target power on! ...
Starting Program Mode [OK]

Reading signature:950F
Searching for image...
  Found "optiboot_atmega328.hex" for atmega328P
  Start address at 7E00
  Total bytes read: 502

Setting fuses for programming
  Lock: 3F FFE000  Low: FF FFA000  High: DE FFA800  Ext: 5 FFA400

Programming bootloader: 512 bytes at 0x3F00
  Commit Page: 3F00:3F00
  Commit Page: 3F40:3F40
  Commit Page: 3F80:3F80
  Commit Page: 3FC0:3FC0

Restoring normal fuses
  Lock: 2F FFE000

Target power OFF!

Type 'G' or hit RESET for next chip

# Credits

Based on optiLoader.ino

Copyright (c) 2011 by Bill Westfield ("WestfW")
Distributed under the terms of the "MIT OSSW License."

----------------------------------------------------------------------

The following credits are from AVRISP.  It turns out that there isn't
a lot of AVRISP left in this sketch, but probably if AVRISP had never
existed,  this sketch would not have been written.

October 2009 by David A. Mellis
- Added support for the read signature command

February 2009 by Randall Bohn
- Added support for writing to EEPROM (what took so long?)
Windows users should consider WinAVR's avrdude instead of the
avrdude included with Arduino software.

January 2008 by Randall Bohn
- Thanks to Amplificar for helping me with the STK500 protocol
- The AVRISP/STK500 (mk I) protocol is used in the arduino bootloader
- The SPI functions herein were developed for the AVR910_ARD programmer 
- More information at http://code.google.com/p/mega-isp


