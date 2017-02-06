
// autoprogrammer.ino
//
// this sketch allows an AVR to program another AVR containing one
// of a number of different MCUs
//
// It is based on Optiloader.ino, which in turn was based on AVRISP
//
// Optiloader is Copyright (c) 2011, 2015 by Bill Westfield ("WestfW")
//

//-------------------------------------------------------------------------------------
// "MIT Open Source Software License":
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in the
// Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//-------------------------------------------------------------------------------------


#include <avr/pgmspace.h>
#include "autoprogrammer.h"
#include "images.h"

char Arduino_preprocessor_hint;













#define PROG_FLICKER true

// Configure SPI clock (in Hz).
// E.g. for an attiny @128 kHz: the datasheet states that both the high
// and low spi clock pulse must be > 2 cpu cycles, so take 3 cycles i.e.
// divide target f_cpu by 6:
//     #define SPI_CLOCK            (128000/6)
//
// A clock slow enough for an attiny85 @ 1MHz, is a reasonable default:

#define SPI_CLOCK     (1000000/6)



// Configure which pins to use:

#define RESET     13 // Use pin 10 to reset the target rather than SS
#define LED_HB    9
#define LED_ERR   8
#define LED_PMODE 7


// By default, use hardware SPI pins:
#ifndef PIN_MOSI
#define PIN_MOSI  MOSI
#endif

#ifndef PIN_MISO
#define PIN_MISO  MISO
#endif

#ifndef PIN_SCK
#define PIN_SCK   SCK
#endif




#define POWER 9

#define LED_PIN 18
#define BLINK_TIME 30

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...




#include "SPI.h"



// Global Variables

int in_programming_mode = 0;
// address for reading and writing, set by 'U' command
int target_addr;

uint16_t target_type = 0;		/* type of target_cpu */
uint16_t target_startaddr;
uint8_t target_pagesize;       /* Page size for flash programming (bytes) */
const image_t *target_flashptr; 	       /* ptr to target info in flash */
uint16_t image_data_offset;
uint8_t record_checksum = 0;

void setup (void) {
  Serial.begin(9600); 			/* Initialize Serial for status msgs */
  delay(5000);

  pinMode(LED_PMODE, OUTPUT);
  pulse(LED_PMODE, 2);
  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR, 2);
  pinMode(LED_HB, OUTPUT);
  pulse(LED_HB, 2);



  Serial.print("\nKeyboardio AVR Flasher\n\n");

}

// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
  static unsigned long last_time = 0;
  unsigned long now = millis();
  if ((now - last_time) < 40)
    return;
  last_time = now;
  if (hbval > 192) hbdelta = -hbdelta;
  if (hbval < 32) hbdelta = -hbdelta;
  hbval += hbdelta;
  analogWrite(LED_HB, hbval);
}




void reset_target(bool reset) {
  digitalWrite(RESET, reset ? LOW: HIGH);
  
}



#define PTIME 30
void pulse(int pin, int times) {
  do {
    digitalWrite(pin, HIGH);
    delay(PTIME);
    digitalWrite(pin, LOW);
    delay(PTIME);
  } while (times--);
}

void prog_lamp(int state) {
  if (PROG_FLICKER) {
    digitalWrite(LED_PMODE, state);
  }
}


void loop (void) {
  Serial.print("\nKeyboardio AVR Flasher\n\n");
  uint8_t did_flash = attempt_flash();
  if (did_flash) {
    Serial.println("OK!");
  } else {
    Serial.println("FAIL");
  }

  target_poweroff(); 			/* turn power off */
  Serial.print("\nTarget power OFF!\n");
  Serial.print("\nType 'G' or RESET for next chip\n");
  while (1) {
    if (Serial.read() == 'G')
      break;
  }
}

/*
   Low level support functions
*/

/*
   blink_led
   turn a pin on and off a few times; indicates life via LED
*/
void blink_led (int pin, int times) {
  do {
    digitalWrite(pin, HIGH);
    delay(BLINK_TIME);
    digitalWrite(pin, LOW);
    delay(BLINK_TIME);
  } while (times--);
}


boolean target_programming_mode() {
  // Reset target before driving PIN_SCK or PIN_MOSI

  // SPI.begin() will configure SS as output,
  // so SPI master mode is selected.
  // We have defined RESET as pin 10,
  // which for many arduino's is not the SS pin.
  // So we have to configure RESET as output here,
  // (reset_target() first sets the correct level)
  reset_target(true);
  pinMode(RESET, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));

  // See avr datasheets, chapter "Serial_PRG Programming Algorithm":

  // Pulse RESET after PIN_SCK is low:
  digitalWrite(PIN_SCK, LOW);
  delay(20); // discharge PIN_SCK, value arbitrally chosen
  reset_target(false);
  // Pulse must be minimum 2 target CPU clock cycles
  // so 100 usec is ok for CPU speeds above 20KHz
  delayMicroseconds(100);
  reset_target(true);

  // Send the enable programming command:
  delay(50); // datasheet: must be > 20 msec
  spi_transaction(0xAC, 0x53, 0x00, 0x00);

  in_programming_mode = 1;
  Serial.println("starting spi txn");

  return true;
}




/* Functions specific to ISP programming of an AVR */

/* target_identify
   read the signature bytes (if possible) and check whether it's
   a legal value (atmega8, atmega168, atmega328) */



uint16_t read_signature() {

  Serial.print((char) STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);


  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);

  Serial.print(high, HEX);
  Serial.print(middle, HEX);  Serial.println(low, HEX);
  return ((middle << 8) + low);

}



boolean target_identify () {
  target_type = read_signature();
  if (target_type == 0 || target_type == 0xFFFF) {
    return false;
  } else {
    return true;
  }
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  SPI.transfer(a);
  SPI.transfer(b);
  SPI.transfer(c);
  return SPI.transfer(d);
}


uint8_t attempt_flash(void) {
  Serial.print("Target power on! ...");
  if (!target_poweron() ) {
    Serial.print("No RESET pullup detected! - no target?");
    return 0;   /* Turn on target power */
  }

  Serial.print("\nStarting Program Mode");
  if (!target_programming_mode() ) {
    Serial.println("FAIL: Enable programming mode.");
    return 0;
  }

  Serial.print("\nReading signature:");
  if (!target_identify() ) {
    Serial.print(" Bad value: ");
    Serial.println(target_type, HEX);
    if (target_type == 0 ) {
      Serial.print("(no target attached?)\n");
    }
    return 0;   /* Figure out what kind of MCU */
  }

  Serial.print("Searching for image...\n");

  if (resolve_chip_alias())  {
    Serial.print("  Compatible bootloader for ");
    Serial.println(target_type);
  } // it's ok if alias lookup fails

  if (!target_findimage() ) {
    Serial.print(" Not Found\n");
    return 0;   /* look for an image */
  } else {
    Serial.print("  Found \"");
//    Serial.println(pgm_read_byte(&target_flashptr->image_chipname[0]));



  // read back a char
  int len = strlen_P(&target_flashptr->image_chipname[0]);
  for (auto k = 0; k < len; k++)
  {
    char myChar =  pgm_read_byte_near(&target_flashptr->image_chipname[0] + k);
    Serial.print(myChar);
  }
    

  }
  Serial.println("Reading the image...");

  if (!read_image()) {
    Serial.print("ERROR: Bad checksum: ");
    Serial.println(record_checksum, HEX);
    return 0;
  }
  Serial.print("\nSetting fuses for programming");
  if (!target_progfuses() ) {
    return 0;   /* get fuses ready to program */
  }

  Serial.print("\nErasing device: ");
  target_erase();

  Serial.print("\nProgramming device: ");
  if (!target_program_from_storage() ) {
    Serial.println("Flash Write Failed");
    return 0;   /* Program the image */
  }

  Serial.print("\nRestoring normal fuses");
  if (!target_normfuses() ) {
    return 0;   /* reset fuses to normal mode */
  }



  return 1;
}


/*
   read_image

   Read an intel hex image from a string in pgm memory.
   We assume that the image does not exceed the 512 bytes that we have
   allowed for it to have.  that would be bad.
   Also read other data from the image, such as fuse and protecttion byte
   values during programming, and for after we're done.
*/

uint8_t read_byte_from_image() {
  uint8_t my_byte = pgm_read_byte_near(attiny_image + image_data_offset++);
  Serial.print(my_byte,HEX);
  Serial.print(", ");
  record_checksum += my_byte;
  return my_byte;
}

bool read_image () {
  target_startaddr = 0;
  image_data_offset = 0 ;
  
  while (1) {
    record_checksum = 0;
    uint16_t addr;
    uint16_t line_length = read_byte_from_image();
    addr = ( read_byte_from_image() << 8) + read_byte_from_image();
    /* address - first byte, address - second byte */
    if (target_startaddr == 0) {
      target_startaddr = addr;
    } else if (addr == 0) {
      break;
    }
    read_byte_from_image(); /* record type - we discard this */
    for (uint8_t i = 0; i < line_length; i++) {
      read_byte_from_image(); /* data */
    }
    read_byte_from_image(); /* checksum */
    Serial.println("");
    if (record_checksum != 0) {
      Serial.println("Checksum fail");
      return false;
    }
    Serial.println("Checksum pass!");
  }
  return true;
}

/* Search through our table of chip aliases first */
boolean resolve_chip_alias() {
  for (uint8_t i = 0; i < sizeof(aliases) / sizeof(aliases[0]); i++) {
    const alias_t *a = &aliases[i];
    if (a->real_chipsig == target_type) {
      target_type = a->alias_chipsig;  /* Overwrite chip signature */
      return true;
    }
  }
  return false;
}

/*
   target_findimage

   given target_type loaded with the relevant part of the device signature,
   search the hex images that we have programmed in flash, looking for one
   that matches.
*/

boolean target_findimage () {
  /* Search through our table of self-contained images.  */
  for (uint8_t i = 0; i < sizeof(images) / sizeof(images[0]); i++) {
    if (images[i] && (pgm_read_word(&images[i]->image_chipsig) == target_type)) {
      target_flashptr = images[i];
      target_pagesize = pgm_read_byte(&target_flashptr->image_pagesize);
      Serial.println(target_pagesize);
      Serial.println("We found the right image!!");
      return true;
    }
  }
  return (false);
}

/*
   target_progfuses
   given initialized target image data, re-program the fuses to allow
   the optiboot image to be programmed.
*/

void target_setfuse(uint8_t f, uint8_t fuse_byte) {
  if (f) {
    Serial.print(f, HEX);
    Serial.print(" ");
    Serial.print(spi_transaction(0xAC, fuse_byte, 0x00, f), HEX);
  }
}


boolean target_progfuses () {
  Serial.print("\nLCK: ");
  target_setfuse( pgm_read_byte(&target_flashptr->image_progfuses[FUSE_PROT]), 0xE0);
  Serial.print(" L:");
  target_setfuse( pgm_read_byte(&target_flashptr->image_progfuses[FUSE_LOW]), 0xA0);
  Serial.print("H:");
  target_setfuse( pgm_read_byte(&target_flashptr->image_progfuses[FUSE_HIGH]), 0xA8);
  Serial.print(" E:");
  target_setfuse( pgm_read_byte(&target_flashptr->image_progfuses[FUSE_EXT]), 0xA4);
  Serial.println();
  return true; 			/* */
}

void target_erase() {
  spi_transaction(0xAC, 0x80, 0, 0); 	/* chip erase */
  delay(1000);
}

/*
   target_program
   Actually program the image into the target chip
*/


boolean target_program_from_storage () {
  target_startaddr = 0;
  image_data_offset = 0;
  
  while (1) {
    record_checksum = 0;
    uint16_t line_length = read_byte_from_image();
    uint16_t addr = ( read_byte_from_image() << 8) + read_byte_from_image();
    /* address - first byte, address - second byte */
    if (target_startaddr == 0) {
      target_startaddr = addr;
    } else if (addr == 0) {
      break;
    }
    read_byte_from_image(); /* record type - we discard this */

    target_addr = (addr++ - target_startaddr) >> 1;
    int page = current_page(target_addr);

    for (uint8_t i = 0; i < line_length; i++) {
      if (page != current_page(target_addr)) {
        commit(page);
        page = current_page(target_addr);
      }
      spi_transaction(0x40, target_addr >> 8 & 0xFF, target_addr & 0xFF, read_byte_from_image());
      spi_transaction(0x48, target_addr >> 8 & 0xFF, target_addr & 0xFF, read_byte_from_image());
      target_addr++;
    }

    commit(page);

    read_byte_from_image(); /* read the checksum */
    if (record_checksum != 0) { /* at this point, the checksum + the content should be 0 */
      return false;
    }
  }
  return true;
}

/*
   target_normfuses
   reprogram the fuses to the state they should be in for bootloader
   based programming
*/
boolean target_normfuses () {
  Serial.print("\n  Lock: ");
  target_setfuse( pgm_read_byte(&target_flashptr->image_normfuses[FUSE_PROT]), 0xE0);
  Serial.print(" L:");
  target_setfuse( pgm_read_byte(&target_flashptr->image_normfuses[FUSE_LOW]), 0xA0);
  Serial.print("  H:");
  target_setfuse( pgm_read_byte(&target_flashptr->image_normfuses[FUSE_HIGH]), 0xA8);
  Serial.print("  E:");
  target_setfuse( pgm_read_byte(&target_flashptr->image_normfuses[FUSE_EXT]), 0xA4);

  Serial.println();
  return true; 			/* */
}

/*
   target_poweron
   Turn on power to the target chip (assuming that it is powered through
   the relevant IO pin of THIS arduino.)
*/
boolean target_poweron () {

  reset_target(true);
  return true;
}

boolean target_poweroff () {




  SPI.end();
  // We're about to take the target out of reset
  // so configure SPI pins as input
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_SCK, INPUT);
  reset_target(false);
  pinMode(RESET, INPUT);

  in_programming_mode = 0;
  return true;







}

void commit (int addr) {
  Serial.print("  Commit Page: ");
  Serial.print(addr, HEX);
  Serial.print(":");
  Serial.println(spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0), HEX);
  delay(100);
}

int current_page (int addr) {
  switch (target_pagesize) {
    case 32:
      return target_addr & 0xFFFFFFF0;
    case 64:
      return target_addr & 0xFFFFFFE0;
    case 128:
      return target_addr & 0xFFFFFFC0;
    default:
      return target_addr;
  }
}


