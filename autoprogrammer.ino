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

/*
   Pins to target
*/
#define SCK 13
#define MISO 12
#define MOSI 11
#define RESET 10
#define POWER 9

#define LED_PIN 14
#define BLINK_TIME 30

// STK Definitions; we can still use these as return codes
#define STK_OK 0x10
#define STK_FAILED 0x11



// Global Variables

int in_programming_mode = 0;
// address for reading and writing, set by 'U' command
int target_addr;

uint16_t target_type = 0;		/* type of target_cpu */
uint16_t target_startaddr;
uint8_t target_pagesize;       /* Page size for flash programming (bytes) */
const image_t *target_flashptr; 	       /* ptr to target info in flash */
uint8_t target_code[512];	       /* The whole code */
uint8_t image_data_ptr;
uint8_t record_checksum = 0;

void setup (void) {
    Serial.begin(19200); 			/* Initialize serial for status msgs */
    pinMode(LED_PIN, OUTPUT); 			/* Blink the pin LED a few times */
    blink_led(LED_PIN, 20);
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

/*
   spi_init
   initialize the AVR SPI peripheral
*/
void spi_init (void) {
    uint8_t x;
    SPCR =  SPIE | MSTR | SPR1 | SPR0;
    x = SPSR;
    x = SPDR;
}

/*
   spi_wait
   wait for SPI transfer to complete
*/
void spi_wait (void) {
    do { } while (!(SPSR & (1 << SPIF)));
}

/*
   spi_send
   send a byte via SPI, wait for the transfer.
*/
uint8_t spi_send (uint8_t send_byte) {
    uint8_t reply;
    SPDR = send_byte;
    spi_wait();
    reply = SPDR;
    return reply;
}


/* Functions specific to ISP programming of an AVR */

/* target_identify
   read the signature bytes (if possible) and check whether it's
   a legal value (atmega8, atmega168, atmega328) */

uint16_t read_signature () {
    uint8_t sig_middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
    uint8_t sig_low = spi_transaction(0x30, 0x00, 0x02, 0x00);
    return ((sig_middle << 8) + sig_low);
}

boolean target_identify () {
    target_type = read_signature();
    if (target_type == 0 || target_type == 0xFFFF) {
        return false;
    } else {
        return true;
    }
}

unsigned long spi_transaction (uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    uint8_t n, m;
    spi_send(a);
    n = spi_send(b);
    m = spi_send(c);
    return 0xFFFFFF & ((((uint32_t)n) << 16) + (m << 8) + spi_send(d));
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
        Serial.println(&target_flashptr->image_chipname[0]);

    }

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
    if (!target_program() ) {
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
    uint8_t byte = pgm_read_byte(image_data_ptr++);
    record_checksum += byte;
    return byte;
}

bool read_image () {
    target_startaddr = 0;
    image_data_ptr = &target_flashptr->image_hexcode_ptr;

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
            target_code[addr++ - target_startaddr] = read_byte_from_image(); /* data */
        }
        read_byte_from_image(); /* checksum */
        if (record_checksum != 0) {
            return false;
        }
    }
    return true;
}

/*
   target_findimage

   given target_type loaded with the relevant part of the device signature,
   search the hex images that we have programmed in flash, looking for one
   that matches.
*/



boolean resolve_chip_alias() {
    /*
       Search through our table of chip aliases first
    */
    for (uint8_t i = 0; i < sizeof(aliases) / sizeof(aliases[0]); i++) {
        const alias_t *a = &aliases[i];
        if (a->real_chipsig == target_type) {
            target_type = a->alias_chipsig;  /* Overwrite chip signature */
            return true;
        }
    }
    return false;
}

boolean target_findimage () {
    /* Search through our table of self-contained images.  */
    for (uint8_t i = 0; i < sizeof(images) / sizeof(images[0]); i++) {
        target_flashptr = images[i];
        if (target_flashptr && (pgm_read_word(&target_flashptr->image_chipsig) == target_type)) {
            target_pagesize = pgm_read_byte(&target_flashptr->image_pagesize);
            return true;
        }
    }
    target_flashptr = 0;
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

boolean target_program () {
    int length = 512; 				/* actual length */
    int byte_to_send = 0;

    target_addr = target_startaddr >> 1; 		/* word address */
    int page = current_page(target_addr);

    while (byte_to_send < length) {
        if (page != current_page(target_addr)) {
            commit(page);
            page = current_page(target_addr);
        }
        spi_transaction(0x40, target_addr >> 8 & 0xFF, target_addr & 0xFF, target_code[byte_to_send++]);
        spi_transaction(0x48, target_addr >> 8 & 0xFF, target_addr & 0xFF, target_code[byte_to_send++]);
        target_addr++;
    }

    commit(page);
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
    digitalWrite(POWER, LOW);
    pinMode(POWER, OUTPUT);
    digitalWrite(POWER, HIGH);
    digitalWrite(RESET, LOW);  // reset it right away.
    pinMode(RESET, OUTPUT);
    /*
       Check if the target is pulling RESET HIGH by reverting to input
    */
    delay(5);
    pinMode(RESET, INPUT);
    delay(1);
    if (digitalRead(RESET) != HIGH) {
        return false;
    }
    pinMode(RESET, OUTPUT);

    delay(200);
    return true;
}

boolean target_programming_mode() {
    pinMode(LED_PIN, INPUT); // restore to default
    spi_init();
    // following delays may not work on all targets...
    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, HIGH);
    pinMode(SCK, OUTPUT);
    digitalWrite(SCK, LOW);
    delay(50);
    digitalWrite(RESET, LOW);
    delay(50);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    in_programming_mode = 1;

    if (( spi_transaction(0xAC, 0x53, 0x00, 0x00) & 0xFF00) != 0x5300) {
        return false;
    }
    return true;
}

boolean target_poweroff () {
    SPCR = 0;         /* reset SPI */
    digitalWrite(MISO, 0);    /* Make sure pullups are off too */
    pinMode(MISO, INPUT);
    digitalWrite(MOSI, 0);
    pinMode(MOSI, INPUT);
    digitalWrite(SCK, 0);
    pinMode(SCK, INPUT);
    digitalWrite(RESET, 0);
    pinMode(RESET, INPUT);
    digitalWrite(POWER, LOW);
    delay(200);
    pinMode(POWER, INPUT);

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


