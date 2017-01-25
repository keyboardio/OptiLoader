/*
 * Table of defined images
 */
const image_t * images[] = {
    &image_32u4, &image_tiny88, 0
};

/*
 * Table of "Aliases."  Chips that are effectively the same as chips
 * that we have a bootloader for.  These work by simply overriding the
 * signature read with the signature of the chip we "know."
 */
const alias_t aliases[] = {
//  { "ATmega168PA", 0x940B, 0x9406 },	/* Treat 168P same as 168 */
//  { "ATmega168PB", 0x9415, 0x9406 },	/* Treat 168PB same as 168 */
//  { "ATmega328PB", 0x9516, 0x950F },	/* Treat 328PB same as 328P */
//  { "ATmega328",   0x9514, 0x950F },	/* Treat 328 same as 328P */
};

/*
 * Bootload images.
 * These are the intel Hex files produced by the optiboot makefile,
 * with a small amount of automatic editing to turn them into C strings,
 * and a header attched to identify them
 *
 * Emacs keyboard macro:

      4*SPC			;; self-insert-command
      "			;; self-insert-command
      C-e			;; move-end-of-line
      \			;; self-insert-command
      n"			;; self-insert-command * 2
      C-n			;; next-line
      C-a			;; move-beginning-of-line

 */


const image_t PROGMEM image_32u4 = {
    { "firmware_atmega32u4.hex"    } ,
    { "atmega32u4"    } ,
    0x9587,				/* Signature bytes for 32u4 */
    { 0x3F,0xFF,0xD8,0xCB,0    } ,
    { 0x2F,0,0,0,0    } ,
    128,
    {
#include "generated/atmega32u4.h"
    }
};


const image_t PROGMEM image_tiny88 = {
    { "firmware_attiny88.hex"    } ,
    { "attiny88"    } ,
    0x9311,				/* Signature bytes for Tiny88 */
    { 0x3F, 0xEE, 0xDD, 0xFE, 0    } ,
    { 0x2F,0,0,0,0    } ,
    64,
    {
#include "generated/attiny88.h"
    }
};


