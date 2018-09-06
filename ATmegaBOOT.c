/**********************************************************/
/* Serial Bootloader for Atmel megaAVR Controllers        */
/*                                                        */
/* tested with ATmega8, ATmega128 and ATmega168           */
/* should work with other mega's, see code for details    */
/*                                                        */
/* ATmegaBOOT.c                                           */
/*                                                        */
/* 20180824: Added support for wireless uploading and     */
/*           removed parts not needed for ATmega128RFA1   */
/*           Larry Bank (bitbank@pobox.com)               */
/*                                                        */
/* 20130613: Added support for ATmega128RFA1              */
/* 20090308: integrated Mega changes into main bootloader */
/*           source by D. Mellis                          */
/* 20080930: hacked for Arduino Mega (with the 1280       */
/*           processor, backwards compatible)             */
/*           by D. Cuartielles                            */
/* 20070626: hacked for Arduino Diecimila (which auto-    */
/*           resets when a USB connection is made to it)  */
/*           by D. Mellis                                 */
/* 20060802: hacked for Arduino by D. Cuartielles         */
/*           based on a previous hack by D. Mellis        */
/*           and D. Cuartielles                           */
/*                                                        */
/* Monitor and debug functions were added to the original */
/* code by Dr. Erik Lins, chip45.com. (See below)         */
/*                                                        */
/* Thanks to Karl Pitrich for fixing a bootloader pin     */
/* problem and more informative LED blinking!             */
/*                                                        */
/* For the latest version see:                            */
/* http://www.chip45.com/                                 */
/*                                                        */
/* ------------------------------------------------------ */
/*                                                        */
/* based on stk500boot.c                                  */
/* Copyright (c) 2003, Jason P. Kyle                      */
/* All rights reserved.                                   */
/* see avr1.org for original file and information         */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/* Target = Atmel AVR m128,m64,m32,m16,m8,m162,m163,m169, */
/* m8515,m8535. ATmega161 has a very small boot block so  */
/* isn't supported.                                       */
/*                                                        */
/* Tested with m168                                       */
/**********************************************************/

/* $Id$ */

/* some includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <string.h>
/* the current avr-libc eeprom functions do not support the ATmega168 */
/* own eeprom write/read functions are used instead */
#if !defined(__AVR_ATmega168__) || !defined(__AVR_ATmega328P__)
#include <avr/eeprom.h>
#endif

/* Use the F_CPU defined in Makefile */

/* 20060803: hacked by DojoCorp */
/* 20070626: hacked by David A. Mellis to decrease waiting time for auto-reset */
/* set the waiting time for the bootloader */
/* get this from the Makefile instead */
/* #define MAX_TIME_COUNT (F_CPU>>4) */

/* 20070707: hacked by David A. Mellis - after this many errors give up and launch application */
//#define MAX_ERROR_COUNT 5
#define MAX_ERROR_COUNT 20

/* set the UART baud rate */
/* 20060803: hacked by DojoCorp */
//#define BAUD_RATE   115200
#ifndef BAUD_RATE
#define BAUD_RATE   19200
#endif


/* SW_MAJOR and MINOR needs to be updated from time to time to avoid warning message from AVR Studio */
/* never allow AVR Studio to do an update !!!! */
#define HW_VER	 0x02
#define SW_MAJOR 0x01
#define SW_MINOR 0x10


/* Adjust to suit whatever pin your hardware uses to enter the bootloader */
/* ATmega128 has two UARTS so two pins are used to enter bootloader and select UART */
/* ATmega1280 has four UARTS, but for Arduino Mega, we will only use RXD0 to get code */
/* BL0... means UART0, BL1... means UART1 */
#ifdef __AVR_ATmega128__
#define BL_DDR  DDRF
#define BL_PORT PORTF
#define BL_PIN  PINF
#define BL0     PINF7
#define BL1     PINF6
/*#elif defined __AVR_ATmega128RFA1__*/
/* ToDO Enable bootloading via both UARTs */
#elif defined __AVR_ATmega1280__ 
/* we just don't do anything for the MEGA and enter bootloader on reset anyway*/
#else
/* other ATmegas have only one UART, so only one pin is defined to enter bootloader */
#define BL_DDR  DDRD
#define BL_PORT PORTD
#define BL_PIN  PIND
#define BL      PIND6
#endif

/* monitor functions will only be compiled when using ATmega128, due to bootblock size constraints */
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
#define MONITOR 1
#endif

/* define device id's */
/* manufacturer byte is always the same */
#define SIG1	0x1E	// Yep, Atmel is the only manufacturer of AVR micros.  Single source :(
// __AVR_ATmega128RFA1__
#define SIG2	0xA7
#define SIG3	0x01
#define PAGE_SIZE	0x80U	// 128 words

/* function prototypes */
void putch(char);
char getch(void);
void getNch(uint8_t);
void byte_response(uint8_t);
void nothing_response(void);

/* some variables */
union address_union {
	uint16_t word;
	uint8_t  byte[2];
} address;

union length_union {
	uint16_t word;
	uint8_t  byte[2];
} length;

struct flags_struct {
	unsigned eeprom : 1;
	unsigned rampz  : 1;
} flags;

uint8_t buff[256];
uint8_t ucTemp[256];
uint8_t rxbuf[256];
uint8_t txbuf[128];
uint8_t address_high;
volatile uint8_t txbusy, txlen;
volatile uint8_t rxhead, rxtail;
uint8_t pagesz=0x80;

uint8_t i;
uint8_t bootuart = 0;

uint8_t error_count = 0;

volatile uint8_t wirelessActive = 0;

void (*app_start)(void) = 0x0000;

// I/O definitions
#define HIGH 1
#define OUTPUT 1
#define LOW 0
#define INPUT 0
#define INPUT_PULLUP 2

// Keyboard info
#define ROWS 6
#define COLS 10

// Mapping of keyboard to GPIO pins
//uint8_t rowPins[ROWS] = {6,35,34,8,9,0};
//uint8_t colPins[COLS] = {4,A1,A3,2,1,25,16,19,23,22};
// new pin numbering
const uint8_t rowPins[ROWS] PROGMEM = {0xe6, 0xb7, 0xb6, 0xb5, 0xb4, 0xe0};
const uint8_t colPins[COLS] PROGMEM = {0xe4, 0xf1, 0xf3, 0xe2, 0xe1, 0xd7, 0xa0, 0xa5, 0xd5, 0xd4};
uint8_t bKeyMap[COLS]; // bits indicating pressed keys
uint8_t bOldKeyMap[COLS]; // previous map to look for pressed/released keys
uint8_t iCSPin, iDCPin, iResetPin;
uint8_t rssiRaw;

//#define DEBUG_COMMS
//#ifdef DEBUG_COMMS
uint8_t cursor_x=0, cursor_y=0;
//#endif

int SRXEWriteString(int x, int y, const char *szMsg, int iLen);
void SRXEFill(uint8_t ucData);
void SRXEWriteCommand(uint8_t c);
void SRXEWriteDataBlock(uint8_t *ucBuf, int iLen);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
uint8_t digitalRead(uint8_t pin);
void delay(long value);

typedef enum
{
  MODE_DATA = 0,
  MODE_COMMAND
} DC_MODE;

//Keyboard
//Logical Layout (SK# are screen keys: top to bottom 1-5 on left, 6-10 on right):
//                ROW1|ROW2|ROW3|ROW4|ROW5|ROW6|ROW7|ROW8|ROW9|ROW10
//           COL1    1|   2|   3|   4|   5|   6|   7|   8|   9|    0
//           COL2    Q|   W|   E|   R|   T|   Y|   U|   I|   O|    P
//           COL3    A|   S|   D|   F|   G|   H|   J|   K|   L| Bksp
//           COL4 Shft|   Z|   X|   C|   V|   B|   N|   M|Entr|   Up
//           COL5  Sym|Frac|Root| Exp| Spc|   ,|   .|Down|Left|Right
//           COL6  SK1| SK2| SK3| SK4| SK5| SK6| SK7| SK8| SK9| SK10
const uint8_t OriginalKeys[] PROGMEM = {'1','2','3','4','5','6','7','8','9','0',
  'q','w','e','r','t','y','u','i','o','p',
  'a','s','d','f','g','h','j','k','l',8,
  0  ,'z','x','c','v','b','n',0x5,0,0x4, // 5 = down, 4 = up
  0  ,0xd,  0,  0,' ',',','.','m',  2,  3, // 2 = left, 3 = right
  0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9};

const char menu0[31] PROGMEM = "*** Larry's SRXE Bootloader ***";
const char menu1[23] PROGMEM = "1) Run resident program";
const char menu2[24] PROGMEM = "2) Download from Serial0";
const char menu3[24] PROGMEM = "3) Download from Serial1";
const char menu4[25] PROGMEM = "4) Download from wireless";
const char menu5[18] PROGMEM = "Loading code from ";
const char szDevice2[7] PROGMEM = "Serial0";
const char szDevice3[7] PROGMEM = "Serial1";
const char szDevice4[8] PROGMEM = "Wireless";
const char szError0[31] PROGMEM = "Error: wireless failed to start";
const char szError1[24] PROGMEM = "Press a key to try again";

//
// Power on the LCD
//
const char powerup[] PROGMEM = {
  1, 0x01, // soft reset
  99, 120, // 120ms delay
  1, 0x11,  // sleep out
  1, 0x28,  // display off
  99, 50, // 50ms delay
  3, 0xc0, 0xf0, 0x00, // Vop = 0xF0
  2, 0xc3, 0x04, // BIAS = 1/14
  2, 0xc4, 0x05, // Booster = x8
  2, 0xd0, 0x1d, // Enable analog circuit
  2, 0xb3, 0x00, // Set FOSC divider
  2, 0xb5, 0x8b, // N-Line = 0
  1, 0x38,       // Set grayscale mode (0x39 = monochrome mode)
  2, 0x3a, 0x02, // Enable DDRAM interface
  2, 0x36, 0x00, // Scan direction setting
  2, 0xB0, 0x9f, // Duty setting (0x87?)
  5, 0xf0, 0x12,0x12,0x12,0x12, // 77Hz frame rate in all temperatures
  1, 0x20, // Display inversion off
  1, 0x29, // Display ON
  0};

// 5x7 font (in 6x8 cell)
// from 0x20 to 0x7f
const unsigned char ucSmallFont[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x5f,0x06,0x00,0x00,0x07,0x03,0x00,
  0x07,0x03,0x00,0x24,0x7e,0x24,0x7e,0x24,0x00,0x24,0x2b,0x6a,0x12,0x00,0x00,0x63,
  0x13,0x08,0x64,0x63,0x00,0x36,0x49,0x56,0x20,0x50,0x00,0x00,0x07,0x03,0x00,0x00,
  0x00,0x00,0x3e,0x41,0x00,0x00,0x00,0x00,0x41,0x3e,0x00,0x00,0x00,0x08,0x3e,0x1c,
  0x3e,0x08,0x00,0x08,0x08,0x3e,0x08,0x08,0x00,0x00,0xe0,0x60,0x00,0x00,0x00,0x08,
  0x08,0x08,0x08,0x08,0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,
  0x00,0x3e,0x51,0x49,0x45,0x3e,0x00,0x00,0x42,0x7f,0x40,0x00,0x00,0x62,0x51,0x49,
  0x49,0x46,0x00,0x22,0x49,0x49,0x49,0x36,0x00,0x18,0x14,0x12,0x7f,0x10,0x00,0x2f,
  0x49,0x49,0x49,0x31,0x00,0x3c,0x4a,0x49,0x49,0x30,0x00,0x01,0x71,0x09,0x05,0x03,
  0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x06,0x49,0x49,0x29,0x1e,0x00,0x00,0x6c,0x6c,
  0x00,0x00,0x00,0x00,0xec,0x6c,0x00,0x00,0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x24,
  0x24,0x24,0x24,0x24,0x00,0x00,0x41,0x22,0x14,0x08,0x00,0x02,0x01,0x59,0x09,0x06,
  0x00,0x3e,0x41,0x5d,0x55,0x1e,0x00,0x7e,0x11,0x11,0x11,0x7e,0x00,0x7f,0x49,0x49,
  0x49,0x36,0x00,0x3e,0x41,0x41,0x41,0x22,0x00,0x7f,0x41,0x41,0x41,0x3e,0x00,0x7f,
  0x49,0x49,0x49,0x41,0x00,0x7f,0x09,0x09,0x09,0x01,0x00,0x3e,0x41,0x49,0x49,0x7a,
  0x00,0x7f,0x08,0x08,0x08,0x7f,0x00,0x00,0x41,0x7f,0x41,0x00,0x00,0x30,0x40,0x40,
  0x40,0x3f,0x00,0x7f,0x08,0x14,0x22,0x41,0x00,0x7f,0x40,0x40,0x40,0x40,0x00,0x7f,
  0x02,0x04,0x02,0x7f,0x00,0x7f,0x02,0x04,0x08,0x7f,0x00,0x3e,0x41,0x41,0x41,0x3e,
  0x00,0x7f,0x09,0x09,0x09,0x06,0x00,0x3e,0x41,0x51,0x21,0x5e,0x00,0x7f,0x09,0x09,
  0x19,0x66,0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x01,0x01,0x7f,0x01,0x01,0x00,0x3f,
  0x40,0x40,0x40,0x3f,0x00,0x1f,0x20,0x40,0x20,0x1f,0x00,0x3f,0x40,0x3c,0x40,0x3f,
  0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x07,0x08,0x70,0x08,0x07,0x00,0x71,0x49,0x45,
  0x43,0x00,0x00,0x00,0x7f,0x41,0x41,0x00,0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00,
  0x41,0x41,0x7f,0x00,0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x80,0x80,0x80,0x80,0x80,
  0x00,0x00,0x03,0x07,0x00,0x00,0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x7f,0x44,0x44,
  0x44,0x38,0x00,0x38,0x44,0x44,0x44,0x28,0x00,0x38,0x44,0x44,0x44,0x7f,0x00,0x38,
  0x54,0x54,0x54,0x08,0x00,0x08,0x7e,0x09,0x09,0x00,0x00,0x18,0xa4,0xa4,0xa4,0x7c,
  0x00,0x7f,0x04,0x04,0x78,0x00,0x00,0x00,0x00,0x7d,0x40,0x00,0x00,0x40,0x80,0x84,
  0x7d,0x00,0x00,0x7f,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x7f,0x40,0x00,0x00,0x7c,
  0x04,0x18,0x04,0x78,0x00,0x7c,0x04,0x04,0x78,0x00,0x00,0x38,0x44,0x44,0x44,0x38,
  0x00,0xfc,0x44,0x44,0x44,0x38,0x00,0x38,0x44,0x44,0x44,0xfc,0x00,0x44,0x78,0x44,
  0x04,0x08,0x00,0x08,0x54,0x54,0x54,0x20,0x00,0x04,0x3e,0x44,0x24,0x00,0x00,0x3c,
  0x40,0x20,0x7c,0x00,0x00,0x1c,0x20,0x40,0x20,0x1c,0x00,0x3c,0x60,0x30,0x60,0x3c,
  0x00,0x6c,0x10,0x10,0x6c,0x00,0x00,0x9c,0xa0,0x60,0x3c,0x00,0x00,0x64,0x54,0x54,
  0x4c,0x00,0x00,0x08,0x3e,0x41,0x41,0x00,0x00,0x00,0x00,0x77,0x00,0x00,0x00,0x00,
  0x41,0x41,0x3e,0x08,0x00,0x02,0x01,0x02,0x01,0x00,0x00,0x3c,0x26,0x23,0x26,0x3c};

#define pgm_read_byte_far(address_long) \
__ELPM((uint32_t)(address_long))

uint8_t getFarByte(PGM_P src)
{
  unsigned long p = 0x10000ul + (unsigned)src;
  return pgm_read_byte_far(p);
}
void memcpyFar(uint8_t *dest, PGM_P src, int len)
{
  int i;
  unsigned long p = 0x10000ul + (unsigned)src;
  for (i=0; i<len; i++)
  {
    *dest++ = pgm_read_byte_far(p++);
  }
} /* memcpyFar() */

void ShowHex(uint8_t val, uint8_t bSent)
{
  uint8_t temp[8], l, h;
  temp[0] = (bSent) ? 'S':'R';
  l = val & 0xf; h = val >> 4;
  temp[1] = (h >= 0xa) ? h+55 : h+48;
  temp[2] = (l >= 0xa) ? l+55 : l+48;
  temp[3] = ' ';
  temp[4] = '*'; // shows where the cursor is
  SRXEWriteString(cursor_x * 48, (cursor_y * 16), (const char *)temp, -5);
  cursor_x++;
  if (cursor_x >= 8)
  {
    cursor_x = 0;
    cursor_y++;
    cursor_y &= 7; // wrap around when it goes past the bottom
  }
} // ShowHex()

void delay(long value)
{
  volatile long i;
  
  value = 400L * value; // scale for number of instructions
  for (i=0; i<value; i++)
  {
    // waste time
  }
} /* delay() */

uint8_t getPinInfo(uint8_t pin, volatile uint8_t **iPort, volatile uint8_t **iDDR, int bInput)
{
  uint8_t port, bit;
  
  port = pin >> 4; // hex port (A,B,D,E,F)
  bit = pin & 0x7;
  switch (port)
  {
    case 0xA: // really port G
      *iPort = (bInput) ? &PING : &PORTG;
      *iDDR = &DDRG;
      break;
    case 0xB:
      *iPort = (bInput) ? &PINB : &PORTB;
      *iDDR = &DDRB;
      break;
    case 0xD:
      *iPort = (bInput) ? &PIND : &PORTD;
      *iDDR = &DDRD;
      break;
    case 0xE:
      *iPort = (bInput) ? &PINE : &PORTE;
      *iDDR = &DDRE;
      break;
    case 0xF:
      *iPort = (bInput) ? &PINF : &PORTF;
      *iDDR = &DDRF;
      break;
  }
  return bit;
} /* getPinInfo() */
//
// Simplified pin numbering scheme uses a hex number to specify the port number
// and bit. Top 4 bits = port (B/D/E/F/G), bottom 3 bits specify the bit of the port
// e.g. 0xB4 = PORTB, bit 4, 0Ax is for port G
//
void pinMode(uint8_t pin, uint8_t mode)
{
  uint8_t bit;
  volatile uint8_t *iPort, *iDDR;

  bit = getPinInfo(pin, &iPort, &iDDR, 0);
  switch (mode)
  {
    case INPUT:
      *iDDR &= ~(1<<bit);
      break;
    case INPUT_PULLUP:
      *iDDR |= (1<<bit);
      *iPort |= (1<<bit); // set the output high, then set it as an input
      *iDDR &= ~(1<<bit);
      break;
    case OUTPUT:
      *iDDR |= (1<<bit);
      break;
  }
} /* pinMode() */

void digitalWrite(uint8_t pin, uint8_t value)
{
  uint8_t bit;
  volatile uint8_t *iPort, *iDDR;
  
  bit = getPinInfo(pin, &iPort, &iDDR, 0);
  if (value == LOW)
  {
    *iPort &= ~(1<<bit);
  }
  else
  {
    *iPort |= (1<<bit);
  }
} /* digitalWrite() */

uint8_t digitalRead(uint8_t pin)
{
  uint8_t bit;
  volatile uint8_t *iPort, *iDDR;
  
  bit = getPinInfo(pin, &iPort, &iDDR, 1);
  if (*iPort & (1<<bit))
    return HIGH;
  else
    return LOW;
} /* digitalRead() */

// Sets the D/C pin to data or command mode
void SRXESetMode(int iMode)
{
  digitalWrite(iDCPin, (iMode == MODE_DATA));
} /* SRXESetMode() */
uint8_t SPI_transfer(volatile char data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait for the end of the transmission
  {
  };
  return SPDR;                    // return the received byte
} /* SPI_transfer() */

//
// Turn off the LCD display (lowest power mode)
//
void SRXEPowerDown()
{
  SRXEFill(0); // fill memory with zeros to go to lowest power mode
  SRXEWriteCommand(0x28); // Display OFF
  SRXEWriteCommand(0x10); // Sleep in
} /* SRXEPowerDown() */
//
// Command sequence to power up the LCD controller
//
void SRXEPowerUp(void)
{
  PGM_P pList = powerup;
  uint8_t val, count, len = 1;
  
  while (len != 0)
  {
    len = getFarByte(pList++);
    if (len == 99) // delay
    {
      val = getFarByte(pList++);
      delay(val);
    }
    else if (len != 0) // send command with optional data
    {
      val = getFarByte(pList++); // command
      SRXEWriteCommand(val);
      count = len-1;
      if (count != 0)
      {
        memcpyFar(ucTemp, pList, count);
        pList += count;
        SRXEWriteDataBlock(ucTemp, count);
      }
    }
  }
} /* SRXEPowerUp() */
//
// Initialize SPI using direct register access
//
void SPI_Init(void)
{
  uint8_t temp;
  // Initialize SPI
  // Set SS to high so a connected chip will be "deselected" by default
  digitalWrite(0xb0, HIGH);
  
  // When the SS pin is set as OUTPUT, it can be used as
  // a general purpose output port (it doesn't influence
  // SPI operations).
  pinMode(0xb0, OUTPUT); // SS as output

  // SPCR = 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate (fastest)
  SPCR = (1<<SPE)|(1<<MSTR);
  temp=SPSR; // clear old data
  temp=SPDR;
  if (temp != 0) {}; // suppress compiler warning
  // Set SCK as output
  pinMode(0xb1, OUTPUT);
  // set MOSI as output
  pinMode(0xb2, OUTPUT);

} /* SPI_Init() */
//
// Initializes the LCD controller
// Parameters: GPIO pin numbers used for the CS/DC/RST control lines
//
int SRXEInit(int iCS, int iDC, int iReset)
{
  iCSPin = iCS;
  iDCPin = iDC;
  iResetPin = iReset;
  
  SPI_Init();
  pinMode(iCSPin,OUTPUT);
  digitalWrite(iCSPin, HIGH);
  
  pinMode(iDCPin, OUTPUT);
  pinMode(iResetPin, OUTPUT);
  
  // Start by reseting the LCD controller
  digitalWrite(iResetPin, HIGH);
  delay(50);
  digitalWrite(iResetPin, LOW);
  delay(5);
  digitalWrite(iResetPin, HIGH); // take it out of reset
  delay(150); // datasheet says it must be at least 120ms
  
  digitalWrite(iCSPin, LOW); // leave CS low forever
  
  SRXEPowerUp(); // turn on and initialize the display
  
  SRXEFill(0); // erase memory
  return 0;
  
} /* SRXEInit() */
//
// Write a 1 byte command to the LCD controller
//
void SRXEWriteCommand(uint8_t c)
{
  SRXESetMode(MODE_COMMAND);
  SPI_transfer(c);
  SRXESetMode(MODE_DATA);
} /* SRXEWriteCommand() */
//
// Send commands to position the "cursor" to the given
// row and column
//
void SRXESetPosition(int x, int y, int cx, int cy)
{
  if (x > 383 || y > 135 || cx > 384 || cy > 136)
    return; // invalid
  SRXEWriteCommand(0x2a); // set column address
  ucTemp[0] = 0; // start column high byte
  ucTemp[1] = x/3; // start column low byte
  ucTemp[2] = 0; // end col high byte
  ucTemp[3] = (x+cx-1)/3; // end col low byte
  SRXEWriteDataBlock(ucTemp, 4);
  SRXEWriteCommand(0x2b); // set row address
  ucTemp[0] = 0; // start row high byte
  ucTemp[1] = y; // start row low byte
  ucTemp[2] = 0; // end row high byte
  ucTemp[3] = y+cy-1; // end row low byte
  SRXEWriteDataBlock(ucTemp, 4);
  SRXEWriteCommand(0x2c); // write RAM
} /* SRXESetPosition() */

// Write a block of pixel data to the LCD
// Length can be anything from 1 to 504 (whole display)
void SRXEWriteDataBlock(uint8_t *ucBuf, int iLen)
{
  int i;
  
  //  digitalWrite(iCSPin, LOW);
  for (i=0; i<iLen; i++)
    SPI_transfer(ucBuf[i]);
  //  digitalWrite(iCSPin, HIGH);
}
//
// Draw a string of normal (8x8), small (6x8) or large (16x24) characters
// At the given col+row
//
int SRXEWriteString(int x, int y, const char *szMsg, int iLen)
{
  int i;
  int iWidth, iDelta;
  uint8_t msgTemp[32];
  
    iWidth = 12;
    iDelta = 4;
    if (iLen < 0) // coming from RAM
    {
      iLen = 0-iLen;
      memcpy(msgTemp, szMsg, iLen);
    }
    else // combing from FLASH
    {
       memcpyFar(msgTemp, szMsg, iLen);
    }
    if ((iWidth*iLen) + x > 384) iLen = (384 - x)/iWidth; // can't display it all
    if (iLen < 0)return -1;
    for (i=0; i<iLen; i++)
    {
      int tx, ty, c;
      uint8_t bTemp[84], bMask, bOut, bOut2, *d, *s;
      c = msgTemp[i];
      c -= 32; // font starts from space (0x20)
      memcpyFar(ucTemp, (const char *)&ucSmallFont[c*6], 6);
      // convert from 1-bpp to 2/3-bpp
      d = bTemp;
      s = ucTemp;
      bMask = 1;
      for (ty=0; ty<8; ty++)
      {
        for (tx=0; tx<iWidth-6; tx+=3) // 3 sets of 3 pixels
        {
          bOut = bOut2 = 0;
          if (s[tx] & bMask)
          {
            bOut |= 0xfc; // first 2 pixels (6 bits)
          }
          if (s[tx+1] & bMask)
          {
            bOut |= 0x03; // third pixel (2 bits)
            bOut2 |= 0xe0; // first pixel
          }
          if (s[tx+2] & bMask)
          {
            bOut2 |= 0x1f; // 2nd & 3rd pixels of second byte
          }
          d[0] = d[iDelta] = bOut;
          if (tx != 6)
            d[1] = d[iDelta+1] = bOut2;
          d += 2;
        } // for tx
        d += 4; // skip extra line (add 4 since we incremented by 6 already)
        bMask <<= 1;
      } // for ty
      SRXESetPosition(x, y, iWidth, 16);
      SRXEWriteDataBlock(bTemp, 16*iDelta); // write character pattern
      x += iWidth;
    } // for each character
  return 0;
} /* SRXEWriteString() */
//
// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
//
void SRXEFill(uint8_t ucData)
{
  int y;
  uint8_t temp[128];
  
  SRXESetPosition(0, 0, 384, 136);
  for (y=0; y<136; y++)
  {
    memset(temp, ucData, 128); // have to do this because the bytes get overwritten
    SRXEWriteDataBlock(temp, 128); // fill with data byte
  }
} /* SRXEFill() */
//
// Scan the rows and columns and store the results in the key map
// returns 0 if no keys are currently pressed, non-zero if any are pressed
//
uint8_t SRXEScanKeyboard(void)
{
  uint8_t r, c;
  uint8_t allkeys = 0;
  
  for (r=0; r<ROWS; r++)
  {
    pinMode(getFarByte((const char *)&rowPins[r]), INPUT_PULLUP);
  }
  // save current keymap to compare for pressed/released keys
  memcpy(bOldKeyMap, bKeyMap, sizeof(bKeyMap));
  
  for (c=0; c<COLS; c++)
  {
    bKeyMap[c] = 0;
    pinMode(getFarByte((const char *)&colPins[c]), OUTPUT);
    digitalWrite(getFarByte((const char *)&colPins[c]), LOW); // test this column
    for (r=0; r<ROWS; r++)
    {
      if (digitalRead(getFarByte((const char *)&rowPins[r])) == LOW)
        bKeyMap[c] |= (1 << r); // set a bit for each pressed key
    } // for r
    digitalWrite(getFarByte((const char *)&colPins[c]), HIGH); // leave pin in high impedance state
    pinMode(getFarByte((const char *)&colPins[c]), INPUT);
    allkeys |= bKeyMap[c];
  } // for c
  return allkeys;
} /* SRXEScanKeyboard() */
//
// Return the current key pressed
// includes code to provide shift + sym adjusted output
//
uint8_t SRXEGetKey(void)
{
//  uint8_t bShift, bSym;
  PGM_P pKeys;
  uint8_t iCol, iRow;
  uint8_t bMask;
  uint8_t bKey = 0;
  
  SRXEScanKeyboard();
// Saved some space by removing the shift/sym key support
//  bShift =  bKeyMap[0] & 0x08;
//  bSym = bKeyMap[0] & 0x10;
  for (iCol = 0; iCol < COLS; iCol++)
  {
    bMask = 1;
    for (iRow=0; iRow < ROWS; iRow++, bMask <<= 1)
    {
      if ((bKeyMap[iCol] & bMask) == bMask && (bOldKeyMap[iCol] & bMask) == 0)
      {
        // make sure it's not shift/sym
        if (iCol == 0 && (iRow == 3 || iRow == 4)) // shift/sym, ignore
          continue;
        // valid key, adjust it and return
        pKeys = (PGM_P)OriginalKeys;
//        if (bShift) pKeys = (PGM_P)ShiftedKeys;
//        else if (bSym) pKeys = (PGM_P)SymKeys;
        bKey = getFarByte(&pKeys[(iRow*COLS)+iCol]);
      }
    } // for iRow
  } // for iCol
  return bKey; // 0 if no keys pressed
} /* SRXEGetKey() */

// Initialize the RFA1's low-power 2.4GHz transciever.
// Sets up the state machine, and gets the radio into
// the RX_ON state. Interrupts are enabled for RX
// begin and end, as well as TX end.
uint8_t rfBegin(uint8_t channel)
{
  // outgoing and incoming buffer management
  txlen = 0;
  rxhead = rxtail = 0;
  txbusy = 0;
  
  // Transceiver Pin Register -- TRXPR.
  // This register can be used to reset the transceiver, without
  // resetting the MCU.
  TRXPR |= (1<<TRXRST);   // TRXRST = 1 (Reset state, resets all registers)
  
  // Transceiver Interrupt Enable Mask - IRQ_MASK
  // This register disables/enables individual radio interrupts.
  // First, we'll disable IRQ and clear any pending IRQ's
  IRQ_MASK = 0;  // Disable all IRQs
  
  // Transceiver State Control Register -- TRX_STATE
  // This regiseter controls the states of the radio.
  // First, we'll set it to the TRX_OFF state.
  TRX_STATE = ((TRX_STATE & 0xE0) | TRX_OFF);  // Set to TRX_OFF state
  delay(1);
  
  // Transceiver Status Register -- TRX_STATUS
  // This read-only register contains the present state of the radio transceiver.
  // After telling it to go to the TRX_OFF state, we'll make sure it's actually
  // there.
  if ((TRX_STATUS & 0x1F) != TRX_OFF) // Check to make sure state is correct
    return 0;    // Error, TRX isn't off
  
  // Transceiver Control Register 1 - TRX_CTRL_1
  // We'll use this register to turn on automatic CRC calculations.
  TRX_CTRL_1 |= (1<<TX_AUTO_CRC_ON);  // Enable automatic CRC calc.
  
  // Enable RX start/end and TX end interrupts
  IRQ_MASK = (1<<RX_START_EN) | (1<<RX_END_EN) | (1<<TX_END_EN);
  
  // Transceiver Clear Channel Assessment (CCA) -- PHY_CC_CCA
  // This register is used to set the channel. CCA_MODE should default
  // to Energy Above Threshold Mode.
  // Channel should be between 11 and 26 (2405 MHz to 2480 MHz)
  if ((channel < 11) || (channel > 26)) channel = 11;
  PHY_CC_CCA = (PHY_CC_CCA & 0xE0) | channel; // Set the channel
  
  // Finally, we'll enter into the RX_ON state. Now waiting for radio RX's, unless
  // we go into a transmitting state.
  TRX_STATE = (TRX_STATE & 0xE0) | RX_ON; // Default to receiver
  
  return 1;
} /* rfBegin() */
//
// This function sends a set of bytes out of the radio.
// It will set the first byte of the transmission packet to the total length
//
void rfWrite(void)
{
  if (txlen == 0)
    return; // nothing to do
  while (txbusy) // need to wait for the last packet to finish transmitting
  { };
  
  // Transceiver State Control Register -- TRX_STATE
  // This regiseter controls the states of the radio.
  // Set to the PLL_ON state - this state begins the TX.
  TRX_STATE = (TRX_STATE & 0xE0) | PLL_ON;  // Set to TX start state
  while(!(TRX_STATUS & PLL_ON))
    ;  // Wait for PLL to lock
  
  // Start of frame buffer - TRXFBST
  // This is the first byte of the (up to 128 byte) frame. It should contain
  // the length of the transmission.
  TRXFBST = txlen + 2; // len + data to send (2 bytes for 16-bit CRC)
  memcpy((void *)(&TRXFBST+1), txbuf, txlen);
  txlen = 0; // reset output buffer pointer
  // Transceiver Pin Register -- TRXPR.
  // From the PLL_ON state, setting SLPTR high will initiate the TX.
  TRXPR |= (1<<SLPTR);   // SLPTR high
  TRXPR &= ~(1<<SLPTR);  // SLPTR low
  txbusy = 1; // flag indicating the transmitter is busy
  
  // After sending the bytes, set the radio back into the RX waiting state.
  TRX_STATE = (TRX_STATE & 0xE0) | RX_ON;
} /* rfWrite() */

// This interrupt is called when radio TX is complete. We'll just
ISR(TRX24_TX_END_vect)
{
  txbusy = 0; // indicate the packet is finished transmitting
}

// This interrupt is called the moment data is received by the radio.
// We'll use it to gather information about RSSI -- signal strength --
ISR(TRX24_RX_START_vect)
{
  rssiRaw = PHY_RSSI;  // Read in the received signal strength
}

// This interrupt is called at the end of data receipt. Here we'll gather
// up the data received. And store it into a global variable. We'll

ISR(TRX24_RX_END_vect)
{
  uint8_t length, *s;
  
  // The received signal must be above a certain threshold.
  if (rssiRaw & RX_CRC_VALID)
  {
    // The length of the message will be the first byte received.
    length = TST_RX_LENGTH; // last 2 bytes are CRC, skip
    s = (uint8_t *)(void*)&TRXFBST;
    while (length > 2) // max length is 127
    {
      rxbuf[rxhead++] = *s++;
      length--;
    }
  }
}

void WriteFlashPage(void)
{
  cli();          //Disable interrupts, just to be sure
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega128RFA1__)
  while(bit_is_set(EECR,EEPE));      //Wait for previous EEPROM writes to complete
#else
  while(bit_is_set(EECR,EEWE));      //Wait for previous EEPROM writes to complete
#endif
  asm volatile(
               "clr  r17    \n\t"  //page_word_count
               "lds  r30,address  \n\t"  //Address of FLASH location (in bytes)
               "lds  r31,address+1  \n\t"
               "ldi  r28,lo8(buff)  \n\t"  //Start of buffer array in RAM
               "ldi  r29,hi8(buff)  \n\t"
               "lds  r24,length  \n\t"  //Length of data to be written (in bytes)
               "lds  r25,length+1  \n\t"
               "length_loop:    \n\t"  //Main loop, repeat for number of words in block
               "cpi  r17,0x00  \n\t"  //If page_word_count=0 then erase page
               "brne  no_page_erase  \n\t"
               "wait_spm1:    \n\t"
               "lds  r16,%0    \n\t"  //Wait for previous spm to complete
               "andi  r16,1           \n\t"
               "cpi  r16,1           \n\t"
               "breq  wait_spm1       \n\t"
               "ldi  r16,0x03  \n\t"  //Erase page pointed to by Z
               "sts  %0,r16    \n\t"
               "spm      \n\t"
#ifdef __AVR_ATmega163__
               ".word 0xFFFF    \n\t"
               "nop      \n\t"
#endif
               "wait_spm2:    \n\t"
               "lds  r16,%0    \n\t"  //Wait for previous spm to complete
               "andi  r16,1           \n\t"
               "cpi  r16,1           \n\t"
               "breq  wait_spm2       \n\t"
               
               "ldi  r16,0x11  \n\t"  //Re-enable RWW section
               "sts  %0,r16    \n\t"
               "spm      \n\t"
#ifdef __AVR_ATmega163__
               ".word 0xFFFF    \n\t"
               "nop      \n\t"
#endif
               "no_page_erase:    \n\t"
               "ld  r0,Y+    \n\t"  //Write 2 bytes into page buffer
               "ld  r1,Y+    \n\t"
               
               "wait_spm3:    \n\t"
               "lds  r16,%0    \n\t"  //Wait for previous spm to complete
               "andi  r16,1           \n\t"
               "cpi  r16,1           \n\t"
               "breq  wait_spm3       \n\t"
               "ldi  r16,0x01  \n\t"  //Load r0,r1 into FLASH page buffer
               "sts  %0,r16    \n\t"
               "spm      \n\t"
               
               "inc  r17    \n\t"  //page_word_count++
               "cpi r17,%1          \n\t"
               "brlo  same_page  \n\t"  //Still same page in FLASH
               "write_page:    \n\t"
               "clr  r17    \n\t"  //New page, write current one first
               "wait_spm4:    \n\t"
               "lds  r16,%0    \n\t"  //Wait for previous spm to complete
               "andi  r16,1           \n\t"
               "cpi  r16,1           \n\t"
               "breq  wait_spm4       \n\t"
#ifdef __AVR_ATmega163__
               "andi  r30,0x80  \n\t"  // m163 requires Z6:Z1 to be zero during page write
#endif
               "ldi  r16,0x05  \n\t"  //Write page pointed to by Z
               "sts  %0,r16    \n\t"
               "spm      \n\t"
#ifdef __AVR_ATmega163__
               ".word 0xFFFF    \n\t"
               "nop      \n\t"
               "ori  r30,0x7E  \n\t"  // recover Z6:Z1 state after page write (had to be zero during write)
#endif
               "wait_spm5:    \n\t"
               "lds  r16,%0    \n\t"  //Wait for previous spm to complete
               "andi  r16,1           \n\t"
               "cpi  r16,1           \n\t"
               "breq  wait_spm5       \n\t"
               "ldi  r16,0x11  \n\t"  //Re-enable RWW section
               "sts  %0,r16    \n\t"
               "spm      \n\t"
#ifdef __AVR_ATmega163__
               ".word 0xFFFF    \n\t"
               "nop      \n\t"
#endif
               "same_page:    \n\t"
               "adiw  r30,2    \n\t"  //Next word in FLASH
               "sbiw  r24,2    \n\t"  //length-2
               "breq  final_write  \n\t"  //Finished
               "rjmp  length_loop  \n\t"
               "final_write:    \n\t"
               "cpi  r17,0    \n\t"
               "breq  block_done  \n\t"
               "adiw  r24,2    \n\t"  //length+2, fool above check on length after short page write
               "rjmp  write_page  \n\t"
               "block_done:    \n\t"
               "clr  __zero_reg__  \n\t"  //restore zero register
#if defined __AVR_ATmega168__  || __AVR_ATmega328P__ || __AVR_ATmega128__ || __AVR_ATmega1280__ || __AVR_ATmega1281__ || __AVR_ATmega128RFA1__
               : "=m" (SPMCSR) : "M" (PAGE_SIZE) : "r0","r16","r17","r24","r25","r28","r29","r30","r31"
#else
               : "=m" (SPMCR) : "M" (PAGE_SIZE) : "r0","r16","r17","r24","r25","r28","r29","r30","r31"
#endif
               );
} /* WriteFlashPage() */

//
// Called when the power button is pressed to wake up the system
// Power up the display
//
ISR (INT2_vect)
{
  // cancel sleep as a precaution
  sleep_disable();
}
//
// Put the device in a deep sleep to save power
// Wakes up when pressing the "power" button
//
void SRXESleep(void)
{
  // Turn off the LCD
  SRXEPowerDown();
  
  TRXPR = 1 << SLPTR; // send transceiver to sleep
  
  // disable ADC
  ADCSRA = 0;
  DDRD &= ~(1 << PORTD2);       //PIN INT2 as input
  PORTD |= (1 << PORTD2); // pull-up resistor, the pin is forced to 1 if nothing is connected
  EIMSK &= ~(1 << INT2); //disabling interrupt on INT2
  EICRA &= ~((1<<ISC21) | (1<<ISC20)); // low level triggers interrupt
  EIFR |= (1 << INTF2); //clear interrupt flag
  EIMSK |= (1 << INT2); //enabling interrupt flag on INT2
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  // turn off brown-out enable in software
  // BODS must be set to one and BODSE must be set to zero within four clock cycles
  //  MCUCR = bit (BODS) | bit (BODSE);
  // The BODS bit is automatically cleared after three clock cycles
  //  MCUCR = bit (BODS);
  
  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  sei();
  sleep_cpu ();   // one cycle
  SRXEPowerUp(); // we're back - turn on the LCD
} /* SRXESleep() */

/* main program starts here */
int main(void)
{
	uint8_t ch,ch2;
	uint16_t w;
  uint8_t bKey, bDevice = 255;
  uint32_t ulTimeout;
  
//  if (SRXEScanKeyboard() == 0) // no keys pressed, just start running the program
//    app_start();
  
#ifdef WATCHDOG_MODS
	ch = MCUSR;
	MCUSR = 0;

	WDTCSR |= _BV(WDCE) | _BV(WDE);
	WDTCSR = 0;

	// Check if the WDT was used to reset, in which case we dont bootload and skip straight to the code. woot.
	if (! (ch &  _BV(EXTRF))) // if its a not an external reset...
		app_start();  // skip bootloader
#else
	asm volatile("nop\n\t");
#endif

	/* set pin direction for bootloader pin and enable pullup */
	/* for ATmega128, two pins need to be initialized */
#ifdef __AVR_ATmega128__
	BL_DDR &= ~_BV(BL0);
	BL_DDR &= ~_BV(BL1);
	BL_PORT |= _BV(BL0);
	BL_PORT |= _BV(BL1);
#else
	/* We run the bootloader regardless of the state of this pin.  Thus, don't
	put it in a different state than the other pins.  --DAM, 070709
	This also applies to Arduino Mega -- DC, 080930
	BL_DDR &= ~_BV(BL);
	BL_PORT |= _BV(BL);
	*/
#endif


#ifdef __AVR_ATmega128__
	/* check which UART should be used for booting */
	if(bit_is_clear(BL_PIN, BL0)) {
		bootuart = 1;
	}
	else if(bit_is_clear(BL_PIN, BL1)) {
		bootuart = 2;
	}
#endif

// Initialize the LCD and display the custom bootloader menu
// chip select, D/C, Reset
  SRXEInit(0xe7, 0xd6, 0xa2); // initialize display
menu_start:
  wirelessActive = 0; // assume we're not receiving over wireless
  
  SRXEWriteString(0, 00, menu0, sizeof(menu0));
  SRXEWriteString(0, 30, menu1, sizeof(menu1));
  SRXEWriteString(0, 60, menu2, sizeof(menu2));
  SRXEWriteString(0, 90, menu3, sizeof(menu3));
  SRXEWriteString(0, 120, menu4, sizeof(menu4));
  // Wait for user to make a selection
  ulTimeout = 0;
  while (bDevice == 255)
  {
    bKey = SRXEGetKey(); // try to read a key from the keyboard
    if ((bKey >= 0xf1 && bKey <= 0xf4) || (bKey >= '1' && bKey <= '4')) // pressed a valid key
    {
      bDevice = bKey & 7;
      ulTimeout = 0;
    }
    else
    {
      delay(50); // no need to read the keyboard too frequently
      ulTimeout++; // if the user sits here too long, turn off the system
      if (ulTimeout > 120*20) // 2 minutes of no activity
      {
        SRXESleep(); // power down
        goto menu_start; // re-draw the menu when we wake up
      }
    }
  }
  SRXEFill(0);
  if (bDevice == 1) // the user just wants to run the resident program
  {
    // Put interrupts back in app land
    MCUCR = (1<<IVCE);
    MCUCR = 0;
    app_start();
  }
  SRXEWriteString(0,0,menu5, sizeof(menu5));
  // compiler won't seem to allow a string array, so do it manually
  if (bDevice == 2)
    SRXEWriteString(216,0, szDevice2, sizeof(szDevice2));
  else if (bDevice == 3)
    SRXEWriteString(216,0, szDevice3, sizeof(szDevice3));
  else SRXEWriteString(216,0, szDevice4, sizeof(szDevice4));
  
  if (bDevice == 4) // user wants to load from wireless
  {
     MCUCR = (1<<IVCE); // enable interrupt change
     MCUCR = (1<<IVSEL); // move interrupts to the BOOTLOADER area
     sei();
     // Initialize the radio to the least interfered channel (26)
     wirelessActive = rfBegin(26);
     if (!wirelessActive)
     {
       SRXEWriteString(0,16, szError0, sizeof(szError0));
       SRXEWriteString(0,32, szError1, sizeof(szError1));
       while (SRXEScanKeyboard() != 0) // wait for any previous key to be released
       {};
       
       while (SRXEScanKeyboard() == 0) // wait for a new key to be pressed
       {};
       cli();
       goto menu_start;
     }
    // Use a custom protocol to receive and flash the program
    {
      uint8_t iPage = 0, bDone = 0;
      int iLen;
      uint8_t i, j, sum, u8Page, u8Segment, u8Bits = 0;
      volatile uint32_t iTimeout = 0;
      cursor_x = 0;
      cursor_y = 1;
      while (!bDone)
      {
        iTimeout = 0;
         while (rxtail == rxhead && iTimeout < 4000000L)
         {
           iTimeout++;
         }
        if (iTimeout >= 4000000L) // we timed out, restart
        {
          bDone = 1;
          continue;
        }
        iLen = (int)(rxhead - rxtail) & 0xff;
           if (iLen >= 67) // a data packet (could be multiple)
           { // byte 0 = page number, byte 1 = segment (0-3)
             u8Page = rxbuf[rxtail++];
             u8Segment = rxbuf[rxtail++];
             if (u8Page != iPage) // wrong page, ignore it
             {
               rxtail += 65;
               continue;
             }
             if (u8Page == iPage && u8Segment < 4) // valid sub-packet?
             {
               j = 64;
               i = u8Segment * 64;
               while (j)
               {
                 buff[i++] = rxbuf[rxtail++];
                 j--;
               }
               sum = 0; // calculate check sum
               // With RF CRC checking enabled, we don't really need to do this, but just in case
               i = u8Segment * 64;
               for (j=0; j<64; j++)
               {
                 sum += buff[i+j];
               }
               i = rxbuf[rxtail++]; // checksum sent by XE hub
               if (i == sum) // good checksum?
               {
                  u8Bits |= (1<<u8Segment); // mark this segment as good
               }
             }
             if (u8Bits == 0xf) // we've got a page (all 4 segments arrived)
             {
               u8Bits = 0; // reset flags
               // Show progress on display
               i = '#';
               SRXEWriteString(cursor_x*12,cursor_y*16, (const char *)&i, -1);
               cursor_x++;
               if (cursor_x == 32)
               {
                 cursor_y++;
                 cursor_x=0;
               }
               // send ack
               txbuf[0] = 0x10; // STK_OK
               txlen = 1;
               rfWrite();
               // Need to wait for transmission to finish or shutting off ints
               // might leave our busy flag set
               while (txbusy != 0)
               {
                 iTimeout++;
               }
               length.word = 256;
               address.byte[0] = 0;
               address.byte[1] = iPage; // 256 bytes
               RAMPZ = 0; // 16th address bit (limits us to writing 64k of flash)
               WriteFlashPage();
               sei(); // re-enable interrupts
               iPage++;
             } // complete page received
           } // data block
           else if (iLen == 2) // end of data
           {
              if (rxbuf[rxtail] == 0x51 && rxbuf[rxtail+1] == 0) // STK_LEAVE_PROGMODE
                bDone = 1;
           }
      } // while !bDone
      if (iTimeout >= 4000000L) // we timed out, restart
      {
        SRXEWriteString(0,16,"Timed out!",-10);
      }
      else
      {
        app_start(); // hopefully we got it
      }
    }
  }
  else // user wants serial0 or serial1
  {
    bootuart = bDevice - 1; // 1 == serial0, 2 = serial1
  }

	/* initialize UART(s) depending on CPU defined */
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega128RFA1__)
  if(bootuart == 1) {
		UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
		UBRR0H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
		UCSR0A = 0x00;
		UCSR0C = 0x06;
		UCSR0B = _BV(TXEN0)|_BV(RXEN0);
	}
	if(bootuart == 2) {
		UBRR1L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
		UBRR1H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
		UCSR1A = 0x00;
		UCSR1C = 0x06;
		UCSR1B = _BV(TXEN1)|_BV(RXEN1);
	}
#elif defined __AVR_ATmega163__
	UBRR = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRRHI = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
	UCSRA = 0x00;
	UCSRB = _BV(TXEN)|_BV(RXEN);	
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

#ifdef DOUBLE_SPEED
	UCSR0A = (1<<U2X0); //Double speed mode USART0
	UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*8L)-1);
	UBRR0H = (F_CPU/(BAUD_RATE*8L)-1) >> 8;
#else
	UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRR0H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
#endif

	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);

	/* Enable internal pull-up resistor on pin D0 (RX), in order
	to supress line noise that prevents the bootloader from
	timing out (DAM: 20070509) */
	DDRD &= ~_BV(PIND0);
	PORTD |= _BV(PIND0);
#elif defined __AVR_ATmega8__
	/* m8 */
	UBRRH = (((F_CPU/BAUD_RATE)/16)-1)>>8; 	// set baud rate
	UBRRL = (((F_CPU/BAUD_RATE)/16)-1);
	UCSRB = (1<<RXEN)|(1<<TXEN);  // enable Rx & Tx
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  // config USART; 8N1
#else
	/* m16,m32,m169,m8515,m8535 */
	UBRRL = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRRH = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
	UCSRA = 0x00;
	UCSRC = 0x06;
	UCSRB = _BV(TXEN)|_BV(RXEN);
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega128RFA1__)
	/* Enable internal pull-up resistor on pin D0 (RX), in order
	to supress line noise that prevents the bootloader from
	timing out (DAM: 20070509) */
	/* feature added to the Arduino Mega --DC: 080930 */
  DDRE &= ~_BV(PINE0);
	PORTE |= _BV(PINE0);
#endif

	/* 20050803: by DojoCorp, this is one of the parts provoking the
		 system to stop listening, cancelled from the original */
	//putch('\0');

	/* forever loop */
	for (;;) {

	/* get character from UART */
	ch = getch();

	/* A bunch of if...else if... gives smaller code than switch...case ! */

	/* Hello is anyone home ? */ 
	if(ch=='0') {
		nothing_response();
	}


	/* Request programmer ID */
	/* Not using PROGMEM string due to boot block in m128 being beyond 64kB boundry  */
	/* Would need to selectively manipulate RAMPZ, and it's only 9 characters anyway so who cares.  */
	else if(ch=='1') {
		if (getch() == ' ') {
			putch(0x14);
			putch('A');
			putch('V');
			putch('R');
			putch(' ');
			putch('I');
			putch('S');
			putch('P');
			putch(0x10);
		} else {
			if (++error_count == MAX_ERROR_COUNT)
				app_start();
		}
	}


	/* AVR ISP/STK500 board commands  DON'T CARE so default nothing_response */
	else if(ch=='@') {
		ch2 = getch();
		if (ch2>0x85) getch();
		nothing_response();
	}


	/* AVR ISP/STK500 board requests */
	else if(ch=='A') {
		ch2 = getch();
		if(ch2==0x80) byte_response(HW_VER);		// Hardware version
		else if(ch2==0x81) byte_response(SW_MAJOR);	// Software major version
		else if(ch2==0x82) byte_response(SW_MINOR);	// Software minor version
		else if(ch2==0x98) byte_response(0x03);		// Unknown but seems to be required by avr studio 3.56
		else byte_response(0x00);				// Covers various unnecessary responses we don't care about
	}


	/* Device Parameters  DON'T CARE, DEVICE IS FIXED  */
	else if(ch=='B') {
      getNch(20);
		nothing_response();
	}


	/* Parallel programming stuff  DON'T CARE  */
	else if(ch=='E') {
		getNch(5);
		nothing_response();
	}


	/* P: Enter programming mode  */
	/* R: Erase device, don't care as we will erase one page at a time anyway.  */
	else if(ch=='P' || ch=='R') {
		nothing_response();
	}


	/* Leave programming mode  */
	else if(ch=='Q') {
		nothing_response();
#ifdef WATCHDOG_MODS
		// autoreset via watchdog (sneaky!)
		WDTCSR = _BV(WDE);
		while (1); // 16 ms
#endif
	}


	/* Set address, little endian. EEPROM in bytes, FLASH in words  */
	/* Perhaps extra address bytes may be added in future to support > 128kB FLASH.  */
	/* This might explain why little endian was used here, big endian used everywhere else.  */
	else if(ch=='U') {
		address.byte[0] = getch();
		address.byte[1] = getch();
		nothing_response();
	}


	/* Universal SPI programming command, disabled.  Would be used for fuses and lock bits.  */
	else if(ch=='V') {
		if (getch() == 0x30) {
			getch();
			ch = getch();
			getch();
			if (ch == 0) {
				byte_response(SIG1);
			} else if (ch == 1) {
				byte_response(SIG2); 
			} else {
				byte_response(SIG3);
			} 
		} else {
			getNch(3);
			byte_response(0x00);
		}
	}


	/* Write memory, length is big endian and is in bytes  */
	else if(ch=='d') {
		length.byte[1] = getch();
		length.byte[0] = getch();
		flags.eeprom = 0;
		if (getch() == 'E') flags.eeprom = 1; // 'F' means flash
		for (w=0;w<length.word;w++) {
			buff[w] = getch();	                        // Store data in buffer, can't keep up with serial data stream whilst programming pages
		}
		if (getch() == ' ') {
			if (flags.eeprom) {		                //Write to EEPROM one byte at a time
				address.word <<= 1;
				for(w=0;w<length.word;w++) {
#if defined(__AVR_ATmega168__)  || defined(__AVR_ATmega328P__)
					while(EECR & (1<<EEPE));
					EEAR = (uint16_t)(void *)address.word;
					EEDR = buff[w];
					EECR |= (1<<EEMPE);
					EECR |= (1<<EEPE);
#else
					eeprom_write_byte((void *)address.word,buff[w]);
#endif
					address.word++;
				}			
			}
			else {					        //Write to FLASH one page at a time
				if (address.byte[1]>127) address_high = 0x01;	//Only possible with m128, m256 will need 3rd address byte. FIXME
				else address_high = 0x00;
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega128RFA1__)
				RAMPZ = address_high;
#endif
				address.word = address.word << 1;	        //address * 2 -> byte location
				/* if ((length.byte[0] & 0x01) == 0x01) length.word++;	//Even up an odd number of bytes */
				if ((length.byte[0] & 0x01)) length.word++;	//Even up an odd number of bytes
        WriteFlashPage();
				/* Should really add a wait for RWW section to be enabled, don't actually need it since we never */
				/* exit the bootloader without a power cycle anyhow */
			}
			putch(0x14);
			putch(0x10);
		} else {
			if (++error_count == MAX_ERROR_COUNT)
				app_start();
		}		
	}


	/* Read memory block mode, length is big endian.  */
	else if(ch=='t') {
		length.byte[1] = getch();
		length.byte[0] = getch();
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega128RFA1__)
		if (address.word>0x7FFF) flags.rampz = 1;		// No go with m256, FIXME
		else flags.rampz = 0;
#endif
		address.word = address.word << 1;	        // address * 2 -> byte location
		if (getch() == 'E') flags.eeprom = 1;
		else flags.eeprom = 0;
		if (getch() == ' ') {		                // Command terminator
			putch(0x14);
			for (w=0;w < length.word;w++) {		        // Can handle odd and even lengths okay
				if (flags.eeprom) {	                        // Byte access EEPROM read
#if defined(__AVR_ATmega168__)  || defined(__AVR_ATmega328P__)
					while(EECR & (1<<EEPE));
					EEAR = (uint16_t)(void *)address.word;
					EECR |= (1<<EERE);
					putch(EEDR);
#else
					putch(eeprom_read_byte((void *)address.word));
#endif
					address.word++;
				}
				else {

					if (!flags.rampz) putch(pgm_read_byte_near(address.word));
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega128RFA1__)
					else putch(pgm_read_byte_far(address.word + 0x10000));
					// Hmmmm, yuck  FIXME when m256 arrvies
#endif
					address.word++;
				}
			}
			putch(0x10);
		}
	}


	/* Get device signature bytes  */
	else if(ch=='u') {
		if (getch() == ' ') {
			putch(0x14);
			putch(SIG1);
			putch(SIG2);
			putch(SIG3);
			putch(0x10);
		} else {
			if (++error_count == MAX_ERROR_COUNT)
				app_start();
		}
	}


	/* Read oscillator calibration byte */
	else if(ch=='v') {
		byte_response(0x00);
	}
	else if (++error_count == MAX_ERROR_COUNT) {
		app_start();
	}
	} /* end of forever loop */
}

void putch(char ch)
{
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega128RFA1__)
	if(bootuart == 1) {
		while (!(UCSR0A & _BV(UDRE0)));
		UDR0 = ch;
	}
	else if (bootuart == 2) {
		while (!(UCSR1A & _BV(UDRE1)));
		UDR1 = ch;
	}
#elif defined(__AVR_ATmega168__)  || defined(__AVR_ATmega328P__)
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = ch;
#else
	/* m8,16,32,169,8515,8535,163 */
	while (!(UCSRA & _BV(UDRE)));
	UDR = ch;
#endif
}


char getch(void)
{
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega128RFA1__)
	uint32_t count = 0;
	if(bootuart == 1) {
		while(!(UCSR0A & _BV(RXC0))) {
			/* 20060803 DojoCorp:: Addon coming from the previous Bootloader*/               
			/* HACKME:: here is a good place to count times*/
			count++;
			if (count > MAX_TIME_COUNT)
				app_start();
			}

			return UDR0;
		}
	else if(bootuart == 2) {
		while(!(UCSR1A & _BV(RXC1))) {
			/* 20060803 DojoCorp:: Addon coming from the previous Bootloader*/               
			/* HACKME:: here is a good place to count times*/
			count++;
			if (count > MAX_TIME_COUNT)
				app_start();
		}

		return UDR1;
	}
	return 0;
#elif defined(__AVR_ATmega168__)  || defined(__AVR_ATmega328P__)
	uint32_t count = 0;
	while(!(UCSR0A & _BV(RXC0))){
		/* 20060803 DojoCorp:: Addon coming from the previous Bootloader*/               
		/* HACKME:: here is a good place to count times*/
		count++;
		if (count > MAX_TIME_COUNT)
			app_start();
	}
	return UDR0;
#else
	/* m8,16,32,169,8515,8535,163 */
	uint32_t count = 0;
	while(!(UCSRA & _BV(RXC))){
		/* 20060803 DojoCorp:: Addon coming from the previous Bootloader*/               
		/* HACKME:: here is a good place to count times*/
		count++;
		if (count > MAX_TIME_COUNT)
			app_start();
	}
	return UDR;
#endif
}


void getNch(uint8_t count)
{
	while(count--)
  {
    getch();
  } // while count
} /* getNch() */


void byte_response(uint8_t val)
{
	if (getch() == ' ') {
		putch(0x14);
		putch(val);
		putch(0x10);
	} else {
		if (++error_count == MAX_ERROR_COUNT)
			app_start();
	}
}


void nothing_response(void)
{
	if (getch() == ' ') {
		putch(0x14);
		putch(0x10);
	} else {
		if (++error_count == MAX_ERROR_COUNT)
			app_start();
	}
}
/* end of file ATmegaBOOT.c */
