#include <SmartResponseXE.h>
//
// Wireless uploader
// 
// Listens on Serial0 or Serial1 for commands coming from the skt500 protocol in avrdude
// to upload a new program to the SMART Response XE over 802.15.4 (ZigBee) wireless
//
// Written by Larry Bank
// Project started 8/26/2018
// Copyright (c) 2018 BitBank Software, Inc.
// bitbank@pobox.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

//
// STK500 Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20
#define STK_GET_SYNC 0x30
#define STK_GET_PARAM 0x41
#define STK_SET_DEVICE 0x42
#define STK_SET_DEVICE_EXT 0x45
#define STK_ENTER_PROGMODE 0x50
#define STK_LEAVE_PROGMODE 0x51
#define STK_LOAD_ADDRESS 0x55
#define STK_UNIVERSAL 0x56
#define STK_PROGRAM_PAGE 0x64
#define STK_READ_PAGE 0x74
#define STK_READ_SIGN 0x75
// The states describing the passing of messages from the PC to the wireless receiver
// Most of the STK500 operations are atomic except for the address used to read and write
// the FLASH memory of the target device.
enum
{
  STATE_IDLE = 0,
  STATE_GET_SYNC,
  STATE_GET_PARAM,
  STATE_GET_PARAM_END,
  STATE_GET_SIGNATURE,
  STATE_ENTER_PROGMODE,
  STATE_ADDRESS,
  STATE_PROGRAMMING,
  STATE_READING,
  STATE_GET_USELESS,
  STATE_GET_IGNORE
};
// number of data blocks for displaying status on the LCD
int iSent=0, iReceived=0;
int bTriedRead; // indicates we've received the whole sketch and avrdude is at the verify stage

// variables used for wireless activity
volatile uint8_t rxhead, rxtail, rssiRaw, txbusy;
volatile uint16_t txlen;
uint8_t txbuf[284]; // allows for maximum STK500 packet size + a few extra bytes
uint8_t rxbuf[256];

// Keeps track of the SPI FLASH erase address since it is on 4K byte blocks while
// the writing to FLASH is with 256-byte blocks
volatile uint32_t ulEraseAddr; // last sector erased

// Change this to Serial1 if that's how your XE "hub" unit is set up
#define SERIAL Serial


// Initialize the RFA1's low-power 2.4GHz transciever.
// Sets up the state machine, and gets the radio into
// the RX_ON state. Interrupts are enabled for RX
// begin and end, as well as TX end.
uint8_t rfBegin(uint8_t channel)
{
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
  TRX_STATE = (TRX_STATE & 0xE0) | TRX_OFF;  // Set to TRX_OFF state
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
  uint8_t bLen;
  uint8_t *s = &txbuf[0];
  
  if (txlen == 0) return; // nothing to do
  // Transceiver State Control Register -- TRX_STATE
  // This regiseter controls the states of the radio.
  // Set to the PLL_ON state - this state begins the TX.
  TRX_STATE = (TRX_STATE & 0xE0) | PLL_ON;  // Set to TX start state
  while(!(TRX_STATUS & PLL_ON))
    ;  // Wait for PLL to lock

// Data may be more than 125 bytes, so we need to break it up
  while (txlen > 0)
  {
     while (txbusy) // wait for last packet to finish transmitting
     { };
     
     if (txlen > 125)
        bLen = 125;
     else
        bLen = txlen;
  // Start of frame buffer - TRXFBST
  // This is the first byte of the (up to 128 byte) frame. It should contain
  // the length of the transmission.
  TRXFBST = bLen + 2; // 16-bit CRC + data to send
  memcpy((void *)(&TRXFBST+1), s, bLen);
  s += bLen;
  txlen -= (uint16_t)bLen;
  // Transceiver Pin Register -- TRXPR.
  // From the PLL_ON state, setting SLPTR high will initiate the TX.
  TRXPR |= (1<<SLPTR);   // SLPTR high
  TRXPR &= ~(1<<SLPTR);  // SLPTR low
  txbusy = 1; // set busy flag which will get reset by the TRX24_TX_END interrupt
  } // while transmitting packets
  // After sending the bytes, set the radio back into the RX waiting state.
  TRX_STATE = (TRX_STATE & 0xE0) | RX_ON;
} /* rfWrite() */

// This interrupt is called when radio TX is complete. We'll just
ISR(TRX24_TX_END_vect)
{
  txbusy = 0; // indicate the packet has finished transmitting
}

// This interrupt is called the moment data is received by the radio.
// We'll use it to gather information about RSSI -- signal strength --
ISR(TRX24_RX_START_vect)
{
  rssiRaw = PHY_RSSI;  // Read in the received signal strength
}

//
// This interrupt is called at the end of data receipt.
// It will be placed in a circular buffer and transmitted to the local
// serial port as soon as it's received
//
ISR(TRX24_RX_END_vect)
{
  uint8_t length, *s;
  // The received signal must be above a certain threshold.
  if (rssiRaw & RX_CRC_VALID)
  {
    // The length of the message will be the first byte received.
    length = TST_RX_LENGTH;
    // The remaining bytes will be our received data.
    s = (uint8_t *)(void*)&TRXFBST;
    while (length > 2) // ignore 16-bit CRC at the end of the received data
    { // copy the bytes into our circular buffer
       rxbuf[rxhead++] = *s++; // 8-bit pointer will automatically wrap
       length--;
       // if the head wraps around and passes the tail, something else is very broken
    }
  }
}
//
// At the start of a sketch upload, reset the display and variables
//
void PrepGUI(void)
{
  SRXEFill(0);
  bTriedRead = 0;
  ulEraseAddr = 0xffffffff; // reset last sector erased
  SRXEWriteString(0,0,"** Larry's Wireless Uploader **",FONT_MEDIUM,3,0);
  SRXEWriteString(0,32,"Blocks written: 0     ",FONT_MEDIUM,3,0);
  SRXEWriteString(0,64,"Blocks read   : 0     ",FONT_MEDIUM,3,0);
  iSent = iReceived = 0;
} /* PrepGUI() */
//
// Display the number of FLASH pages written and read by avrdude
//
void UpdateGUI(int bWritten)
{
char szTemp[32];
int y, i;

   if (bWritten)
   {
      i = iSent;
      y = 32;
   }
   else
   {
      i = iReceived;
      y = 64;
   }
   sprintf(szTemp, "%d",i);
   SRXEWriteString(16*12, y, szTemp, FONT_MEDIUM,3,0);
} /* UpdateGUI() */

//
// Transmit the program over wireless to the receiving XE
//
#define MAX_RF_WAIT 300000L
#define MAX_RETRIES 10
void SendOverRF(void)
{
int iPage;
uint32_t ulAddr;
volatile uint32_t iTimeout;
int iRetries;
uint8_t c, sum, j, k;
uint8_t ucTemp[256]; // to read a page from the SPI flash

  iPage = 0;
  iRetries = 0;
  while (iPage < iSent && iRetries < MAX_RETRIES) // loop through all of the received pages
  {
     ulAddr = (uint32_t)iPage;
     ulAddr <<= 8L;
     SRXEFlashRead(ulAddr, ucTemp, 256);
     // break each page into 4 blocks of 64 bytes + some header info
     // send the 4 blocks, then check that they arrived successfully
     for (j=0; j<4; j++)
     {
        txbuf[0] = (uint8_t)iPage;
        txbuf[1] = j; // block number
        memcpy(&txbuf[2], &ucTemp[j*64], 64);
        // compute checksum
        sum = 0;
        for (k=0; k<64; k++)
           sum += txbuf[2+k];
        txbuf[66] = sum;
        txlen = 67;
        rfWrite();
     }
     iTimeout = 0; // wait for response
     while (rxtail == rxhead && iTimeout < MAX_RF_WAIT)
     {
       iTimeout++;
     }
     if (iTimeout >= MAX_RF_WAIT) // something went wrong, we should have gotten an answer
     {
        iRetries++;
        sprintf((char *)ucTemp,"Page %d, Retry %d", iPage, iRetries);
        SRXEWriteString(0,120,(char *)ucTemp,FONT_MEDIUM,3,0);
        continue; // send it again
     }
     c = rxbuf[rxtail++]; // see if all is ok
     if (c == STK_OK) // we're good, continue
     {
        iPage++;
        iRetries = 0;
        // displaying this string also allows more time for XE to write to flash
        sprintf((char *)ucTemp, "Pages sent: %d  ", iPage);
        SRXEWriteString(0,120,(char *)ucTemp, FONT_MEDIUM,3,0);
        delay(60); // allow time for page to be written to flash            
        continue;
     }
     iRetries++;
  } // while sending
  if (iPage == iSent) // we're done
  { // tell receiver to restart
     txbuf[0] = STK_LEAVE_PROGMODE;
     txbuf[1] = 0x00;
     txlen = 2;
     rfWrite();
     SRXEWriteString(0,120,"Success!          ", FONT_MEDIUM,3,0);
  }
  else
  {  // Since we have the sketch in SPI FLASH, we can resend it without asking the PC
     SRXEWriteString(0,120,"Failed! (r)etry / (s)tart over", FONT_MEDIUM,3,0);
  }
} /* SendOverRF() */

void setup() {
  SRXEInit(0xe7, 0xd6, 0xa2); // Initialize the LCD
  PrepGUI(); // Draw our empty status text
  rfBegin(26); // default to 802.15.4 channel 26 to upload code to the custom bootloader
  delay(3000); // wait a few seconds since the same serial port is used to update this device
  SERIAL.begin(57600); // default Arduino IDE upload baud rate
} // setup

void loop() {
uint8_t state = STATE_IDLE; // current state machine state of sender
uint16_t msg_len;
uint8_t c, i, useless_count, param, response;
uint32_t ulAddress; // address to read or write from flash
uint8_t bUniversal; // indicates a specific response for a command we really ignore
char szTemp[256];
unsigned long timeout = 0;

 while (1)
 {
      if (bTriedRead)
      {
         c = SRXEGetKey();
         if (c  == 'r') // retry sending to remote XE
            SendOverRF();
         else if (c == 's') // start over
         {
            PrepGUI();
         }
      }
      if (SERIAL.available() > 0)
      {
          c = SERIAL.read();
          switch (state)
          {
             case STATE_ADDRESS: // capture the flash read/write address
                txbuf[txlen++] = c; // gather up all the bytes to write at once
                if (txlen == 4 && c == CRC_EOP) // got it, send ack
                {
                   SERIAL.write(STK_INSYNC); // ack
                   SERIAL.write(STK_OK);
                   txlen = 0;
                   ulAddress = ((uint32_t)txbuf[1]<<1L) + ((uint32_t)txbuf[2]<<9L); // sent as little endian word addr
                   state = STATE_IDLE; // go back to waiting for more data
                }
                break;
             case STATE_READING: // read page request
                txbuf[txlen++] = c; // gather up all the bytes to write at once
                if (txlen == 5 && c == CRC_EOP) // pass to remote device
                {
                   iReceived++;
                   UpdateGUI(0);
                   state = STATE_IDLE; // go back to waiting for more data
                   txlen = 0;
                   SERIAL.write(STK_INSYNC); // ack request
                   SRXEFlashRead(ulAddress, (uint8_t *)szTemp, 256);
                   SERIAL.write(szTemp, 256);
                   SERIAL.write(STK_OK);
                }
                break;
             case STATE_PROGRAMMING: // data writing to flash on remote device
                txbuf[txlen++] = c; // gather up all the bytes to write at once
                if (txlen == 5+256 && c == CRC_EOP) // a whole packet was received (256 bytes)
                {
                   int i;
                   iSent++;
                   UpdateGUI(1);
                   state = STATE_IDLE; // go back to waiting for more data
                   txlen = 0;
                   SERIAL.write(STK_INSYNC); // start by sending ack
                   // write data to SPI flash
                   if ((ulAddress & 0xfffff000L) != ulEraseAddr) // need to erase this sector
                   {
                      SRXEFlashEraseSector(ulAddress, 1); // wait for it to complete
                      ulEraseAddr = ulAddress; // new erase address
                   }
                   i = 0; // wait until it succeeds (might be busy from a previous write)
                   while (i == 0)
                   {
                     i = SRXEFlashWritePage(ulAddress, &txbuf[4]); // write 256 bytes to flash
                   }
                   SERIAL.write(STK_OK); // finish by sending ok
                }
                break;
             case STATE_IDLE:
                if (c == STK_LEAVE_PROGMODE) // we're done, tell remote device to restart
                {
                   delay(2); // give time for second byte to arrive (it may have already)
                   txbuf[txlen++] = c;
                   txbuf[txlen++] = SERIAL.read();
                   if (bTriedRead) // writing has succeeded, so we can send the data
                   {
                      SRXEWriteString(0,96,"Ready! Press 4 on remote XE",FONT_MEDIUM,3,0);
                      SendOverRF();
                   }
                }
                if (c == STK_LOAD_ADDRESS) // programming has begun
                {
                   txbuf[txlen++] = c; // pass through to wireless
                   state = STATE_ADDRESS;
                }
                else if (c == STK_PROGRAM_PAGE) // data block to write
                {
                   txbuf[txlen++] = c;
                   state = STATE_PROGRAMMING;
                }
                else if (c == STK_READ_PAGE)
                {
                   txbuf[txlen++] = c;
                   state = STATE_READING;
                   bTriedRead = 1; // indicate that writing has succeeded
                }
                else if (c == STK_GET_SYNC) // requesting sync
                   state = STATE_GET_SYNC;
                else if (c == STK_GET_PARAM)
                   state = STATE_GET_PARAM;
                else if (c == STK_SET_DEVICE)
                {
                   state = STATE_GET_IGNORE;
                   bUniversal = 0;
                   useless_count = 20;
                }
                else if (c == STK_SET_DEVICE_EXT)
                {
                   state = STATE_GET_IGNORE;
                   bUniversal = 0;
                   useless_count = 5;
                }
                else if (c == STK_UNIVERSAL)
                {
                   state = STATE_GET_IGNORE;
                   bUniversal = 1;
                   useless_count = 4;
                }
                else if (c == STK_READ_SIGN)
                {
                   state = STATE_GET_SIGNATURE;
                   PrepGUI(); // a new session is starting
                }
                else if (c == STK_ENTER_PROGMODE)
                   state = STATE_ENTER_PROGMODE;
                break;
             case STATE_ENTER_PROGMODE:
                if (c == CRC_EOP) // send ok
                {
                   SERIAL.write(STK_INSYNC);
                   SERIAL.write(STK_OK);
                   state = STATE_IDLE;                  
                }
                break;
             case STATE_GET_SIGNATURE:
                if (c == CRC_EOP) // send device signature
                {
                   SERIAL.write(STK_INSYNC);
                   SERIAL.write(0x1e); // Atmel
                   SERIAL.write(0xa7); // ATmega128rfa1 byte 1
                   SERIAL.write(0x01); // ATmega128rfa1 byte 2
                   SERIAL.write(STK_OK);
                   state = STATE_IDLE;                  
                }
                break;
             case STATE_GET_IGNORE: // read bytes and throw them away
                if (useless_count)
                {
                   useless_count--;
                }
                else // final character needs to be a space
                   if (c == 0x20) // send good/useless response
                   {
                      SERIAL.write(STK_INSYNC);
                      if (bUniversal) // fake EEPROM byte read for STK_UNIVERSAL command
                         SERIAL.write(0x00);
                      SERIAL.write(STK_OK);
                      state = STATE_IDLE;
                   }
                   else
                   {
                      // some kind of error
                   }
                break;
             case STATE_GET_SYNC:
                if (c == CRC_EOP || c == 0xa0) // corrupted CRC_EOP byte
                {
                   SERIAL.write(STK_INSYNC);
                   SERIAL.write(STK_OK);
                   state = STATE_IDLE;
                }
                break;
             case STATE_GET_PARAM:
                param = c;
                state = STATE_GET_PARAM_END; // need 1 more byte to finish
                break;
             case STATE_GET_PARAM_END:
                response = 0x00; // assume nothing matches
                if (c == 0x20) // valid end of request
                {
                  if (param == 0x80) // Hardware version
                     response = 0x02;
                  else if (param == 0x81) // SW Major
                     response = 0x01;
                  else if (param == 0x82) // SW Minor
                     response = 0x10;
                  else if (param == 0x98) // unknown
                     response = 0x03;
                  // send "byte response"
                  SERIAL.write(STK_INSYNC);
                  SERIAL.write(response);
                  SERIAL.write(STK_OK);
                }
                else // some kind of error
                {
                  
                }
                state = STATE_IDLE;
                break;
          } // switch on state
      } // serial available
      else
      {
        timeout++;
        if (timeout > 1000000L) // about 5 seconds 
        {
           state = STATE_IDLE; // reset to idle state
           txlen = 0;
//           SRXEWriteString(0,120,"Timeout!",FONT_MEDIUM,3,0);
           timeout = 0;
        }
      }
       while (rxtail != rxhead) // data was received and must be passed to the local serial port
       {
         SERIAL.write(rxbuf[rxtail++]);
       }
 } // while
} // loop
