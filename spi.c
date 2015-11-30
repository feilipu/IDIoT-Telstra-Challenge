/*
   Copyright (c) 2015 by Phillip Stevens
   SPI library for AVR.

   This file is free software; you can redistribute it and/or modify
   it under the terms of either the GNU General Public License version 2
   or the GNU Lesser General Public License version 2.1, both as
   published by the Free Software Foundation.
*/

// AVR include files.
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "spi.h"

/*******************************************************/

void spiBegin(void)
{
  // Set direction register for SCK and MOSI pin.
  // MISO pin automatically overrides to INPUT.
  // The SS pin MUST be set as OUTPUT, but it can be used as
  // a general purpose output port (it doesn't influence
  // SPI operations).

  // Warning: if the default SS pin ever becomes a LOW INPUT then SPI
  // automatically switches to Slave, so the data direction of
  // the SS pin MUST be kept as OUTPUT.
  // This is true even if an alternative SS pin is also being used.
  // Be warned!

  // Set the MOSI and SCK to low, as is standardised.
  // Also, turn on internal pull-up resistor on MISO, to keep it defined.

  register uint8_t tmp __attribute__ ((unused));

  SPI_PORT_DIR |=  SPI_BIT_SCK | SPI_BIT_MOSI | SPI_BIT_SS;
  SPI_PORT_DIR &= ~SPI_BIT_MISO;

  SPI_PORT |= SPI_BIT_MISO | SPI_BIT_SS;

  // Set the control register to turn on the SPI interface as Master.
  SPCR |= _BV(MSTR) | _BV(SPE);

  // Reading SPI Status Register & SPI Data Register
  // has side effect of zeroing out both
  tmp = SPSR;
  tmp = SPDR;
}

void spiEnd(void)
{

  SPCR &= ~( _BV(MSTR) | _BV(SPE) );
  // Don't bother to tidy up. This function is not likely to be actually used.
}

void spiSetClockDivider(SPI_CLOCK_DIV_t rate)
{
  SPCR = (SPCR & ~SPI_CLOCK_MASK) | ((uint8_t)rate & SPI_CLOCK_MASK);
  SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | (((uint8_t)rate >> 2) & SPI_2XCLOCK_MASK);
}

void spiSetBitOrder(uint8_t bitOrder)
{
  if (bitOrder == SPI_LSBFIRST) {
    SPCR |= _BV(DORD);
  } else {
    SPCR &= ~(_BV(DORD));
  }
}

void spiSetDataMode(SPI_MODE_t mode)
{
  SPCR = (SPCR & ~SPI_MODE_MASK) | (uint8_t) mode;
}


void spiAttachInterrupt(void)
{
  SPCR |= _BV(SPIE);
}

void spiDetachInterrupt(void)
{
  SPCR &= ~_BV(SPIE);
}

/*-----------------------------------------------------------------------*/
/* Select the SPI device                                    */
/*-----------------------------------------------------------------------*/

uint8_t spiSelect(void)	/* 1:Successful, 0:Timeout */
{
  SPI_PORT &= ~SPI_BIT_SS;
  return 1;		// OK /
}


/*-----------------------------------------------------------------------*/
/* Deselect the SPI device                                 */
/*-----------------------------------------------------------------------*/

void spiDeselect(void)
{
  // Pull SS high to Deselect the card.
  SPI_PORT |= SPI_BIT_SS;
}


uint8_t spiTransfer(uint8_t data)
{
  // Make sure you manually pull slave select low to indicate start of transfer.
  // That is NOT done by this function..., because...
  // Some devices need to have their SS held low across multiple transfer calls.
  // Using spiSelect ();

  // If the SPI module has not been enabled yet, then return with nothing.
  if ( !(SPCR & _BV(SPE)) ) return 0;

  // The SPI module is enabled, but it is in slave mode, so we can not
  // transmit the byte. This can happen if SSbar is an input and it went low.
  // We will try to recover by setting the MSTR bit.
  if ( !(SPCR & _BV(MSTR)) )
  {
    SPCR |= _BV(MSTR);
    if ( !(SPCR & _BV(MSTR)) ) return 0;
  }

  SPDR = data; 	// Begin transmission

  /*
     The following NOP introduces a small delay that can prevent the wait
     loop from iterating when running at the maximum speed. This gives
     about 10% more speed, even if it seems counter-intuitive. At lower
     speeds it is unnoticed.
  */
  __asm__ __volatile__("nop");

  while ( !(SPSR & _BV(SPIF)) )
  {
    if ( !(SPCR & _BV(MSTR)) ) return 0;
    // The SPI module has left master mode, so return.
    // Otherwise, this will be an infinite loop.
  }

  return SPDR;

  // Make sure you pull slave select high to indicate end of transfer.
  // That is NOT done by this function.
  // Using spiDeselect (SS_pin);
}

/*
   In testing with a Freetronics EtherMega driving an SD card
   the system achieved the following results.

   Single byte transfer MOSI 3.750uS MISO 3.6250us
   Multi- byte transfer MOSI 1.333uS MISO 1.3750uS

   Performance increase MOSI 2.8x    MISO 2.64x

   Worth doing if you can!

*/

uint8_t spiMultiByteTx(const uint8_t *data, const uint16_t length)
{
  uint16_t index = 0;
  uint8_t TxByte;

  // Make sure you manually pull slave select low to indicate start of transfer.
  // That is NOT done by this function..., because...
  // Some devices need to have their SS held low across multiple transfer calls.
  // Using spiSelect();

  // If the SPI module has not been enabled yet, then return with nothing.
  if ( !(SPCR & _BV(SPE)) ) return 0;

  // The SPI module is enabled, but it is in slave mode, so we can not
  // transmit the byte.  This can happen if SSbar is an input and it went low.
  // We will try to recover by setting the MSTR bit.
  // Check this once only at the start. Assume that things don't change.
  if ( !(SPCR & _BV(MSTR)) )
  {
    SPCR |= _BV(MSTR);
    if ( !(SPCR & _BV(MSTR)) ) return 0;
  }

  SPDR = data[ index++ ]; // Begin transmission

  while ( index < length )
  {
    TxByte = data[ index++ ]; // pre-load the byte to be transmitted
    while ( !(SPSR & _BV(SPIF)) );
    SPDR = TxByte; // Continue transmission
  }
  while ( !(SPSR & _BV(SPIF)) );
  return 1;

  // Make sure you pull slave select high to indicate end of transfer.
  // That is NOT done by this function.
  // Using spiDeselect (SS_pin);
}

uint8_t spiMultiByteTx_P(const uint8_t *data, const uint16_t length)
{
  uint16_t index = 0;
  uint8_t TxByte;

  // Transfers PROGMEM data.
  // Ensure the data is below 128 kByte limit on ATmega2560.

  // Make sure you manually pull slave select low to indicate start of transfer.
  // That is NOT done by this function..., because...
  // Some devices need to have their SS held low across multiple transfer calls.
  // Using spiSelect();

  // If the SPI module has not been enabled yet, then return with nothing.
  if ( !(SPCR & _BV(SPE)) ) return 0;

  // The SPI module is enabled, but it is in slave mode, so we can not
  // transmit the byte.  This can happen if SSbar is an input and it went low.
  // We will try to recover by setting the MSTR bit.
  // Check this once only at the start. Assume that things don't change.
  if ( !(SPCR & _BV(MSTR)) )
  {
    SPCR |= _BV(MSTR);
    if ( !(SPCR & _BV(MSTR)) ) return 0;
  }

  SPDR = pgm_read_byte( &data[ index++ ]); // Begin transmission
  while ( index < length )
  {
    TxByte = pgm_read_byte( &data[ index++ ]); // pre-load the byte to be transmitted
    while ( !(SPSR & _BV(SPIF)) );
    SPDR = TxByte; // Continue transmission
  }
  while ( !(SPSR & _BV(SPIF)) );
  return 1;

  // Make sure you pull slave select high to indicate end of transfer.
  // That is NOT done by this function.
  // Using spiDeselect (SS_pin);
}

uint8_t spiMultiByteRx(uint8_t *data, const uint16_t length)
{
  uint16_t index = 0;
  uint8_t RxByte;

  // Make sure you manually pull slave select low to indicate start of transfer.
  // That is NOT done by this function..., because...
  // Some devices need to have their SS held low across multiple transfer calls.
  // Using spiSelect();

  // If the SPI module has not been enabled yet, then return with nothing.
  if ( !(SPCR & _BV(SPE)) ) return 0;

  // The SPI module is enabled, but it is in slave mode, so we can not
  // transmit the byte.  This can happen if SSbar is an input and it went low.
  // We will try to recover by setting the MSTR bit.
  // Check this once only at the start. Assume that things don't change.
  if ( !(SPCR & _BV(MSTR)) )
  {
    SPCR |= _BV(MSTR);
    if ( !(SPCR & _BV(MSTR)) ) return 0;
  }

  SPDR = 0xFF; // Begin dummy transmission
  while ( index < length - 1 )
  {
    while ( !(SPSR & _BV(SPIF)) );
    RxByte = SPDR; // copy received byte
    SPDR = 0xFF;   // Continue dummy transmission
    data[ index++ ] = RxByte;
  }
  while ( !(SPSR & _BV(SPIF)) );

  data[ index ] = SPDR;	// store the last byte that was read
  return 1;

  // Make sure you pull slave select high to indicate end of transfer.
  // That is NOT done by this function.
  // Using spiDeselect (SS_pin);
}

uint8_t spiMultiByteTransfer(uint8_t *data, const uint16_t length)
{
  uint16_t index = 0;
  uint8_t TxByte, RxByte;

  // Make sure you manually pull slave select low to indicate start of transfer.
  // That is NOT done by this function..., because...
  // Some devices need to have their SS held low across multiple transfer calls.
  // Using spiSelect();

  // If the SPI module has not been enabled yet, then return with nothing.
  if ( !(SPCR & _BV(SPE)) ) return 0;

  // The SPI module is enabled, but it is in slave mode, so we can not
  // transmit the byte.  This can happen if SSbar is an input and it went low.
  // We will try to recover by setting the MSTR bit.
  // Check this once only at the start. Assume that things don't change.
  if ( !(SPCR & _BV(MSTR)) )
  {
    SPCR |= _BV(MSTR);
    if ( !(SPCR & _BV(MSTR)) ) return 0;
  }

  SPDR = data[ index ]; // Begin first byte transfer
  while (index < length - 1)
  {
    TxByte = data[ index + 1 ]; 	// pre-load the next byte to be transmitted, while transferring
    while ( !(SPSR & _BV(SPIF)) ); 	// wait for the transfer success
    RxByte = SPDR; 					// copy received byte
    SPDR = TxByte; 					// Continue transmission
    data[ index++ ] = RxByte;		// store the byte that was read, while transferring
  }

  while ( !(SPSR & _BV(SPIF)) );

  data[ index ] = SPDR;	// store the last byte that was read
  return 1;

  // Make sure you pull slave select high to indicate end of transfer.
  // That is NOT done by this function.
  // Using spiDeselect (SS_pin);
}
