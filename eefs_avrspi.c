/*
   eefs_avrspi.c

    Created on: 04/01/2015
        Author: phillip
*/
/*
   Includes
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <util/delay.h>

/* SPI interface include file. */
#include "spi.h"

#include "common_types.h"

#include "eefs_avrspi.h"


int8_t eefs_avrspi_begin(void)
{
  spiSetDataMode(SPI_MODE0);		// Enable SPI function in mode 0
  spiSetClockDivider(SPI_CLOCK_DIV2);	// SPI at maximum speed
  spiBegin();

#if defined (RAM0)
  RAM_DDR |= _BV(RAM0_SS);		// Set the RAM0 SS to Output
  RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
#endif

#if defined (RAM1)
  RAM_DDR |= _BV(RAM1_SS); 	// Set the RAM1 SS to Output
  RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
#endif

  return EEFS_SUCCESS;
}


int8_t eefs_avrspi_read(uint8_t * Dest, const uint_farptr_t Src, uint32_t Length)
{
  uint_farptr_t index = 0;
  const addr_farptr_t localSrc = (addr_farptr_t)Src;
  uint8_t TxRxByte;
  int8_t ReturnCode = EEFS_SUCCESS;

  // Length is 0 so just return
  if ( Length == 0 ) return EEFS_SUCCESS;

  // If the SPI module has not been enabled yet, then return with nothing.
  if ( !(SPCR & _BV(SPE)) ) return EEFS_BUS_IS_NOT_READY;

  // The SPI module is enabled, but it is in slave mode, so we can not
  // transmit the byte. This can happen if SSbar is an input and it went low.
  // We will try to recover by setting the MSTR bit.
  if ( !(SPCR & _BV(MSTR)) )
  {
    SPCR |= _BV(MSTR);
    if ( !(SPCR & _BV(MSTR)) ) return EEFS_BUS_IS_NOT_READY;
  }


  switch ( localSrc.bAddr.device_byte )
  {
    case RAM0_DEVICE_ADDR:
#if defined (RAM0)

#if defined (RAM0_BUSY_MODE)
      TxRxByte = MODE_BUSY;
      while ((TxRxByte & MODE_BUSY) != 0x00)
      {
        RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low
        SPDR = RDSR; 				// Send read the Status Register
        while ( !(SPSR & _BV(SPIF)) );
        SPDR = 0xff; 				// Send a dummy byte to receive the Status Register
        while ( !(SPSR & _BV(SPIF)) );
        TxRxByte = SPDR;
        RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
        if ((TxRxByte & MODE_BUSY) != 0x00)
          _delay_ms(1);
      }
#endif

      RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low

      // Transmit the mode command
      SPDR = READ;

#if (RAM0_ADDR_BITS > 16)
      // Prepare & transmit the Address High Byte if warranted
      TxRxByte = localSrc.bAddr.high_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;
#endif

      // Prepare & transmit the Address Mid Byte
      TxRxByte = localSrc.bAddr.mid_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;

      // Prepare & transmit the Address Low Byte
      TxRxByte = localSrc.bAddr.low_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;

      while ( !(SPSR & _BV(SPIF)) );
      SPDR = 0xFF; // Begin dummy transmission
      while ( index < Length - 1 )
      {
        while ( !(SPSR & _BV(SPIF)) );
        TxRxByte = SPDR; // copy received byte
        SPDR = 0xFF;   // Continue dummy transmission
        Dest[ index++ ] = TxRxByte;
      }
      while ( !(SPSR & _BV(SPIF)) );
      Dest[ index ] = SPDR;	// store the last byte that was read

      RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High

      ReturnCode = EEFS_SUCCESS;
#else
      ReturnCode = EEFS_NO_SUCH_DEVICE;
#endif
      break;

    case RAM1_DEVICE_ADDR:
#if defined (RAM1)

#if defined (RAM1_BUSY_MODE)
      TxRxByte = MODE_BUSY;
      while ((TxRxByte & MODE_BUSY) != 0x00)
      {
        RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low
        SPDR = RDSR; 				// Send read the Status Register
        while ( !(SPSR & _BV(SPIF)) );
        SPDR = 0xff; 				// Send a dummy byte to receive the Status Register
        while ( !(SPSR & _BV(SPIF)) );
        TxRxByte = SPDR;
        RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
        if ((TxRxByte & MODE_BUSY) != 0x00)
          _delay_ms(1);
      }
#endif

      RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low

      // Transmit the mode command
      SPDR = READ;

#if (RAM1_ADDR_BITS > 16)
      // Prepare & transmit the Address High Byte if warranted
      TxRxByte = localSrc.bAddr.high_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;
#endif

      // Prepare & transmit the Address Mid Byte
      TxRxByte = localSrc.bAddr.mid_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;

      // Prepare & transmit the Address Low Byte
      TxRxByte = localSrc.bAddr.low_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;

      while ( !(SPSR & _BV(SPIF)) );
      SPDR = 0xFF; // Begin dummy transmission
      while ( index < Length - 1 )
      {
        while ( !(SPSR & _BV(SPIF)) );
        TxRxByte = SPDR; // copy received byte
        SPDR = 0xFF;   // Continue dummy transmission
        Dest[ index++ ] = TxRxByte;
      }
      while ( !(SPSR & _BV(SPIF)) );
      Dest[ index ] = SPDR;	// store the last byte that was read

      RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High

      ReturnCode = EEFS_SUCCESS;
#else
      ReturnCode = EEFS_NO_SUCH_DEVICE;
#endif
      break;

    default:
      ReturnCode = EEFS_NO_SUCH_DEVICE;
      break;
  }

  return ReturnCode;
}


int8_t eefs_avrspi_write(const uint_farptr_t Dest, const uint8_t * Src, uint32_t Length)
{
  uint_farptr_t index = 0;
  const addr_farptr_t localDest = (addr_farptr_t)Dest;
  addr_farptr_t writeDest;
  
  uint8_t TxRxByte;
  int8_t ReturnCode = EEFS_SUCCESS;

  // Length is 0 so just return
  if ( Length == 0 ) return EEFS_SUCCESS;

  // If the SPI module has not been enabled yet, then return.
  if ( !(SPCR & _BV(SPE)) ) return EEFS_BUS_IS_NOT_READY;

  // The SPI module is enabled, but it is in slave mode, so we can not
  // transmit the byte. This can happen if SSbar is an input and it went low.
  // We will try to recover by setting the MSTR bit.
  if ( !(SPCR & _BV(MSTR)) )
  {
    SPCR |= _BV(MSTR);
    if ( !(SPCR & _BV(MSTR)) ) return EEFS_BUS_IS_NOT_READY;
  }

  switch ( localDest.bAddr.device_byte )
  {
    case RAM0_DEVICE_ADDR:
#if defined (RAM0)

#if defined (RAM0_BUSY_MODE)
      TxRxByte = MODE_BUSY;
      while ((TxRxByte & MODE_BUSY) != 0x00)
      {
        RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low
        SPDR = RDSR; 				// Send read the Status Register
        while ( !(SPSR & _BV(SPIF)) );
        SPDR = 0xff; 				// Send a dummy byte to receive the Status Register
        while ( !(SPSR & _BV(SPIF)) );
        TxRxByte = SPDR;
        RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
        if ((TxRxByte & MODE_BUSY) != 0x00)
          _delay_ms(2);
      }
#endif


#if defined (RAM0_WRITE_LATCH)
      RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low
      // Transmit the Write Latch command
      SPDR = WREN;
      while ( !(SPSR & _BV(SPIF)) );
      RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
#endif

      RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low

      // Transmit the mode command
      SPDR = WRITE;

#if (RAM0_ADDR_BITS > 16)
      // Prepare & transmit the Address High Byte if warranted
      TxRxByte = localDest.bAddr.high_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;
#endif

      // Prepare & transmit the Address Mid Byte
      TxRxByte = localDest.bAddr.mid_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;

      // Prepare & transmit the Address Low Byte
      TxRxByte = localDest.bAddr.low_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;

      while ( !(SPSR & _BV(SPIF)) );
      SPDR = Src[ index++ ]; // Begin transmission from the Src to the Dest
      while ( index < Length )
      {

#if defined (RAM0_PAGE_ACCESS)
        // New write cycle must be initiated if writing across page boundaries
        if ((localDest.lAddr + index) % RAM0_PAGE_BYTES == 0)
        {
          while ( !(SPSR & _BV(SPIF)) );
          RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High

          writeDest.lAddr = localDest.lAddr + index;

#if defined (RAM0_BUSY_MODE)
          TxRxByte = MODE_BUSY;
          while ((TxRxByte & MODE_BUSY) != 0x00)
          {
            RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low
            SPDR = RDSR; 				// Send read the Status Register
            while ( !(SPSR & _BV(SPIF)) );
            SPDR = 0xff; 				// Send a dummy byte to receive the Status Register
            while ( !(SPSR & _BV(SPIF)) );
            TxRxByte = SPDR;
            RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
            if ((TxRxByte & MODE_BUSY) != 0x00)
              _delay_ms(2);
          }
#endif

#if defined (RAM0_WRITE_LATCH)
          RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low
          // Transmit the Write Latch command
          SPDR = WREN;
          while ( !(SPSR & _BV(SPIF)) );
          RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
#endif

          RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low

          // Transmit the mode command
          SPDR = WRITE;

#if (RAM0_ADDR_BITS > 16)
          // Prepare & transmit the Address High Byte if warranted
          TxRxByte = writeDest.bAddr.high_byte;
          while ( !(SPSR & _BV(SPIF)) );
          SPDR = TxRxByte;
#endif

          // Prepare & transmit the Address Mid Byte
          TxRxByte = writeDest.bAddr.mid_byte;
          while ( !(SPSR & _BV(SPIF)) );
          SPDR = TxRxByte;

          // Prepare & transmit the Address Low Byte
          TxRxByte = writeDest.bAddr.low_byte;
          while ( !(SPSR & _BV(SPIF)) );
          SPDR = TxRxByte;
        }
#endif

        TxRxByte = Src[ index++ ]; // pre-load the byte to be transmitted
        while ( !(SPSR & _BV(SPIF)) );
        SPDR = TxRxByte; // Continue transmission
      }
      while ( !(SPSR & _BV(SPIF)) );

      RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High

      ReturnCode = EEFS_SUCCESS;
#else
      ReturnCode = EEFS_NO_SUCH_DEVICE;
#endif

      break;

    case RAM1_DEVICE_ADDR:
#if defined (RAM1)

#if defined (RAM1_BUSY_MODE)
      TxRxByte = MODE_BUSY;
      while ((TxRxByte & MODE_BUSY) != 0x00)
      {
        RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low
        SPDR = RDSR; 				// Send read the Status Register
        while ( !(SPSR & _BV(SPIF)) );
        SPDR = 0xff; 				// Send a dummy byte to receive the Status Register
        while ( !(SPSR & _BV(SPIF)) );
        TxRxByte = SPDR;
        RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
        if ((TxRxByte & MODE_BUSY) != 0x00)
          _delay_ms(2);
      }
#endif


#if defined (RAM1_WRITE_LATCH)
      RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low
      // Transmit the Write Latch command
      SPDR = WREN;
      while ( !(SPSR & _BV(SPIF)) );
      RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
#endif

      RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low

      // Transmit the mode command
      SPDR = WRITE;

#if (RAM1_ADDR_BITS > 16)
      // Prepare & transmit the Address High Byte if warranted
      TxRxByte = localDest.bAddr.high_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;
#endif

      // Prepare & transmit the Address Mid Byte
      TxRxByte = localDest.bAddr.mid_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;

      // Prepare & transmit the Address Low Byte
      TxRxByte = localDest.bAddr.low_byte;
      while ( !(SPSR & _BV(SPIF)) );
      SPDR = TxRxByte;

      while ( !(SPSR & _BV(SPIF)) );
      SPDR = Src[ index++ ]; // Begin transmission from the Src to the Dest
      while ( index < Length )
      {

#if defined (RAM1_PAGE_ACCESS)
        // New write cycle must be initiated if writing across page boundaries
        if ((localDest.lAddr + index) % RAM1_PAGE_BYTES == 0)
        {
          while ( !(SPSR & _BV(SPIF)) );
          RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High

          writeDest.lAddr = localDest.lAddr + index;

#if defined (RAM1_BUSY_MODE)
          TxRxByte = MODE_BUSY;
          while ((TxRxByte & MODE_BUSY) != 0x00)
          {
            RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low
            SPDR = RDSR; 				// Send read the Status Register
            while ( !(SPSR & _BV(SPIF)) );
            SPDR = 0xff; 				// Send a dummy byte to receive the Status Register
            while ( !(SPSR & _BV(SPIF)) );
            TxRxByte = SPDR;
            RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
            if ((TxRxByte & MODE_BUSY) != 0x00)
              _delay_ms(2);
          }
#endif

#if defined (RAM1_WRITE_LATCH)
          RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low
          // Transmit the Write Latch command
          SPDR = WREN;
          while ( !(SPSR & _BV(SPIF)) );
          RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
#endif

          RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low

          // Transmit the mode command
          SPDR = WRITE;

#if (RAM1_ADDR_BITS > 16)
          // Prepare & transmit the Address High Byte if warranted
          TxRxByte = writeDest.bAddr.high_byte;
          while ( !(SPSR & _BV(SPIF)) );
          SPDR = TxRxByte;
#endif

          // Prepare & transmit the Address Mid Byte
          TxRxByte = writeDest.bAddr.mid_byte;
          while ( !(SPSR & _BV(SPIF)) );
          SPDR = TxRxByte;

          // Prepare & transmit the Address Low Byte
          TxRxByte = writeDest.bAddr.low_byte;
          while ( !(SPSR & _BV(SPIF)) );
          SPDR = TxRxByte;
        }
#endif

        TxRxByte = Src[ index++ ]; // pre-load the byte to be transmitted
        while ( !(SPSR & _BV(SPIF)) );
        SPDR = TxRxByte; // Continue transmission
      }
      while ( !(SPSR & _BV(SPIF)) );

      RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High

      ReturnCode = EEFS_SUCCESS;

#else
      ReturnCode = EEFS_NO_SUCH_DEVICE;
#endif

      break;

    default:
      ReturnCode = EEFS_NO_SUCH_DEVICE;
      break;
  }

  return ReturnCode;
}



