#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include <Arduino.h>

#include "spi.h"
#include "eefs_ringBuffer.h"
#include "ADC.h"
#include "IIR.h"
#include "g711.h"

#include "ModemClass.h"


/*-----------------------------------------------------------*/
// Definitions

// Remove this to stop the sampling line ping
//#define DEBUG_PING

#define idDevAddr "D0237023"
#define idDevEui "11111111D0237023"
#define idNwSKey "3BD4F443293920FF32DA51BB4D1EE63D"
#define idAppSKey "FC833EE615ED0C54F06883CEDF44AEAC"

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#endif

/*-----------------------------------------------------------*/
// Type Definitions

typedef enum {
  INITIALISE,
  SAMPLE,
  LOGIC,
  TRANSMIT,
  SLEEP
} globalState_t;

typedef enum {
  SETUP,
  PREPARATION,
  SAMPLING,
  PROCESSED
} samplingState_t;

typedef enum {
  NOTREADY,
  READY,
  PREAMBLE,
  PAYLOAD,
  RESPONSE
} transmitState_t;

typedef enum {
  PASSIVE,
  ACTIVE
} operateMode_t; // operational mode for low power or active powered.

typedef enum {
  DR0 = 11,
  DR1 = 53,
  DR2 = 129,
  DR3 = 250
} packetPayload_t; // network packet size based on rate

/*-----------------------------------------------------------*/
// Assembly defines
/*
  Enable the watchdog timer, configuring it for expire after
  (value) timeout (which is a combination of the WDP0
  through WDP3 bits).

  This function is derived from <avr/wdt.h> but enables only
  the interrupt bit (WDIE), rather than the reset bit (WDE).

  Can't find it documented but the WDT, once enabled,
  rolls over and fires a new interrupt each time.

  See also the symbolic constants WDTO_15MS et al.
*/
#define wdt_interrupt_enable(value)     \
  __asm__ __volatile__ (                \
                                        "in __tmp_reg__,__SREG__" "\n\t"    \
                                        "cli" "\n\t"                        \
                                        "wdr" "\n\t"                        \
                                        "sts %0,%1" "\n\t"                  \
                                        "out __SREG__,__tmp_reg__" "\n\t"   \
                                        "sts %0,%2" "\n\t"                  \
                                        : /* no outputs */                  \
                                        : "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
                                        "r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
                                        "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) | \
                                            _BV(WDIF) | _BV(WDIE) | (value & 0x07)) )   \
                                        : "r0"                              \
                       )

/*-----------------------------------------------------------*/
// Prototypes

void setup() __attribute__((flatten));
void loop() __attribute__((flatten));

void samplingEngine(void);      // state machine for running the sampling
void transmissionEngine(void);  // state machine for running the modem and transmission

void _powerDown(void) __attribute__((flatten));  // power down unnecessary peripherals
void _sleep (uint8_t sleepMode, uint8_t WDTValue) __attribute__((flatten));     // set the watchdog and go to sleep

ISR(TIMER2_COMPA_vect) __attribute__ ((hot, flatten)); // This interrupt is used to drive the ADC sampling rate.

/*-----------------------------------------------------------*/
// Globals

globalState_t deviceState; // overall device state
samplingState_t inputState; // acquisition engine state
transmitState_t modemState; // transmission engine state

operateMode_t mode; // mode of operation, low power passive, or active

LoRaModem modem; // instantiating a modem.

filter_t tx_filter; // filter for processing samples from Microphone.

/*-----------------------------------------------------------*/

void setup() {
  // put your setup code here, to run once:

#ifdef DEBUG
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  // wait a moment for the serial port to finish starting up
  _delay_ms(100);
#endif

  deviceState = INITIALISE; // set up initial state for state machines.
  modemState = NOTREADY;
  inputState = SETUP;
}

void loop()
{
  switch (deviceState)
  {
    case INITIALISE:
      DEBUG_PRINT("deviceState INITIALISE");

      _powerDown();

      samplingEngine();

      transmissionEngine();

      if ( modemState == READY && inputState == PREPARATION )
      {
        deviceState = SAMPLE;
      }

      break;

    case SAMPLE:
      DEBUG_PRINT("deviceState SAMPLE");

      samplingEngine();

      if ( modemState == READY && inputState == PROCESSED )
      {
        deviceState = TRANSMIT;
      }
      break;

    case LOGIC:
      break;

    case TRANSMIT:
      DEBUG_PRINT("deviceState TRANSMIT");

      transmissionEngine();

      if ( inputState == PROCESSED && modemState == READY )
      {
        inputState = PREPARATION;
        deviceState = SLEEP;
      }
      break;

    case SLEEP:

      DEBUG_PRINT("deviceState SLEEP");
      Serial.flush(); // empty the serial transmission buffer before we sleep.
      _sleep( SLEEP_MODE_STANDBY, WDTO_8S );
      deviceState = SAMPLE; // woken up from a global sleep, so start sampling
      break;

    default:
      break;

  }
}
/*-----------------------------------------------------------*/

#define FRAM_START_ADDR     RAM0_ADDR
#define FRAM_SIZE           8192

eefs_ringBuffer_t acquisitionBufferXRAM; // where we store the samples from acquisition, G.711 companded or G.726 compressed.

ADC_value_t mod0_value; // address of individual audio sample

void samplingEngine(void)
{
  switch (inputState)
  {
    case SETUP:
      DEBUG_PRINT("inputState SETUP");
      AudioCodec_ADC_init();
      AudioCodec_Timer2_init( SAMPLE_RATE );

      tx_filter.cutoff = 0xc000;        // set filter to 3/8 of sample frequency (0xffff is 1/2 sample frequency)
      setIIRFilterLPF( &tx_filter );    // initialise transmit sample filter

      eefs_ringBuffer_InitBuffer( &acquisitionBufferXRAM, FRAM_START_ADDR, FRAM_SIZE );

      inputState = PREPARATION;
      break;

    case PREPARATION:
      DEBUG_PRINT("inputState PREPARATION");
      eefs_ringBuffer_Flush( &acquisitionBufferXRAM );

      if (modemState == READY)
        inputState = SAMPLING;
      break;

    case SAMPLING:
      DEBUG_PRINT("inputState SAMPLING");

      AudioCodec_Timer2_enable(); // turn on the timer 2 to enable acquisition of audio

      while ( ! eefs_ringBuffer_IsFull( &acquisitionBufferXRAM ) )
      {
        _delay_ms(500);
      }

      AudioCodec_Timer2_disable(); // turn on the timer 2 to enable acquisition of audio

      inputState = PROCESSED;
      break;

    case PROCESSED:
      DEBUG_PRINT("inputState PROCESSED");
      break;

    default:
      break;
  }
}
/*-----------------------------------------------------------*/

void transmissionEngine(void)
{

  String Command; // Final command to the modem
  String Type; // Which command we're going to use.

  uint16_t messageIndex;
  uint16_t messageLengthBytes;

  uint8_t byteG711;
  int16_t processedSample;

  switch (modemState)
  {
    case NOTREADY:
      DEBUG_PRINT("modemState NOTREADY");
      if ( ! modem.Reset() )
      {
        _sleep(SLEEP_MODE_IDLE, WDTO_2S);
        if ( ! modem.setPort("1") )
        {
          modemState = READY;
          _sleep(SLEEP_MODE_IDLE, WDTO_2S);

          modem.checkAT();
          _sleep(SLEEP_MODE_IDLE, WDTO_2S);

          modem.checkDR();
          _sleep(SLEEP_MODE_IDLE, WDTO_2S);

          //  modem.setID(idDevAddr, idDevEui);
          //  _sleep(SLEEP_MODE_IDLE, WDTO_2S);
          //  modem.setKeys(idNwSKey, idAppSKey);
        }
      }
      break;

    case READY: // ready to transmit, but if there is nothing to transmit then break;
      DEBUG_PRINT("modemState READY");
      if (inputState == PROCESSED)
        modemState = PREAMBLE;
      break;

    case PREAMBLE: // prepare the preamble commands
      DEBUG_PRINT("modemState PREAMBLE");

      Type = "100";
      messageIndex = 0xFFFF;
      messageLengthBytes =  eefs_ringBuffer_GetCount( &acquisitionBufferXRAM );

      Command = Type + ',' + messageIndex + ',' + messageLengthBytes;

      DEBUG_PRINT( Command );

      while ( modem.cMsg( Command ) != 1)
        DEBUG_PRINT("Failure");

      modemState = PAYLOAD;
      break;

    case PAYLOAD: // transmit the payload bytes
      DEBUG_PRINT("modemState PAYLOAD");

      Type = "100";
      messageIndex = 0x0000;
      messageLengthBytes = eefs_ringBuffer_GetCount( &acquisitionBufferXRAM );

      do {

        Command = Type + ',' + (messageIndex++);

        for (uint8_t i = 0; i < 7; ++i)
        {
          byteG711 = eefs_ringBuffer_Pop( &acquisitionBufferXRAM );
          Command = Command + ',' + byteG711;
          --messageLengthBytes;
        }
        DEBUG_PRINT( Command);

        uint8_t ack;
        do {
          ack = modem.cMsg( Command );
        } while ( ack != 1);

        // DEBUG_PRINT( byteG711 );
        // alaw_expand1( &byteG711, &processedSample );
        // DEBUG_PRINT( processedSample );
      } while ( messageLengthBytes > 0  &&  messageIndex < 0xFFFF );

      modemState = RESPONSE;
      break; // fall through to RESPONSE

    case RESPONSE: // handle responses from the network side
      DEBUG_PRINT("modemState RESPONSE");

      modemState = READY;
      break;

    default:
      break;
  }
}
/*-----------------------------------------------------------*/

void _powerDown(void)
{
  DEBUG_PRINT("_powerDown()");

  // Analogue Comparator Disable
  // When the ACD bit is written logic one, the power to the Analogue Comparator is switched off.
  // This bit can be set at any time to turn off the Analogue Comparator.
  // This will reduce power consumption in Active and Idle mode.
  // When changing the ACD bit, the Analogue Comparator Interrupt must be disabled by clearing the ACIE bit in ACSR.
  // Otherwise an interrupt can occur when the ACD bit is changed.
  ACSR &= ~_BV(ACIE);
  ACSR |= _BV(ACD);

  // Digital Input Disable on Analogue Pins
  // When this bit is written logic one, the digital input buffer on the corresponding ADC pin is disabled.
  // The corresponding PIN Register bit will always read as zero when this bit is set. When an
  // analogue signal is applied to the ADC5..0 pin and the digital input from this pin is not needed, this
  // bit should be written logic one to reduce power consumption in the digital input buffer.
  DIDR0 = 0x3F;

  // Disable Port D Digitial Pins (except D0 Rx and D1 Tx).
  // Note that we're using B0 and B1 for or transmission to modem.
  // Don't need to do this as putting the device to sleep will lock the pins.
  // Could come back to look at this by setting unused pins to input, with pull-up.
}



void _sleep (uint8_t sleepMode, uint8_t WDTValue)
{
  // There are several macros provided in the header file to actually put
  // the device into sleep mode.

  // SLEEP_MODE_IDLE (0)
  // SLEEP_MODE_ADC _BV(SM0)
  // SLEEP_MODE_PWR_DOWN _BV(SM1)
  // SLEEP_MODE_PWR_SAVE (_BV(SM0) | _BV(SM1))
  // SLEEP_MODE_STANDBY (_BV(SM1) | _BV(SM2))
  // SLEEP_MODE_EXT_STANDBY (_BV(SM0) | _BV(SM1) | _BV(SM2))

  set_sleep_mode( sleepMode );

  /* Watchdog period options:   WDTO_15MS
              WDTO_30MS
              WDTO_60MS
              WDTO_120MS
              WDTO_250MS
              WDTO_500MS
              WDTO_1S
              WDTO_2S
              WDTO_4S
              WDTO_8S
  */
  wdt_interrupt_enable( WDTValue ); // enable the watchdog timer.

  sleep_enable();

#if defined(BODS) && defined(BODSE) // Only if there is support to disable the brown-out detection.
  sleep_bod_disable();
#endif

  sleep_cpu(); // good night.

  sleep_disable(); // Ugh. I've been woken up. Better disable sleep mode.

  wdt_disable(); // we've got to disable the watchdog or it will trigger again.

}

/*-----------------------------------------------------------*/

ISR(TIMER2_COMPA_vect) // This interrupt for generating Audio samples. Should be between 4000 and 16000 samples per second.
{
  uint8_t byteG711;
  int16_t sample;

  while ( ! eefs_ringBuffer_IsFull( &acquisitionBufferXRAM ) )
  {
#if defined(DEBUG_PING)
    // start mark - check for start of interrupt - for debugging only
    PORTD |=  _BV(PORTD7);        // Ping IO line.
#endif

    AudioCodec_ADC( &mod0_value.u16 );

    sample = mod0_value.u16 - 0x3e00; // This is offset using 5V supply. Will need to read adjust once it is running off battery.

    IIRFilter( &tx_filter, &sample);  // filter the sample train prior to companding, with corner frequency set to 3/8 of sample rate.

    alaw_compress1( &sample, &byteG711 );

    eefs_ringBuffer_Poke( &acquisitionBufferXRAM, byteG711 );

#if defined(DEBUG_PING)
    // end mark - check for end of interrupt - for debugging only
    PORTD &= ~_BV(PORTD7);
#endif
  }
}

/*-----------------------------------------------------------*/

