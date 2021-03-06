#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
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
#include "g726.h"

#include "ModemClass.h"


/*-----------------------------------------------------------*/
// Definitions

// Remove this to stop the sampling line ping
//#define DEBUG_PING

#define idDevAddr "D0237023"
#define idDevEui "11111111D0237023"
#define idNwSKey "3BD4F443293920FF32DA51BB4D1EE63D"
#define idAppSKey "FC833EE615ED0C54F06883CEDF44AEAC"

// ADR is a bit slow to switch from DR0 to DR3. Set to DR3 and hope for the best?
#define maxDR "DR3"

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

  DR0 = 11,   // DR0 = 11,
  DR1 = 11,   // DR1 = 53,
  DR2 = 11,  // DR2 = 129,
  DR3 = 11   // DR3 = 250
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
/*-----------------------------------------------------------*/

void loop()
{
  switch (deviceState)
  {
    case INITIALISE:
      DEBUG_PRINT(F("deviceState INITIALISE"));

      _powerDown();

      samplingEngine();

      transmissionEngine();

      if ( modemState == READY && inputState == PREPARATION )
      {
        deviceState = SAMPLE;
      }

      break;

    case SAMPLE:
      DEBUG_PRINT(F("deviceState SAMPLE"));

      samplingEngine();

      if ( modemState == READY && inputState == PROCESSED )
      {
        deviceState = TRANSMIT;
      }

      break;

    case TRANSMIT:
      DEBUG_PRINT(F("deviceState TRANSMIT"));

      transmissionEngine();

      if ( inputState == PROCESSED && modemState == READY )
      {
        deviceState = SLEEP;
      }
      break;

    case SLEEP:
      DEBUG_PRINT(F("deviceState SLEEP"));

      modem.lowPower();

      Serial.flush(); // empty the serial transmission buffer before we sleep.

      _sleep( SLEEP_MODE_IDLE, WDTO_8S );

      modem.checkAT();
      delay(50);

      modem.checkAT();

      deviceState = SAMPLE; // woken up from a global sleep, so start sampling
      inputState = PREPARATION;
      break;

    default:
      break;

  }
}
/*-----------------------------------------------------------*/
// Globals

#define FRAM_START_ADDR     RAM0_ADDR
#define FRAM_SIZE           8192

#define NOISE_TRIGGER       10000

ADC_value_t mod0Value; // address of individual audio sample

filter_t txFilter; // filter for processing samples from Microphone.

g726_state g726State;  // state for the g.726 encoder, maintaining predictors etc.

eefs_ringBuffer_t acquisitionBufferXRAM; // where we store the samples from acquisition, G.711 companded or G.726 compressed.

int32_t maximumSampleDelta;

enum {
  FALSE = 0,
  TRUE = 1
} noiseEvent;

/*-----------------------------------------------------------*/

void samplingEngine(void)
{
  uint16_t messageLengthBytes;

  int16_t filteredSample;
  uint8_t g711Byte;
  uint8_t g726Byte;
  uint8_t twoBits;

  int16_t minimumSample = 0;
  int16_t maximumSample = 0;

  switch (inputState)
  {
    case SETUP:
      DEBUG_PRINT(F("inputState SETUP"));

      AudioCodec_ADC_init();
      AudioCodec_Timer2_init( SAMPLE_RATE );

      txFilter.cutoff = 0xc000;        // set filter to 3/8 of sample frequency (0xffff is 1/2 sample frequency)
      setIIRFilterLPF( &txFilter );    // initialise transmit sample filter

      g726_init_state( &g726State );    // initialise the 16kbit/s compression engine. 16 bits/sample -> 2 bits/sample

      eefs_ringBuffer_InitBuffer( &acquisitionBufferXRAM, FRAM_START_ADDR, FRAM_SIZE );

      inputState = PREPARATION;
      break;

    case PREPARATION:
      DEBUG_PRINT(F("inputState PREPARATION"));

      eefs_ringBuffer_Flush( &acquisitionBufferXRAM );

      if (modemState == READY)
        inputState = SAMPLING;
      break;

    case SAMPLING:
      DEBUG_PRINT(F("inputState SAMPLING"));

      // AUDIO SAMPLING

      AudioCodec_Timer2_enable(); // turn on the timer 2 to enable acquisition of audio

      while ( ! eefs_ringBuffer_IsFull( &acquisitionBufferXRAM ) )
      {
        _delay_ms(100);
      }

      AudioCodec_Timer2_disable(); // turn on the timer 2 to enable acquisition of audio

      // AUDIO ENCODING

      messageLengthBytes = eefs_ringBuffer_GetCount( &acquisitionBufferXRAM );
      maximumSampleDelta = 0;

      for ( uint16_t i = 0; i < (messageLengthBytes >> 2); ++i ) // iterate over the message buffer, noting that we will capture 4x bytes per cycle.
      {

        g726Byte = 0; // prepare the g.726 byte for 4 x 2 bits.

        for ( uint8_t j = 0; j < 4; ++j )
        {
          g711Byte = eefs_ringBuffer_Pop( &acquisitionBufferXRAM ); // get the 8 bit A Law encoded byte

          // Quickly grab it back and find the peak delta value across the sample range.

          alaw_expand1( &g711Byte, &filteredSample ); // expand the G.711 A Law encoded byte back to PCM 16 bits.

          AudioCodec_samplePeaks( filteredSample, &minimumSample, &maximumSample );

          // Now do the G.726 Encoding Section

          twoBits = (uint8_t) g726_16_encoder( (uint16_t)g711Byte, AUDIO_ENCODING_ALAW, &g726State ); // capture the two bits from the g.726 encoder

          g726Byte <<= 2; // shift the current g.726 byte along two places (Would do this in fewer steps, but this is clear for debugging).

          g726Byte |= (twoBits & 0x03); // mask in the new two bits, and do this four times.
        }

        eefs_ringBuffer_Poke( &acquisitionBufferXRAM, g726Byte ); // poke in the g.726 byte.
        // At the end we should have only 1/4 the g.711 (or 1/8 the PCM) bytes, so messageLengthBytes should reflect that the XRAM buffer is 1/4 full.
      }

      maximumSampleDelta = abs(maximumSample) + abs( minimumSample); // This is the maximum for this sampling second.
      DEBUG_PRINT(maximumSampleDelta);


      if ( (maximumSampleDelta - NOISE_TRIGGER) > 0 ) {
        noiseEvent = TRUE;
      } else {
        noiseEvent = FALSE;
      }

      inputState = PROCESSED;
      break;

    case PROCESSED:
      DEBUG_PRINT(F("inputState PROCESSED"));

      // Success!
      // We wait here until the deviceState logic asks us to sample again.

      break;

    default:
      break;
  }
}
/*-----------------------------------------------------------*/
// Globals

packetPayload_t packetPayloadSize;

/*-----------------------------------------------------------*/

void transmissionEngine(void)
{

  String Command; // Final command to the modem
  uint16_t messageLengthBytes;

  uint8_t ack;

  switch (modemState)
  {
    case NOTREADY:
      DEBUG_PRINT(F("modemState NOTREADY"));

      if ( ! modem.Reset() )
      {
        _sleep(SLEEP_MODE_IDLE, WDTO_1S);
        if ( ! modem.disableADR() )
        {
          _sleep(SLEEP_MODE_IDLE, WDTO_1S);
          if ( ! modem.setDR(maxDR) )
          {
            _sleep(SLEEP_MODE_IDLE, WDTO_1S);
            if ( ! modem.setPort("1") )
            {
              modemState = READY;
              _sleep(SLEEP_MODE_IDLE, WDTO_1S);

              modem.checkAT();
              _sleep(SLEEP_MODE_IDLE, WDTO_1S);

              //  modem.setID(idDevAddr, idDevEui);
              //  _sleep(SLEEP_MODE_IDLE, WDTO_1S);
              //  modem.setKeys(idNwSKey, idAppSKey);
            }
          }
        }
      }
      break;

    case READY: // ready to transmit, but if there is nothing to transmit then break;
      DEBUG_PRINT(F("modemState READY"));

      if (inputState == PROCESSED)
        modemState = PREAMBLE;
      break;

    case PREAMBLE: // prepare the preamble commands
      DEBUG_PRINT(F("modemState PREAMBLE"));

      switch (modem.getDR()) {
        case '0':
          packetPayloadSize = DR0;//11
          break;
        case '1':
          packetPayloadSize = DR1;//53
          break;
        case '2':
          packetPayloadSize = DR2;//129
          break;
        case '3':
          packetPayloadSize = DR3;//250
          break;
        default:
          packetPayloadSize = DR0;//11
      }
      DEBUG_PRINT(packetPayloadSize);

      Command = "108";
      Command = Command + ',' + maximumSampleDelta;

      do {
        if ( (ack = modem.cMsg( Command )) == 1)
          DEBUG_PRINT(F("108, failure, resend."));
      } while ( ack == 1);

      if ( noiseEvent == TRUE ) {
        modemState = PAYLOAD;
      } else {
        modemState = RESPONSE;
      }

      break;

    case PAYLOAD: // transmit the payload bytes
      DEBUG_PRINT(F("modemState PAYLOAD"));

      //     Command = "DEADBEEFCAFEF00D00BABE";
      //     if ( modem.cMsgHEX( Command ) )
      //        DEBUG_PRINT(F("Test HEX Failure"));

      struct {
        uint8_t messageType;
        uint8_t bytes[DR1]; // xxx WARNING This is a memory limit !!!
      } payloadStructure;

      messageLengthBytes =  eefs_ringBuffer_GetCount( &acquisitionBufferXRAM );

      Command = "101";
      Command = Command + ',' + messageLengthBytes;
      // send an audio stream begin command with "101", and the number of bytes we expect to send.

      DEBUG_PRINT( Command );


      do {
        if ( (ack = modem.cMsg( Command )) == 1)
          DEBUG_PRINT(F("101, failure, resend."));
      } while ( ack == 1);

      payloadStructure.messageType = 1;

      do {

        for (uint8_t i = 0; i < (uint8_t)(packetPayloadSize - 1 ); ++i)
        {
          if (messageLengthBytes > 0 )
          {
            payloadStructure.bytes[i] = eefs_ringBuffer_Pop( &acquisitionBufferXRAM );
            --messageLengthBytes;
          }
        }

        DEBUG_PRINT(F("Remaining messageLengthBytes"));
        DEBUG_PRINT(messageLengthBytes);

        do {
          DEBUG_PRINT(F("Sending 01xxyyzzaabbcc..."));
          if ( (ack = modem.cMsgBytes( (uint8_t *)&payloadStructure, (uint16_t)packetPayloadSize )) == 1)
            DEBUG_PRINT(F("... failure, resend."));
        } while ( ack == 1);

      } while ( messageLengthBytes > 0 );

      Command = "102";
      Command = Command + ',' + 0xFFFF;
      // End an audio stream  with End command "102" with 0xFFFF.

      DEBUG_PRINT( Command);

      do {
        if ( (ack = modem.cMsg( Command )) == 1)
          DEBUG_PRINT(F("102, failure, resend."));
      } while ( ack == 1);

      modemState = RESPONSE;
      break;

    case RESPONSE: // handle responses from the network side
      DEBUG_PRINT(F("modemState RESPONSE"));

      modemState = READY;
      break;

    default:
      break;
  }
}
/*-----------------------------------------------------------*/

void _powerDown(void)
{
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
/*-----------------------------------------------------------*/


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
// Globals

//Will need to adjust the offset of 1.25V/Vcc = "Mic Amp Offset"/"Operating Voltage" once it is running off battery.
#define ADC_NULL_OFFSET 0x3e00 // USB Powered 5.0V Vcc.
//#define ADC_NULL_OFFSET 0x4e00 // Battery Powered 4.1V Vcc.

/*-----------------------------------------------------------*/

ISR(TIMER2_COMPA_vect) // This interrupt for generating Audio samples. Should be between 4000 and 16000 samples per second.
{
  uint8_t g711Byte;

  while ( ! eefs_ringBuffer_IsFull( &acquisitionBufferXRAM ) )
  {
#if defined(DEBUG_PING)
    // start mark - check for start of interrupt - for debugging only
    PORTD |=  _BV(PORTD7);        // Ping IO line.
#endif

    AudioCodec_ADC( &mod0Value.u16 );

    mod0Value.i16 = mod0Value.u16 - ADC_NULL_OFFSET; // This is offset based on the operating voltage.

    IIRFilter( &txFilter, &mod0Value.i16);  // filter the sample train prior to companding, with corner frequency set to 3/8 of sample rate.

    alaw_compress1( &mod0Value.i16, &g711Byte );

    eefs_ringBuffer_Poke( &acquisitionBufferXRAM, g711Byte );

#if defined(DEBUG_PING)
    // end mark - check for end of interrupt - for debugging only
    PORTD &= ~_BV(PORTD7);
#endif
  }
}


/*-----------------------------------------------------------*/

