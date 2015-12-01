
#ifndef _ADC_H
#define _ADC_H

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>

#include <Arduino.h>

typedef union _ADC_value_t {
  uint16_t u16;
  int16_t  i16;
  uint8_t  u8[2];
} ADC_value_t;


void AudioCodec_ADC_init(void);                         // setup ADC
void AudioCodec_Timer2_init(uint16_t const samplesSecond);    // set up the sampling Timer2, runs at audio sampling rate in Hz.
void AudioCodec_ADC(uint16_t* _modvalue) __attribute__((flatten));   // adc sampling routine

void AudioCodec_Timer2_enable(void) __attribute__((flatten));  // enable sampling Timer2, runs at audio sampling rate in Hz.
void AudioCodec_Timer2_disable(void) __attribute__((flatten));  // disable sampling Timer2, runs at audio sampling rate in Hz.

void AudioCodec_samplePeaks( int16_t const inputSample, int16_t * const minimumSample, int16_t * const maximumSample ); // store the maximum and minimum values of a sample train.

/*-----------------------------------------------------------*/

void AudioCodec_ADC_init(void)
{
  ADMUX  = _BV(REFS0) | _BV(ADLAR); // AVcc reference with external capacitor at AREF pin - left justify - start with MIC input ADC0 which is ADMUX=0b000
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADPS2) | _BV(ADPS0); // ADC enable, auto trigger, ck/32 = 250kHz
  ADCSRB =  0x00;	// free running mode
  DIDR0  =  0xFF;	// turn off digital input for all Analogue Pins.
}

void AudioCodec_Timer2_init(uint16_t const samplesSecond)
{
  if (TIMSK2 && _BV(OCIE2A))          // if the timer has already been set, then just adjust the sample rate.
  {
#if (F_CPU > 80000000L)
    OCR2A = (F_CPU / ((uint32_t)samplesSecond * 256 )) - 1;
#else
    OCR2A = (F_CPU / ((uint32_t)samplesSecond * 128 )) - 1;
#endif
  }
  else                                // otherwise we have to set up the timer before enabling the interrupt routine.
  {
#if defined(DEBUG_PING)
    DDRD |= _BV(DDD7);              // set the debugging ping
    PORTD &= ~_BV(PORTD7);
#endif

    // setup Timer2 for codec clock division
    TCCR2A = _BV(WGM21);            // set to  CTC Mode 2.  TOP = OCR2A. Normal port operation, OC2A disconnected.

#if (F_CPU > 80000000L)
    TCCR2B = _BV(CS22) | _BV(CS21); // Clk I/O scaler 256 (From prescaler) CTC Mode. TOP = OCR0A. Clock on rising edge.
    TCNT2 =  0x00;                  // clear the counter.
    OCR2A = (F_CPU / ((uint32_t)samplesSecond * 256 )) - 1;
#else
    TCCR2B = _BV(CS22) | _BV(CS20); // Clk I/O scaler 128 (From prescaler) CTC Mode. TOP = OCR0A. Clock on rising edge.
    TCNT2 =  0x00;                  // clear the counter.
    OCR2A = (F_CPU / ((uint32_t)samplesSecond * 128 )) - 1;
#endif

    TIMSK2 = _BV(OCIE2A);           // turn on compare match interrupt
  }
}


void AudioCodec_Timer2_enable(void)   // enable sampling Timer2, runs at audio sampling rate in Hz.
{
  TIMSK2 = _BV(OCIE2A);           // turn on compare match interrupt
}


void AudioCodec_Timer2_disable(void)  // disable sampling Timer2, runs at audio sampling rate in Hz.
{
  TIMSK2 &= ~_BV(OCIE2A);         // turn off compare match interrupt
}

void AudioCodec_ADC(uint16_t* _modvalue)
{
  if (ADCSRA & _BV(ADIF))				  // check if sample ready
  {
    *_modvalue = ADCW;				  // fetch ADCL first to freeze sample, then ADCH. It is done by the compiler.
    ADCSRA |= _BV(ADIF);			  // reset the interrupt flag
  }
}


void AudioCodec_samplePeaks(  int16_t const inputSample, int16_t * const minimumSample, int16_t * const maximumSample ) // store the maximum and minimum values of a sample train.
{
  if (inputSample < *minimumSample)
    *minimumSample = inputSample;
  if (inputSample > *maximumSample)
    *maximumSample = inputSample;
}


/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif // _ADC_H
