// IIR.h

#ifndef IIR_h // include guard
#define IIR_h


/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#include <stdlib.h>
#include <math.h>

#include "mult16x16.h"
#include "mult16x8.h"
#include "mult32x16.h"

//==================================================//
//******************* IIR Filter *******************//
//==================================================//


/*--------------------------------------------------*/
/*------------------Definitions---------------------*/
/*--------------------------------------------------*/


#ifndef SAMPLE_RATE
#define SAMPLE_RATE		8000	// set up the sampling Timer1 to 48000Hz, 44100Hz (or lower), runs at audio sampling rate in samples per Second.
#endif					// 384 = 3 x 2^7 divisors 2k, 3k, 4k, 6k, 8k, 12k, 16k, 24k, 48k

// IIR filter coefficient scaling
// multiply the coefficients by 32, assume a(1) is 32 * (1 + alpha), to get better accuracy in fixed format.
#define IIRSCALEFACTOR		32
#define IIRSCALEFACTORSHIFT	5

// IIR Resonance (maximum) at the corner frequency.
#define Q_MAXIMUM 			(6.0f)
#define Q_LINEAR			(M_SQRT1_2)

// float-fix conversion macros
// assuming we're using 8.8 for the coefficients.
#define float2int(a) ((int16_t)((a)*256.0))
#define int2float(a) ((double)(a)/256.0)


/*--------------------------------------------------*/
/*--------------------Typedefs----------------------*/
/*--------------------------------------------------*/

typedef struct __attribute__ ((packed)) _filter_t {
  uint16_t sample_rate;	// sample rate in Hz
  uint16_t cutoff;	// normalised cutoff frequency, 0-65536. maximum is sample_rate/2
  uint16_t peak;		// normalised Q factor, 0-65536. maximum is Q_MAXIMUM
  int16_t b0, b1, b2, a1, a2; // Coefficients in 8.8 format
  int16_t xn_1, xn_2;	//IIR state variables
  int16_t yn_1, yn_2; //IIR state variables
} filter_t;


/*--------------------------------------------------*/
/*----------Public Function Definitions-------------*/
/*--------------------------------------------------*/

//==================================================
//****************** IIR Filter ******************//
//==================================================
// second order IIR -- "Direct Form I Transposed"
//  a(0)*y(n) = b(0)*x(n) + b(1)*x(n-1) +  b(2)*x(n-2)
//                   - a(1)*y(n-1) -  a(2)*y(n-2)
// assumes a(0) = IIRSCALEFACTOR = 32

// Compute filter function coefficients
// http://en.wikipedia.org/wiki/Digital_biquad_filter
// https://www.hackster.io/bruceland/dsp-on-8-bit-microcontroller
// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt

void setIIRFilterLPF( filter_t *filter ); // Low Pass
void setIIRFilterBPF( filter_t *filter ); // Band Pass
void setIIRFilterHPF( filter_t *filter ); // High Pass

// returns y(n) filtered by the biquad IIR process in place of x(n)
void IIRFilter( filter_t *filter, int16_t * xn ) __attribute__ ((hot, flatten));


//========================================================
//********************* IIR Filter *********************//
//========================================================
// second order IIR -- "Direct Form I Transposed"
//  a(0)*y(n) = b(0)*x(n) + b(1)*x(n-1) +  b(2)*x(n-2)
//                   - a(1)*y(n-1) -  a(2)*y(n-2)
// assumes a(0) = IIRSCALEFACTOR = 32

// Compute filter function coefficients
// http://en.wikipedia.org/wiki/Digital_biquad_filter
// https://www.hackster.io/bruceland/dsp-on-8-bit-microcontroller
// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt

void setIIRFilterLPF( filter_t *filter ) // Low Pass Filter Setting
{
  if ( !(filter->sample_rate) )
    filter->sample_rate = SAMPLE_RATE;

  if ( !(filter->cutoff) )
    filter->cutoff = UINT16_MAX >> 1; // 1/4 of sample rate = filter->sample_rate>>2

  if ( !(filter->peak) )
    filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM); // 1/sqrt(2) effectively

  double frequency = ((double)filter->cutoff * (filter->sample_rate >> 1)) / UINT16_MAX;
  double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
  double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
  double sinW0 = sin(w0);
  double cosW0 = cos(w0);
  double alpha = sinW0 / (q * 2.0f);
  double scale = IIRSCALEFACTOR / (1 + alpha); // a0 = 1 + alpha

  filter->b0	= \
                filter->b2	= float2int( ((1.0 - cosW0) / 2.0) * scale );
  filter->b1	= float2int(  (1.0 - cosW0) * scale );

  filter->a1	= float2int( (-2.0 * cosW0) * scale );
  filter->a2	= float2int( (1.0 - alpha) * scale );
}

void setIIRFilterHPF( filter_t *filter ) // High Pass Filter Setting
{
  if ( !(filter->sample_rate) )
    filter->sample_rate = SAMPLE_RATE;

  if ( !(filter->cutoff) )
    filter->cutoff = UINT16_MAX >> 1; // 1/4 of sample rate = filter->sample_rate>>2

  if ( !(filter->peak) )
    filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM); // 1/sqrt(2) effectively

  double frequency = ((double)filter->cutoff * (filter->sample_rate >> 1)) / UINT16_MAX;
  double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
  double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
  double sinW0 = sin(w0);
  double cosW0 = cos(w0);
  double alpha = sinW0 / (q * 2.0f);
  double scale = IIRSCALEFACTOR / (1 + alpha); // a0 = 1 + alpha

  filter->b0	= float2int( ((1.0 + cosW0) / 2.0) * scale );
  filter->b1	= float2int( -(1.0 + cosW0) * scale );
  filter->b2	= float2int( ((1.0 + cosW0) / 2.0) * scale );

  filter->a1	= float2int( (-2.0 * cosW0) * scale );
  filter->a2	= float2int( (1.0 - alpha) * scale );
}

void setIIRFilterBPF( filter_t *filter ) // Band Pass Filter Setting
{
  if ( !(filter->sample_rate) )
    filter->sample_rate = SAMPLE_RATE;

  if ( !(filter->cutoff) )
    filter->cutoff = UINT16_MAX >> 1; // 1/4 of sample rate = filter->sample_rate>>2

  if ( !(filter->peak) )
    filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM); // 1/sqrt(2) effectively

  double frequency = ((double)filter->cutoff * (filter->sample_rate >> 1)) / UINT16_MAX;
  double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
  double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
  double sinW0 = sin(w0);
  double cosW0 = cos(w0);
  double alpha = sinW0 / (q * 2.0f);
  double scale = IIRSCALEFACTOR / (1 + alpha); // a0 = 1 + alpha

  filter->b0	= float2int( alpha * scale );
  filter->b1	= 0;
  filter->b2	= float2int( -alpha * scale );

  filter->a1	= float2int( (-2.0 * cosW0) * scale );
  filter->a2	= float2int( (1.0 - alpha) * scale );
}

// Coefficients in 8.8 format
// interim values in 24.8 format
// returns y(n) in place of x(n)
void IIRFilter( filter_t *filter, int16_t * xn )
{
  int32_t yn;			// current output
  int32_t  accum;		// temporary accumulator

  // sum the 5 terms of the biquad IIR filter
  // and update the state variables
  // as soon as possible
  MultiS16X16to32(yn, filter->xn_2, filter->b2);
  filter->xn_2 = filter->xn_1;

  MultiS16X16to32(accum, filter->xn_1, filter->b1);
  yn += accum;
  filter->xn_1 = *xn;

  MultiS16X16to32(accum, *xn, filter->b0);
  yn += accum;

  MultiS16X16to32(accum, filter->yn_2, filter->a2);
  yn -= accum;
  filter->yn_2 = filter->yn_1;

  MultiS16X16to32(accum, filter->yn_1, filter->a1);
  yn -= accum;

  filter->yn_1 = yn >> (IIRSCALEFACTORSHIFT + 8); // divide by a(0) = 32 & shift to 16.0 bit outcome from 24.8 interim steps

  *xn = filter->yn_1; // being 16 bit yn, so that's what we return.
}

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif // IIR_h end include guard
