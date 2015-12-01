
/*
   This source code is a product of Sun Microsystems, Inc. and is provided
   for unrestricted use.  Users may copy or modify this source code without
   charge.

   SUN SOURCE CODE IS PROVIDED AS IS WITH NO WARRANTIES OF ANY KIND INCLUDING
   THE WARRANTIES OF DESIGN, MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
   PURPOSE, OR ARISING FROM A COURSE OF DEALING, USAGE OR TRADE PRACTICE.

   Sun source code is provided with no support and without any obligation on
   the part of Sun Microsystems, Inc. to assist in its use, correction,
   modification or enhancement.

   SUN MICROSYSTEMS, INC. SHALL HAVE NO LIABILITY WITH RESPECT TO THE
   INFRINGEMENT OF COPYRIGHTS, TRADE SECRETS OR ANY PATENTS BY THIS SOFTWARE
   OR ANY PART THEREOF.

   In no event will Sun Microsystems, Inc. be liable for any lost revenue
   or profits or other special, indirect and consequential damages, even if
   Sun has been advised of the possibility of such damages.

   Sun Microsystems, Inc.
   2550 Garcia Avenue
   Mountain View, California  94043
*/

/*
   g726.h

   Header file for CCITT conversion routines.

*/


/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#ifndef _G726_H
#define	_G726_H

/********************* PUBLIC **********************/

#define	AUDIO_ENCODING_ULAW   (1)	/* ISDN u-law */
#define	AUDIO_ENCODING_ALAW   (2)	/* ISDN A-law */
#define	AUDIO_ENCODING_LINEAR (3)	/* PCM 2's-complement (0-center) */

/*
   The following is the definition of the state structure
   used by the G.721/G.723 encoder and decoder to preserve their internal
   state between successive calls.  The meanings of the majority
   of the state structure fields are explained in detail in the
   CCITT Recommendation G.721.  The field names are essentially indentical
   to variable names in the bit level description of the coding algorithm
   included in this Recommendation.
*/
typedef struct g726_state_s {
  long yl;	/* Locked or steady state step size multiplier. */
  int16_t yu;		/* Unlocked or non-steady state step size multiplier. */
  int16_t dms;	/* Short term energy estimate. */
  int16_t dml;	/* Long term energy estimate. */
  int16_t ap;		/* Linear weighting coefficient of 'yl' and 'yu'. */

  int16_t a[2];	/* Coefficients of pole portion of prediction filter. */
  int16_t b[6];	/* Coefficients of zero portion of prediction filter. */
  int16_t pk[2];	/* Signs of previous two samples of a partially
				  reconstructed signal. */
  int16_t dq[6];/* int here fails in newupdate on encode!
				  Previous 6 samples of the quantized difference
				  signal represented in an internal floating point
				  format.
*/
  int16_t sr[2];	/* Previous 2 samples of the quantized difference
				  signal represented in an internal floating point
				  format. */
  int16_t td;		/* delayed tone detect, new in 1988 version */
} g726_state;

/********************* PUBLIC **********************/
/* External function definitions. */

void g726_init_state( g726_state *);

int16_t g726_16_encoder(
  int16_t sample,
  int16_t in_coding,
  g726_state *state_ptr);

int16_t g726_16_decoder(
  int16_t code,
  int16_t out_coding,
  g726_state *state_ptr);


/********************* PRIVATE **********************/

int16_t linear2ulaw(int16_t pcm_val);
int16_t ulaw2linear(int16_t u_val);
int16_t linear2alaw(int16_t pcm_val);
int16_t alaw2linear(int16_t u_val);

static int16_t fmult(int16_t an, int16_t srn);
int16_t predictor_zero(	g726_state *state_ptr);
int16_t predictor_pole(	g726_state *state_ptr);
int16_t step_size(	g726_state *state_ptr);
int16_t quantize(	int16_t d,	/* Raw difference signal sample */
                  int16_t y,	/* Step size multiplier */
                  int16_t *	table,	/* quantization table */
                  int16_t size);	/* table size of short integers */
int16_t reconstruct(	int16_t sign,	/* 0 for non-negative value */
                      int16_t dqln,	/* G.72x codeword */
                      int16_t y);	/* Step size multiplier */
void update(	int16_t code_size,	/* distinguish 723_40 with others */
              int16_t y,		/* quantizer step size */
              int16_t wi,		/* scale factor multiplier */
              int16_t fi,		/* for long/short term energies */
              int16_t dq,		/* quantized prediction difference */
              int16_t sr,		/* reconstructed signal */
              int16_t dqsez,		/* difference from 2-pole predictor */
              g726_state *state_ptr);	/* coder state pointer */
int16_t tandem_adjust_alaw(
  int16_t sr,	/* decoder output linear PCM sample */
  int16_t se,	/* predictor estimate sample */
  int16_t y,	/* quantizer step size */
  int16_t i,	/* decoder input code */
  int16_t sign,
  int16_t *	qtab);
int16_t tandem_adjust_ulaw(
  int16_t sr,	/* decoder output linear PCM sample */
  int16_t se,	/* predictor estimate sample */
  int16_t y,	/* quantizer step size */
  int16_t i,	/* decoder input code */
  int16_t sign,
  int16_t *	qtab);

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif // _G726_H end include guard
