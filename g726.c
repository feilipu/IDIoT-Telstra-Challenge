
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

/* 16kbps version created, used 24kbps code and changing as little as possible.
   G.726 specs are available from ITU's gopher or WWW site (http://www.itu.ch)
   If any errors are found, please contact me at mrand@tamu.edu
        -Marc Randolph
*/

/*
   g726_16.c

   Description:

   g723_16_encoder(), g723_16_decoder()

   These routines comprise an implementation of the CCITT G.726 16 Kbps
   ADPCM coding algorithm.  Essentially, this implementation is identical to
   the bit level description except for a few deviations which take advantage
   of workstation attributes, such as hardware 2's complement arithmetic.

   The ITU-T G.726 coder is an adaptive differential pulse code modulation
   (ADPCM) waveform coding algorithm, suitable for coding of digitized
   telephone bandwidth (0.3-3.4 kHz) speech or audio signals sampled at 8 kHz.
   This coder operates on a sample-by-sample basis. Input samples may be
   represented in linear PCM or companded 8-bit G.711 (m-law/A-law) formats
   (i.e., 64 kbps). For 32 kbps operation, each sample is converted into a
   4-bit quantized difference signal resulting in a compression ratio of
   2:1 over the G.711 format. For 24 kbps 40 kbps operation, the quantized
   difference signal is 3 bits and 5 bits, respectively.

   $Log: g726_16.c,v $
   Revision 1.4  2002/11/20 04:29:13  robertj
   Included optimisations for G.711 and G.726 codecs, thanks Ted Szoczei

   Revision 1.1  2002/02/11 23:24:23  robertj
   Updated to openH323 v1.8.0

   Revision 1.2  2002/02/10 21:14:54  dereks
   Add cvs log history to head of the file.
   Ensure file is terminated by a newline.


*/


#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "g726.h"

/*
   Maps G.723_16 code word to reconstructed scale factor normalized log
   magnitude values.  Comes from Table 11/G.726
*/
static int16_t	_dqlntab[4] = { 116, 365, 365, 116};

/* Maps G.723_16 code word to log of scale factor multiplier.

   _witab[4] is actually {-22 , 439, 439, -22}, but FILTD wants it
   as WI << 5  (multiplied by 32), so we'll do that here
*/
static int16_t	_witab[4] = { -704, 14048, 14048, -704};

/*
   Maps G.723_16 code words to a set of values whose long and short
   term averages are computed and then compared to give an indication
   how stationary (steady state) the signal is.
*/

/* Comes from FUNCTF */
static int16_t	_fitab[4] = {0, 0xE00, 0xE00, 0};

/* Comes from quantizer decision level tables (Table 7/G.726)
*/
static int16_t qtab_723_16[1] = {261};


/*
   g723_16_encoder()

   Encodes a linear PCM, A-law or u-law input sample and returns its 2-bit code.
   Returns -1 if invalid input coding value.
*/
int16_t
g726_16_encoder(
  int16_t sl,
  int16_t in_coding,
  g726_state *state_ptr)
{
  int16_t sezi;
  int16_t sez;			/* ACCUM */
  int16_t sei;
  int16_t se;
  int16_t d;				/* SUBTA */
  int16_t y;				/* MIX */
  int16_t i;
  int16_t dq;
  int16_t sr;				/* ADDB */
  int16_t dqsez;			/* ADDC */

  switch (in_coding) {	/* linearize input sample to 14-bit PCM */
    case AUDIO_ENCODING_ALAW:
      sl = alaw2linear(sl) >> 2;
      break;
    case AUDIO_ENCODING_ULAW:
      sl = ulaw2linear(sl) >> 2;
      break;
    case AUDIO_ENCODING_LINEAR:
      sl >>= 2;		/* sl of 14-bit dynamic range */
      break;
    default:
      return (-1);
  }

  sezi = predictor_zero(state_ptr);
  sez = sezi >> 1;
  sei = sezi + predictor_pole(state_ptr);
  se = sei >> 1;			/* se = estimated signal */

  d = sl - se;			/* d = estimation diff. */

  /* quantize prediction difference d */
  y = step_size(state_ptr);	/* quantizer step size */
  i = quantize(d, y, qtab_723_16, 1);  /* i = ADPCM code */

  /* Since quantize() only produces a three level output
     (1, 2, or 3), we must create the fourth one on our own
  */
  if (i == 3)                          /* i code for the zero region */
    if ((d & 0x8000) == 0)             /* If d > 0, i=3 isn't right... */
      i = 0;

  dq = reconstruct(i & 2, _dqlntab[i], y); /* quantized diff. */

  sr = (dq < 0) ? se - (dq & 0x3FFF) : se + dq; /* reconstructed signal */

  dqsez = sr + sez - se;		/* pole prediction diff. */

  update(2, y, _witab[i], _fitab[i], dq, sr, dqsez, state_ptr);

  return (i);
}



/*
   g723_16_decoder()

   Decodes a 2-bit CCITT G.723_16 ADPCM code and returns
   the resulting 16-bit linear PCM, A-law or u-law sample value.
   -1 is returned if the output coding is unknown.
*/
int16_t
g726_16_decoder(
  int16_t i,
  int16_t out_coding,
  g726_state *state_ptr)
{
  int16_t sezi;
  int16_t sez;			/* ACCUM */
  int16_t sei;
  int16_t se;
  int16_t y;				/* MIX */
  int16_t dq;
  int16_t sr;				/* ADDB */
  int16_t dqsez;

  i &= 0x03;			/* mask to get proper bits */
  sezi = predictor_zero(state_ptr);
  sez = sezi >> 1;
  sei = sezi + predictor_pole(state_ptr);
  se = sei >> 1;			/* se = estimated signal */

  y = step_size(state_ptr);	/* adaptive quantizer step size */
  dq = reconstruct(i & 0x02, _dqlntab[i], y); /* unquantize pred diff */

  sr = (dq < 0) ? (se - (dq & 0x3FFF)) : (se + dq); /* reconst. signal */

  dqsez = sr - se + sez;			/* pole prediction diff. */

  update(2, y, _witab[i], _fitab[i], dq, sr, dqsez, state_ptr);

  switch (out_coding) {
    case AUDIO_ENCODING_ALAW:
      return (tandem_adjust_alaw(sr, se, y, i, 2, qtab_723_16));
    case AUDIO_ENCODING_ULAW:
      return (tandem_adjust_ulaw(sr, se, y, i, 2, qtab_723_16));
    case AUDIO_ENCODING_LINEAR:
      return (sr << 2);	/* sr was of 14-bit dynamic range */
    default:
      return (-1);
  }
}


/*
   Common routines for G.721 and G.723 conversions.
*/

static int16_t power2[15] = {1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80,
                             0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000
                            };
/*
   quan()

   quantizes the input val against the table of size short integers.
   It returns i if table[i - 1] <= val < table[i].

   Using linear search for simple coding.
*/
static int16_t
quan(
  int16_t val,
  int16_t *	table,
  int16_t size)
{
  int16_t i;

  for (i = 0; i < size; i++)
    if (val < *table++)
      break;
  return (i);
}

/*
   fmult()

   returns the integer product of the 14-bit integer "an" and
   "floating point" representation (4-bit exponent, 6-bit mantessa) "srn".
*/
static int16_t
fmult(
  int16_t an,
  int16_t srn)
{
  int16_t anmag;
  int16_t anexp;
  int16_t anmant;
  int16_t wanexp;
  int16_t wanmant;
  int16_t retval;

  anmag = (an > 0) ? an : ((-an) & 0x1FFF);
  anexp = quan(anmag, power2, 15) - 6;
  anmant = (anmag == 0) ? 32 :
           (anexp >= 0) ? anmag >> anexp : anmag << -anexp;
  wanexp = anexp + ((srn >> 6) & 0xF) - 13;

  wanmant = (anmant * (srn & 077) + 0x30) >> 4;
  retval = (wanexp >= 0) ? ((wanmant << wanexp) & 0x7FFF) :
           (wanmant >> -wanexp);

  return (((an ^ srn) < 0) ? -retval : retval);
}

/*
   g72x_init_state()

   This routine initializes and/or resets the g72x_state structure
   pointed to by 'state_ptr'.
   All the initial state values are specified in the CCITT G.721 document.
*/
void
g726_init_state(
  g726_state *state_ptr)
{
  int16_t cnta;

  state_ptr->yl = 34816;
  state_ptr->yu = 544;
  state_ptr->dms = 0;
  state_ptr->dml = 0;
  state_ptr->ap = 0;
  for (cnta = 0; cnta < 2; cnta++) {
    state_ptr->a[cnta] = 0;
    state_ptr->pk[cnta] = 0;
    state_ptr->sr[cnta] = 32;
  }
  for (cnta = 0; cnta < 6; cnta++) {
    state_ptr->b[cnta] = 0;
    state_ptr->dq[cnta] = 32;
  }
  state_ptr->td = 0;
}

/*
   predictor_zero()

   computes the estimated signal from 6-zero predictor.

*/
int16_t
predictor_zero(
  g726_state *state_ptr)
{
  int16_t i;
  int16_t sezi;

  sezi = fmult(state_ptr->b[0] >> 2, state_ptr->dq[0]);
  for (i = 1; i < 6; i++)			/* ACCUM */
    sezi += fmult(state_ptr->b[i] >> 2, state_ptr->dq[i]);
  return (sezi);
}
/*
   predictor_pole()

   computes the estimated signal from 2-pole predictor.

*/
int16_t
predictor_pole(
  g726_state *state_ptr)
{
  return (fmult(state_ptr->a[1] >> 2, state_ptr->sr[1]) +
          fmult(state_ptr->a[0] >> 2, state_ptr->sr[0]));
}
/*
   step_size()

   computes the quantization step size of the adaptive quantizer.

*/
int16_t
step_size(
  g726_state *state_ptr)
{
  int16_t y;
  int16_t dif;
  int16_t al;

  if (state_ptr->ap >= 256)
    return (state_ptr->yu);
  else {
    y = state_ptr->yl >> 6;
    dif = state_ptr->yu - y;
    al = state_ptr->ap >> 2;
    if (dif > 0)
      y += (dif * al) >> 6;
    else if (dif < 0)
      y += (dif * al + 0x3F) >> 6;
    return (y);
  }
}

/*
   quantize()

   Given a raw sample, 'd', of the difference signal and a
   quantization step size scale factor, 'y', this routine returns the
   ADPCM codeword to which that sample gets quantized.  The step
   size scale factor division operation is done in the log base 2 domain
   as a subtraction.
*/
int16_t
quantize(
  int16_t d,	/* Raw difference signal sample */
  int16_t y,	/* Step size multiplier */
  int	*	table,	/* quantization table */
  int16_t size)	/* table size of integers */
{
  int16_t dqm;	/* Magnitude of 'd' */
  int16_t exp;	/* Integer part of base 2 log of 'd' */
  int16_t mant;	/* Fractional part of base 2 log */
  int16_t dl;	/* Log of magnitude of 'd' */
  int16_t dln;	/* Step size scale factor normalized log */
  int16_t i;

  /*
     LOG

     Compute base 2 log of 'd', and store in 'dl'.
  */
  dqm = abs(d);
  exp = quan(dqm >> 1, power2, 15);
  mant = ((dqm << 7) >> exp) & 0x7F;	/* Fractional portion. */
  dl = (exp << 7) + mant;

  /*
     SUBTB

     "Divide" by step size multiplier.
  */
  dln = dl - (y >> 2);

  /*
     QUAN

     Obtain codword i for 'd'.
  */
  i = quan(dln, table, size);
  if (d < 0)			/* take 1's complement of i */
    return ((size << 1) + 1 - i);
  else if (i == 0)		/* take 1's complement of 0 */
    return ((size << 1) + 1); /* new in 1988 */
  else
    return (i);
}
/*
   reconstruct()

   Returns reconstructed difference signal 'dq' obtained from
   codeword 'i' and quantization step size scale factor 'y'.
   Multiplication is performed in log base 2 domain as addition.
*/
int16_t
reconstruct(
  int16_t sign,	/* 0 for non-negative value */
  int16_t dqln,	/* G.72x codeword */
  int16_t y)		/* Step size multiplier */
{
  int16_t dql;	/* Log of 'dq' magnitude */
  int16_t dex;	/* Integer part of log */
  int16_t dqt;
  int16_t dq;		/* Reconstructed difference signal sample */

  dql = dqln + (y >> 2);	/* ADDA */

  if (dql < 0) {
    return ((sign) ? -0x8000 : 0);
  } else {		/* ANTILOG */
    dex = (dql >> 7) & 15;
    dqt = 128 + (dql & 127);
    dq = (int16_t)((dqt << 7) >> (14 - dex));
    return ((sign) ? (dq - 0x8000) : dq);
  }
}


/*
   update()

   updates the state variables for each output code
*/
void
update(
  int16_t code_size,	/* distinguish 723_40 with others */
  int16_t y,		/* quantizer step size */
  int16_t wi,		/* scale factor multiplier */
  int16_t fi,		/* for long/short term energies */
  int16_t dq,		/* quantized prediction difference */
  int16_t sr,		/* reconstructed signal */
  int16_t dqsez,		/* difference from 2-pole predictor */
  g726_state *state_ptr)	/* coder state pointer */
{
  int16_t cnt;
  int16_t mag, exp;	/* Adaptive predictor, FLOAT A */
  int16_t a2p;		/* LIMC */
  int16_t a1ul;		/* UPA1 */
  int16_t pks1;		/* UPA2 */
  int16_t fa1;
  int16_t tr;			/* tone/transition detector */
  int16_t ylint, thr2, dqthr;
  int16_t  	ylfrac, thr1;
  int16_t pk0;

  pk0 = (dqsez < 0) ? 1 : 0;	/* needed in updating predictor poles */

  mag = dq & 0x7FFF;		/* prediction difference magnitude */
  /* TRANS */
  ylint = state_ptr->yl >> 15;	/* exponent part of yl */
  ylfrac = (state_ptr->yl >> 10) & 0x1F;	/* fractional part of yl */
  thr1 = (32 + ylfrac) << ylint;		/* threshold */
  thr2 = (ylint > 9) ? 31 << 10 : thr1;	/* limit thr2 to 31 << 10 */
  dqthr = (thr2 + (thr2 >> 1)) >> 1;	/* dqthr = 0.75 * thr2 */
  if (state_ptr->td == 0)		/* signal supposed voice */
    tr = 0;
  else if (mag <= dqthr)		/* supposed data, but small mag */
    tr = 0;			/* treated as voice */
  else				/* signal is data (modem) */
    tr = 1;

  /*
     Quantizer scale factor adaptation.
  */

  /* FUNCTW & FILTD & DELAY */
  /* update non-steady state step size multiplier */
  state_ptr->yu = y + ((wi - y) >> 5);

  /* LIMB */
  if (state_ptr->yu < 544)	/* 544 <= yu <= 5120 */
    state_ptr->yu = 544;
  else if (state_ptr->yu > 5120)
    state_ptr->yu = 5120;

  /* FILTE & DELAY */
  /* update steady state step size multiplier */
  state_ptr->yl += state_ptr->yu + ((-state_ptr->yl) >> 6);

  /*
     Adaptive predictor coefficients.
  */
  if (tr == 1) {			/* reset a's and b's for modem signal */
    state_ptr->a[0] = 0;
    state_ptr->a[1] = 0;
    state_ptr->b[0] = 0;
    state_ptr->b[1] = 0;
    state_ptr->b[2] = 0;
    state_ptr->b[3] = 0;
    state_ptr->b[4] = 0;
    state_ptr->b[5] = 0;
    a2p = 0 ;
  } else {			/* update a's and b's */
    pks1 = pk0 ^ state_ptr->pk[0];		/* UPA2 */

    /* update predictor pole a[1] */
    a2p = state_ptr->a[1] - (state_ptr->a[1] >> 7);
    if (dqsez != 0) {
      fa1 = (pks1) ? state_ptr->a[0] : -state_ptr->a[0];
      if (fa1 < -8191)	/* a2p = function of fa1 */
        a2p -= 0x100;
      else if (fa1 > 8191)
        a2p += 0xFF;
      else
        a2p += fa1 >> 5;

      if (pk0 ^ state_ptr->pk[1])
        /* LIMC */
        if (a2p <= -12160)
          a2p = -12288;
        else if (a2p >= 12416)
          a2p = 12288;
        else
          a2p -= 0x80;
      else if (a2p <= -12416)
        a2p = -12288;
      else if (a2p >= 12160)
        a2p = 12288;
      else
        a2p += 0x80;
    }

    /* TRIGB & DELAY */
    state_ptr->a[1] = a2p;

    /* UPA1 */
    /* update predictor pole a[0] */
    state_ptr->a[0] -= state_ptr->a[0] >> 8;
    if (dqsez != 0)
      if (pks1 == 0)
        state_ptr->a[0] += 192;
      else
        state_ptr->a[0] -= 192;

    /* LIMD */
    a1ul = 15360 - a2p;
    if (state_ptr->a[0] < -a1ul)
      state_ptr->a[0] = -a1ul;
    else if (state_ptr->a[0] > a1ul)
      state_ptr->a[0] = a1ul;

    /* UPB : update predictor zeros b[6] */
    for (cnt = 0; cnt < 6; cnt++) {
      if (code_size == 5)		/* for 40Kbps G.723 */
        state_ptr->b[cnt] -= state_ptr->b[cnt] >> 9;
      else			/* for G.721 and 24Kbps G.723 */
        state_ptr->b[cnt] -= state_ptr->b[cnt] >> 8;
      if (dq & 0x7FFF) {			/* XOR */
        if ((dq ^ state_ptr->dq[cnt]) >= 0)
          state_ptr->b[cnt] += 128;
        else
          state_ptr->b[cnt] -= 128;
      }
    }
  }

  for (cnt = 5; cnt > 0; cnt--)
    state_ptr->dq[cnt] = state_ptr->dq[cnt - 1];
  /* FLOAT A : convert dq[0] to 4-bit exp, 6-bit mantissa f.p. */
  if (mag == 0) {
    state_ptr->dq[0] = (dq >= 0) ? 0x20 : 0xFC20;
  } else {
    exp = quan(mag, power2, 15);
    state_ptr->dq[0] = (int16_t)((dq >= 0) ?
                                 (exp << 6) + ((mag << 6) >> exp) :
                                 (exp << 6) + ((mag << 6) >> exp) - 0x400);
  }

  state_ptr->sr[1] = state_ptr->sr[0];
  /* FLOAT B : convert sr to 4-bit exp., 6-bit mantissa f.p. */
  if (sr == 0) {
    state_ptr->sr[0] = 0x20;
  } else if (sr > 0) {
    exp = quan(sr, power2, 15);
    state_ptr->sr[0] = (exp << 6) + ((sr << 6) >> exp);
  } else if (sr > -32768) {
    mag = -sr;
    exp = quan(mag, power2, 15);
    state_ptr->sr[0] =  (exp << 6) + ((mag << 6) >> exp) - 0x400;
  } else
    state_ptr->sr[0] = 0xFC20;

  /* DELAY A */
  state_ptr->pk[1] = state_ptr->pk[0];
  state_ptr->pk[0] = pk0;

  /* TONE */
  if (tr == 1)		/* this sample has been treated as data */
    state_ptr->td = 0;	/* next one will be treated as voice */
  else if (a2p < -11776)	/* small sample-to-sample correlation */
    state_ptr->td = 1;	/* signal may be data */
  else				/* signal is voice */
    state_ptr->td = 0;

  /*
     Adaptation speed control.
  */
  state_ptr->dms += (fi - state_ptr->dms) >> 5;		/* FILTA */
  state_ptr->dml += (((fi << 2) - state_ptr->dml) >> 7);	/* FILTB */

  if (tr == 1)
    state_ptr->ap = 256;
  else if (y < 1536)					/* SUBTC */
    state_ptr->ap += (0x200 - state_ptr->ap) >> 4;
  else if (state_ptr->td == 1)
    state_ptr->ap += (0x200 - state_ptr->ap) >> 4;
  else if (abs((state_ptr->dms << 2) - state_ptr->dml) >=
           (state_ptr->dml >> 3))
    state_ptr->ap += (0x200 - state_ptr->ap) >> 4;
  else
    state_ptr->ap += (-state_ptr->ap) >> 4;
}

/*
   tandem_adjust(sr, se, y, i, sign)

   At the end of ADPCM decoding, it simulates an encoder which may be receiving
   the output of this decoder as a tandem process. If the output of the
   simulated encoder differs from the input to this decoder, the decoder output
   is adjusted by one level of A-law or u-law codes.

   Input:
 	sr	decoder output linear PCM sample,
 	se	predictor estimate sample,
 	y	quantizer step size,
 	i	decoder input code,
 	sign	sign bit of code i

   Return:
 	adjusted A-law or u-law compressed sample.
*/
int16_t
tandem_adjust_alaw(
  int16_t sr,	/* decoder output linear PCM sample */
  int16_t se,	/* predictor estimate sample */
  int16_t y,	/* quantizer step size */
  int16_t i,	/* decoder input code */
  int16_t sign,
  int16_t *	qtab)
{
  int16_t sp;	/* A-law compressed 8-bit code */
  int16_t dx;	/* prediction error */
  int16_t id;	/* quantized prediction error */
  int16_t sd;	/* adjusted A-law decoded sample value */
  int16_t im;	/* biased magnitude of i */
  int16_t imx;	/* biased magnitude of id */

  if (sr <= -32768)
    sr = -1;
  sp = linear2alaw((sr >> 1) << 3);	/* short to A-law compression */
  dx = (alaw2linear(sp) >> 2) - se;	/* 16-bit prediction error */
  id = quantize(dx, y, qtab, sign - 1);

  if (id == i) {			/* no adjustment on sp */
    return (sp);
  } else {			/* sp adjustment needed */
    /* ADPCM codes : 8, 9, ... F, 0, 1, ... , 6, 7 */
    im = i ^ sign;		/* 2's complement to biased unsigned */
    imx = id ^ sign;

    if (imx > im) {		/* sp adjusted to next lower value */
      if (sp & 0x80) {
        sd = (sp == 0xD5) ? 0x55 :
             ((sp ^ 0x55) - 1) ^ 0x55;
      } else {
        sd = (sp == 0x2A) ? 0x2A :
             ((sp ^ 0x55) + 1) ^ 0x55;
      }
    } else {		/* sp adjusted to next higher value */
      if (sp & 0x80)
        sd = (sp == 0xAA) ? 0xAA :
             ((sp ^ 0x55) + 1) ^ 0x55;
      else
        sd = (sp == 0x55) ? 0xD5 :
             ((sp ^ 0x55) - 1) ^ 0x55;
    }
    return (sd);
  }
}

int16_t
tandem_adjust_ulaw(
  int16_t sr,	/* decoder output linear PCM sample */
  int16_t se,	/* predictor estimate sample */
  int16_t y,	/* quantizer step size */
  int16_t i,	/* decoder input code */
  int16_t sign,
  int16_t *	qtab)
{
  int16_t sp;	/* u-law compressed 8-bit code */
  int16_t dx;	/* prediction error */
  int16_t id;	/* quantized prediction error */
  int16_t sd;	/* adjusted u-law decoded sample value */
  int16_t im;	/* biased magnitude of i */
  int16_t imx;	/* biased magnitude of id */

  if (sr <= -32768)
    sr = 0;
  sp = linear2ulaw(sr << 2);	/* short to u-law compression */
  dx = (ulaw2linear(sp) >> 2) - se;	/* 16-bit prediction error */
  id = quantize(dx, y, qtab, sign - 1);
  if (id == i) {
    return (sp);
  } else {
    /* ADPCM codes : 8, 9, ... F, 0, 1, ... , 6, 7 */
    im = i ^ sign;		/* 2's complement to biased unsigned */
    imx = id ^ sign;
    if (imx > im) {		/* sp adjusted to next lower value */
      if (sp & 0x80)
        sd = (sp == 0xFF) ? 0x7E : sp + 1;
      else
        sd = (sp == 0) ? 0 : sp - 1;

    } else {		/* sp adjusted to next higher value */
      if (sp & 0x80)
        sd = (sp == 0x80) ? 0x80 : sp - 1;
      else
        sd = (sp == 0x7F) ? 0xFE : sp + 1;
    }
    return (sd);
  }
}

/* Allocate and Init. codec struct */



/*
   g711.c

   u-law, A-law and linear PCM conversions.
*/

/*
   December 30, 1994:
   Functions linear2alaw, linear2ulaw have been updated to correctly
   convert unquantized 16 bit values.
   Tables for direct u- to A-law and A- to u-law conversions have been
   corrected.
   Borge Lindberg, Center for PersonKommunikation, Aalborg University.
   bli@cpk.auc.dk

*/

#define	SIGN_BIT	(0x80)		/* Sign bit for a A-law byte. */
#define	QUANT_MASK	(0xf)		/* Quantization field mask. */
#define	NSEGS		(8)		/* Number of A-law segments. */
#define	SEG_SHIFT	(4)		/* Left shift for segment number. */
#define	SEG_MASK	(0x70)		/* Segment field mask. */

static int16_t seg_aend[8] = {0x1F, 0x3F, 0x7F, 0xFF,
                              0x1FF, 0x3FF, 0x7FF, 0xFFF
                             };
static int16_t seg_uend[8] = {0x3F, 0x7F, 0xFF, 0x1FF,
                              0x3FF, 0x7FF, 0xFFF, 0x1FFF
                             };

/* copy from CCITT G.711 specifications */
uint8_t u2a[128] = {			/* u- to A-law conversions */
  1,	1,	2,	2,	3,	3,	4,	4,
  5,	5,	6,	6,	7,	7,	8,	8,
  9,	10,	11,	12,	13,	14,	15,	16,
  17,	18,	19,	20,	21,	22,	23,	24,
  25,	27,	29,	31,	33,	34,	35,	36,
  37,	38,	39,	40,	41,	42,	43,	44,
  46,	48,	49,	50,	51,	52,	53,	54,
  55,	56,	57,	58,	59,	60,	61,	62,
  64,	65,	66,	67,	68,	69,	70,	71,
  72,	73,	74,	75,	76,	77,	78,	79,
  /* corrected:
  	81,	82,	83,	84,	85,	86,	87,	88,
     should be: */
  80,	82,	83,	84,	85,	86,	87,	88,
  89,	90,	91,	92,	93,	94,	95,	96,
  97,	98,	99,	100,	101,	102,	103,	104,
  105,	106,	107,	108,	109,	110,	111,	112,
  113,	114,	115,	116,	117,	118,	119,	120,
  121,	122,	123,	124,	125,	126,	127,	128
};

uint8_t a2u[128] = {			/* A- to u-law conversions */
  1,	3,	5,	7,	9,	11,	13,	15,
  16,	17,	18,	19,	20,	21,	22,	23,
  24,	25,	26,	27,	28,	29,	30,	31,
  32,	32,	33,	33,	34,	34,	35,	35,
  36,	37,	38,	39,	40,	41,	42,	43,
  44,	45,	46,	47,	48,	48,	49,	49,
  50,	51,	52,	53,	54,	55,	56,	57,
  58,	59,	60,	61,	62,	63,	64,	64,
  65,	66,	67,	68,	69,	70,	71,	72,
  /* corrected:
  	73,	74,	75,	76,	77,	78,	79,	79,
     should be: */
  73,	74,	75,	76,	77,	78,	79,	80,

  80,	81,	82,	83,	84,	85,	86,	87,
  88,	89,	90,	91,	92,	93,	94,	95,
  96,	97,	98,	99,	100,	101,	102,	103,
  104,	105,	106,	107,	108,	109,	110,	111,
  112,	113,	114,	115,	116,	117,	118,	119,
  120,	121,	122,	123,	124,	125,	126,	127
};

static int16_t
search(
  int16_t val,	/* changed from "short" *drago* */
  int16_t *	table,
  int16_t size)	/* changed from "short" *drago* */
{
  int16_t i;		/* changed from "short" *drago* */

  for (i = 0; i < size; i++) {
    if (val <= *table++)
      return (i);
  }
  return (size);
}

/*
   linear2alaw() - Convert a 16-bit linear PCM value to 8-bit A-law

   linear2alaw() accepts an 16-bit integer and encodes it as A-law data.

 		Linear Input Code	Compressed Code
 	------------------------	---------------
 	0000000wxyza			000wxyz
 	0000001wxyza			001wxyz
 	000001wxyzab			010wxyz
 	00001wxyzabc			011wxyz
 	0001wxyzabcd			100wxyz
 	001wxyzabcde			101wxyz
 	01wxyzabcdef			110wxyz
 	1wxyzabcdefg			111wxyz

   For further information see John C. Bellamy's Digital Telephony, 1982,
   John Wiley & Sons, pps 98-111 and 472-476.
*/
int16_t linear2alaw(int16_t	pcm_val)        /* 2's complement (16-bit range) */
/* changed from "short" *drago* */
{
  int16_t mask;	/* changed from "short" *drago* */
  int16_t seg;	/* changed from "short" *drago* */
  int16_t aval;

  pcm_val = pcm_val >> 3;

  if (pcm_val >= 0) {
    mask = 0xD5;		/* sign (7th) bit = 1 */
  } else {
    mask = 0x55;		/* sign bit = 0 */
    pcm_val = -pcm_val - 1;
  }

  /* Convert the scaled magnitude to segment number. */
  seg = search(pcm_val, seg_aend, 8);

  /* Combine the sign, segment, and quantization bits. */

  if (seg >= 8)		/* out of range, return maximum value. */
    return (0x7F ^ mask);
  else {
    aval = seg << SEG_SHIFT;
    if (seg < 2)
      aval |= (pcm_val >> 1) & QUANT_MASK;
    else
      aval |= (pcm_val >> seg) & QUANT_MASK;
    return (aval ^ mask);
  }
}

/*
   alaw2linear() - Convert an A-law value to 16-bit linear PCM

*/
int16_t alaw2linear(int16_t	a_val)
{
  int16_t t;      /* changed from "short" *drago* */
  int16_t seg;    /* changed from "short" *drago* */

  a_val ^= 0x55;

  t = (a_val & QUANT_MASK) << 4;
  seg = ((unsigned)a_val & SEG_MASK) >> SEG_SHIFT;
  switch (seg) {
    case 0:
      t += 8;
      break;
    case 1:
      t += 0x108;
      break;
    default:
      t += 0x108;
      t <<= seg - 1;
  }
  return ((a_val & SIGN_BIT) ? t : -t);
}

#define	BIAS		(0x84)		/* Bias for linear code. */
#define CLIP            8159

/*
   linear2ulaw() - Convert a linear PCM value to u-law

   In order to simplify the encoding process, the original linear magnitude
   is biased by adding 33 which shifts the encoding range from (0 - 8158) to
   (33 - 8191). The result can be seen in the following encoding table:

 	Biased Linear Input Code	Compressed Code
 	------------------------	---------------
 	00000001wxyza			000wxyz
 	0000001wxyzab			001wxyz
 	000001wxyzabc			010wxyz
 	00001wxyzabcd			011wxyz
 	0001wxyzabcde			100wxyz
 	001wxyzabcdef			101wxyz
 	01wxyzabcdefg			110wxyz
 	1wxyzabcdefgh			111wxyz

   Each biased linear code has a leading 1 which identifies the segment
   number. The value of the segment number is equal to 7 minus the number
   of leading 0's. The quantization interval is directly available as the
   four bits wxyz.  * The trailing bits (a - h) are ignored.

   Ordinarily the complement of the resulting code word is used for
   transmission, and so the code word is complemented before it is returned.

   For further information see John C. Bellamy's Digital Telephony, 1982,
   John Wiley & Sons, pps 98-111 and 472-476.
*/
int16_t linear2ulaw( int16_t	pcm_val)	/* 2's complement (16-bit range) */
{
  int16_t mask;
  int16_t seg;
  int16_t uval;

  /* Get the sign and the magnitude of the value. */
  pcm_val = pcm_val >> 2;
  if (pcm_val < 0) {
    pcm_val = -pcm_val;
    mask = 0x7F;
  } else {
    mask = 0xFF;
  }
  if ( pcm_val > CLIP ) pcm_val = CLIP;		/* clip the magnitude */
  pcm_val += (BIAS >> 2);

  /* Convert the scaled magnitude to segment number. */
  seg = search(pcm_val, seg_uend, 8);

  /*
     Combine the sign, segment, quantization bits;
     and complement the code word.
  */
  if (seg >= 8)		/* out of range, return maximum value. */
    return (0x7F ^ mask);
  else {
    uval = (seg << 4) | ((pcm_val >> (seg + 1)) & 0xF);
    return (uval ^ mask);
  }

}

/*
   ulaw2linear() - Convert a u-law value to 16-bit linear PCM

   First, a biased linear code is derived from the code word. An unbiased
   output can then be obtained by subtracting 33 from the biased code.

   Note that this function expects to be passed the complement of the
   original code word. This is in keeping with ISDN conventions.
*/
int16_t ulaw2linear( int16_t	u_val)
{
  int16_t t;

  /* Complement to obtain normal u-law value. */
  u_val = ~u_val;

  /*
     Extract and bias the quantization bits. Then
     shift up by the segment number and subtract out the bias.
  */
  t = ((u_val & QUANT_MASK) << 3) + BIAS;
  t <<= (u_val & SEG_MASK) >> SEG_SHIFT;

  return ((u_val & SIGN_BIT) ? (BIAS - t) : (t - BIAS));
}

/* A-law to u-law conversion */
static int16_t alaw2ulaw (int16_t	aval)
{
  aval &= 0xff;
  return ((aval & 0x80) ? (0xFF ^ a2u[aval ^ 0xD5]) :
          (0x7F ^ a2u[aval ^ 0x55]));
}

/* u-law to A-law conversion */
static int16_t ulaw2alaw (int16_t	uval)
{
  uval &= 0xff;
  return ((uval & 0x80) ? (0xD5 ^ (u2a[0xFF ^ uval] - 1)) :
          (0x55 ^ (u2a[0x7F ^ uval] - 1)));
}


