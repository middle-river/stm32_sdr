/*
  Signal processing library.
  2020-09-31  T. Nakagawa
*/

#ifndef DSP_H_
#define DSP_H_

#pragma GCC optimize ("Ofast")
#define OPTIMIZE_LEVEL 2	// 0=Unoptimized, 1=Improved algorithm, 2=Assembly language.

#include <cmath>
#include "Filter.h"

#ifdef ARDUINO_ARCH_STM32
static inline int32_t __SMULBB(uint32_t x, uint32_t y) { int32_t r; asm volatile("smulbb %[r], %[x], %[y]" : [r] "=r" (r) : [x] "r" (x), [y] "r" (y) : ); return r; }
static inline int32_t __SMULTT(uint32_t x, uint32_t y) { int32_t r; asm volatile("smultt %[r], %[x], %[y]" : [r] "=r" (r) : [x] "r" (x), [y] "r" (y) : ); return r; }
#else
static inline uint32_t __SMLAD(uint32_t x, uint32_t y, uint32_t z) { int16_t *xx = (int16_t *)&x, *yy = (int16_t *)&y; return (xx[0] * yy[0] + xx[1] * yy[1] + z); }
static inline uint32_t __SSUB16(uint32_t x, uint32_t y) { int16_t *xx = (int16_t *)&x, *yy = (int16_t *)&y; uint32_t z = 0; int16_t *zz = (int16_t *)&z; zz[0] = xx[0] - yy[0]; zz[1] = xx[1] - yy[1]; return z; }
static inline int32_t __SMULBB(uint32_t x, uint32_t y) { int16_t *xx = (int16_t *)&x, *yy = (int16_t *)&y; return ((int32_t)xx[0] * yy[0]); }
static inline int32_t __SMULTT(uint32_t x, uint32_t y) { int16_t *xx = (int16_t *)&x, *yy = (int16_t *)&y; return ((int32_t)xx[1] * yy[1]); }
static inline int32_t __SSAT(int32_t val, uint32_t sat) { const int32_t lim = 1 << (sat - 1); return (val < -lim) ? -lim : (val >= lim) ? (lim - 1) : val;  }
#endif
static inline int ilog2(int x) { return sizeof(int) * 8 - __builtin_clz(x) - 1; }

extern "C" {
int32_t offset2(int32_t ofst, int size, const int16_t *inp, int16_t *out);
void decimator_cic2(const int16_t *tbl, int32_t *state, int size, const int16_t *inp, float *out);
void decimator_fir2(const float *coeff, float *state, int size, const float *inp, float *out);
void bpf_iir2(const float *coeff, float *state, int size, const float *inp, float *out);
void demodulator_amplitude2(int size, const float *inp, float *out);
void demodulator_mixer2(const float *tbl, int size, const float *inp, float *out);
void demodulator_hilbert2(const float *coeff, float *state, int size, const float *inp, float *out);
float amplifier2(float ofst, float vol, float max, int size, const float *inp, int16_t *out);
void snapshot2(float gain, int size, const float *inp, int16_t *out);
}

/*
  Make a cos/sin table for Numerically Controlled Oscillator.
  <tbl>: int16_t[2 * len].
*/
static inline void nco(int samp, float freq, int len, int16_t *tbl) {
  for (int i = 0; i < len; i++) {
    const float rad = i * 2.0f * (float)M_PI * freq / samp;
    tbl[i      ] = (int16_t)(sinf(rad) * 32767.0f);
    tbl[i + len] = (int16_t)(cosf(rad) * 32767.0f);
  }
}

/*
  DC offset removal.
  <inp>: int16_t[size], <out>: int16_t[size]
*/
static inline void offset(int32_t &ofst, int size, const int16_t *inp, int16_t *out) {
#if OPTIMIZE_LEVEL == 0
  const int16_t bias = (int16_t)(ofst >> 16);
  int32_t sum = 0;
  for (int i = 0; i < size; i++) {
    sum += inp[i];
    out[i] = inp[i] - bias;
  }
  ofst = (int32_t)((uint64_t)ofst * (65536 - 2048) / 65536 + (uint64_t)sum * 2048 / size);
#elif OPTIMIZE_LEVEL == 1
  const uint32_t *inp32 = (const uint32_t *)inp;
  uint32_t *out32 = (uint32_t *)out;
  uint32_t bias = ofst >> 16;
  bias |= bias << 16;
  int32_t sum = 0;
  for (int i = size / 8 / 2; i != 0; i--) {
    uint32_t acc[8];
    for (int j = 0; j < 8; j++) acc[j] = *inp32++;
    for (int j = 0; j < 8; j++) {
      sum = __SMLAD(acc[j], 0x00010001, sum);
      acc[j] = __SSUB16(acc[j], bias);
    }
    for (int j = 0; j < 8; j++) *out32++ = acc[j];
  }
  ofst -= ofst >> (16 - 11);		// ofst * 2048 / 65536.
  ofst += sum >> (ilog2(size) - 11);	// sum * 2048 / size.
#elif OPTIMIZE_LEVEL == 2
  const int32_t sum = offset2(ofst, size, inp, out);
  ofst -= ofst >> (16 - 11);
  ofst += sum >> (ilog2(size) - 11);
#endif
}

/*
  Mixer and CIC decimator (3-stage, 1/32).
  <tbl>: int16_t[size], <state>: int32_t[6], <inp>: int16_t[size], <out>: float[size / 32]
*/
static inline void decimator_cic(const int16_t *tbl, int32_t *state, int size, const int16_t *inp, float *out) {
#if OPTIMIZE_LEVEL == 0
  int32_t *intg = state;
  int32_t *comb = state + 3;
  for (int i = 0; i < size; i++) {
    // Mixer.
    const int32_t sgnl = ((int32_t)tbl[i] * inp[i]) >> 10;
    // CIC decimator.
    for (int j = 0; j < 3; j++) intg[j] += (j == 0) ? sgnl : intg[j - 1];
    if (i % 32 == 31) {
      int32_t accm[3];
      for (int j = 0; j < 3; j++) {
        const int32_t y = (j == 0) ? intg[2] : accm[j - 1];
        accm[j] = y - comb[j];
        comb[j] = y;
      }
      out[i / 32] = (float)accm[2];
    }
  }
#elif OPTIMIZE_LEVEL == 1
  int32_t intg0 = state[0], intg1 = state[1], intg2 = state[2];
  int32_t comb0 = state[3], comb1 = state[4], comb2 = state[5];
  const uint32_t *tbl32 = (uint32_t *)tbl;
  const uint32_t *inp32 = (uint32_t *)inp;
  for (int i = size / 32; i != 0; i--) {
    for (int j = 0; j < 32 / 2; j++) {
      const uint32_t tblv = *tbl32++;
      const uint32_t inpv = *inp32++;
      const int32_t sgnll = __SMULBB(tblv, inpv) >> 10;
      const int32_t sgnlh = __SMULTT(tblv, inpv) >> 10;
      intg0 += sgnll; intg1 += intg0; intg2 += intg1;
      intg0 += sgnlh; intg1 += intg0; intg2 += intg1;
    }
    int32_t accm0, accm1, accm2;
    accm0 = intg2 - comb0; comb0 = intg2;
    accm1 = accm0 - comb1; comb1 = accm0;
    accm2 = accm1 - comb2; comb2 = accm1;
    *out++ = (float)accm2;
  }
  state[0] = intg0; state[1] = intg1; state[2] = intg2;
  state[3] = comb0; state[4] = comb1; state[5] = comb2;
#elif OPTIMIZE_LEVEL == 2
  decimator_cic2(tbl, state, size, inp, out);
#endif
}

/*
  FIR decimator (32-tap, 1/4)
  <coeff>: float[32], <state>: float[32], <inp>: float[size], <out>: float[size]
*/
static inline void decimator_fir(const float *coeff, float *state, int size, const float *inp, float *out) {
#if OPTIMIZE_LEVEL == 0
  for (int i = 0; i < size; i++) {
    for (int j = 31; j >= 1; j--) state[j] = state[j - 1];
    state[0] = inp[i];
    if (i % 4 == 3) {
      float acc = 0.0f;
      for (int j = 0; j < 32; j++) acc += coeff[j] * state[j];
      out[i / 4] = acc / (1 << 31);
    }
  }
#elif OPTIMIZE_LEVEL == 1
  float stt[32];
  for (int i = 0; i < 32; i++) stt[i] = state[i];
  for (int i = size / 32; i != 0; i--) {
    for (int j = 0; j < 8; j++) {
      for (int k = 0; k < 4; k++) stt[j * 4 + k] = *inp++;
      float acc = 0.0f;
      for (int k = 0; k < 16; k++) acc += coeff[k] * (stt[(j * 4 + 3 - k + 32) % 32] + stt[(j * 4 + 4 + k) % 32]);
      *out++ = acc / (1 << 31);
    }
  }
  for (int i = 0; i < 32; i++) state[i] = stt[i];
#elif OPTIMIZE_LEVEL == 2
  decimator_fir2(coeff, state, size, inp, out);
#endif
}

/*
  IIR filter (3-stage biquad).
  <coeff>: float[6], <state>: float[6], <inp>: float[size], <out>: float[size]
*/
static inline void bpf_iir(const float *coeff, float *state, int size, const float *inp, float *out) {
#if OPTIMIZE_LEVEL == 0
  for (int i = 0; i < 3; i++) {
    float *stt = state + i * 2;
    for (int j = 0; j < size; j++) {
      float acc = inp[j];
      for (int k = 1; k < 3; k++) acc -= coeff[k + 3] * stt[k - 1];
      float res = coeff[0] * acc;
      for (int k = 1; k < 3; k++) res += coeff[k] * stt[k - 1];
      stt[1] = stt[0];
      stt[0] = acc;
      out[j] = res;
    }
  }
#elif OPTIMIZE_LEVEL == 1
  float stt0[2] = {state[0], state[1]};
  float stt1[2] = {state[2], state[3]};
  float stt2[2] = {state[4], state[5]};
  for (int i = size / 16; i != 0; i--) {
    for (int j = 0; j < 16; j++) {
      float acc, res = *inp++;
      // 1st stage.
      acc = res - coeff[4] * stt0[0] - coeff[5] * stt0[1];
      res = coeff[0] * acc + coeff[1] * stt0[0] + coeff[2] * stt0[1];
      stt0[1] = stt0[0]; stt0[0] = acc;
      // 2nd stage.
      acc = res - coeff[4] * stt1[0] - coeff[5] * stt1[1];
      res = coeff[0] * acc + coeff[1] * stt1[0] + coeff[2] * stt1[1];
      stt1[1] = stt1[0]; stt1[0] = acc;
      // 3rd stage.
      acc = res - coeff[4] * stt2[0] - coeff[5] * stt2[1];
      res = coeff[0] * acc + coeff[1] * stt2[0] + coeff[2] * stt2[1];
      stt2[1] = stt2[0]; stt2[0] = acc;
      *out++ = res;
    }
  }
  state[0] = stt0[0]; state[1] = stt0[1];
  state[2] = stt1[0]; state[3] = stt1[1];
  state[4] = stt2[0]; state[5] = stt2[1];
#elif OPTIMIZE_LEVEL == 2
  bpf_iir2(coeff, state, size, inp, out);
#endif
}

/*
  Amplitude demodulator.
  <inp>: float[2 * size], <out>: float[size]
*/
static inline void demodulator_amplitude(int size, const float *inp, float *out) {
#if OPTIMIZE_LEVEL == 0
  for (int i = 0; i < size; i++) {
    const float ii = inp[i       ];
    const float iq = inp[i + size];
    out[i] = sqrtf(ii * ii + iq * iq);
  }
#elif OPTIMIZE_LEVEL == 1
  for (int i = size / 16; i != 0; i--) {
    for (int j = 0; j < 16; j++) {
      const float ii = *inp;
      const float iq = *(inp++ + size);
      *out++ = sqrtf(ii * ii + iq * iq);
    }
  }
#elif OPTIMIZE_LEVEL == 2
  demodulator_amplitude2(size, inp, out);
#endif
}

/*
  Demodulator for CW with a mixer.
  <tone>: float[2 * size], <inp>: float[2 * size], <out>: float[size]
*/
static inline void demodulator_mixer(const float *tbl, int size, const float *inp, float *out) {
#if OPTIMIZE_LEVEL == 0
  for (int i = 0; i < size; i++) {
    out[i] = 2.0f * (tbl[i] * inp[i] - tbl[i + size] * inp[i + size]);
  }
#elif OPTIMIZE_LEVEL == 1
  for (int i = size / 16; i != 0; i--) {
    for (int l = 0; l < 16; l++) {
      const float tmp = *tbl * *inp;
      *out++ = 2.0f * (tmp - *(tbl++ + size) * *(inp++ + size));
    }
  }
#elif OPTIMIZE_LEVEL == 2
  demodulator_mixer2(tbl, size, inp, out);
#endif
}

/*
  Demodulator for SSB with the Hilbert transform by 31-tap FIR.
  <coeff>: float[31], <state>: float[46], <inp>: float[2 * size], <out>: float[size]
*/
static inline void demodulator_hilbert(const float *coeff, float *state, int size, const float *inp, float *out) {
#if OPTIMIZE_LEVEL == 0
  float *si = state;	// float[31]
  float *sq = state + 31;	// float[15]
  const float *ii = inp;
  const float *iq = inp + size;
  for (int i = 0; i < size; i++) {
    for (int j = 30; j >= 1; j--) si[j] = si[j - 1];
    for (int j = 14; j >= 1; j--) sq[j] = sq[j - 1];
    si[0] = ii[i];
    sq[0] = iq[i];
    float acc = 0.0f;
    for (int j = 0; j < 31; j++) acc += coeff[j] * si[j];
    out[i] = acc - sq[31 / 2 - 1];
  }
#elif OPTIMIZE_LEVEL == 1
  float si[32]; for (int i = 0; i < 32; i++) si[i] = state[i];	// float[32]
  float *sq = state + 32;	// float[14]
  const float *ii = inp;
  const float *iq = inp + size;
  {
    for (int j = 0; j < 32; j++) {
      si[j] = *ii++;
      float acc = 0.0f;
      for (int k = 0; k < 15; k++) acc += coeff[k] * (si[(j - k + 32) % 32] - si[(j + 2 + k) % 32]);
      *out++ = acc - ((j < 14) ? sq[j] : *iq++);
    }
  }
  for (int i = size / 32 - 1; i != 0; i--) {
    for (int j = 0; j < 32; j++) {
      si[j] = *ii++;
      float acc = 0.0f;
      for (int k = 0; k < 15; k++) acc += coeff[k] * (si[(j - k + 32) % 32] - si[(j + 2 + k) % 32]);
      *out++ = acc - *iq++;
    }
  }
  for (int i = 0; i < 32; i++) state[i] = si[i];
  for (int i = 0; i < 14; i++) state[i + 32] = *iq++;
#elif OPTIMIZE_LEVEL == 2
  demodulator_hilbert2(coeff, state, size, inp, out);
#endif
}

/*
  Adjust volume for DAC.
  <inp>: float[size], <out>: int16_t[size]
*/
static inline void amplifier(float &ofst, float vol, float max, int size, const float *inp, int16_t *out) {
#if OPTIMIZE_LEVEL == 0
  for (int i = 0; i < size; i++) {
    float acc = inp[i] - ofst;
    ofst += 0.05f * acc;
    acc *= vol;
    if (acc < -max) acc = -max; else if (acc > max) acc = max;
    out[i] = (int16_t)(2048.0f + acc);
  }
#elif OPTIMIZE_LEVEL == 1
  for (int i = size / 8; i != 0; i--) {
    for (int j = 0; j < 8; j++) {
      float acc = *inp++ - ofst;
      ofst += 0.05f * acc;
      acc *= vol;
      acc = (acc < -max) ? -max : (acc > max) ? max : acc;
      *out++ = (int16_t)(2048.0f + acc);
    }
  }
#elif OPTIMIZE_LEVEL == 2
  ofst = amplifier2(ofst, vol, max, size, inp, out);
#endif
}

/*
  Copy I/Q signals for FFT.
  inp: float[2 * size], out: int16_t[size * 2]
*/
static inline void snapshot(float gain, int size, const float *inp, int16_t *out) {
#if OPTIMIZE_LEVEL == 0
  for (int i = 0; i < size; i++) {
    out[2 * i    ] = __SSAT((int32_t)(inp[i       ] * gain * 32767.0f), 16);
    out[2 * i + 1] = __SSAT((int32_t)(inp[i + size] * gain * 32767.0f), 16);
  }
#elif OPTIMIZE_LEVEL == 1
  const float *ii = inp;
  const float *iq = inp + size;
  uint32_t *o = (uint32_t *)out;
  for (int i = size / 16; i != 0; i--) {
    for (int j = 0; j < 16; j++) {
      const int16_t tmpl = __SSAT((int32_t)(*ii++ * gain * 32767.0f), 16);
      const int16_t tmph = __SSAT((int32_t)(*iq++ * gain * 32767.0f), 16);
      *o++ = (uint32_t)tmph << 16 | tmpl;
    }
  }
#elif OPTIMIZE_LEVEL == 2
  snapshot2(gain, size, inp, out);
#endif
}

#endif
