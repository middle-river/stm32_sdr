/*
  Signal processing library.
  2020-09-31  T. Nakagawa
*/

#ifndef DSP_H_
#define DSP_H_

#pragma GCC optimize ("Ofast")
#define OPTIMIZE_LEVEL 1	// 0=Unoptimized, 1=Improved algorithm, 2=Assembly language.

#include <cmath>
#include "Filter.h"

#ifdef ARDUINO_ARCH_STM32
static inline int32_t __SMULBB(uint32_t x, uint32_t y) { int32_t r; asm volatile("smulbb %[r], %[x], %[y]" : [r] "=r" (r) : [x] "r" (x), [y] "r" (y) : ); return r; }
static inline int32_t __SMULTT(uint32_t x, uint32_t y) { int32_t r; asm volatile("smultt %[r], %[x], %[y]" : [r] "=r" (r) : [x] "r" (x), [y] "r" (y) : ); return r; }
#else
static inline uint32_t __SMLAD(uint32_t x, uint32_t y, uint32_t z) { int16_t *xx = (int16_t *)&x, *yy = (int16_t *)&y; return (xx[0] * yy[0] + xx[1] * yy[1] + z); }
static inline uint32_t __SSUB16(uint32_t x, uint32_t y) { int16_t *xx = (int16_t *)&x, *yy = (int16_t *)&y; uint32_t z = 0; int16_t *zz = (int16_t *)&z; zz[0] = xx[0] - yy[0]; zz[1] = xx[1] - yy[1]; return z; }
static inline int32_t __SMULBB(uint32_t x, uint32_t y) { int16_t *xx = (int16_t *)&x, *yy = (int16_t *)&y; return (xx[0] * yy[0]); }
static inline int32_t __SMULTT(uint32_t x, uint32_t y) { int16_t *xx = (int16_t *)&x, *yy = (int16_t *)&y; return (xx[1] * yy[1]); }
static inline int32_t __SSAT(int32_t val, uint32_t sat) { const int32_t lim = 1 << (sat - 1); return (val < -lim) ? -lim : (val >= lim) ? (lim - 1) : val;  }
#endif
static inline int ilog2(int x) { return sizeof(int) * 8 - __builtin_clz(x) - 1; }

extern "C" {
int32_t offset2(int32_t ofst, int size, const int16_t *inp, int16_t *out);
void decimator_cic2(const int16_t *tbl, int32_t *state, int size, const int16_t *inp, float *out);
void decimator_fir2(const float *coeff, float *state, int size, const float *inp, float *out);
void bpf_iir2(const float *coeff, float *state, int size, const float *inp, float *out);
float demodulator_amplitude2(float alpha, float state, int size, const float *inp0, const float *inp1, float *out);
void demodulator_mixer2(const float *tbl0, const float *tbl1, int size, const float *inp0, const float *inp1, float *out);
void demodulator_hilbert2(const float *coeff, float *state, int size, const float *inp0, const float *inp1, float *out);
}

/*
  Make a cos/sin table for Numerically Controlled Oscillator.
  <tbl0/tbl1>: int16_t[len].
*/
static inline float nco(int samp, float freq, int size, int16_t *tbl0, int16_t *tbl1) {
  int f = (int)((float)freq * size / samp + 0.5);
  for (int i = 0; i < size; i++) {
    const float rad = i * 2.0f * (float)M_PI * f / size;
    tbl0[i] = __SSAT((int32_t)(sinf(rad) * 32768.0f), 16);
    tbl1[i] = __SSAT((int32_t)(cosf(rad) * 32768.0f), 16);
  }
  return ((float)f * samp / size);
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
    const int32_t sgnl = (tbl[i] * (inp[i] << 4)) >> 15;
    // CIC decimator.
    for (int j = 0; j < 3; j++) intg[j] += (j == 0) ? sgnl : intg[j - 1];
    if (i % 32 == 31) {
      int32_t accm[3];
      for (int j = 0; j < 3; j++) {
        const int32_t y = (j == 0) ? intg[2] : accm[j - 1];
        accm[j] = y - comb[j];
        comb[j] = y;
      }
      out[i / 32] = (float)accm[2] / (1 << (15 + 15));
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
      const int32_t sgnll = __SMULBB(tblv, inpv) >> 11;
      const int32_t sgnlh = __SMULTT(tblv, inpv) >> 11;
      intg0 += sgnll; intg1 += intg0; intg2 += intg1;
      intg0 += sgnlh; intg1 += intg0; intg2 += intg1;
    }
    int32_t accm0, accm1, accm2;
    accm0 = intg2 - comb0; comb0 = intg2;
    accm1 = accm0 - comb1; comb1 = accm0;
    accm2 = accm1 - comb2; comb2 = accm1;
    *out++ = accm2 / 1073741824.0f;
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
      out[i / 4] = acc;
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
      *out++ = acc;
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
  <state>: float, <inp0/inp1>: float[size], <out>: float[size]
*/
static inline void demodulator_amplitude(float alpha, float &state, int size, const float *inp0, const float *inp1, float *out) {
#if OPTIMIZE_LEVEL == 0
  for (int i = 0; i < size; i++) {
    const float ii = inp0[i];
    const float iq = inp1[i];
    const float sig = sqrtf(ii * ii + iq * iq);
    // DC removal filter.
    const float tmp = sig + alpha * state;
    out[i] = tmp - state;
    state = tmp;
  }
#elif OPTIMIZE_LEVEL == 1
  for (int i = size / 16; i != 0; i--) {
    for (int j = 0; j < 16; j++) {
      const float i0 = *inp0++;
      const float i1 = *inp1++;
      const float sig = sqrtf(i0 * i0 + i1 * i1);
      // DC removal filter.
      const float tmp = sig + (alpha - 1.0f) * state;
      *out++ = tmp;
      state += tmp;
    }
  }
#elif OPTIMIZE_LEVEL == 2
  state = demodulator_amplitude2(alpha, state, size, inp0, inp1, out);
#endif
}

/*
  Demodulator for CW with a mixer.
  <tbl0/tbl1>: float[size], <inp0/inp1>: float[size], <out>: float[size]
*/
static inline void demodulator_mixer(const float *tbl0, const float *tbl1, int size, const float *inp0, const float *inp1, float *out) {
#if OPTIMIZE_LEVEL == 0
  for (int i = 0; i < size; i++) {
    out[i] = 2.0f * (tbl0[i] * inp0[i] - tbl1[i] * inp1[i]);
  }
#elif OPTIMIZE_LEVEL == 1
  for (int i = size / 16; i != 0; i--) {
    for (int l = 0; l < 16; l++) {
      *out++ = 2.0f * (*tbl0++ * *inp0++ - *tbl1++ * *inp1++);
    }
  }
#elif OPTIMIZE_LEVEL == 2
  demodulator_mixer2(tbl0, tbl1, size, inp0, inp1, out);
#endif
}

/*
  Demodulator for SSB with the Hilbert transform by 31-tap FIR.
  <coeff>: float[31], <state>: float[47], <inp0/inp1>: float[size], <out>: float[size]
*/
static inline void demodulator_hilbert(const float *coeff, float *state, int size, const float *inp0, const float *inp1, float *out) {
#if OPTIMIZE_LEVEL == 0
  float *s0 = state;	// float[31]
  float *s1 = state + 31;	// float[16]
  for (int i = 0; i < size; i++) {
    for (int j = 30; j >= 1; j--) s0[j] = s0[j - 1];
    for (int j = 15; j >= 1; j--) s1[j] = s1[j - 1];
    s0[0] = inp0[i];
    s1[0] = inp1[i];
    float acc = 0.0f;
    for (int j = 0; j < 31; j++) acc += coeff[j] * s0[j];
    out[i] = acc - s1[31 / 2];
  }
#elif OPTIMIZE_LEVEL == 1
  float s0[32]; for (int i = 0; i < 32; i++) s0[i] = state[i];	// float[32]
  float *s1 = state + 32;	// float[15]
  {
    for (int j = 0; j < 32; j++) {
      s0[j] = *inp0++;
      float acc = 0.0f;
      for (int k = 0; k < 15; k++) acc += coeff[k] * (s0[(j - k + 32) % 32] - s0[(j + 2 + k) % 32]);
      *out++ = acc - ((j < 15) ? s1[j] : *inp1++);
    }
  }
  for (int i = size / 32 - 1; i != 0; i--) {
    for (int j = 0; j < 32; j++) {
      s0[j] = *inp0++;
      float acc = 0.0f;
      for (int k = 0; k < 15; k++) acc += coeff[k] * (s0[(j - k + 32) % 32] - s0[(j + 2 + k) % 32]);
      *out++ = acc - *inp1++;
    }
  }
  for (int i = 0; i < 32; i++) state[i] = s0[i];
  for (int i = 0; i < 15; i++) state[i + 32] = *inp1++;
#elif OPTIMIZE_LEVEL == 2
  demodulator_hilbert2(coeff, state, size, inp0, inp1, out);
#endif
}

/*
  Automatic gain control.
  <inp>: float[size], <out>: int16_t[size]
*/
static inline void agc(const float *config, float &state, int size, const float *inp, float *out) {
  const float &decay = config[0];
  const float &threshold = config[1];
  if (state < threshold) state = threshold;
  for (int i = 0; i < size; i++) {
    const float level = fabs(inp[i]);
    if (level > state) state = level;
    out[i] = inp[i] / state;
    if (state > threshold) state *= decay;
  }
}

/*
  Copy I/Q signals for FFT.
  <inp0/inp1>: float[size], <out>: int16_t[size * 2]
*/
static inline void snapshot(float scale, int size, const float *inp0, const float *inp1, int16_t *out) {
  for (int i = 0; i < size; i++) {
    *out++ = __SSAT((int32_t)(*inp0++ * scale * 32768.0f), 16);
    *out++ = __SSAT((int32_t)(*inp1++ * scale * 32768.0f), 16);
  }
}

#endif
