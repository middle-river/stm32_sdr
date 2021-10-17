#!/usr/bin/python3
# Calculate coefficients of filters for STM32-SDR.
# 2020-10-03  T. Nakagawa

import numpy as np
import scipy.signal


# Decimation FIR filter with CIC compensation.

T = 32       # Number of taps.
L = 4        # Decimation rate.
P = 1024     # Sampling points for CIC compensation.
D = 1        # CIC differential delay.
M = 32       # CIC decimation rate.
O = 3        # CIC number of stages.

def cic(w):
  return (np.abs(np.sin(w * M / 2.) / np.sin(w * D / 2.)) ** O / ((M * D) ** O)
          if w else 1.)

freq = np.arange(P) / (P - 1.)
gain = 1. / np.array(list(map(cic, freq * np.pi / M)))
gain[P // L:] = 0.
fir = scipy.signal.firwin2(T, freq, gain)

print('// Coefficients for decimation FIR filter with CIC compensation.')
print('const float fir_coeff[%d] = {' % T)
for i in range(0, T, 8):
  print('  %s,' % ', '.join(['%+ef' % v for v in fir[i:i + 8]]))
print('};')
print('')


# Low-pass IIR filter.

def biquad(c, s):
  import math
  q = 0.707106781187
  w0 = 2.0 * math.pi * c / s
  alpha = math.sin(w0) / (2.0 * q)
  bb = [(1.0 - math.cos(w0)) / 2.0,
        1.0 - math.cos(w0),
        (1.0 - math.cos(w0)) / 2.0]
  aa = [1.0 + alpha,
        -2.0 * math.cos(w0),
        1.0 - alpha]
  z = aa[0]
  for i in range(3):
    bb[i] /= z
    aa[i] /= z
  return bb, aa

O = 2                                 # Order of the filter.
C = [200., 500.,
     1.e3, 2.e3, 5.e3, 10.e3, 20.e3]  # Cutoff frequencies.
S = 46875.0                           # Sampling rate (Hz).

iir = []
for c in C:
  iir.append(scipy.signal.iirfilter(O, c / (S / 2.0), btype='lowpass'))

print('// Coefficients for low-pass IIR filters.')
print('const float iir_coeff[%d][%d] = {' % (len(C), 2 * (O + 1)))
for i in range(len(C)):
  print('  {%s,' % ', '.join(['%+ef' % v for v in iir[i][0]]) +
        '   %s},' % ', '.join(['%+ef' % v for v in iir[i][1]]))
print('};')
print('const int lpf[%d] = {%s};' %
      (len(C), ', '.join([str(int(c)) for c in C])))
print('')


# Hilbert FIR filter.

T = 31  # Number of taps.

hlb = scipy.signal.remez(T, [0.03, 0.47], [1.0], type='hilbert')

print('// Coefficients for Hilbert FIR filter.')
print('const float hlb_coeff[%d] = {' % T)
for i in range(0, T, 8):
  print('  %s,' % ', '.join(['%+ef' % v for v in hlb[i:i + 8]]))
print('};')
print('')
