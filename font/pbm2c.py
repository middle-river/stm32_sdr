#!/usr/bin/python3

import sys

bmp = []
buf = ''
n = 0;
for line in sys.stdin:
  line = line.strip();
  if line.startswith('#'):
    continue
  n += 1
  if n == 1:
    assert line == 'P1'
  elif n == 2:
    w, h = map(int, line.split())
    assert w % 16 == 0
    assert h == 32
  else:
    buf = buf + line
    while len(buf) >= w:
      bmp.append(buf[:w])
      buf = buf[w:]
assert not buf

data = []
for i in range(w // 16):
  for j in range(32):
    d = 0
    for k in range(16):
      d <<= 1
      if bmp[j][16 * i + k] != '0':
        d |= 1
    data.append(d)

for i, d in enumerate(data):
  out = ''
  if i % 32 == 0:
    print('\n  ', end='')
  print('0x%04x, ' % d, end='')
print('')
