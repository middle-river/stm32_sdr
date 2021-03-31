/*
  Peripheral control library.
  2020-10-04  T. Nakagawa
*/

#ifndef PERIPHERAL_H_
#define PERIPHERAL_H_

#include <cstdint>

void initPeripherals();
void startPeripherals(int adc_buf_size, int16_t *adc_buf, void (*adc_handler)(), int dac_buf_size, int16_t *dac_buf);
void initTimer();
void startTimer(void (*tim_handler)());

#endif
