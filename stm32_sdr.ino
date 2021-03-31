/*
  Direct-Sampling Software Defined Radio using STM32.
  2020-10-01  T. Nakagawa
*/

#include <CMSIS_DSP.h>
#include <arm_const_structs.h>
#include <math.h>
#include "DSP.h"
#include "Font.h"
#include "LCD.h"
#include "Peripheral.h"

#define CCMRAM __attribute__((section(".ccmram")))

constexpr int PIN_KEY_A = PD12;
constexpr int PIN_KEY_B = PD11;
constexpr int PIN_KEY_SW = PD14;
constexpr int PIN_KEY_GND = PD13;
constexpr int PIN_LCD_CS = PB12;
constexpr int PIN_LCD_DC = PB11;
constexpr int PIN_LCD_RES = PD8;
constexpr int PIN_LCD_SCL = PB13;
constexpr int PIN_LCD_SDA = PB15;
constexpr int PIN_LCD_MISO = PB14;				// NC.
constexpr int CIC_DCM_FACTOR = 32;				// CIC decimation factor.
constexpr int FIR_DCM_FACTOR = 4;				// FIR decimation factor.
constexpr int DAC_BUF_SIZE = 128;				// DAC buffer size.
constexpr int WRK_BUF_SIZE = DAC_BUF_SIZE * FIR_DCM_FACTOR;	// Work buffer size.
constexpr int ADC_BUF_SIZE = WRK_BUF_SIZE * CIC_DCM_FACTOR;	// ADC buffer size.
constexpr int ADC_FREQ = 6000000;				// ADC sampling frequency.
constexpr int CWT_FREQ = 800;					// CW tone frequency.
constexpr uint16_t colors[8] = {0x0000, 0x00f8, 0xe007, 0xe0ff, 0x1f00, 0x1ff8, 0xff07, 0xffff};  // Black, red, green, yellow, blue, magenta, cyan, white in little endian.
constexpr float NCO_CALIB = 4000000.0f / 3970936.0f;

static float hlb_coeff_inv[31];
static int16_t adc_buf[2 * ADC_BUF_SIZE];		// ADC buffer (double buffering).
static int16_t nco_tbl[2 * ADC_BUF_SIZE] CCMRAM;	// NCO mixer table.
static float cwt_tbl[2 * DAC_BUF_SIZE];			// CW tone mixer table.
static float wrk_buf[2 * WRK_BUF_SIZE];			// Work buffer.
static int16_t dac_buf[2 * DAC_BUF_SIZE];		// DAC buffer (double buffering).
static int16_t fft_buf[2 * DAC_BUF_SIZE];		// FFT buffer.
static int16_t fft_mag[DAC_BUF_SIZE];
static int nco_frequency = -1;
static uint32_t nco_timer = 0;
static bool fft_empty = true;
static LCD lcd(PIN_LCD_CS, PIN_LCD_DC, PIN_LCD_RES, PIN_LCD_SCL, PIN_LCD_SDA, PIN_LCD_MISO);
static uint16_t heatmap_colors[64];
static int16_t heatmap_values[64];

static struct {
  uint32_t time_dcr;
  uint32_t time_cic;
  uint32_t time_fir;
  uint32_t time_bpf;
  uint32_t time_dem;
  uint32_t time_amp;
  uint32_t freq_fft;
} stat;

static struct {
  int ui;
  int cursor;
  int frequency;
  int demodulation;
  int bpf;
  int volume;
  int fft_gain;
  int fft_delay;
} config;

// Get key events (event: 1=decrement, 2=increment, 3=short push, 4=long push).
int key_event() {
  int event = 0;
  static int key_a_old = 1;
  static int key_b_old = 1;
  static int key_timer = 0;
  const int key_a = digitalRead(PIN_KEY_A);
  const int key_b = digitalRead(PIN_KEY_B);
  const int key_sw = digitalRead(PIN_KEY_SW);
  if (key_a == 0 && key_b == 0 && !(key_a_old == 0 && key_b_old == 0)) event = (key_b_old == 1) ? 1 : 2;
  key_a_old = key_a;
  key_b_old = key_b;
  if (key_sw == 0) {
    if (key_timer <= 1000) key_timer++;
    if (key_timer == 1000) event = 4;	// Long push=1ms*1000.
  } else {
    if (key_timer > 0 && key_timer < 1000) event = 3;
    key_timer = 0;
  }
  return event;
}

// Signal processing handler which is called by ADC DMA interuption.
void dsp_handler() {
  static int16_t *adc_buf_wr = adc_buf, *adc_buf_rd = adc_buf + ADC_BUF_SIZE;
  static int16_t *dac_buf_rd = dac_buf, *dac_buf_wr = dac_buf + DAC_BUF_SIZE;
  std::swap(adc_buf_rd, adc_buf_wr);
  std::swap(dac_buf_rd, dac_buf_wr);
  uint32_t time_end = micros(), time_bgn = time_end;

  // DC offset removal.
  static int32_t adc_ofst = (2048 << 16);
  offset(adc_ofst, ADC_BUF_SIZE, adc_buf_rd, adc_buf_rd);
  time_end = micros(); stat.time_dcr = time_end - time_bgn; time_bgn = time_end;

  // I/Q mixer and CIC decimator (1/32).
  static int32_t cic_state[2 * 6];
  decimator_cic(nco_tbl               , cic_state    , ADC_BUF_SIZE, adc_buf_rd, wrk_buf               );
  decimator_cic(nco_tbl + ADC_BUF_SIZE, cic_state + 6, ADC_BUF_SIZE, adc_buf_rd, wrk_buf + WRK_BUF_SIZE);
  time_end = micros(); stat.time_cic = time_end - time_bgn; time_bgn = time_end;

  // FIR decimator (1/4).
  static float fir_state[2 * 32];
  decimator_fir(fir_coeff, fir_state     , WRK_BUF_SIZE, wrk_buf               , wrk_buf               );
  decimator_fir(fir_coeff, fir_state + 32, WRK_BUF_SIZE, wrk_buf + WRK_BUF_SIZE, wrk_buf + DAC_BUF_SIZE);
  if (fft_empty) {
    snapshot((1 << config.fft_gain) - 1, DAC_BUF_SIZE, wrk_buf, fft_buf);
    fft_empty = false;
  }
  time_end = micros(); stat.time_fir = time_end - time_bgn; time_bgn = time_end;

  // Band pass filter.
  static float bpf_state[2 * 6];
  bpf_iir(iir_coeff[config.bpf], bpf_state    , DAC_BUF_SIZE, wrk_buf               , wrk_buf               );
  bpf_iir(iir_coeff[config.bpf], bpf_state + 6, DAC_BUF_SIZE, wrk_buf + DAC_BUF_SIZE, wrk_buf + DAC_BUF_SIZE);
  time_end = micros(); stat.time_bpf = time_end - time_bgn; time_bgn = time_end;

  // Demodulation.
  if (config.demodulation == 0) {		// AM.
    demodulator_amplitude(DAC_BUF_SIZE, wrk_buf, wrk_buf);
  } else if (config.demodulation == 1) {	// CW.
    demodulator_mixer(cwt_tbl, DAC_BUF_SIZE, wrk_buf, wrk_buf);
  } else {
    static float hlb_state[46];
    if (config.demodulation == 2) {		// USB.
      demodulator_hilbert(hlb_coeff    , hlb_state, DAC_BUF_SIZE, wrk_buf, wrk_buf);
    } else {					// LSB.
      demodulator_hilbert(hlb_coeff_inv, hlb_state, DAC_BUF_SIZE, wrk_buf, wrk_buf);
    }
  }
  time_end = micros(); stat.time_dem = time_end - time_bgn; time_bgn = time_end;

  // Volume adjustment.
  static float vol_ofst = 0.0;
  amplifier(vol_ofst, ((1 << config.volume) - 1) * 1024.0f, 1024.0f, DAC_BUF_SIZE, wrk_buf, dac_buf_wr);
  time_end = micros(); stat.time_amp = time_end - time_bgn; time_bgn = time_end;
}

// Process key inputs. Called by 1000Hz timer interruption.
void key_handler() {
  const int event = key_event();

  int delta = (event == 1) ? -1 : (event == 2) ? +1 : 0;
  if (event == 4) {
    config.ui = (config.ui + 1) % 2;
    config.cursor = 0;
  }
  if (config.ui == 0) {	// Information mode.
    if (event == 3) config.cursor = (config.cursor + 1) % 5;
    if (delta != 0) {
      if (config.cursor <= 1) {
        nco_timer = millis() + 500;
        if (config.cursor == 1) delta *= 1000;
        static uint32_t delta_timer = 0;
        if (millis() - delta_timer < 200) delta *= 10;
        delta_timer = millis();
        config.frequency = constrain(config.frequency + delta, 1, 3000000);
      } else if (config.cursor == 2) {
        config.demodulation = constrain(config.demodulation + delta, 0, 3);
      } else if (config.cursor == 3) {
        config.bpf = constrain(config.bpf + delta, 0, 8);
      } else if (config.cursor == 4) {
        config.volume = constrain(config.volume + delta, 0, 15);
      }
    }
  } else {	// Waterfall mode.
    if (event == 3) config.cursor = (config.cursor + 1) % 3;
    if (delta != 0) {
      if (config.cursor == 0) {
        nco_timer = millis() + 500;
        delta *= 10;
        static uint32_t delta_timer = 0;
        if (millis() - delta_timer < 200) delta *= 10;
        delta_timer = millis();
        config.frequency = constrain(config.frequency + delta, 1, 3000000);
      } else if (config.cursor == 1) {
        config.fft_gain = constrain(config.fft_gain + delta, 0, 15);
      } else if (config.cursor == 2) {
        config.fft_delay = constrain(config.fft_delay - delta, 0, 15);
      }
    }
  }
}

// Serial communicatin.
void communicate() {
  const char cmd = Serial.read();
  const int val = Serial.readStringUntil('\n').toInt();
  if (cmd == 'F') {
    if (1 <= val && val <= 3000000) config.frequency = val;
  } else if (cmd == 'D') {
    if (0 <= val && val <= 3) config.demodulation = val;
  } else if (cmd == 'B') {
    if (0 <= val && val <= 8) config.bpf = val;
  } else if (cmd == 'V') {
    if (0 <= val && val <= 15) config.volume = val;
  } else if (cmd == 'G') {
    if (0 <= val && val <= 15) config.fft_gain = val;
  } else if (cmd == 'L') {
    if (0 <= val && val <= 15) config.fft_delay = val;
  } else if (cmd == 'a') {
    for (int i = 0; i < 32; i++) {Serial.print(fft_buf[i]); Serial.print(" ");} Serial.println("");
  } else if (cmd == 'b') {
    for (int i = 0; i < 32; i++) {Serial.print(fft_mag[i]); Serial.print(" ");} Serial.println("");
  } else if (cmd == 'C') {
    Serial.print("FRQ: "); Serial.println(config.frequency);
    Serial.print("DEM: "); Serial.println(config.demodulation);
    Serial.print("BPF: "); Serial.println(config.bpf);
    Serial.print("VOL: "); Serial.println(config.volume);
    Serial.print("FFG: "); Serial.println(config.fft_gain);
    Serial.print("FFD: "); Serial.println(config.fft_delay);
  } else if (cmd == 'S') {
    Serial.print("DCR: "); Serial.println(stat.time_dcr);
    Serial.print("CIC: "); Serial.println(stat.time_cic);
    Serial.print("FIR: "); Serial.println(stat.time_fir);
    Serial.print("BPF: "); Serial.println(stat.time_bpf);
    Serial.print("DEM: "); Serial.println(stat.time_dem);
    Serial.print("AMP: "); Serial.println(stat.time_amp);
    Serial.print("FFT: "); Serial.println(stat.freq_fft);
  }
}

// Draw a letter to LCD.
void draw_font(uint16_t x, uint16_t y, int idx, uint16_t fgc, uint16_t bgc) {
  uint16_t bmp[16 * 32];
  for (int i = 0; i < 32; i++) {
    uint16_t pat = pgm_read_word_near(font_data + 32 * idx + i);
    for (int j = 0; j < 16; j++, pat <<= 1) {
      bmp[16 * i + j] = ((pat & 0x8000) != 0) ? fgc : bgc;
    }
  }
  lcd.draw(x, y, 16, 32, bmp);
}

// Update the LCD.
void display() {
  static int ui = -1;
  if (config.ui == 0) {	// Information mode.
    if (ui != config.ui) {
      ui = config.ui;
      // Clear screen.
      lcd.reset();
      const uint16_t blank[240] = {};
      for (int i = 0; i < 135; i++) lcd.draw(0, i, 240, 1, blank);
      // Draw labels.
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) draw_font(16 * j, 34 * i, 15 + 4 * i + j, colors[3], colors[0]);	// Label.
        draw_font(16 * 4, 34 * i, 12, colors[2], colors[0]);	// Colon.
      }
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
          draw_font(16 * (13 + j), 34 * 2 * i, 13 + j, colors[1], colors[0]);	// Hz.
        }
      }
    }
    // Draw numbers.
    for (int i = 0, j = 1; i < 7; i++, j *= 10) {
      const int idx = (config.frequency >= j || j == 1) ? (config.frequency / j) % 10 : 10;	// Zero suppress.
      const int bgc = ((config.cursor == 0 && i < 3) || (config.cursor == 1 && i >= 3)) ? colors[4] : colors[0];
      draw_font(16 * (12 - i - ((i < 3) ? 0 : 1)), 0, idx, colors[7], bgc);	// Frequency.
    }
    draw_font(16 * (12 - 3), 0, (config.frequency >= 1000) ? 11 : 10, colors[7], colors[0]);	// Comma.
    for (int i = 0; i < 3; i++) draw_font(16 * (12 + i), 32, 31 + 3 * config.demodulation + i, colors[7], (config.cursor == 2) ? colors[4] : colors[0]);	// Mode.
    for (int i = 0, j = 1; i < 5; i++, j *= 10) {
      const int idx = (lpf[config.bpf] >= j) ? (lpf[config.bpf] / j) % 10 : 10;
      const int bgc = (config.cursor == 3) ? colors[4] : colors[0];
      draw_font(16 * (12 - i - ((i < 3) ? 0 : 1)), 34 * 2, idx, colors[7], bgc);	// BPF.
    }
    draw_font(16 * (12 - 3), 34 * 2, (lpf[config.bpf] >= 1000) ? 11 : 10, colors[7], (config.cursor == 3) ? colors[4] : colors[0]);	// Comma
    for (int i = 0, j = 1; i < 3; i++, j *= 10) {
      const int idx = (config.volume >= j || j == 1) ? (config.volume / j) % 10 : 10;
      const int bgc = (config.cursor == 4) ? colors[4] : colors[0];
      draw_font(16 * (14 - i), 34 * 3, idx, colors[7], bgc);	// Volume.
    }
  } else {	// Waterfall mode.
    if (ui != config.ui) {
      ui = config.ui;
      // Clear screen.
      lcd.reset();
      const uint16_t blank[240] = {};
      for (int i = 0; i < 135; i++) lcd.draw(0, i, 240, 1, blank);
    }
    static uint32_t fft_timer = 0;
    if (millis() >= fft_timer && !fft_empty) {
      fft_timer = millis() + ((1 << config.fft_delay) - 1) / 32;
      // Compute FFT.
      arm_cfft_q15(&arm_cfft_sR_q15_len128, fft_buf, 0, 1);
      arm_cmplx_mag_q15(fft_buf, fft_mag, DAC_BUF_SIZE);
      fft_empty = true;
      // Draw a line.
      uint16_t buf[128];
      for (int i = 0; i < 128; i++) {
        const int16_t v = fft_mag[(i + 64) & 0x7f];
        int n = 0;	// Calculate logarithm by binary search.
        if (v >= heatmap_values[n + 32]) n += 32;
        if (v >= heatmap_values[n + 16]) n += 16;
        if (v >= heatmap_values[n + 8]) n += 8;
        if (v >= heatmap_values[n + 4]) n += 4;
        if (v >= heatmap_values[n + 2]) n += 2;
        if (v >= heatmap_values[n + 1]) n += 1;
        buf[i] = heatmap_colors[n];
      }
      buf[63] = colors[7];	// Center frequency.
      lcd.scroll(1);
      lcd.draw(239, 3, 1, 128, buf);

      static uint32_t freq_timer = 0;
      static int freq_counter = 0;
      if (millis() < freq_timer) {
        freq_counter++;
      } else {
        stat.freq_fft = freq_counter;
        freq_counter = 0;
        freq_timer = millis() + 1000;
      }
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_KEY_A, INPUT_PULLUP);
  pinMode(PIN_KEY_B, INPUT_PULLUP);
  pinMode(PIN_KEY_SW, INPUT_PULLUP);
  pinMode(PIN_KEY_GND, OUTPUT);
  digitalWrite(PIN_KEY_GND, LOW);

  Serial.begin(115200);
  Serial.println("STM32-SDR");
  Serial.println("Clock=" + String(F_CPU) + "MHz.");

  config.ui = 0;
  config.cursor = 0;
  config.frequency = 594000;
  config.demodulation = 0;
  config.bpf = 8;
  config.volume = 12;
  config.fft_gain = 10;
  config.fft_delay = 12;

  // Initialize the inversed hilbert filter coefficients.
  for (int i = 0; i < 31; i++) hlb_coeff_inv[i] = -hlb_coeff[i];

  // Make the CW tone table.
  nco(ADC_FREQ / CIC_DCM_FACTOR / FIR_DCM_FACTOR, NCO_CALIB * CWT_FREQ, DAC_BUF_SIZE, nco_tbl);
  for (int i = 0; i < 2 * DAC_BUF_SIZE; i++) cwt_tbl[i] = (float)nco_tbl[i] / 32767.0f;

  // Make 5-color heatmap palette and logarithm table..
  for (int i = 0; i < 64; i++) {
    uint16_t r, g, b;
    const int n = map(i % 16, 0, 15, 0, 255);
    if (i < 16) {
      r = 0; g = n; b = 255;
    } else if (i < 32) {
      r = 0; g = 255; b = 255 - n;
    } else if (i < 48) {
      r = n; g = 255; b = 0;
    } else {
      r = 255; g = 255 - n; b = 0;
    }
    r >>= 3; g >>= 2; b >>= 3;
    const uint16_t rgb = (r << 11) | (g << 5) | b;
    heatmap_colors[i] = ((rgb & 0xffU) << 8) | (rgb >> 8);
    heatmap_values[i] = (int16_t)expf(0.16503456f * i);	// log_?(32767)=63
  }

  lcd.begin();
  initPeripherals();
  startPeripherals(2 * ADC_BUF_SIZE, adc_buf, dsp_handler, 2 * DAC_BUF_SIZE, dac_buf);
  initTimer();
  startTimer(key_handler);
}

void loop() {
  // LED heartbeat.
  digitalWrite(LED_BUILTIN, millis() & (1 << 9));	// Blink at 1000/512Hz.

  // NCO table update.
  if (nco_frequency != config.frequency && millis() >= nco_timer) {
    nco_frequency = config.frequency;
    nco(ADC_FREQ, NCO_CALIB * config.frequency, ADC_BUF_SIZE, nco_tbl);
  }

  // Serial communication.
  if (Serial.available()) communicate();

  // LCD update.
  display();
}
