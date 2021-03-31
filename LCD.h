/*
  Library for ST7789 135x240 LCD.
  2020-12-15  T. Nakagawa
*/

#ifndef LCD_H_
#define LCD_H_

#include <SPI.h>

class LCD {
public:
  LCD(uint8_t cs, uint8_t dc, uint8_t res, uint8_t scl, uint8_t sda, uint8_t miso = MISO) : dc_(dc), res_(res), spi_(sda, miso, scl, cs), scroll_(0) {
    pinMode(dc_, OUTPUT);
    pinMode(res_, OUTPUT);
    digitalWrite(dc_, LOW);
    digitalWrite(res_, LOW);
  }

  void begin() {
    digitalWrite(dc_, LOW);
    digitalWrite(res_, HIGH);
    spi_.begin();
    spi_.setBitOrder(MSBFIRST);
    spi_.setDataMode(SPI_MODE2);
    spi_.setClockDivider(SPI_CLOCK_DIV2);
    initialize();
    reset();
  }

  void end() {
    digitalWrite(dc_, LOW);
    digitalWrite(res_, LOW);
    spi_.end();
  }

  void reset() {
    scroll_ = 0;
    scroll(0);
  }

  void draw(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *bmp) {
    const uint16_t xs = OFST_X + (scroll_ + x) % WIDTH;
    const uint16_t xe = xs + w - 1;
    const uint16_t ys = y + OFST_Y;
    const uint16_t ye = ys + h - 1;
    // Column address set.
    send_cmd(0x2a);  // CASET
    send_data(xs >> 8);  // XS high.
    send_data(xs & 0xff);  // XS low.
    send_data(xe >> 8);  // XE high.
    send_data(xe & 0xff);  // XE low.
    // Row address set.
    send_cmd(0x2b);  // RASET
    send_data(ys >> 8);  // YS high.
    send_data(ys & 0xff);  // YS low.
    send_data(ye >> 8);  // YE high.
    send_data(ye & 0xff);  // YE low.
    send_cmd(0x2c);  // RAMWR
    send_data((uint8_t *)bmp, (uint32_t)w * h * 2);
  }

  void scroll(int16_t n) {
    while (n < 0) n += WIDTH - 1;
    scroll_ = (scroll_ + n) % WIDTH;
    // Vertical scroll start address of RAM.
    send_cmd(0x37);  // VSCSAD
    send_data((scroll_ + OFST_X) >> 8);  // VSP high.
    send_data((scroll_ + OFST_X) & 0xff);  // VSP low.
  }

  static constexpr int16_t WIDTH = 240;
  static constexpr int16_t HEIGHT = 135;
  static constexpr uint16_t ROW = 320;
  static constexpr uint16_t OFST_X = 40;
  static constexpr uint16_t OFST_Y = 53;

private:
  void send_cmd(uint8_t cmd) {
    digitalWrite(dc_, LOW);
    spi_.transfer(cmd);
  }

  void send_data(uint8_t data) {
    digitalWrite(dc_, HIGH);
    spi_.transfer(data);
  }

  void send_data(const uint8_t *data, uint32_t size) {
    digitalWrite(dc_, HIGH);
    while (size--) {
      spi_.transfer(*data++);
    }
  }

  void initialize() {
    // Hardware reset.
    digitalWrite(res_, LOW);
    delay(1);
    digitalWrite(res_, HIGH);
    delay(120);
    // Software reset.
    send_cmd(0x01);	// SWRESET
    delay(120);
    // Memory data access control.
    send_cmd(0x36);
    send_data(0x60);
    // Vertical scrolling definition.
    send_cmd(0x33);				// VSCRDEF
    send_data(OFST_X >> 8);			// TFA high.
    send_data(OFST_X & 0xff);			// TFA low.
    send_data(WIDTH >> 8);			// VSA high.
    send_data(WIDTH & 0xff);			// VSA low.
    send_data((ROW - OFST_X - WIDTH) >> 8);	// BFA high.
    send_data((ROW - OFST_X - WIDTH) & 0xff);	// BFA low.
    // Interface pixel format.
    send_cmd(0x3a);	// COLMOD
    send_data(0x55);	// 65K of RGB interface, 16bit/pixel.
    // Display inversion on.
    send_cmd(0x21);	// INVON
    // Sleep out.
    send_cmd(0x11);	// SLPOUT
    delay(5);
    // Display on.
    send_cmd(0x29);	// DISPON
  }

  uint8_t dc_;
  uint8_t res_;
  SPIClass spi_;
  uint16_t scroll_;
};

#endif
