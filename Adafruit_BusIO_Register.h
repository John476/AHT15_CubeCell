#include "D:\Arduino_project\AHT15\aht4\Adafruit_I2CDevice.h"
#include <Arduino.h>

#ifndef Adafruit_BusIO_Register_h
#define Adafruit_BusIO_Register_h

/*!
 * @brief The class which defines a device register (a location to read/write
 * data from)
 */
class Adafruit_BusIO_Register {
public:
  Adafruit_BusIO_Register(Adafruit_I2CDevice *i2cdevice, uint16_t reg_addr,
                          uint8_t width = 1, uint8_t byteorder = LSBFIRST,
                          uint8_t address_width = 1);

  bool read(uint8_t *buffer, uint8_t len);
  bool read(uint8_t *value);
  bool read(uint16_t *value);
  uint32_t read(void);
  uint32_t readCached(void);
  bool write(uint8_t *buffer, uint8_t len);
  bool write(uint32_t value, uint8_t numbytes = 0);

  uint8_t width(void);

  void setWidth(uint8_t width);
  void setAddress(uint16_t address);
  void setAddressWidth(uint16_t address_width);

  void print(Stream *s = &Serial);
  void println(Stream *s = &Serial);

private:
  Adafruit_I2CDevice *_i2cdevice;
  uint16_t _address;
  uint8_t _width, _addrwidth, _byteorder;
  uint8_t _buffer[4]; // we wont support anything larger than uint32 for
                      // non-buffered read
  uint32_t _cached = 0;
};

/*!
 * @brief The class which defines a slice of bits from within a device register
 * (a location to read/write data from)
 */
class Adafruit_BusIO_RegisterBits {
public:
  Adafruit_BusIO_RegisterBits(Adafruit_BusIO_Register *reg, uint8_t bits,
                              uint8_t shift);
  bool write(uint32_t value);
  uint32_t read(void);

private:
  Adafruit_BusIO_Register *_register;
  uint8_t _bits, _shift;
};

#endif // BusIO_Register_h
