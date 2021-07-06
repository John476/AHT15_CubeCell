#include "Adafruit_BusIO_Register.h"

Adafruit_BusIO_Register::Adafruit_BusIO_Register(Adafruit_I2CDevice *i2cdevice,
                                                 uint16_t reg_addr,
                                                 uint8_t width,
                                                 uint8_t byteorder,
                                                 uint8_t address_width) {
  _i2cdevice = i2cdevice;
  _addrwidth = address_width;
  _address = reg_addr;
  _byteorder = byteorder;
  _width = width;
}

bool Adafruit_BusIO_Register::write(uint8_t *buffer, uint8_t len) {

  uint8_t addrbuffer[2] = {(uint8_t)(_address & 0xFF),
                           (uint8_t)(_address >> 8)};

  if (_i2cdevice){
    return _i2cdevice->write(buffer, len, true, addrbuffer, _addrwidth);}
  return false;
}

/*!
 *    @brief  Write up to 4 bytes of data to the register location
 *    @param  value Data to write
 *    @param  numbytes How many bytes from 'value' to write
 *    @return True on successful write (only really useful for I2C as SPI is
 * uncheckable)
 */
bool Adafruit_BusIO_Register::write(uint32_t value, uint8_t numbytes) {
  if (numbytes == 0) {
    numbytes = _width;
  }
  if (numbytes > 4) {
    return false;
  }

  // store a copy
  _cached = value;

  for (int i = 0; i < numbytes; i++) {
    if (_byteorder == LSBFIRST) {
      _buffer[i] = value & 0xFF;
    } else {
      _buffer[numbytes - i - 1] = value & 0xFF;
    }
    value >>= 8;
  }
  return write(_buffer, numbytes);
}

uint32_t Adafruit_BusIO_Register::read(void) {
  if (!read(_buffer, _width)) {
    return -1;
  }

  uint32_t value = 0;

  for (int i = 0; i < _width; i++) {
    value <<= 8;
    if (_byteorder == LSBFIRST) {
      value |= _buffer[_width - i - 1];
    } else {
      value |= _buffer[i];
    }
  }

  return value;
}

uint32_t Adafruit_BusIO_Register::readCached(void) { return _cached; }

bool Adafruit_BusIO_Register::read(uint8_t *buffer, uint8_t len) {
  uint8_t addrbuffer[2] = {(uint8_t)(_address & 0xFF),
                           (uint8_t)(_address >> 8)};

  if (_i2cdevice) {
    return _i2cdevice->write_then_read(addrbuffer, _addrwidth, buffer, len);
  }
  return false;
}


bool Adafruit_BusIO_Register::read(uint16_t *value) {
  if (!read(_buffer, 2)) {
    return false;
  }

  if (_byteorder == LSBFIRST) {
    *value = _buffer[1];
    *value <<= 8;
    *value |= _buffer[0];
  } else {
    *value = _buffer[0];
    *value <<= 8;
    *value |= _buffer[1];
  }
  return true;
}

bool Adafruit_BusIO_Register::read(uint8_t *value) {
  if (!read(_buffer, 1)) {
    return false;
  }

  *value = _buffer[0];
  return true;
}

/*!
 *    @brief  Pretty printer for this register
 *    @param  s The Stream to print to, defaults to &Serial
 */
void Adafruit_BusIO_Register::print(Stream *s) {
  uint32_t val = read();
  s->print("0x");
  s->print(val, HEX);
}

/*!
 *    @brief  Pretty printer for this register
 *    @param  s The Stream to print to, defaults to &Serial
 */
void Adafruit_BusIO_Register::println(Stream *s) {
  print(s);
  s->println();
}

/*!
 *    @brief  Create a slice of the register that we can address without
 * touching other bits
 *    @param  reg The Adafruit_BusIO_Register which defines the bus/register
 *    @param  bits The number of bits wide we are slicing
 *    @param  shift The number of bits that our bit-slice is shifted from LSB
 */
Adafruit_BusIO_RegisterBits::Adafruit_BusIO_RegisterBits(
    Adafruit_BusIO_Register *reg, uint8_t bits, uint8_t shift) {
  _register = reg;
  _bits = bits;
  _shift = shift;
}

/*!
 *    @brief  Read 4 bytes of data from the register
 *    @return  data The 4 bytes to read
 */
uint32_t Adafruit_BusIO_RegisterBits::read(void) {
  uint32_t val = _register->read();
  val >>= _shift;
  return val & ((1 << (_bits)) - 1);
}

bool Adafruit_BusIO_RegisterBits::write(uint32_t data) {
  uint32_t val = _register->read();

  // mask off the data before writing
  uint32_t mask = (1 << (_bits)) - 1;
  data &= mask;

  mask <<= _shift;
  val &= ~mask;          // remove the current data at that spot
  val |= data << _shift; // and add in the new data

  return _register->write(val, _register->width());
}

/*!
 *    @brief  The width of the register data, helpful for doing calculations
 *    @returns The data width used when initializing the register
 */
uint8_t Adafruit_BusIO_Register::width(void) { return _width; }

/*!
 *    @brief  Set the default width of data
 *    @param width the default width of data read from register
 */
void Adafruit_BusIO_Register::setWidth(uint8_t width) { _width = width; }

/*!
 *    @brief  Set register address
 *    @param address the address from register
 */
void Adafruit_BusIO_Register::setAddress(uint16_t address) {
  _address = address;
}

/*!
 *    @brief  Set the width of register address
 *    @param address_width the width for register address
 */
void Adafruit_BusIO_Register::setAddressWidth(uint16_t address_width) {
  _addrwidth = address_width;
}
