/*!
 * @file Adafruit_MCP23008.cpp
 *
 * @mainpage Adafruit MCP23008 Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the MCP23008 i2c port expander
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 * 
 * Originally from https://github.com/adafruit/Adafruit-MCP23008-library
 */

#include "Arduino.h"
#include <i2c_driver.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
#include "Adafruit_MCP23008.h"


////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 implementation

void Adafruit_MCP23008::begin(uint8_t addr) {
  if (addr > 7) {
    addr = 7;
  }
  uint8_t new_addr = MCP23008_ADDRESS | addr;
  if (is_initialized && new_addr == i2caddr)
  {
    return; // Begin allready called
  }
  i2caddr = new_addr;
  master.begin(400 * 1000U);

  // set defaults!
  const int len_to_write = 11;
  uint8_t to_write[len_to_write] = {
    MCP23008_IODIR,
    0xFF,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
  };
  master.write_async(i2caddr, to_write, len_to_write, true);
  while (!master.finished()){
    delayMicroseconds(20);
  }
  is_initialized = true;
}

void Adafruit_MCP23008::begin(void) { begin(0); }

void Adafruit_MCP23008::pinMode(uint8_t p, uint8_t d) {
  // only 8 bits!
  if (p > 7)
    return;

  io_flags = read8(MCP23008_IODIR);

  // set the pin and direction
  if (d == INPUT) {
    io_flags |= 1 << p;
  } else {
    io_flags &= ~(1 << p);
  }

  // write the new IODIR
  write8(MCP23008_IODIR, io_flags);
}

uint8_t Adafruit_MCP23008::readGPIO(void) {
  return gpio_state;
}

MCPState Adafruit_MCP23008::update(bool force){
  static MCPState state = WAITING;
  unsigned long duration = millis() - gpio_stamp;
  switch (state)
  {
  case DONE:
    state = WAITING;
  case WAITING:
    // Immediately start write-read if state has been updated
    if (next_gpio_state != gpio_state) {
      state = SENDING_STATE;
    // just read the state if it's time or force flag is enabled
    } else if (duration > update_interval || force) {
      state = READ_STATE_SENDING_ADDR;
    } else {
      break;
    }
  case SENDING_STATE:
    if (master.finished()) {
      write_buffer[0] = MCP23008_GPIO;
      write_buffer[1] = next_gpio_state;
      master.write_async(i2caddr, write_buffer, 2, true);
      state = READ_STATE_SENDING_ADDR;
    }
    break;
  case READ_STATE_SENDING_ADDR:
    if (master.finished()) {
      write_buffer[0] = MCP23008_GPIO;
      master.write_async(i2caddr, write_buffer, 1, false);
      state = READ_STATE_REQUESTING_STATE;
    } 
    break;
  case READ_STATE_REQUESTING_STATE:
    if (master.finished()) {
      master.read_async(i2caddr, read_buffer, 1, true);
      state = WAITING_FOR_READ;
    }
    break;
  case WAITING_FOR_READ:
    if (master.finished()) {
      gpio_state = read_buffer[0];
      gpio_stamp = millis();
      state = DONE;
    }
    break;
  }
  return state;
}

void Adafruit_MCP23008::forceUpdate() {
  while (update(true) != DONE){
    ;
  }
}

void Adafruit_MCP23008::writeGPIO(uint8_t gpio) { 
  next_gpio_state = gpio;
}

void Adafruit_MCP23008::digitalWrite(uint8_t p, uint8_t d) {
  // only 8 bits!
  if (p > 7)
    return;

  // set the pin and direction
  if (d == HIGH) {
    next_gpio_state |= 1 << p;
  } else {
    next_gpio_state &= ~(1 << p);
  }
}

void Adafruit_MCP23008::pullUp(uint8_t p, uint8_t d) {
  uint8_t gppu;

  // only 8 bits!
  if (p > 7)
    return;

  gppu = read8(MCP23008_GPPU);
  // set the pin and direction
  if (d == HIGH) {
    gppu |= 1 << p;
  } else {
    gppu &= ~(1 << p);
  }
  // write the new GPIO
  write8(MCP23008_GPPU, gppu);
}

uint8_t Adafruit_MCP23008::digitalRead(uint8_t p) {
  // only 8 bits!
  if (p > 7)
    return 0;
  // read the current GPIO
  return (gpio_state >> p) & 0x1;
}

uint8_t Adafruit_MCP23008::read8(uint8_t addr) {
  // Wire1.beginTransmission(MCP23008_ADDRESS | i2caddr);
  // Wire1.write((byte)addr);
  // Wire1.endTransmission();
  // Wire1.requestFrom(MCP23008_ADDRESS | i2caddr, 1);
  // return Wire1.read();

  uint8_t buffer[] = {addr};
  while(!master.finished()) {
    delayMicroseconds(10);
  }
  master.write_async(i2caddr, buffer, 1, false);
  while(!master.finished()) {
    delayMicroseconds(10);
  }
  uint8_t reading;
  master.read_async(i2caddr, &reading, 1, true);
  while(!master.finished()) {
    delayMicroseconds(10);
  }
  return reading;
}

void Adafruit_MCP23008::write8(uint8_t addr, uint8_t data) {
  uint8_t buffer[] = {addr, data};
  while(!master.finished()) {
    delayMicroseconds(10);
  }
  master.write_async(i2caddr, buffer, 2, true);
  while(!master.finished()) {
    delayMicroseconds(10);
  }
  // Wire1.beginTransmission(MCP23008_ADDRESS | i2caddr);
  // Wire1.write((byte)addr);
  // Wire1.write((byte)data);
  // Wire1.endTransmission();
}
