/*
  DevLab_BMI323.cpp

  Implementation file for the DevLab_BMI323 Arduino library.

  This library communicates with the Bosch BMI323 IMU using direct
  16-bit I2C register access.

  Author:
  Adrian Rabadan Ortiz

  Organization:
  UNIT Electronics - DevLab Ecosystem

  License:
  MIT License
*/

#include "DevLab_BMI323.h"

// --------------------------------------------------
// BMI323 register definitions
// --------------------------------------------------
#define REG_ACC_X     0x03
#define REG_ACC_CONF  0x20
#define REG_GYR_CONF  0x21
#define REG_CMD       0x7E

#define SOFT_RESET_CMD 0xDEAF

// --------------------------------------------------
// Constructor
// --------------------------------------------------
DevLab_BMI323::DevLab_BMI323(TwoWire &wire, uint8_t address) {
  _wire = &wire;
  _address = address;
}

// --------------------------------------------------
// Initialize I2C interface and configure BMI323
// --------------------------------------------------
bool DevLab_BMI323::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock) {
  _wire->begin(sdaPin, sclPin);
  _wire->setClock(clock);

  delay(100);

  softReset();
  delay(100);

  return configure();
}

// --------------------------------------------------
// Configure accelerometer and gyroscope
// --------------------------------------------------
bool DevLab_BMI323::configure() {
  /*
    ACC_CONF = 0x708B

    Mode      : High performance
    Average   : No averaging
    Filtering : ODR / 4
    Range     : ±2g
    ODR       : 800 Hz
  */
  writeRegister16(REG_ACC_CONF, 0x708B);
  delay(10);

  /*
    GYR_CONF = 0x708B

    Mode      : High performance
    Average   : No averaging
    Filtering : ODR / 4
    Range     : ±125 dps
    ODR       : 800 Hz
  */
  writeRegister16(REG_GYR_CONF, 0x708B);
  delay(10);

  return true;
}

// --------------------------------------------------
// Perform BMI323 soft reset
// --------------------------------------------------
void DevLab_BMI323::softReset() {
  writeRegister16(REG_CMD, SOFT_RESET_CMD);
  delay(50);
}

// --------------------------------------------------
// Read accelerometer, gyroscope and temperature data
// --------------------------------------------------
bool DevLab_BMI323::readData(SensorData &dataOut) {
  _wire->beginTransmission(_address);
  _wire->write(REG_ACC_X);
  _wire->endTransmission(false);

  /*
    BMI323 I2C burst reads include 2 dummy bytes at the beginning.
    Requested bytes:
    - 2 dummy bytes
    - 6 bytes accelerometer XYZ
    - 6 bytes gyroscope XYZ
    - 2 bytes temperature
  */
  _wire->requestFrom(_address, (uint8_t)16);

  uint8_t buffer[16];
  uint8_t i = 0;

  while (_wire->available() && i < 16) {
    buffer[i++] = _wire->read();
  }

  if (i < 16) {
    return false;
  }

  const uint8_t offset = 2;  // Skip dummy bytes

  dataOut.accX = (int16_t)(buffer[offset + 0]  | ((uint16_t)buffer[offset + 1]  << 8));
  dataOut.accY = (int16_t)(buffer[offset + 2]  | ((uint16_t)buffer[offset + 3]  << 8));
  dataOut.accZ = (int16_t)(buffer[offset + 4]  | ((uint16_t)buffer[offset + 5]  << 8));

  dataOut.gyrX = (int16_t)(buffer[offset + 6]  | ((uint16_t)buffer[offset + 7]  << 8));
  dataOut.gyrY = (int16_t)(buffer[offset + 8]  | ((uint16_t)buffer[offset + 9]  << 8));
  dataOut.gyrZ = (int16_t)(buffer[offset + 10] | ((uint16_t)buffer[offset + 11] << 8));

  dataOut.tempRaw = (int16_t)(buffer[offset + 12] | ((uint16_t)buffer[offset + 13] << 8));

  // BMI323 temperature conversion formula
  dataOut.temperatureC = (dataOut.tempRaw / 512.0f) + 23.0f;

  return true;
}

// --------------------------------------------------
// Write a 16-bit value to a BMI323 register
// --------------------------------------------------
void DevLab_BMI323::writeRegister16(uint8_t reg, uint16_t value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value & 0xFF);         // LSB
  _wire->write((value >> 8) & 0xFF);  // MSB
  _wire->endTransmission();
}

// --------------------------------------------------
// Read a 16-bit value from a BMI323 register
// --------------------------------------------------
uint16_t DevLab_BMI323::readRegister16(uint8_t reg) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission(false);

  // 2 dummy bytes + 2 data bytes
  _wire->requestFrom(_address, (uint8_t)4);

  if (_wire->available() < 4) {
    return 0xFFFF;
  }

  _wire->read();  // Dummy byte 1
  _wire->read();  // Dummy byte 2

  uint8_t lsb = _wire->read();
  uint8_t msb = _wire->read();

  return ((uint16_t)msb << 8) | lsb;
}