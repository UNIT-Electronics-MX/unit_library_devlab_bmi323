/*
  DevLab_BMI323.h

  Arduino library for Bosch BMI323 IMU
  using direct I2C register communication.

  Features:
  - Accelerometer reading
  - Gyroscope reading
  - Temperature reading
  - Direct 16-bit register communication
  - Compatible with ESP32, RP2040, and Arduino platforms

  Author:
  Adrian Rabadan Ortiz

  Organization:
  UNIT Electronics - DevLab Ecosystem

  License:
  MIT License
*/

#ifndef DEVLAB_BMI323_H
#define DEVLAB_BMI323_H

#include <Arduino.h>
#include <Wire.h>

class DevLab_BMI323 {

public:

  /*
    Structure containing all BMI323 sensor data
  */
  struct SensorData {
    int16_t accX;
    int16_t accY;
    int16_t accZ;

    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;

    int16_t tempRaw;
    float temperatureC;
  };

  struct acc_cfg {
    uint8_t acc_odr;
    uint8_t acc_range;
    uint8_t acc_bw;
    uint8_t acc_avgnum;
    uint8_t acc_mode;
  };

  struct gyr_cfg {
    uint8_t gyr_odr;
    uint8_t gyr_range;
    uint8_t gyr_bw;
    uint8_t gyr_avgnum;
    uint8_t gyr_mode;
  };


  /*
    Constructor

    @param wire    I2C interface instance
    @param address I2C device address
  */
  DevLab_BMI323(TwoWire &wire = Wire, uint8_t address = 0x69);

  /*
    Initialize BMI323 sensor

    @param sdaPin  SDA pin
    @param sclPin  SCL pin
    @param clock   I2C clock frequency

    @return true if initialization succeeds
  */
  bool begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock = 400000);

  /*
    Configure accelerometer and gyroscope
  */
  bool configure();

  void configureAccGyr(acc_cfg acc_Cfg, gyr_cfg gyr_Cfg);

  void configAccBMI323(acc_cfg acc_Cfg);

  void configGyrBMI323(gyr_cfg gyr_Cfg);

  /*
    Read accelerometer, gyroscope and temperature data

    @param data Structure containing sensor values

    @return true if read succeeds
  */
  bool readData(SensorData &data);

  /*
    Perform BMI323 soft reset
  */
  void softReset();

  void test_chip_id(int BMI323_CHIP_ID, int REG_CHIP_ID);
  
protected:

    // ── Variables globales del checklist ──────────────────────
  struct ValidationResult {
    bool chip_detected   = false;
    bool no_fatal_error  = false;
    bool por_ok          = false;
    bool acc_configured  = false;
    bool gyr_configured  = false;
    bool acc_x_ok        = false;
    bool acc_y_ok        = false;
    bool acc_z_ok        = false;
    bool gyr_x_ok        = false;
    bool gyr_y_ok        = false;
    bool gyr_z_ok        = false;
    bool temp_ok         = false;
    bool stability_ok    = false;
  };

  //Declaraciones de validacion 
  ValidationResult vr;


  void print_separator();

  void print_pass(const char* test);

  void print_fail(const char* test);

  void print_warn(const char* test);
private:

  TwoWire *_wire;
  uint8_t _address;

  /*
    Write 16-bit value to register
  */
  void writeRegister16(uint8_t reg, uint16_t value);

  /*
    Read 16-bit value from register
  */
  uint16_t readRegister16(uint8_t reg);
};

#endif