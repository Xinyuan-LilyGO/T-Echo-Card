/*!
 * @file Adafruit_QMC5883P.h
 *
 * This is a library for the QMC5883P 3-axis magnetometer
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by ladyada for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef _ADAFRUIT_QMC5883P_H_
#define _ADAFRUIT_QMC5883P_H_

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>

#include "Arduino.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define QMC5883P_DEFAULT_ADDR 0x2C ///< Default I2C address
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
#define QMC5883P_REG_CHIPID 0x00   ///< Chip ID register
#define QMC5883P_REG_XOUT_LSB 0x01 ///< X-axis output LSB register
#define QMC5883P_REG_XOUT_MSB 0x02 ///< X-axis output MSB register
#define QMC5883P_REG_YOUT_LSB 0x03 ///< Y-axis output LSB register
#define QMC5883P_REG_YOUT_MSB 0x04 ///< Y-axis output MSB register
#define QMC5883P_REG_ZOUT_LSB 0x05 ///< Z-axis output LSB register
#define QMC5883P_REG_ZOUT_MSB 0x06 ///< Z-axis output MSB register
#define QMC5883P_REG_STATUS 0x09   ///< Status register
#define QMC5883P_REG_CONTROL1 0x0A ///< Control register 1
#define QMC5883P_REG_CONTROL2 0x0B ///< Control register 2
/*=========================================================================*/

/*=========================================================================
    CONTROL REGISTER 1 ENUMS
    -----------------------------------------------------------------------*/
/*!
 * @brief Operating mode options
 */
typedef enum {
  QMC5883P_MODE_SUSPEND = 0x00,   ///< Suspend mode
  QMC5883P_MODE_NORMAL = 0x01,    ///< Normal mode
  QMC5883P_MODE_SINGLE = 0x02,    ///< Single measurement mode
  QMC5883P_MODE_CONTINUOUS = 0x03 ///< Continuous mode
} qmc5883p_mode_t;

/*!
 * @brief Output data rate options
 */
typedef enum {
  QMC5883P_ODR_10HZ = 0x00,  ///< 10 Hz output data rate
  QMC5883P_ODR_50HZ = 0x01,  ///< 50 Hz output data rate
  QMC5883P_ODR_100HZ = 0x02, ///< 100 Hz output data rate
  QMC5883P_ODR_200HZ = 0x03  ///< 200 Hz output data rate
} qmc5883p_odr_t;

/*!
 * @brief Over sample ratio options
 */
typedef enum {
  QMC5883P_OSR_8 = 0x00, ///< Over sample ratio = 8
  QMC5883P_OSR_4 = 0x01, ///< Over sample ratio = 4
  QMC5883P_OSR_2 = 0x02, ///< Over sample ratio = 2
  QMC5883P_OSR_1 = 0x03  ///< Over sample ratio = 1
} qmc5883p_osr_t;

/*!
 * @brief Downsample ratio options
 */
typedef enum {
  QMC5883P_DSR_1 = 0x00, ///< Downsample ratio = 1
  QMC5883P_DSR_2 = 0x01, ///< Downsample ratio = 2
  QMC5883P_DSR_4 = 0x02, ///< Downsample ratio = 4
  QMC5883P_DSR_8 = 0x03  ///< Downsample ratio = 8
} qmc5883p_dsr_t;
/*=========================================================================*/

/*=========================================================================
    CONTROL REGISTER 2 ENUMS
    -----------------------------------------------------------------------*/
/*!
 * @brief Field range options
 */
typedef enum {
  QMC5883P_RANGE_30G = 0x00, ///< ±30 Gauss range
  QMC5883P_RANGE_12G = 0x01, ///< ±12 Gauss range
  QMC5883P_RANGE_8G = 0x02,  ///< ±8 Gauss range
  QMC5883P_RANGE_2G = 0x03   ///< ±2 Gauss range
} qmc5883p_range_t;

/*!
 * @brief Set/Reset mode options
 */
typedef enum {
  QMC5883P_SETRESET_ON = 0x00,      ///< Set and reset on
  QMC5883P_SETRESET_SETONLY = 0x01, ///< Set only on
  QMC5883P_SETRESET_OFF = 0x02      ///< Set and reset off
} qmc5883p_setreset_t;
/*=========================================================================*/

/*!
 * @brief Class for hardware interfacing with the QMC5883P 3-axis magnetometer
 */
class Adafruit_QMC5883P {
 public:
  Adafruit_QMC5883P(void);
  ~Adafruit_QMC5883P(void);

  bool begin(uint8_t i2c_addr = QMC5883P_DEFAULT_ADDR, TwoWire* wire = &Wire);
  bool getRawMagnetic(int16_t* x, int16_t* y, int16_t* z);
  bool getGaussField(float* x, float* y, float* z);
  bool isDataReady();
  bool isOverflow();
  void setMode(qmc5883p_mode_t mode);
  qmc5883p_mode_t getMode();
  void setODR(qmc5883p_odr_t odr);
  qmc5883p_odr_t getODR();
  void setOSR(qmc5883p_osr_t osr);
  qmc5883p_osr_t getOSR();
  void setDSR(qmc5883p_dsr_t dsr);
  qmc5883p_dsr_t getDSR();
  bool softReset();
  bool selfTest();
  void setRange(qmc5883p_range_t range);
  qmc5883p_range_t getRange();
  void setSetResetMode(qmc5883p_setreset_t mode);
  qmc5883p_setreset_t getSetResetMode();

 private:
  Adafruit_I2CDevice* i2c_dev; ///< Pointer to I2C bus interface
};

#endif