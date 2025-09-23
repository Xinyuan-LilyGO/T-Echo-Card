/*!
 * @file Adafruit_QMC5883P.cpp
 *
 * @mainpage Adafruit QMC5883P 3-axis magnetometer library
 *
 * @section intro_sec Introduction
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
 * @section author Author
 *
 * Written by ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_QMC5883P.h"

/*!
 *  @brief  Instantiates a new QMC5883P class
 */
Adafruit_QMC5883P::Adafruit_QMC5883P(void) {
  i2c_dev = NULL;
}

/*!
 *  @brief  Cleans up the QMC5883P
 */
Adafruit_QMC5883P::~Adafruit_QMC5883P(void) {
  if (i2c_dev) {
    delete i2c_dev;
  }
}

/*!
 *  @brief  Sets up the hardware and initializes I2C
 *  @param  i2c_addr
 *          The I2C address to be used.
 *  @param  wire
 *          The Wire object to be used for I2C connections.
 *  @return True if initialization was successful, otherwise false.
 */
bool Adafruit_QMC5883P::begin(uint8_t i2c_addr, TwoWire* wire) {
  if (i2c_dev) {
    delete i2c_dev;
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  // Check chip ID
  Adafruit_BusIO_Register chip_id_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CHIPID);
  uint8_t chip_id = chip_id_reg.read();
  if (chip_id != 0x80) {
    return false;
  }

  return true;
}

/*!
 *  @brief  Reads raw magnetic field data from all three axes
 *  @param  x
 *          Pointer to store X-axis raw data (2's complement)
 *  @param  y
 *          Pointer to store Y-axis raw data (2's complement)
 *  @param  z
 *          Pointer to store Z-axis raw data (2's complement)
 *  @return True if read was successful, otherwise false.
 */
bool Adafruit_QMC5883P::getRawMagnetic(int16_t* x, int16_t* y, int16_t* z) {
  uint8_t buffer[6];

  // Read all 6 bytes (X,Y,Z LSB+MSB) starting from X LSB register
  Adafruit_BusIO_Register data_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_XOUT_LSB, 6);
  if (!data_reg.read(buffer, 6)) {
    return false;
  }

  // Combine LSB and MSB for each axis (2's complement)
  *x = (int16_t)((buffer[1] << 8) | buffer[0]); // X = MSB[1] | LSB[0]
  *y = (int16_t)((buffer[3] << 8) | buffer[2]); // Y = MSB[3] | LSB[2]
  *z = (int16_t)((buffer[5] << 8) | buffer[4]); // Z = MSB[5] | LSB[4]

  return true;
}

/*!
 *  @brief  Reads magnetic field data and converts to Gauss
 *  @param  x
 *          Pointer to store X-axis field in Gauss
 *  @param  y
 *          Pointer to store Y-axis field in Gauss
 *  @param  z
 *          Pointer to store Z-axis field in Gauss
 *  @return True if read was successful, otherwise false.
 */
bool Adafruit_QMC5883P::getGaussField(float* x, float* y, float* z) {
  int16_t raw_x, raw_y, raw_z;

  // Get raw magnetic data
  if (!getRawMagnetic(&raw_x, &raw_y, &raw_z)) {
    return false;
  }

  // Get current range to determine conversion factor
  qmc5883p_range_t range = getRange();
  float lsb_per_gauss;

  switch (range) {
    case QMC5883P_RANGE_30G:
      lsb_per_gauss = 1000.0;
      break;
    case QMC5883P_RANGE_12G:
      lsb_per_gauss = 2500.0;
      break;
    case QMC5883P_RANGE_8G:
      lsb_per_gauss = 3750.0;
      break;
    case QMC5883P_RANGE_2G:
      lsb_per_gauss = 15000.0;
      break;
    default:
      return false;
  }

  // Convert to Gauss
  *x = (float)raw_x / lsb_per_gauss;
  *y = (float)raw_y / lsb_per_gauss;
  *z = (float)raw_z / lsb_per_gauss;

  return true;
}

/*!
 *  @brief  Checks if new magnetic data is ready
 *  @return True if new data is ready, false otherwise
 */
bool Adafruit_QMC5883P::isDataReady() {
  Adafruit_BusIO_Register status_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_STATUS);
  Adafruit_BusIO_RegisterBits drdy_bit =
      Adafruit_BusIO_RegisterBits(&status_reg, 1, 0);
  return drdy_bit.read();
}

/*!
 *  @brief  Checks if data overflow has occurred
 *  @return True if overflow occurred, false otherwise
 */
bool Adafruit_QMC5883P::isOverflow() {
  Adafruit_BusIO_Register status_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_STATUS);
  Adafruit_BusIO_RegisterBits ovfl_bit =
      Adafruit_BusIO_RegisterBits(&status_reg, 1, 1);
  return ovfl_bit.read();
}

/*!
 *  @brief  Sets the operating mode
 *  @param  mode
 *          The operating mode to set
 */
void Adafruit_QMC5883P::setMode(qmc5883p_mode_t mode) {
  Adafruit_BusIO_Register ctrl1_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL1);
  Adafruit_BusIO_RegisterBits mode_bits =
      Adafruit_BusIO_RegisterBits(&ctrl1_reg, 2, 0);
  mode_bits.write(mode);
}

/*!
 *  @brief  Gets the current operating mode
 *  @return The current operating mode
 */
qmc5883p_mode_t Adafruit_QMC5883P::getMode() {
  Adafruit_BusIO_Register ctrl1_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL1);
  Adafruit_BusIO_RegisterBits mode_bits =
      Adafruit_BusIO_RegisterBits(&ctrl1_reg, 2, 0);
  return (qmc5883p_mode_t)mode_bits.read();
}

/*!
 *  @brief  Sets the output data rate
 *  @param  odr
 *          The output data rate to set
 */
void Adafruit_QMC5883P::setODR(qmc5883p_odr_t odr) {
  Adafruit_BusIO_Register ctrl1_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL1);
  Adafruit_BusIO_RegisterBits odr_bits =
      Adafruit_BusIO_RegisterBits(&ctrl1_reg, 2, 2);
  odr_bits.write(odr);
}

/*!
 *  @brief  Gets the current output data rate
 *  @return The current output data rate
 */
qmc5883p_odr_t Adafruit_QMC5883P::getODR() {
  Adafruit_BusIO_Register ctrl1_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL1);
  Adafruit_BusIO_RegisterBits odr_bits =
      Adafruit_BusIO_RegisterBits(&ctrl1_reg, 2, 2);
  return (qmc5883p_odr_t)odr_bits.read();
}

/*!
 *  @brief  Sets the over sample ratio
 *  @param  osr
 *          The over sample ratio to set
 */
void Adafruit_QMC5883P::setOSR(qmc5883p_osr_t osr) {
  Adafruit_BusIO_Register ctrl1_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL1);
  Adafruit_BusIO_RegisterBits osr_bits =
      Adafruit_BusIO_RegisterBits(&ctrl1_reg, 2, 4);
  osr_bits.write(osr);
}

/*!
 *  @brief  Gets the current over sample ratio
 *  @return The current over sample ratio
 */
qmc5883p_osr_t Adafruit_QMC5883P::getOSR() {
  Adafruit_BusIO_Register ctrl1_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL1);
  Adafruit_BusIO_RegisterBits osr_bits =
      Adafruit_BusIO_RegisterBits(&ctrl1_reg, 2, 4);
  return (qmc5883p_osr_t)osr_bits.read();
}

/*!
 *  @brief  Sets the downsample ratio
 *  @param  dsr
 *          The downsample ratio to set
 */
void Adafruit_QMC5883P::setDSR(qmc5883p_dsr_t dsr) {
  Adafruit_BusIO_Register ctrl1_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL1);
  Adafruit_BusIO_RegisterBits dsr_bits =
      Adafruit_BusIO_RegisterBits(&ctrl1_reg, 2, 6);
  dsr_bits.write(dsr);
}

/*!
 *  @brief  Gets the current downsample ratio
 *  @return The current downsample ratio
 */
qmc5883p_dsr_t Adafruit_QMC5883P::getDSR() {
  Adafruit_BusIO_Register ctrl1_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL1);
  Adafruit_BusIO_RegisterBits dsr_bits =
      Adafruit_BusIO_RegisterBits(&ctrl1_reg, 2, 6);
  return (qmc5883p_dsr_t)dsr_bits.read();
}

/*!
 *  @brief  Performs a soft reset of the chip
 *  @return True if reset was successful, false otherwise
 */
bool Adafruit_QMC5883P::softReset() {
  Adafruit_BusIO_Register ctrl2_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL2);
  Adafruit_BusIO_RegisterBits reset_bit =
      Adafruit_BusIO_RegisterBits(&ctrl2_reg, 1, 7);

  // Set reset bit
  reset_bit.write(1);

  // Wait for reset to complete (datasheet doesn't specify time, use 50ms)
  delay(50);

  // Check if chip ID is still valid after reset
  Adafruit_BusIO_Register chip_id_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CHIPID);
  uint8_t chip_id = chip_id_reg.read();
  return (chip_id == 0x80);
}

/*!
 *  @brief  Performs self-test of the chip
 *  @return True if self-test passed, false otherwise
 */
bool Adafruit_QMC5883P::selfTest() {
  Adafruit_BusIO_Register ctrl2_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL2);
  Adafruit_BusIO_RegisterBits test_bit =
      Adafruit_BusIO_RegisterBits(&ctrl2_reg, 1, 6);

  // Set self-test bit
  test_bit.write(1);

  // Wait for self-test to complete (datasheet suggests 5ms)
  delay(5);

  // Check if self-test bit auto-cleared (indicates completion)
  uint8_t test_status = test_bit.read();
  return (test_status == 0);
}

/*!
 *  @brief  Sets the magnetic field range
 *  @param  range
 *          The magnetic field range to set
 */
void Adafruit_QMC5883P::setRange(qmc5883p_range_t range) {
  Adafruit_BusIO_Register ctrl2_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL2);
  Adafruit_BusIO_RegisterBits range_bits =
      Adafruit_BusIO_RegisterBits(&ctrl2_reg, 2, 2);
  range_bits.write(range);
}

/*!
 *  @brief  Gets the current magnetic field range
 *  @return The current magnetic field range
 */
qmc5883p_range_t Adafruit_QMC5883P::getRange() {
  Adafruit_BusIO_Register ctrl2_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL2);
  Adafruit_BusIO_RegisterBits range_bits =
      Adafruit_BusIO_RegisterBits(&ctrl2_reg, 2, 2);
  return (qmc5883p_range_t)range_bits.read();
}

/*!
 *  @brief  Sets the set/reset mode
 *  @param  mode
 *          The set/reset mode to set
 */
void Adafruit_QMC5883P::setSetResetMode(qmc5883p_setreset_t mode) {
  Adafruit_BusIO_Register ctrl2_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL2);
  Adafruit_BusIO_RegisterBits setreset_bits =
      Adafruit_BusIO_RegisterBits(&ctrl2_reg, 2, 0);
  setreset_bits.write(mode);
}

/*!
 *  @brief  Gets the current set/reset mode
 *  @return The current set/reset mode
 */
qmc5883p_setreset_t Adafruit_QMC5883P::getSetResetMode() {
  Adafruit_BusIO_Register ctrl2_reg =
      Adafruit_BusIO_Register(i2c_dev, QMC5883P_REG_CONTROL2);
  Adafruit_BusIO_RegisterBits setreset_bits =
      Adafruit_BusIO_RegisterBits(&ctrl2_reg, 2, 0);
  return (qmc5883p_setreset_t)setreset_bits.read();
}