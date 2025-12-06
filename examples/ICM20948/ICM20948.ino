/*
 * @Description: ICM20948 Sensor Test
 * @Author: LILYGO_L
 * @Date: 2024-11-07 12:04:52
 * @LastEditTime: 2025-12-06 14:46:11
 * @License: GPL 3.0
 */

#include <Wire.h>
#include "ICM20948_WE.h"
#include <Adafruit_TinyUSB.h>
#include "t_echo_card_config.h"

/* There are several ways to create your ICM20948 object:
 * ICM20948_WE myIMU = ICM20948_WE()              -> uses Wire / I2C Address = 0x68
 * ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR) -> uses Wire / ICM20948_ADDR
 * ICM20948_WE myIMU = ICM20948_WE(&wire2)        -> uses the TwoWire object wire2 / ICM20948_ADDR
 * ICM20948_WE myIMU = ICM20948_WE(&wire2, ICM20948_ADDR) -> all together
 * ICM20948_WE myIMU = ICM20948_WE(CS_PIN, spi);  -> uses SPI, spi is just a flag, see SPI example
 * ICM20948_WE myIMU = ICM20948_WE(&SPI, CS_PIN, spi);  -> uses SPI / passes the SPI object, spi is just a flag, see SPI example
 */
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDRESS);

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");

    // 3.3V Power ON
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    Wire.setPins(ICM20948_SDA, ICM20948_SCL);
    Wire.begin();
    while (myIMU.init() == false)
    {
        printf("ICM20948 AG initialization failed\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (myIMU.initMagnetometer() == false)
    {
        printf("ICM20948 M initialization failed\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("ICM20948 initialization successful\n");

    printf("Position your ICM20948 flat and don't move it - calibrating...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    myIMU.autoOffsets();
    printf("Done!\n");

    myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
    myIMU.setAccDLPF(ICM20948_DLPF_6);
    myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
}

void loop()
{
    myIMU.readSensor();

    xyzFloat gValue;
    myIMU.getGValues(&gValue);
    xyzFloat angle;
    myIMU.getAngles(&angle);
    float pitch = myIMU.getPitch();
    float roll = myIMU.getRoll();

    // 获取磁力计的 x, y 值以计算航向角（Yaw）
    xyzFloat magValues;
    myIMU.getMagValues(&magValues);
    float yaw = atan2(magValues.y, magValues.x) * (180.0 / M_PI); // 计算航向角

    printf("gValue (x,y,z): %f , %f , %f\n", gValue.x, gValue.y, gValue.z);
    printf("angle (x,y,z): %f , %f , %f\n", angle.x, angle.y, angle.z);
    printf("magValues (x,y,z): %f , %f , %f\n", magValues.x, magValues.y, magValues.z);

    printf("Euler angles (pitch,Roll,Yaw): %f , %f , %f\n", pitch, roll, yaw);

    delay(100);
}
