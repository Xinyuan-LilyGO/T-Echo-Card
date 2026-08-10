/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-09-12 16:42:57
 * @LastEditTime: 2026-08-10 09:44:54
 * @License: GPL 3.0
 */
#include <Adafruit_TinyUSB.h>
#include "t_echo_card_config.h"

constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t GNSS_SERIAL_BAUD = 9600;

void setup()
{
    Serial.begin(USB_SERIAL_BAUD);

    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);
    delay(100);
    digitalWrite(RT9080_EN, LOW);
    delay(100);
    digitalWrite(RT9080_EN, HIGH);
    delay(100);

    pinMode(GPS_WAKE_UP, OUTPUT);
    digitalWrite(GPS_WAKE_UP, HIGH);

    pinMode(GPS_RF_EN, OUTPUT);
    digitalWrite(GPS_RF_EN, HIGH);

    pinMode(GPS_1PPS, INPUT);

    Serial2.setPins(GPS_UART_TX, GPS_UART_RX);
    Serial2.begin(GNSS_SERIAL_BAUD);
}

void loop()
{
    while (Serial2.available() > 0)
    {
        Serial.write(static_cast<uint8_t>(Serial2.read()));
    }

    while (Serial.available() > 0)
    {
        Serial2.write(static_cast<uint8_t>(Serial.read()));
    }
}
