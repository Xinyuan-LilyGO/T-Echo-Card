/*
 * @Description: GPS test
 * @Author: LILYGO_L
 * @Date: 2024-10-25 17:57:30
 * @LastEditTime: 2025-09-12 15:24:21
 * @License: GPL 3.0
 */
#include "cpp_bus_driver_library.h"
#include "t_echo_card_config.h"
#include <Adafruit_TinyUSB.h>

size_t CycleTime = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");

    // 3.3V Power ON
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    pinMode(BUZZER_DATA, OUTPUT);

    tone(BUZZER_DATA, 2000, 1000);
}

void loop()
{
    if (millis() > CycleTime)
    {
        tone(BUZZER_DATA, 2000, 1000);
        CycleTime = millis() + 3000;
    }
}