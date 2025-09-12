/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-09-12 16:42:57
 * @LastEditTime: 2025-09-12 18:02:42
 * @License: GPL 3.0
 */
#include "Adafruit_NeoPixel.h"
#include "t_echo_card_config.h"

#define NUM_LEDS 1

Adafruit_NeoPixel Led_1(NUM_LEDS, WS2812_DATA_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel Led_2(NUM_LEDS, WS2812_DATA_2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel Led_3(NUM_LEDS, WS2812_DATA_3, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel *Led[] =
    {
        &Led_1,
        &Led_2,
        &Led_3,
};

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");

    // 3.3V Power ON
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->begin();
        Led[i]->show();
        Led[i]->setBrightness(50);
    }
}

void loop()
{
    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(255, 0, 0));
        Led[i]->show();
    }
    delay(500);

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(0, 255, 0));
        Led[i]->show();
    }
    delay(500);

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(0, 0, 255));
        Led[i]->show();
    }
    delay(500);

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(255, 255, 255));
        Led[i]->show();
    }
    delay(500);
}