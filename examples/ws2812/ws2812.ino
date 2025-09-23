/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-09-12 16:42:57
 * @LastEditTime: 2025-09-22 15:28:52
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

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait)
{
    // Hue of first pixel runs 5 complete loops through the color wheel.
    // Color wheel has a range of 65536 but it's OK if we roll over, so
    // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
    // means we'll make 5*65536/256 = 1280 passes through this loop:
    for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256)
    {
        for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
        {
            // strip.rainbow() can take a single argument (first pixel hue) or
            // optionally a few extras: number of rainbow repetitions (default 1),
            // saturation and value (brightness) (both 0-255, similar to the
            // ColorHSV() function, default 255), and a true/false flag for whether
            // to apply gamma correction to provide 'truer' colors (default true).
            Led[i]->rainbow(firstPixelHue);
            // Above line is equivalent to:
            // strip.rainbow(firstPixelHue, 1, 255, 255, true);
            Led[i]->show(); // Update strip with new contents
        }
        delay(wait); // Pause for a moment
    }
}

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

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(0, 0, 0));
        Led[i]->show();
    }
    delay(500);
}

void loop()
{
    rainbow(100); // Flowing rainbow cycle along the whole strip
}