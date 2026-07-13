#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "t_echo_card_config.h"
#include "Display_Fonts.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, SCREEN_RST);

constexpr uint8_t SSD1315_SET_IREF = 0xAD;
constexpr uint8_t SSD1315_INTERNAL_IREF_19UA = 0x10;

// OLED 没有背光，SSD1315 通过对比度寄存器调节发光亮度。
void setOledBrightness(uint8_t brightness)
{
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(brightness);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("OLED screen and brightness test");

    // 3.3V Power ON
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    Wire.setPins(SCREEN_SDA, SCREEN_SCL);
    Wire.begin();

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println("SSD1315 allocation failed");
        while (1)
        {
            /* code */
        }
    }

    // SSD1315-specific: use the internal 19 uA IREF while the display is on.
    // This avoids an incorrect or saturated external IREF masking contrast changes.
    display.ssd1306_command(SSD1315_SET_IREF);
    display.ssd1306_command(SSD1315_INTERNAL_IREF_19UA);

    display.setOffsetCursor(28, 24);

    display.clearDisplay();

    display.fillScreen(WHITE);
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(BLACK); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner

    display.println("Brightness");
    display.print("Test");

    display.display();
}

void loop()
{
    static int16_t brightness = 1;
    static int8_t direction = 1;

    setOledBrightness((uint8_t)brightness);

    // Reduce serial output while keeping useful progress markers.
    if ((brightness % 16 == 1) || (brightness == 255))
    {
        Serial.print("SSD1315 brightness: ");
        Serial.println(brightness);
    }

    if (brightness >= 255)
    {
        direction = -1;
    }
    else if (brightness <= 1)
    {
        direction = 1;
    }

    brightness += direction;

    // About 5 seconds from minimum to maximum brightness.
    delay(20);
}
