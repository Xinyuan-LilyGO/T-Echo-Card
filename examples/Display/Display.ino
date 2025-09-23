#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "t_echo_card_config.h"
#include "Display_Fonts.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, SCREEN_RST);

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");

    // 3.3V Power ON
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    Wire.setPins(SCREEN_SDA, SCREEN_SCL);
    Wire.begin();

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println("SSD1306 allocation failed");
        while (1)
        {
            /* code */
        }
    }

    display.setOffsetCursor(28, 24);

    display.clearDisplay();

    display.fillScreen(WHITE);
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(BLACK); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner

    display.write("Ciallo");

    display.display();
    delay(100);
}

void loop()
{
    // Scroll in various directions, pausing in-between:
    // display.startscrollleft(0x00, 0x0F);
    // delay(2500);
    // display.stopscroll();
    // delay(1000);
    // display.startscrollright(0x00, 0x0F);
    // delay(2500);
    // display.stopscroll();
    delay(1000);
    Serial.println("Ciallo");
}
