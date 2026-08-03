/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-01-07 09:50:04
 * @LastEditTime: 2026-01-07 09:54:21
 * @License: GPL 3.0
 */

#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include "zd25_flash_config.h"
#include "t_echo_card_config.h"

// QSPI
Adafruit_FlashTransport_QSPI flashTransport(ZD25WQ32C_SCLK, ZD25WQ32C_CS,
                                            ZD25WQ32C_IO0, ZD25WQ32C_IO1,
                                            ZD25WQ32C_IO2, ZD25WQ32C_IO3);

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;
File32 myFile;

void setup()
{
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10); // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Ciallo");

    // 3.3V Power ON
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    delay(500);

    Serial.println("Initializing Filesystem on external flash...");

    // Init external flash
    while (flash.begin(ZD25WQ32_DEVICES, ZD25WQ32_DEVICE_COUNT) == false)
    {
        Serial.println("Flash initialization failed");
        delay(1000);
    }
    Serial.println("Flash initialization successful");

    // QSPI
    flashTransport.setClockSpeed(32000000UL, 0);

    // Open file system on the flash
    if (!fatfs.begin(&flash))
    {
        Serial.println("Error: filesystem is not existed. Please try SdFat_format "
                       "example to make one.");
        while (1)
        {
            yield();
            delay(1);
        }
    }

    Serial.println("initialization done.");

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    myFile = fatfs.open("test.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile)
    {
        Serial.print("Writing to test.txt...");
        myFile.println("testing 1, 2, 3.");
        // close the file:
        myFile.close();
        Serial.println("done.");
    }
    else
    {
        // if the file didn't open, print an error:
        Serial.println("error opening test.txt");
    }

    // re-open the file for reading:
    myFile = fatfs.open("test.txt");
    if (myFile)
    {
        Serial.println("test.txt:");

        // read from the file until there's nothing else in it:
        while (myFile.available())
        {
            Serial.write(myFile.read());
        }
        // close the file:
        myFile.close();
    }
    else
    {
        // if the file didn't open, print an error:
        Serial.println("error opening test.txt");
    }
}

void loop()
{
    // nothing happens after setup
}
