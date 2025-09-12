/*
 * @Description: t_echo_card_config
 * @Author: LILYGO
 * @Date: 2024-12-06 14:37:43
 * @LastEditTime: 2025-09-12 16:45:55
 * @License: GPL 3.0
 */
#pragma once

#define _PINNUM(port, pin) ((port) * 32 + (pin))

////////////////////////////////////////////////// gpio config //////////////////////////////////////////////////

// IIC
#define IIC_1_SDA _PINNUM(1, 4)
#define IIC_1_SCL _PINNUM(1, 2)

// ZD25WQ32CEIGR SPI
#define ZD25WQ32C_CS _PINNUM(0, 12)
#define ZD25WQ32C_SCLK _PINNUM(0, 4)
#define ZD25WQ32C_MOSI _PINNUM(0, 6)
#define ZD25WQ32C_MISO _PINNUM(0, 8)
#define ZD25WQ32C_IO0 _PINNUM(0, 6)
#define ZD25WQ32C_IO1 _PINNUM(0, 8)
#define ZD25WQ32C_IO2 _PINNUM(1, 9)
#define ZD25WQ32C_IO3 _PINNUM(0, 26)

// GDEM0122T16
#define SCREEN_WIDTH 176
#define SCREEN_HEIGHT 192
#define SCREEN_BS1 _PINNUM(1, 12)
#define SCREEN_BUSY _PINNUM(0, 3)
#define SCREEN_RST _PINNUM(0, 28)
#define SCREEN_DC _PINNUM(0, 21)
#define SCREEN_CS _PINNUM(0, 22)
#define SCREEN_SCLK _PINNUM(0, 19)
#define SCREEN_MOSI _PINNUM(0, 20)
#define SCREEN_SRAM_CS -1
#define SCREEN_MISO -1

// BOOT
#define nRF52840_BOOT _PINNUM(0, 24)

// key
#define KEY_1 _PINNUM(1, 10)

// Battery
#define BATTERY_MEASUREMENT_CONTROL _PINNUM(0, 31)
#define BATTERY_ADC_DATA _PINNUM(0, 2)

// RT9080
#define RT9080_EN _PINNUM(0, 30)

// GPS
#define GPS_WAKE_UP _PINNUM(0, 25)
#define GPS_1PPS _PINNUM(0, 23)
#define GPS_UART_TX _PINNUM(0, 21)
#define GPS_UART_RX _PINNUM(0, 19)

// buzzer
#define BUZZER_DATA _PINNUM(1, 6)

// microphone
#define MICROPHONE_SCLK _PINNUM(1, 3)
#define MICROPHONE_DATA _PINNUM(1, 5)

// speaker
#define SPEAKER_EN _PINNUM(1, 11)
#define SPEAKER_EN_2 _PINNUM(0, 3)
#define SPEAKER_BCLK _PINNUM(0, 16)
#define SPEAKER_DATA _PINNUM(0, 20)
#define SPEAKER_WS_LRCK _PINNUM(0, 22)

// ws2812
#define WS2812_DATA_1 _PINNUM(1, 7)
#define WS2812_DATA_2 _PINNUM(1, 12)
#define WS2812_DATA_3 _PINNUM(0, 28)

////////////////////////////////////////////////// gpio config //////////////////////////////////////////////////
