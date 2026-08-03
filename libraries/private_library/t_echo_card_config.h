/*
 * @Description: t_echo_card_config
 * @Author: LILYGO
 * @Date: 2024-12-06 14:37:43
 * @LastEditTime: 2026-08-03 14:24:53
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

// SSD1315
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define SCREEN_RST -1
#define SCREEN_SDA IIC_1_SDA
#define SCREEN_SCL IIC_1_SCL

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
#define GPS_EN _PINNUM(1, 15)
#define GPS_WAKE_UP _PINNUM(0, 25)
#define GPS_1PPS _PINNUM(0, 23)
#define GPS_UART_TX _PINNUM(0, 19)
#define GPS_UART_RX _PINNUM(0, 21)
#define GPS_RF_EN _PINNUM(0, 29)

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

// ICM20948
#define ICM20948_SDA IIC_1_SDA
#define ICM20948_SCL IIC_1_SCL

// Lora S62F(SX1262)
#define SX1262_CS _PINNUM(0, 11)
#define SX1262_RST _PINNUM(0, 7)
#define SX1262_SCLK _PINNUM(0, 13)
#define SX1262_MOSI _PINNUM(0, 15)
#define SX1262_MISO _PINNUM(0, 17)
#define SX1262_BUSY _PINNUM(0, 14)
#define SX1262_INT _PINNUM(1, 8)
#define SX1262_DIO1 _PINNUM(1, 8)
#define SX1262_DIO2 _PINNUM(0, 5)

// 本板使用 AcSiP 射频开关控制模式 A，由 MCU 分别驱动两个 RF_VC 引脚。
// DIO2 为单独引出的信号，不能替代这两个 GPIO 控制引脚。
#define SX1262_RF_VC1 _PINNUM(0, 27)
#define SX1262_RF_VC2 _PINNUM(1, 1)

////////////////////////////////////////////////// gpio config //////////////////////////////////////////////////

////////////////////////////////////////////////// other define config //////////////////////////////////////////////////

// ICM20948
#define ICM20948_ADDRESS 0x68

// S62F 使用内置的 32 MHz TCXO，由 SX1262 的 DIO3 提供 3.0 V 电源。
// VREG 与 DCC_SW 通过外置 15 uH 电感连接，因此使用 SX1262 的 DC-DC
// 稳压模式，不强制使用纯 LDO 模式。
#define SX1262_TCXO_VOLTAGE 3.0
#define SX1262_USE_REGULATOR_LDO false

////////////////////////////////////////////////// other define config //////////////////////////////////////////////////
