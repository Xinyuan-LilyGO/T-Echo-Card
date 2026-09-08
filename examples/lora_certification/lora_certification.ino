/*
 * @Description: T-Echo-Card 868 MHz LoRa 射频认证发包测试
 * @Author: LILYGO_L
 * @Date: 2026-09-07 11:01:11
 * @LastEditTime: 2026-09-08 15:36:35
 * @License: GPL 3.0
 */
#include <Arduino.h>
#include <SPI.h>
#include "RadioLib.h"
#include "t_echo_card_config.h"

// LoRa 射频测试参数。
static constexpr float rf_frequency_mhz = 868.0;
static constexpr float rf_bandwidth_khz = 500.0;
static constexpr uint8_t lora_spreading_factor = 5;
static constexpr uint8_t lora_coding_rate = 8;
static constexpr uint8_t lora_sync_word = 0xAB;
static constexpr int8_t rf_output_power_dbm = 7;
static constexpr float rf_current_limit_ma = 140.0;
static constexpr uint16_t lora_preamble_length = 16;
static constexpr bool lora_crc_enabled = false;

// 目标占空比为 0.1%，使用 ppm 避免浮点计算误差。
static constexpr uint32_t target_duty_cycle_ppm = 1000;
static constexpr size_t payload_size = 32;

// SX1262 使用 SPIM3 总线。
SPIClass radio_spi(NRF_SPIM3, SX1262_MISO, SX1262_SCLK, SX1262_MOSI);
SX1262 radio = new Module(SX1262_CS, SX1262_DIO1, SX1262_RST, SX1262_BUSY, radio_spi);

static uint8_t payload[payload_size];
static uint32_t packet_counter = 0;
static uint32_t next_transmission_ms = 0;
static uint32_t tx_interval_ms = 0;
static uint32_t time_on_air_us = 0;

/**
 * @brief 输出射频驱动错误并停止测试程序。
 * @param operation 发生错误的操作名称。
 * @param state RadioLib 错误码。
 */
static void haltOnRadioError(const char *operation, int16_t state)
{
    Serial.print("[SX1262] ");
    Serial.print(operation);
    Serial.print(" failed, code ");
    Serial.println(state);

    while (true)
    {
        delay(1000);
    }
}

/**
 * @brief 生成由计数器和交替测试字节组成的固定长度载荷。
 */
static void preparePayload()
{
    for (size_t index = 0; index < payload_size; ++index)
    {
        payload[index] = (index & 1U) ? 0xAA : 0x55;
    }

    payload[0] = static_cast<uint8_t>(packet_counter >> 24);
    payload[1] = static_cast<uint8_t>(packet_counter >> 16);
    payload[2] = static_cast<uint8_t>(packet_counter >> 8);
    payload[3] = static_cast<uint8_t>(packet_counter);
}

/**
 * @brief 根据单包空中时间计算满足目标占空比的发包周期。
 * @param packet_time_on_air_us 单包空中时间，单位为微秒。
 * @return 两次发包开始时刻之间的最小间隔，单位为毫秒。
 */
static uint32_t calculateTransmissionInterval(uint32_t packet_time_on_air_us)
{
    const uint64_t interval_numerator =
        static_cast<uint64_t>(packet_time_on_air_us) * 1000ULL;

    return static_cast<uint32_t>(
        (interval_numerator + target_duty_cycle_ppm - 1ULL) / target_duty_cycle_ppm);
}

/**
 * @brief 按照整机测试时序重新开启 3.3 V 外设电源。
 */
static void enablePeripheralPower()
{
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);
    delay(100);
    digitalWrite(RT9080_EN, LOW);
    delay(100);
    digitalWrite(RT9080_EN, HIGH);
    delay(100);
}

void setup()
{
    Serial.begin(115200);

    // 未连接串口监视器时不阻塞射频测试，最多等待三秒。
    const uint32_t serial_wait_started_ms = millis();
    while (!Serial && (millis() - serial_wait_started_ms < 3000))
    {
        delay(10);
    }

    enablePeripheralPower();
    radio_spi.begin();

    int16_t state = radio.begin(
        rf_frequency_mhz,
        rf_bandwidth_khz,
        lora_spreading_factor,
        lora_coding_rate,
        lora_sync_word,
        rf_output_power_dbm,
        lora_preamble_length,
        SX1262_TCXO_VOLTAGE,
        SX1262_USE_REGULATOR_LDO);
    if (state != RADIOLIB_ERR_NONE)
    {
        haltOnRadioError("initialization", state);
    }

    // RadioLib 参数顺序为接收使能引脚、发送使能引脚。
    radio.setRfSwitchPins(SX1262_RF_VC2, SX1262_RF_VC1);

    state = radio.setCurrentLimit(rf_current_limit_ma);
    if (state != RADIOLIB_ERR_NONE)
    {
        haltOnRadioError("current limit configuration", state);
    }

    state = radio.setCRC(lora_crc_enabled);
    if (state != RADIOLIB_ERR_NONE)
    {
        haltOnRadioError("CRC configuration", state);
    }

    time_on_air_us = static_cast<uint32_t>(radio.getTimeOnAir(payload_size));
    tx_interval_ms = calculateTransmissionInterval(time_on_air_us);

    Serial.println();
    Serial.println("T-Echo-Card LoRa certification test");
    Serial.print("Frequency: ");
    Serial.print(rf_frequency_mhz, 1);
    Serial.println(" MHz");
    Serial.print("Bandwidth: ");
    Serial.print(rf_bandwidth_khz, 1);
    Serial.println(" kHz");
    Serial.print("Spreading factor: ");
    Serial.println(lora_spreading_factor);
    Serial.print("Coding rate: 4/");
    Serial.println(lora_coding_rate);
    Serial.print("Sync word: 0x");
    Serial.println(lora_sync_word, HEX);
    Serial.print("Output power: ");
    Serial.print(rf_output_power_dbm);
    Serial.println(" dBm");
    Serial.print("Current limit: ");
    Serial.print(rf_current_limit_ma, 1);
    Serial.println(" mA");
    Serial.print("Preamble length: ");
    Serial.println(lora_preamble_length);
    Serial.print("CRC: ");
    Serial.println(lora_crc_enabled ? "enabled" : "disabled");
    Serial.print("Payload size: ");
    Serial.print(payload_size);
    Serial.println(" bytes");
    Serial.print("Target duty cycle: ");
    Serial.print(static_cast<float>(target_duty_cycle_ppm) / 10000.0F, 3);
    Serial.println("%");
    Serial.print("Time on air: ");
    Serial.print(time_on_air_us);
    Serial.println(" us");
    Serial.print("Transmit interval: ");
    Serial.print(tx_interval_ms);
    Serial.println(" ms");

    next_transmission_ms = millis();
}

void loop()
{
    const uint32_t current_ms = millis();
    if (static_cast<int32_t>(current_ms - next_transmission_ms) < 0)
    {
        delay(1);
        return;
    }

    next_transmission_ms = current_ms + tx_interval_ms;
    preparePayload();

    Serial.print("[SX1262] Transmitting packet ");
    Serial.print(packet_counter);
    Serial.print(" ... ");

    const int16_t state = radio.transmit(payload, payload_size);
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println("success");
    }
    else
    {
        Serial.print("failed, code ");
        Serial.println(state);
    }

    ++packet_counter;
}
