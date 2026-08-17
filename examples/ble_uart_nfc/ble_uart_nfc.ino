/*
 * @Description: Bluefruit BLE UART 与 nRF52840 NFC 标签共存测试
 * @Author: LILYGO_L
 * @Date: 2026-08-17 09:34:56
 * @LastEditTime: 2026-08-17 09:58:20
 * @License: GPL 3.0
 */
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include "arduino_nordicnrf52_nfc_library.h"

// BLE UART 服务。
BLEUart ble_uart;

// NFC 标签文本。
static const char nfc_text[] = "T-Echo-Card NFC Tag";

// BLE UART 发送和状态心跳输出间隔。
static constexpr uint32_t heartbeat_interval_ms = 1000;
static uint32_t heartbeat_previous_ms = 0;
static uint32_t heartbeat_count = 0;
static uint32_t ble_uart_send_count = 0;
static bool ble_running = false;

/**
 * @brief 获取当前 NFC 工作模式名称。
 * @param mode NFC 工作模式。
 * @return NFC 工作模式名称。
 */
static const char *nfcModeName(NordicNrf52NfcTag::Mode mode)
{
    switch (mode)
    {
    case NordicNrf52NfcTag::ModeType2Ndef:
        return "Type 2 NDEF read-only";
    case NordicNrf52NfcTag::ModeType2Raw:
        return "Type 2 raw read-only";
    case NordicNrf52NfcTag::ModeType4ReadOnly:
        return "Type 4 NDEF read-only";
    case NordicNrf52NfcTag::ModeType4ReadWrite:
        return "Type 4 NDEF read-write";
    case NordicNrf52NfcTag::ModeType4Raw:
        return "Type 4 raw ISO-DEP";
    default:
        return "none";
    }
}

/**
 * @brief 处理 BLE 连接事件。
 * @param connection_handle BLE 连接句柄。
 */
static void bleConnectCallback(uint16_t connection_handle)
{
    (void)connection_handle;
    Serial.println("BLE connected.");
}

/**
 * @brief 处理 BLE 断开事件。
 * @param connection_handle BLE 连接句柄。
 * @param reason BLE 断开原因。
 */
static void bleDisconnectCallback(uint16_t connection_handle, uint8_t reason)
{
    (void)connection_handle;
    Serial.print("BLE disconnected, reason: 0x");
    Serial.println(reason, HEX);
}

/**
 * @brief 启动 BLE UART 服务并持续广播。
 * @return true 启动成功，false 启动失败。
 */
static bool startBleUart()
{
    if (!Bluefruit.begin())
    {
        return false;
    }

    Bluefruit.setName("T-Echo-Card BLE NFC");
    Bluefruit.setTxPower(8);
    Bluefruit.Periph.setConnectCallback(bleConnectCallback);
    Bluefruit.Periph.setDisconnectCallback(bleDisconnectCallback);

    ble_uart.begin();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(ble_uart);
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);

    return true;
}

void setup()
{
    Serial.begin(115200);

    // 最多等待 USB 串口三秒，避免未打开串口时阻塞测试。
    const uint32_t serial_wait_start = millis();
    while (!Serial && (millis() - serial_wait_start < 3000))
    {
        delay(10);
    }

    Serial.println();
    Serial.println("T-Echo-Card BLE UART and NFC coexistence test");

    if (!startBleUart())
    {
        Serial.println("BLE UART startup failed.");
        return;
    }
    Serial.println("BLE UART started. Device name: T-Echo-Card BLE NFC");
    ble_running = true;

    if (!NordicNfc.beginText(nfc_text, "en"))
    {
        Serial.print("NFC startup failed: ");
        Serial.println(NordicNfc.lastErrorMessage());
        Serial.print("NFC driver error code: 0x");
        Serial.println(NordicNfc.lastDriverErrorCode(), HEX);
        return;
    }
    Serial.print("NFC tag started. Text: ");
    Serial.println(nfc_text);
}

void loop()
{
    const uint32_t events = NordicNfc.takeEvents();

    if ((events & NordicNrf52NfcTag::EventFieldOn) != 0)
    {
        Serial.println("NFC event: RF field detected and tag selected.");
    }
    if ((events & NordicNrf52NfcTag::EventDataRead) != 0)
    {
        Serial.println("NFC event: NDEF message read completed.");
    }
    if ((events & NordicNrf52NfcTag::EventDataUpdated) != 0)
    {
        Serial.println("NFC event: NDEF message updated.");
    }
    if ((events & NordicNrf52NfcTag::EventDataReceived) != 0)
    {
        Serial.println("NFC event: raw APDU fragment received.");
    }
    if ((events & NordicNrf52NfcTag::EventDataTransmitted) != 0)
    {
        Serial.println("NFC event: raw response APDU transmitted.");
    }
    if ((events & NordicNrf52NfcTag::EventFieldOff) != 0)
    {
        Serial.println("NFC event: RF field removed.");
    }
    if ((events & NordicNrf52NfcTag::EventStopped) != 0)
    {
        Serial.println("NFC event: tag emulation stopped.");
    }

    const uint32_t current_ms = millis();
    if (current_ms - heartbeat_previous_ms >= heartbeat_interval_ms)
    {
        heartbeat_previous_ms = current_ms;
        ++heartbeat_count;

        if (ble_running && Bluefruit.connected())
        {
            ++ble_uart_send_count;
            ble_uart.print("BLE UART #");
            ble_uart.println(ble_uart_send_count);
        }

        Serial.print("Heartbeat #");
        Serial.print(heartbeat_count);
        Serial.print(" | uptime_ms=");
        Serial.print(current_ms);
        Serial.print(" | ble_running=");
        Serial.print(ble_running ? "yes" : "no");
        Serial.print(" | ble_connected=");
        Serial.print((ble_running && Bluefruit.connected()) ? "yes" : "no");
        Serial.print(" | ble_tx_count=");
        Serial.print(ble_uart_send_count);
        Serial.print(" | nfc_running=");
        Serial.print(NordicNfc.isRunning() ? "yes" : "no");
        Serial.print(" | field_present=");
        Serial.print(NordicNfc.isFieldPresent() ? "yes" : "no");
        Serial.print(" | mode=");
        Serial.print(nfcModeName(NordicNfc.mode()));
        Serial.print(" | last_error=");
        Serial.print(NordicNfc.lastErrorMessage());
        Serial.print(" | driver_code=0x");
        Serial.println(NordicNfc.lastDriverErrorCode(), HEX);
    }

    delay(10);
}
