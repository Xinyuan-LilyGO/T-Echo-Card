/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-09-12 16:42:57
 * @LastEditTime: 2025-09-22 15:28:52
 * @License: GPL 3.0
 */
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "arduino_nordicnrf52_nfc_library.h"
#include <nrf.h>

static const char nfc_text[] = "T-Echo-Card NFC Text Record";
static const char nfc_language[] = "en";
static constexpr uint32_t heartbeat_interval_ms = 1000;
static uint32_t heartbeat_previous_ms = 0;
static uint32_t heartbeat_count = 0;
static uint8_t ndef_buffer[NordicNrf52NfcTag::Type2MaximumNdefSize];
static uint8_t configured_nfc_id[7];

static void printHex(const uint8_t *data, size_t length)
{
    for (size_t index = 0; index < length; ++index)
    {
        if (data[index] < 0x10)
        {
            Serial.print('0');
        }
        Serial.print(data[index], HEX);
        if ((index + 1) != length)
        {
            Serial.print(':');
        }
    }
}

static const char *modeName(NordicNrf52NfcTag::Mode mode)
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

static bool printNdefRecord(const NfcNdefRecordView &record, void *context)
{
    size_t *index = static_cast<size_t *>(context);
    ++(*index);
    Serial.print("NDEF record #");
    Serial.print(*index);
    Serial.print(" | tnf=");
    Serial.print(record.tnf);
    Serial.print(" | type=");
    Serial.write(record.type, record.type_length);
    Serial.print(" | id_length=");
    Serial.print(record.id_length);
    Serial.print(" | payload_length=");
    Serial.print(record.payload_length);
    Serial.print(" | message_begin=");
    Serial.print(record.message_begin ? "yes" : "no");
    Serial.print(" | message_end=");
    Serial.println(record.message_end ? "yes" : "no");
    return true;
}

static void printNfcConfiguration(const NfcNdefMessage &message)
{
    const uint32_t nfc_pin_setting =
        (NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) >> UICR_NFCPINS_PROTECT_Pos;

    Serial.println("NFC capability: NFC-A Listen/Tag only; Poller/Reader is not supported by NFCT.");
    Serial.print("UICR NFCPINS raw value: 0x");
    Serial.println(NRF_UICR->NFCPINS, HEX);
    Serial.print("NFC pins function: ");
    Serial.println(nfc_pin_setting == UICR_NFCPINS_PROTECT_NFC ? "NFC antenna" : "GPIO");
    Serial.print("NFC mode: ");
    Serial.println(modeName(NordicNfc.mode()));
    Serial.println("NFC technology: NFC-A / ISO 14443-3A / NFC Forum Type 2 Tag");
    Serial.println("NDEF access: read-only");
    uint8_t nfc_id[10];
    const size_t nfc_id_length = NordicNfc.getConfiguredNfcId(nfc_id, sizeof(nfc_id));
    Serial.print("NFCID1 length: ");
    Serial.println(nfc_id_length);
    Serial.print("NFCID1 value: ");
    printHex(nfc_id, nfc_id_length);
    Serial.println();
    Serial.print("NDEF message length: ");
    Serial.println(message.size());
    Serial.print("NDEF record count: ");
    Serial.println(message.recordCount());
    Serial.print("NDEF message bytes: ");
    printHex(message.data(), message.size());
    Serial.println();

    size_t record_index = 0;
    if (!NfcNdefParser::parse(message.data(),
                              message.size(),
                              printNdefRecord,
                              &record_index))
    {
        Serial.println("NDEF parse check: failed");
    }
    else
    {
        Serial.println("NDEF parse check: passed");
    }
}

void setup()
{
    Serial.begin(115200);

    /* 给 USB 串口短暂的枚举时间，但不因未打开串口而阻塞 NFC。 */
    const uint32_t serial_wait_start = millis();
    while (!Serial && (millis() - serial_wait_start < 3000))
    {
        delay(10);
    }

    Serial.println();
    Serial.println("T-Echo-Card NFC complete information test");
    Serial.println("Place an NFC phone or reader near the onboard NFC antenna.");

    NfcNdefMessage message(ndef_buffer, sizeof(ndef_buffer));
    if (!message.addText(nfc_text, nfc_language))
    {
        Serial.println("NDEF message generation failed.");
        return;
    }

    /* 使用芯片设备标识生成稳定的 7 字节随机类 NFCID1。 */
    configured_nfc_id[0] = 0x08;
    configured_nfc_id[1] = static_cast<uint8_t>(NRF_FICR->DEVICEID[0]);
    configured_nfc_id[2] = static_cast<uint8_t>(NRF_FICR->DEVICEID[0] >> 8);
    configured_nfc_id[3] = static_cast<uint8_t>(NRF_FICR->DEVICEID[0] >> 16);
    configured_nfc_id[4] = static_cast<uint8_t>(NRF_FICR->DEVICEID[0] >> 24);
    configured_nfc_id[5] = static_cast<uint8_t>(NRF_FICR->DEVICEID[1]);
    configured_nfc_id[6] = static_cast<uint8_t>(NRF_FICR->DEVICEID[1] >> 8);

    if (!NordicNfc.setNfcId(configured_nfc_id, sizeof(configured_nfc_id)) ||
        !NordicNfc.beginType2(message.data(), message.size()))
    {
        Serial.print("NFC startup failed: ");
        Serial.println(NordicNfc.lastErrorMessage());
        Serial.print("NFC driver error code: 0x");
        Serial.println(NordicNfc.lastDriverErrorCode(), HEX);

        if (NordicNfc.lastError() == NordicNrf52NfcTag::ErrorNfcPinsDisabled)
        {
            Serial.println("The UICR configures the NFC pins as GPIO.");
            Serial.println("Flash a bootloader that enables NFCT pins or erase UICR with a debug probe.");
        }
        return;
    }

    Serial.print("NFC tag started. Text: ");
    Serial.println(nfc_text);
    Serial.print("Text language: ");
    Serial.println(nfc_language);
    Serial.println("Text encoding: UTF-8");
    printNfcConfiguration(message);
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

    /* 每秒输出一次完整心跳，便于观察主循环和 NFC 状态。 */
    const uint32_t current_ms = millis();
    if (current_ms - heartbeat_previous_ms >= heartbeat_interval_ms)
    {
        heartbeat_previous_ms = current_ms;
        ++heartbeat_count;

        Serial.print("Heartbeat #");
        Serial.print(heartbeat_count);
        Serial.print(" | uptime_ms=");
        Serial.print(current_ms);
        Serial.print(" | nfc_running=");
        Serial.print(NordicNfc.isRunning() ? "yes" : "no");
        Serial.print(" | field_present=");
        Serial.print(NordicNfc.isFieldPresent() ? "yes" : "no");
        Serial.print(" | mode=");
        Serial.print(modeName(NordicNfc.mode()));
        Serial.print(" | last_error=");
        Serial.print(NordicNfc.lastErrorMessage());
        Serial.print(" | driver_code=0x");
        Serial.println(NordicNfc.lastDriverErrorCode(), HEX);
    }

    delay(10);
}
