/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-12-30 09:49:37
 * @LastEditTime: 2026-01-07 17:07:50
 * @License: GPL 3.0
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TinyUSB.h>
#include "t_echo_card_config.h"
#include "cpp_bus_driver_library.h"
#include "PDM.h"
#include "codec2.h"
#include <vector>
#include <malloc.h>
#include "Adafruit_SPIFlash.h"

#define PDM_SAMPLE_RATE 16000
#define IIS_SAMPLE_RATE 8000

#define MAX_IIS_DATA_TRANSMIT_SIZE 1024

#define MAX_IIS_TX_BUFFER_COUNT 2

#define MAX_PDM_DATA_TRANSMIT_SIZE 512
#define MAX_PDM_DATA_TRANSMIT_BLOCK_COUNT 16

#define MAX_CODEC2_ENCODE_BUFFER_COUNT 3
#define MAX_CODEC2_DECODE_BUFFER_COUNT 10

#define MAX_CODEC2_TRANSMIT_BUFFER_COUNT 30

#define MAX_RECORD_AUDIO_TIME_SECONDS 5

#define MAX_FLASH_BUFFER_BLOCK_SIZE 4 * 1024

#define MAX_FLASH_SAMPLE_16KHZ_INT8_T_AUDIO_SIZE (PDM_SAMPLE_RATE * 16 * 1 * MAX_RECORD_AUDIO_TIME_SECONDS) / 8

#define FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET MAX_FLASH_SAMPLE_16KHZ_INT8_T_AUDIO_SIZE

enum class Audio_Operating_Status
{
    IDLE,
    RECORDING,
    ENCODE,
    DECODE,
    PLAY,
};

SPIFlash_Device_t ZD25WQ32C =
    {
        total_size : (1UL << 22), /* 4 MiB */
        start_up_time_us : 12000,
        manufacturer_id : 0xBA,
        memory_type : 0x60,
        capacity : 0x16,
        max_clock_speed_mhz : 104,
        quad_enable_bit_mask : 0x02,
        has_sector_protection : false,
        supports_fast_read : true,
        supports_qspi : true,
        supports_qspi_writes : true,
        write_status_register_split : false,
        single_status_byte : false,
        is_fram : false,
    };

struct Flash_Buffer_Struct
{
    alignas(4) uint8_t data[MAX_FLASH_BUFFER_BLOCK_SIZE];
};

// 4 byte aligned buffer has best result with nRF QSPI
// uint8_t Flash_Buffer_Write[MAX_FLASH_BUFFER_BLOCK_SIZE] __attribute__((aligned(4)));
// uint8_t Flash_Buffer_Read[MAX_FLASH_BUFFER_BLOCK_SIZE] __attribute__((aligned(4)));
auto Flash_Buffer = std::make_unique<Flash_Buffer_Struct>();

CODEC2 *Codec2_Status;

std::vector<int16_t> Pdm_Stream;

int32_t Sample_8khz_Int16_t_Size;
int32_t Sample_8khz_Int8_t_Size;
int32_t Sample_16khz_Int8_t_Size;
int32_t Codec2_Encode_Size;

uint8_t Current_Iis_Tx_Buffer_Count = 0;
bool Iis_Tx_Buffer_Full_Flag[MAX_IIS_TX_BUFFER_COUNT] = {false};
// int32_t Iis_Tx_Buffer_Count[MAX_IIS_TX_BUFFER_COUNT] = {0};

// std::vector<std::unique_ptr<int16_t[]>> Codec2_Encode_Buffer; // 存储从 PDM 读取的原始 PCM 数据
// std::vector<std::unique_ptr<int16_t[]>> Codec2_Decode_Buffer;

// std::vector<std::unique_ptr<uint8_t[]>> Codec2_Transmit_Buffer;

std::vector<uint8_t> Codec2_Transmit_Buffer;

int32_t Iis_Data_Transmit_Size = 0;

bool Codec2_Init_Flag = false;

size_t Cycle_Time = 0;

uint32_t Iis_Tx_Buffer[MAX_IIS_TX_BUFFER_COUNT][MAX_IIS_DATA_TRANSMIT_SIZE] = {0};

uint32_t Flash_Write_Sample_16khz_Int8_t_Audio_Index = 0;
uint32_t Flash_Read_Sample_16khz_Int8_t_Audio_Index = 0;

uint32_t Flash_Write_Sample_8khz_Int8_t_Audio_Index = 0;
uint32_t Flash_Read_Sample_8khz_Int8_t_Audio_Index = 0;

bool Iis_Start_Transmit_Flag = false;
bool Iis_Start_Transmit_Flag_Lock = true;

Audio_Operating_Status Audio_Operation_Current_Status = Audio_Operating_Status::IDLE;

// QSPI
Adafruit_FlashTransport_QSPI flashTransport(ZD25WQ32C_SCLK, ZD25WQ32C_CS,
                                            ZD25WQ32C_IO0, ZD25WQ32C_IO1,
                                            ZD25WQ32C_IO2, ZD25WQ32C_IO3);

Adafruit_SPIFlash flash(&flashTransport);

auto IIS_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iis>(-1, SPEAKER_DATA, SPEAKER_WS_LRCK, SPEAKER_BCLK, -1);

void Iis_Data_Convert(const void *input_data, void *out_buffer, size_t input_data_start_index, size_t byte)
{
    const uint8_t *input_ptr = (const uint8_t *)input_data + input_data_start_index;
    uint8_t *out_ptr = (uint8_t *)out_buffer;

    memcpy(out_ptr, input_ptr, byte);
}

// 降采样函数：16kHz -> 8kHz，使用平均法
void downsample_16k_to_8k(const int16_t *input_buffer_16k, int16_t *output_buffer_8k, int32_t output_samples_8k)
{
    // 2:1降采样，每2个16kHz样本平均为1个8kHz样本
    for (int i = 0; i < output_samples_8k; i++)
    {
        int32_t sum = input_buffer_16k[i * 2] + input_buffer_16k[i * 2 + 1];
        output_buffer_8k[i] = static_cast<int16_t>(sum / 2);
    }
}

// 升采样函数：8kHz -> 16kHz，使用线性插值
void upsample_8k_to_16k(const int16_t *input_buffer_8k, int16_t *output_buffer_16k, int32_t input_samples_8k)
{
    // 1:2升采样，使用线性插值
    for (int i = 0; i < input_samples_8k - 1; i++)
    {
        // 第一个点：原始8kHz样本
        output_buffer_16k[i * 2] = input_buffer_8k[i];

        // 第二个点：线性插值（当前样本和下一个样本的平均）
        output_buffer_16k[i * 2 + 1] = static_cast<int16_t>(
            (static_cast<int32_t>(input_buffer_8k[i]) + static_cast<int32_t>(input_buffer_8k[i + 1])) / 2);
    }

    // 处理最后一个采样点
    output_buffer_16k[(input_samples_8k - 1) * 2] = input_buffer_8k[input_samples_8k - 1];
    output_buffer_16k[(input_samples_8k - 1) * 2 + 1] = input_buffer_8k[input_samples_8k - 1];
}

void Pdm_Task(void *parameter)
{
    Serial.println("Pdm_Task start");

    PDM.setPins(MICROPHONE_DATA, MICROPHONE_SCLK, -1);
    PDM.onReceive([]()
                  {
                    if (PDM.available() >= MAX_PDM_DATA_TRANSMIT_SIZE)
                    {
                        auto pcm_buffer = std::make_unique<int16_t[]>(MAX_PDM_DATA_TRANSMIT_SIZE / sizeof(int16_t));

                        int32_t buffer_length = PDM.read(pcm_buffer.get(), MAX_PDM_DATA_TRANSMIT_SIZE);

                        if (Audio_Operation_Current_Status == Audio_Operating_Status::RECORDING)
                         {
                            if (Pdm_Stream.size() < (MAX_PDM_DATA_TRANSMIT_SIZE * MAX_PDM_DATA_TRANSMIT_BLOCK_COUNT))
                            {
                                Pdm_Stream.insert(Pdm_Stream.end(), pcm_buffer.get(), pcm_buffer.get() + buffer_length / sizeof(int16_t));
                            }
                        }
                    } });

    while (PDM.begin(1, PDM_SAMPLE_RATE) == false)
    {
        printf("PDM.begin fail\n");
        delay(1000);
    }

    PDM.setGain(100);

    while (1)
    {
        if (Audio_Operation_Current_Status == Audio_Operating_Status::RECORDING)
        {
            if (Pdm_Stream.size() >= MAX_FLASH_BUFFER_BLOCK_SIZE)
            {
                uint32_t buffer_size = min(MAX_FLASH_SAMPLE_16KHZ_INT8_T_AUDIO_SIZE - Flash_Write_Sample_16khz_Int8_t_Audio_Index,
                                           MAX_FLASH_BUFFER_BLOCK_SIZE);

                if (buffer_size == 0)
                {
                    Serial.println("Audio_Operating_Status::RECORDING finish");

                    Audio_Operation_Current_Status = Audio_Operating_Status::ENCODE;
                }
                else
                {
                    memcpy(Flash_Buffer->data, Pdm_Stream.data(), buffer_size);
                    Pdm_Stream.erase(Pdm_Stream.begin(), Pdm_Stream.begin() + (buffer_size / sizeof(int16_t)));

                    if (flash.writeBuffer(Flash_Write_Sample_16khz_Int8_t_Audio_Index, Flash_Buffer->data, buffer_size) != buffer_size)
                    {
                        Serial.printf("flash.writeBuffer fail\n");
                        Audio_Operation_Current_Status = Audio_Operating_Status::IDLE;
                    }

                    Flash_Write_Sample_16khz_Int8_t_Audio_Index += buffer_size;

                    Serial.printf("Audio_Operating_Status::RECORDING progress: %.01f%%\n",
                                  (static_cast<float>(Flash_Write_Sample_16khz_Int8_t_Audio_Index) / static_cast<float>(MAX_FLASH_SAMPLE_16KHZ_INT8_T_AUDIO_SIZE)) *
                                      100.0f);
                }
            }
        }

        delay(10);
    }
}

void Codec2_Encode_Task(void *parameter)
{
    Serial.println("Codec2_Encode_Task start");
    Codec2_Status = codec2_create(CODEC2_MODE_3200);

    while (!Codec2_Status)
    {
        printf("codec2_create fail\n");
        delay(1000);
    }

    Sample_8khz_Int16_t_Size = codec2_samples_per_frame(Codec2_Status);   // 8kHz下每帧的int16_t数据
    Sample_8khz_Int8_t_Size = Sample_8khz_Int16_t_Size * sizeof(int16_t); // 8kHz下的int8_t数据
    Sample_16khz_Int8_t_Size = Sample_8khz_Int8_t_Size * 2;               // 16kHz下的int8_t数据

    int32_t nbit = codec2_bits_per_frame(Codec2_Status);
    // 经过Codec2压缩后的数据量
    Codec2_Encode_Size = (nbit + 7) / 8;

    printf("Sample_8khz_Int8_t_Size: %d\n", Sample_8khz_Int8_t_Size);
    printf("Sample_16khz_Int8_t_Size: %d\n", Sample_16khz_Int8_t_Size);
    printf("Codec2_Encode_Size: %d\n", Codec2_Encode_Size);

    Codec2_Init_Flag = true;

    while (1)
    {
        if (Audio_Operation_Current_Status == Audio_Operating_Status::ENCODE)
        {
            if (Flash_Write_Sample_16khz_Int8_t_Audio_Index == MAX_FLASH_SAMPLE_16KHZ_INT8_T_AUDIO_SIZE)
            {
                if (Flash_Write_Sample_16khz_Int8_t_Audio_Index - Flash_Read_Sample_16khz_Int8_t_Audio_Index < Sample_16khz_Int8_t_Size)
                {
                    Serial.println("Audio_Operating_Status::ENCODE finish");

                    Audio_Operation_Current_Status = Audio_Operating_Status::DECODE;
                }
                else
                {
                    if (flash.readBuffer(Flash_Read_Sample_16khz_Int8_t_Audio_Index, Flash_Buffer->data, Sample_16khz_Int8_t_Size) != Sample_16khz_Int8_t_Size)
                    {
                        Serial.printf("flash.readBuffer fail\n");
                        Audio_Operation_Current_Status = Audio_Operating_Status::IDLE;
                    }

                    Flash_Read_Sample_16khz_Int8_t_Audio_Index += Sample_16khz_Int8_t_Size;

                    Serial.printf("Audio_Operating_Status::ENCODE progress: %.01f%%\n",
                                  (static_cast<float>(Flash_Read_Sample_16khz_Int8_t_Audio_Index) / static_cast<float>(Flash_Write_Sample_16khz_Int8_t_Audio_Index)) *
                                      100.0f);

                    // 降采样：从16kHz降到8kHz
                    auto downsampled_buffer = std::make_unique<int16_t[]>(Sample_8khz_Int16_t_Size);

                    // 调用降采样函数
                    downsample_16k_to_8k(reinterpret_cast<int16_t *>(Flash_Buffer->data), downsampled_buffer.get(), Sample_8khz_Int16_t_Size);

                    auto codec2_encode_buffer = std::make_unique<uint8_t[]>(Codec2_Encode_Size);

                    // 编码 (压缩) - 使用8kHz数据
                    codec2_encode(Codec2_Status, codec2_encode_buffer.get(), downsampled_buffer.get());

                    Codec2_Transmit_Buffer.insert(Codec2_Transmit_Buffer.end(), codec2_encode_buffer.get(), codec2_encode_buffer.get() + Codec2_Encode_Size);
                }
            }
        }

        delay(10);
    }
}

void Codec2_Decode_Task(void *parameter)
{
    Serial.println("Codec2_Decode_Task start");

    while (1)
    {
        if (Audio_Operation_Current_Status == Audio_Operating_Status::DECODE)
        {
            if (Codec2_Transmit_Buffer.empty() == false)
            {
                // auto codec2_decode_buffer = std::make_unique<int16_t[]>(Sample_8khz_Int16_t_Size);

                // 解码 (还原) - 得到8kHz数据
                codec2_decode(Codec2_Status, reinterpret_cast<int16_t *>(Flash_Buffer->data), Codec2_Transmit_Buffer.data());
                Codec2_Transmit_Buffer.erase(Codec2_Transmit_Buffer.begin(), Codec2_Transmit_Buffer.begin() + Codec2_Encode_Size);

                // // 升采样：从8kHz升到16kHz
                // auto upsampled_buffer = std::make_unique<int16_t[]>(Sample_16khz_Int8_t_Size / sizeof(int16_t));

                // 调用升采样函数
                // upsample_8k_to_16k(codec2_decode_buffer.get(), upsampled_buffer.get(), Sample_8khz_Int16_t_Size);
                // upsample_8k_to_16k(downsampled_buffer.get(), upsampled_buffer.get(), Sample_8khz_Int16_t_Size);

                // printf("codec2_encode_decode finish\n");

                if (flash.writeBuffer(Flash_Write_Sample_8khz_Int8_t_Audio_Index, Flash_Buffer->data, Sample_8khz_Int8_t_Size) != Sample_8khz_Int8_t_Size)
                {
                    Serial.printf("flash.writeBuffer fail\n");
                    Audio_Operation_Current_Status = Audio_Operating_Status::IDLE;
                }

                Flash_Write_Sample_8khz_Int8_t_Audio_Index += Sample_8khz_Int8_t_Size;

                Serial.printf("Audio_Operating_Status::DECODE progress: %.01f%%\n",
                              (static_cast<float>(Flash_Write_Sample_8khz_Int8_t_Audio_Index - FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET) /
                               static_cast<float>(MAX_FLASH_SAMPLE_16KHZ_INT8_T_AUDIO_SIZE / 2)) *
                                  100.0f);
            }
            else
            {
                Iis_Start_Transmit_Flag_Lock = false;

                Serial.println("Audio_Operating_Status::DECODE finish");

                Audio_Operation_Current_Status = Audio_Operating_Status::PLAY;
            }
        }

        delay(10);
    }
}

void Iis_Tx_Handle(void *parameter)
{
    Serial.println("Iis_Tx_Handle start");

    while (1)
    {
        if (Audio_Operation_Current_Status == Audio_Operating_Status::PLAY)
        {
            for (uint8_t i = 0; i < MAX_IIS_TX_BUFFER_COUNT; i++)
            {
                if (Iis_Tx_Buffer_Full_Flag[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] == false)
                {
                    uint32_t buffer_size = min(Flash_Write_Sample_8khz_Int8_t_Audio_Index - Flash_Read_Sample_8khz_Int8_t_Audio_Index,
                                               MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t));

                    if (buffer_size == 0)
                    {
                        IIS_Bus->stop_transmit();

                        Serial.println("Audio_Operating_Status::PLAY finish");

                        Audio_Operation_Current_Status = Audio_Operating_Status::IDLE;
                    }
                    else
                    {
                        if (flash.readBuffer(Flash_Read_Sample_8khz_Int8_t_Audio_Index, Flash_Buffer->data, buffer_size) != buffer_size)
                        {
                            Serial.printf("flash.readBuffer fail\n");
                            Audio_Operation_Current_Status = Audio_Operating_Status::IDLE;
                        }

                        Flash_Read_Sample_8khz_Int8_t_Audio_Index += buffer_size;

                        Serial.printf("Audio_Operating_Status::PLAY progress: %.01f%%\n",
                                      (static_cast<float>(Flash_Read_Sample_8khz_Int8_t_Audio_Index - FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET) /
                                       static_cast<float>(MAX_FLASH_SAMPLE_16KHZ_INT8_T_AUDIO_SIZE / 2)) *
                                          100.0f);

                        Iis_Data_Convert(Flash_Buffer->data, Iis_Tx_Buffer[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT],
                                         0, buffer_size);

                        // Serial.printf("Iis_Tx_Buffer_Count[%d] finish\n", (Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT);

                        Iis_Tx_Buffer_Full_Flag[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] = true;

                        if (Iis_Start_Transmit_Flag_Lock == false)
                        {
                            Iis_Start_Transmit_Flag = true;

                            Iis_Start_Transmit_Flag_Lock = true;
                        }

                        break;
                    }
                }
            }
        }

        delay(10);
    }
}

void setup()
{
    Serial.begin(115200);

    uint8_t serial_init_count = 0;
    while (!Serial)
    {
        delay(100); // wait for native usb
        serial_init_count++;
        if (serial_init_count > 30)
        {
            break;
        }
    }

    Serial.println("Ciallo");

    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);
    pinMode(SPEAKER_EN, OUTPUT);
    digitalWrite(SPEAKER_EN, HIGH);
    pinMode(SPEAKER_EN_2, OUTPUT);
    digitalWrite(SPEAKER_EN_2, HIGH);

    pinMode(nRF52840_BOOT, INPUT_PULLUP);

    pinMode(KEY_1, INPUT);

    delay(500);

    while (flash.begin(&ZD25WQ32C) == false)
    {
        Serial.println("Flash initialization failed");
        delay(1000);
    }
    Serial.println("Flash initialization successful");

    // QSPI
    flashTransport.setClockSpeed(32000000UL, 0);

    IIS_Bus->begin(nrf_i2s_ratio_t::NRF_I2S_RATIO_128X, IIS_SAMPLE_RATE, nrf_i2s_swidth_t::NRF_I2S_SWIDTH_16BIT, nrf_i2s_channels_t::NRF_I2S_CHANNELS_LEFT);

    xTaskCreate(&Pdm_Task, "Pdm_Task", 1 * 1024, NULL, 3, NULL);

    xTaskCreate(&Codec2_Encode_Task, "Codec2_Encode_Task", 10 * 1024, NULL, 3, NULL);
    xTaskCreate(&Codec2_Decode_Task, "Codec2_Decode_Task", 10 * 1024, NULL, 3, NULL);

    xTaskCreate(&Iis_Tx_Handle, "Iis_Tx_Handle", 1 * 1024, NULL, 5, NULL);
}

void loop()
{
    if (digitalRead(nRF52840_BOOT) == LOW && Audio_Operation_Current_Status == Audio_Operating_Status::IDLE)
    {
        delay(300);

        Serial.println("audio operating start");

        flash.eraseChip();
        flash.waitUntilReady();

        Pdm_Stream.clear();

        Flash_Write_Sample_16khz_Int8_t_Audio_Index = 0;
        Flash_Read_Sample_16khz_Int8_t_Audio_Index = 0;

        Codec2_Transmit_Buffer.clear();

        Flash_Write_Sample_8khz_Int8_t_Audio_Index = FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET;
        Flash_Read_Sample_8khz_Int8_t_Audio_Index = FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET;

        for (size_t i = 0; i < MAX_IIS_TX_BUFFER_COUNT; i++)
        {
            Iis_Tx_Buffer_Full_Flag[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] = false;
        }

        Audio_Operation_Current_Status = Audio_Operating_Status::RECORDING;
    }

    if (digitalRead(KEY_1) == LOW && Audio_Operation_Current_Status == Audio_Operating_Status::IDLE)
    {
        delay(300);

        Serial.println("audio replay start");

        Flash_Read_Sample_8khz_Int8_t_Audio_Index = FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET;

        for (size_t i = 0; i < MAX_IIS_TX_BUFFER_COUNT; i++)
        {
            Iis_Tx_Buffer_Full_Flag[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] = false;
        }

        Iis_Start_Transmit_Flag_Lock = false;

        Audio_Operation_Current_Status = Audio_Operating_Status::PLAY;
    }

    if (Audio_Operation_Current_Status == Audio_Operating_Status::PLAY)
    {
        if (Iis_Start_Transmit_Flag == true)
        {
            if (IIS_Bus->start_transmit(Iis_Tx_Buffer[Current_Iis_Tx_Buffer_Count], nullptr, MAX_IIS_DATA_TRANSMIT_SIZE) == true)
            {
                Iis_Tx_Buffer_Full_Flag[Current_Iis_Tx_Buffer_Count] = false;
                Current_Iis_Tx_Buffer_Count = (Current_Iis_Tx_Buffer_Count + 1) % MAX_IIS_TX_BUFFER_COUNT;
                printf("iis start_transmit success\n");
            }
            else
            {
                printf("iis start_transmit fail\n");

                Audio_Operation_Current_Status = Audio_Operating_Status::IDLE;
            }

            Iis_Start_Transmit_Flag = false;
        }

        if (IIS_Bus->get_write_event_flag() == true)
        {
            if (Iis_Tx_Buffer_Full_Flag[Current_Iis_Tx_Buffer_Count] == true)
            {
                IIS_Bus->set_next_write_data(Iis_Tx_Buffer[Current_Iis_Tx_Buffer_Count]);
                Iis_Tx_Buffer_Full_Flag[Current_Iis_Tx_Buffer_Count] = false;

                Current_Iis_Tx_Buffer_Count = (Current_Iis_Tx_Buffer_Count + 1) % MAX_IIS_TX_BUFFER_COUNT;
            }
        }
    }

    if (millis() > Cycle_Time)
    {
        printf("Pdm_Stream size: %d\n", Pdm_Stream.size());
        printf("Iis_Tx_Buffer: %d\n", static_cast<int16_t>(Iis_Tx_Buffer[Current_Iis_Tx_Buffer_Count][0] >> 16));
        printf("Audio_Operation_Current_Status: %d\n", Audio_Operation_Current_Status);

        Cycle_Time = millis() + 1000;
    }

    delay(10);
}