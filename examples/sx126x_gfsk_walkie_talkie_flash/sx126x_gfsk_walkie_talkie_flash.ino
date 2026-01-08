/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-12-30 09:49:37
 * @LastEditTime: 2026-01-08 17:43:37
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
#include <sstream>

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

#define MAX_SX126X_TRANSMIT_HEADER_SIZE 30
#define MAX_SX126X_TRANSMIT_DATA_SIZE 170
#define MAX_SX126X_TRANSMIT_SIZE MAX_SX126X_TRANSMIT_HEADER_SIZE + MAX_SX126X_TRANSMIT_DATA_SIZE

enum class Audio_Operating_Status
{
    IDLE,
    RECORDING,
    ENCODE,
    DECODE,
    PLAY,
    SX1262_SEND,
    SX1262_RECEIVE,
};

enum class Sx1262_Rf_Switch_Status
{
    SEND,
    RECEIVE,
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

std::vector<uint8_t> Codec2_Send_Buffer;
std::vector<uint8_t> Codec2_Receive_Buffer;

uint32_t Codec2_Receive_Buffer_Size = 0;

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

bool Sx1262_Send_Flag = false;

uint32_t Codec2_Send_Buffer_Size = 0;

bool Flash_Sample_8khz_Int8_t_Audio_Exist_Flag = false;

Audio_Operating_Status Audio_Operation_Current_Status = Audio_Operating_Status::IDLE;

// QSPI
Adafruit_FlashTransport_QSPI flashTransport(ZD25WQ32C_SCLK, ZD25WQ32C_CS,
                                            ZD25WQ32C_IO0, ZD25WQ32C_IO1,
                                            ZD25WQ32C_IO2, ZD25WQ32C_IO3);

Adafruit_SPIFlash flash(&flashTransport);

auto IIS_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iis>(-1, SPEAKER_DATA, SPEAKER_WS_LRCK, SPEAKER_BCLK, -1);

auto SPI_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Spi>(SX1262_MOSI, SX1262_SCLK, SX1262_MISO, NRF_SPIM3, 0);

auto Sx1262 = std::make_unique<Cpp_Bus_Driver::Sx126x>(SPI_Bus, Cpp_Bus_Driver::Sx126x::Chip_Type::SX1262, SX1262_BUSY, SX1262_CS, SX1262_RST);

void Set_Sx1262_Rf_Switch(Sx1262_Rf_Switch_Status status)
{
    switch (status)
    {
    case Sx1262_Rf_Switch_Status::SEND:
        digitalWrite(SX1262_RF_VC1, HIGH); // send
        digitalWrite(SX1262_RF_VC2, LOW);
        break;
    case Sx1262_Rf_Switch_Status::RECEIVE:
        digitalWrite(SX1262_RF_VC1, LOW); // receive
        digitalWrite(SX1262_RF_VC2, HIGH);
        break;

    default:
        break;
    }
}

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

                    Codec2_Send_Buffer.clear();

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

                    Codec2_Send_Buffer_Size = Codec2_Send_Buffer.size();

                    Audio_Operation_Current_Status = Audio_Operating_Status::SX1262_SEND;
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

                    Codec2_Send_Buffer.insert(Codec2_Send_Buffer.end(), codec2_encode_buffer.get(), codec2_encode_buffer.get() + Codec2_Encode_Size);
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
            if (Codec2_Receive_Buffer.empty() == false)
            {
                if (Codec2_Receive_Buffer.size() / Codec2_Encode_Size > 0)
                {
                    // auto codec2_decode_buffer = std::make_unique<int16_t[]>(Sample_8khz_Int16_t_Size);

                    // 解码 (还原) - 得到8kHz数据
                    codec2_decode(Codec2_Status, reinterpret_cast<int16_t *>(Flash_Buffer->data), Codec2_Receive_Buffer.data());
                    Codec2_Receive_Buffer.erase(Codec2_Receive_Buffer.begin(), Codec2_Receive_Buffer.begin() + Codec2_Encode_Size);

                    Serial.printf("Audio_Operating_Status::DECODE progress: %.01f%%\n",
                                  (static_cast<float>(Codec2_Receive_Buffer_Size - Codec2_Receive_Buffer.size()) / static_cast<float>(Codec2_Receive_Buffer_Size)) * 100.0f);

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
                }
                else
                {
                    Codec2_Receive_Buffer.clear();
                }
            }
            else
            {
                Iis_Start_Transmit_Flag_Lock = false;

                for (size_t i = 0; i < MAX_IIS_TX_BUFFER_COUNT; i++)
                {
                    Iis_Tx_Buffer_Full_Flag[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] = false;
                }

                Serial.println("Audio_Operating_Status::DECODE finish");

                Flash_Sample_8khz_Int8_t_Audio_Exist_Flag = true;

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

bool Sx126x_Gfsk_Receive(uint8_t *data, uint32_t *length)
{
    if (digitalRead(SX1262_INT) == 1) // 接收完成中断
    {
        // 检查中断
        Cpp_Bus_Driver::Sx126x::Irq_Status is;
        // 判断中断正确性
        if (Sx1262->parse_irq_status(Sx1262->get_irq_flag(), is) == false)
        {
            printf("parse_irq_status fail\n");
            return false;
        }
        else
        {
            if (is.all_flag.tx_rx_timeout == true)
            {
                printf("receive timeout\n");
                Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TIMEOUT);
                return false;
            }
            else if (is.all_flag.crc_error == true)
            {
                printf("receive crc error\n");
                Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::CRC_ERROR);
                return false;
            }
            else
            {
                // 判断传输包正确性
                Cpp_Bus_Driver::Sx126x::Gfsk_Packet_Status gps;
                uint32_t packet_buffer = Sx1262->get_gfsk_packet_status();
                if (Sx1262->parse_gfsk_packet_status(packet_buffer, gps) == false)
                {
                    printf("parse_gfsk_packet_status fail\n");
                    return false;
                }
                else
                {
                    if (gps.abort_error_flag == true) // 中止错误
                    {
                        printf("receive abort_error_flag error\n");
                        return false;
                    }
                    else if (gps.length_error_flag == true) // 长度错误
                    {
                        printf("receive length_error_flag error\n");
                        return false;
                    }
                    else if (gps.crc_error_flag == true) // CRC校验错误
                    {
                        printf("receive crc_error_flag error\n");
                        return false;
                    }
                    else if (gps.address_error_flag == true) // 地址错误
                    {
                        printf("receive address_error_flag error\n");
                        return false;
                    }
                    else if (gps.sync_word_flag == true) // 同步错误
                    {
                        printf("receive sync_word_flag error\n");
                        return false;
                    }
                    else if (gps.preamble_error_flag == true) // 前导错误
                    {
                        printf("receive preamble_error_flag error\n");
                        return false;
                    }
                    else
                    {
                        uint8_t length_buffer = Sx1262->receive_data(data);
                        if (length_buffer == 0)
                        {
                            printf("Sx1262 receive fail (error assert: %d)\n", Sx1262->_assert);
                            return false;
                        }

                        Cpp_Bus_Driver::Sx126x::Packet_Metrics pm;
                        Sx1262->parse_gfsk_packet_metrics(packet_buffer, pm);

                        printf("Sx1262 receive rssi_average: %.01f rssi_sync: %.01f\n", pm.gfsk.rssi_average, pm.gfsk.rssi_sync);

                        *length = length_buffer;

                        // 接收完成中断清除需要成功接收到数据后才能清除
                        // 不能放在其他接收到错误数据的位置上
                        Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);

                        return true;
                    }
                }
            }
        }
    }

    return false;
}

void Sx1262_Task(void *parameter)
{
    Serial.println("Sx1262_Task start");

    Set_Sx1262_Rf_Switch(Sx1262_Rf_Switch_Status::RECEIVE);

    Sx1262->begin(5000000);
    Sx1262->config_gfsk_params(850.0, 200.0, Cpp_Bus_Driver::Sx126x::Gfsk_Bw::BW_467000HZ, 140, 22);
    Sx1262->clear_buffer();

    Sx1262->start_gfsk_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
    Sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE,
                             Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
                             Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE);
    Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);

    auto codec2_receive_buffer = std::make_unique<uint8_t[]>(MAX_SX126X_TRANSMIT_SIZE);
    uint32_t codec2_receive_buffer_size;
    uint32_t receive_packet_id = 0;
    uint32_t last_receive_total_size = 0;

    while (1)
    {
        switch (Audio_Operation_Current_Status)
        {
        case Audio_Operating_Status::SX1262_SEND:
        {

            uint32_t packet_id = 0;
            char header[MAX_SX126X_TRANSMIT_HEADER_SIZE];

            std::unique_ptr<uint8_t[]> packet = std::make_unique<uint8_t[]>(MAX_SX126X_TRANSMIT_SIZE);

            Set_Sx1262_Rf_Switch(Sx1262_Rf_Switch_Status::SEND);

            while (1)
            {
                printf("Sx1262 send start\n");

                uint32_t data_size = min(Codec2_Send_Buffer.size(), MAX_SX126X_TRANSMIT_DATA_SIZE);

                if (data_size == 0)
                {
                    printf("Audio_Operating_Status::SX1262_SEND finish\n");

                    Set_Sx1262_Rf_Switch(Sx1262_Rf_Switch_Status::RECEIVE);

                    // 还原接收模式
                    Sx1262->start_gfsk_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
                    Sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE,
                                             Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
                                             Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE);
                    Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);

                    Audio_Operation_Current_Status = Audio_Operating_Status::IDLE;
                    break;
                }
                else
                {
                    uint16_t timeout_count = 0;

                    // CODEC2_TRANSMIT:packetId:totalSize:
                    uint32_t header_len = snprintf(header, sizeof(header),
                                                   "CODEC2_TRANSMIT:%lu:%lu:", packet_id, Codec2_Send_Buffer_Size);

                    memcpy(packet.get(), header, header_len);
                    memcpy(packet.get() + header_len, Codec2_Send_Buffer.data(), data_size);

                    // 设置发送模式，发送完成后进入快速切换模式（FS模式）
                    Sx1262->start_gfsk_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::TX, 0, Cpp_Bus_Driver::Sx126x::Fallback_Mode::FS);
                    Sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE,
                                             Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
                                             Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE);
                    Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);

                    if (Sx1262->send_data(packet.get(), data_size + header_len) == true)
                    {
                        while (1) // 等待发送完成
                        {
                            if (digitalRead(SX1262_INT) == 1) // 发送完成标志
                            {
                                // 检查标志
                                Cpp_Bus_Driver::Sx126x::Irq_Status is;
                                if (Sx1262->parse_irq_status(Sx1262->get_irq_flag(), is) == false)
                                {
                                    printf("parse_irq_status fail\n");
                                }
                                else
                                {
                                    if (is.all_flag.tx_done == true) // 发送完成
                                    {
                                        printf("Sx1262 send success\n");
                                        Codec2_Send_Buffer.erase(Codec2_Send_Buffer.begin(), Codec2_Send_Buffer.begin() + data_size);
                                        packet_id++;
                                        break;
                                    }
                                }
                            }

                            timeout_count++;
                            if (timeout_count > 500) // 超时
                            {
                                printf("Sx1262 send timeout\n");
                                break;
                            }

                            delay(10);
                        }
                    }
                    else
                    {
                        printf("Sx1262 send fail\n");
                    }

                    // 大数据传输保证传输稳定
                    delay(100);
                }
            }
        }

        break;

        case Audio_Operating_Status::IDLE:
            if (Sx126x_Gfsk_Receive(codec2_receive_buffer.get(), &codec2_receive_buffer_size) == true)
            {
                // 将接收到的数据转为字符串
                std::string received_data(reinterpret_cast<char *>(codec2_receive_buffer.get()), codec2_receive_buffer_size);

                std::stringstream ss(received_data);
                std::string token;
                std::vector<std::string> parts;

                // 按 ':' 分割字符串
                while (std::getline(ss, token, ':'))
                {
                    parts.push_back(token);
                }

                // 检查格式
                if (parts.size() >= 4 && parts[0] == "CODEC2_TRANSMIT")
                {
                    uint32_t packet_id = std::stoul(parts[1]);
                    uint32_t total_size = std::stoul(parts[2]);

                    // parts[3] 包含音频数据
                    std::string &codec2_data_str = parts[3];
                    if (codec2_data_str.empty() == false)
                    {
                        // 获取音频数据
                        const uint8_t *codec2_data = reinterpret_cast<const uint8_t *>(codec2_data_str.data());
                        uint32_t codec2_data_length = codec2_data_str.size();

                        printf("received packet id: %u, total size: %u, codec2_data size: %u bytes\n", packet_id, total_size, codec2_data_length);

                        Codec2_Receive_Buffer_Size = 0;
                        Codec2_Receive_Buffer.clear();

                        if (Codec2_Receive_Buffer.size() < MAX_CODEC2_TRANSMIT_BUFFER_COUNT * MAX_SX126X_TRANSMIT_DATA_SIZE)
                        {
                            Codec2_Receive_Buffer.insert(Codec2_Receive_Buffer.end(), codec2_data, codec2_data + codec2_data_length);

                            Codec2_Receive_Buffer_Size += codec2_data_length;

                            Serial.printf("Audio_Operating_Status::SX1262_RECEIVE progress: %.01f%%\n",
                                          (static_cast<float>(Codec2_Receive_Buffer_Size) / static_cast<float>(total_size)) * 100.0f);
                        }
                        else
                        {
                            printf("Codec2_Receive_Buffer is full (Codec2_Receive_Buffer size: %d)\n", Codec2_Receive_Buffer.size());
                        }

                        receive_packet_id = packet_id;
                        last_receive_total_size = total_size;

                        Audio_Operation_Current_Status = Audio_Operating_Status::SX1262_RECEIVE;
                    }
                }
                else
                {
                    printf("codec2 receive format error\n");

                    // for (size_t i = 0; i < codec2_receive_buffer_size; i++)
                    // {
                    //     printf("error receive[%d]: %c\n", i, codec2_receive_buffer[i]);
                    // }
                }
            }

            break;

        case Audio_Operating_Status::SX1262_RECEIVE:
        {
            uint16_t timeout_count = 0;

            while (1)
            {
                if (Sx126x_Gfsk_Receive(codec2_receive_buffer.get(), &codec2_receive_buffer_size) == true)
                {
                    // 将接收到的数据转为字符串
                    std::string received_data(reinterpret_cast<char *>(codec2_receive_buffer.get()), codec2_receive_buffer_size);

                    std::stringstream ss(received_data);
                    std::string token;
                    std::vector<std::string> parts;

                    // 按 ':' 分割字符串
                    while (std::getline(ss, token, ':'))
                    {
                        parts.push_back(token);
                    }

                    // 检查格式
                    if (parts.size() >= 4 && parts[0] == "CODEC2_TRANSMIT")
                    {
                        uint32_t packet_id = std::stoul(parts[1]);
                        uint32_t total_size = std::stoul(parts[2]);

                        // parts[3] 包含音频数据
                        std::string &codec2_data_str = parts[3];
                        if (codec2_data_str.empty() == false)
                        {
                            // 获取音频数据
                            const uint8_t *codec2_data = reinterpret_cast<const uint8_t *>(codec2_data_str.data());
                            uint32_t codec2_data_length = codec2_data_str.size();

                            printf("received packet id: %u, total size: %u, codec2_data size: %u bytes\n", packet_id, total_size, codec2_data_length);

                            if (Codec2_Receive_Buffer.size() < MAX_CODEC2_TRANSMIT_BUFFER_COUNT * MAX_SX126X_TRANSMIT_DATA_SIZE)
                            {
                                Codec2_Receive_Buffer.insert(Codec2_Receive_Buffer.end(), codec2_data, codec2_data + codec2_data_length);

                                Codec2_Receive_Buffer_Size += codec2_data_length;

                                Serial.printf("Audio_Operating_Status::SX1262_RECEIVE progress: %.01f%%\n",
                                              (static_cast<float>(Codec2_Receive_Buffer_Size) / static_cast<float>(total_size)) * 100.0f);
                            }
                            else
                            {
                                printf("Codec2_Receive_Buffer is full (Codec2_Receive_Buffer size: %d)\n", Codec2_Receive_Buffer.size());
                            }

                            receive_packet_id++;
                            last_receive_total_size = total_size;

                            if (Codec2_Receive_Buffer_Size == total_size)
                            {
                                printf("Audio_Operating_Status::SX1262_RECEIVE finish\n");
                                printf("total receive bytes size: %d\n", Codec2_Receive_Buffer_Size);

                                flash.eraseChip();
                                flash.waitUntilReady();
                                printf("Waiting for flash to erase\n");
                                delay(MAX_RECORD_AUDIO_TIME_SECONDS * 1000);

                                Flash_Write_Sample_8khz_Int8_t_Audio_Index = FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET;
                                Flash_Read_Sample_8khz_Int8_t_Audio_Index = FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET;

                                Audio_Operation_Current_Status = Audio_Operating_Status::DECODE;
                                break;
                            }

                            timeout_count = 0;
                        }
                    }
                    else
                    {
                        printf("codec2 receive format error\n");

                        // for (size_t i = 0; i < codec2_receive_buffer_size; i++)
                        // {
                        //     printf("error receive[%d]: %c\n", i, codec2_receive_buffer[i]);
                        // }
                    }
                }

                timeout_count++;
                if (timeout_count > 500) // 超时
                {
                    printf("Audio_Operating_Status::SX1262_RECEIVE finish\n");
                    printf("Sx1262 receive timeout\n");
                    printf("receive packet size: %d\n", receive_packet_id);
                    printf("missing receive bytes size: %d\n", last_receive_total_size - Codec2_Receive_Buffer_Size);
                    printf("total receive bytes size: %d\n", Codec2_Receive_Buffer_Size);

                    flash.eraseChip();
                    flash.waitUntilReady();
                    printf("Waiting for flash to erase\n");
                    delay(MAX_RECORD_AUDIO_TIME_SECONDS * 1000);

                    Flash_Write_Sample_8khz_Int8_t_Audio_Index = FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET;
                    Flash_Read_Sample_8khz_Int8_t_Audio_Index = FLASH_SAMPLE_8KHZ_INT8_T_AUDIO_OFFSET;

                    Audio_Operation_Current_Status = Audio_Operating_Status::DECODE;
                    break;
                }

                delay(10);
            }
        }

        break;

        default:
            break;
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

    pinMode(SX1262_INT, INPUT);

    pinMode(SX1262_RF_VC1, OUTPUT);
    pinMode(SX1262_RF_VC2, OUTPUT);

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

    xTaskCreate(&Sx1262_Task, "Sx1262_Task", 1 * 1024, NULL, 3, NULL);
}

void loop()
{
    if ((digitalRead(nRF52840_BOOT) == LOW) && (Audio_Operation_Current_Status == Audio_Operating_Status::IDLE))
    {
        delay(300);

        Serial.println("audio operating start");

        flash.eraseChip();
        flash.waitUntilReady();

        Pdm_Stream.clear();

        Flash_Write_Sample_16khz_Int8_t_Audio_Index = 0;
        Flash_Read_Sample_16khz_Int8_t_Audio_Index = 0;

        Audio_Operation_Current_Status = Audio_Operating_Status::RECORDING;
    }

    if ((digitalRead(KEY_1) == LOW) &&
        (Audio_Operation_Current_Status == Audio_Operating_Status::IDLE) &&
        (Flash_Sample_8khz_Int8_t_Audio_Exist_Flag == true))
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