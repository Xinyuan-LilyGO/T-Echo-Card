/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-12-30 09:49:37
 * @LastEditTime: 2026-01-06 15:53:42
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

#define PDM_SAMPLE_RATE 16000
#define IIS_SAMPLE_RATE 8000

#define MAX_IIS_DATA_TRANSMIT_COUNT 10
#define MAX_IIS_DATA_TRANSMIT_SIZE 80 * MAX_IIS_DATA_TRANSMIT_COUNT

#define MAX_IIS_TX_BUFFER_COUNT 3

#define MAX_PDM_DATA_TRANSMIT_SIZE 512
#define MAX_PDM_DATA_TRANSMIT_BLOCK_COUNT 3

#define MAX_CODEC2_ENCODE_BUFFER_COUNT 3
#define MAX_CODEC2_DECODE_BUFFER_COUNT 10

#define MAX_CODEC2_TRANSMIT_BUFFER_COUNT 5

#define MAX_SX126X_TRANSMIT_BUFFER_SIZE 200

enum class Sx1262_Rf_Switch_Status
{
    SEND,
    RECEIVE,
};

CODEC2 *Codec2_Status;

std::vector<int16_t> Pdm_Stream;

int32_t Sample_8khz_Int16_t_Size;
int32_t Sample_8khz_Int8_t_Size;
int32_t Sample_16khz_Int8_t_Size;
int32_t Codec2_Encode_Size;

uint8_t Current_Iis_Tx_Buffer_Count = 0;
bool Iis_Tx_Buffer_Full_Flag[MAX_IIS_TX_BUFFER_COUNT] = {false};
int32_t Iis_Tx_Buffer_Count[MAX_IIS_TX_BUFFER_COUNT] = {0};

bool Sx1262_Send_Flag = false;

std::vector<std::unique_ptr<int16_t[]>> Codec2_Encode_Buffer; // 存储从 PDM 读取的原始 PCM 数据
std::vector<std::unique_ptr<int16_t[]>> Codec2_Decode_Buffer;

std::vector<uint8_t> Codec2_Send_Buffer;
std::vector<uint8_t> Codec2_Receive_Buffer;

int32_t Iis_Data_Transmit_Size = 0;

bool Codec2_Init_Flag = false;

size_t Cycle_Time = 0;

uint32_t Iis_Tx_Buffer[MAX_IIS_TX_BUFFER_COUNT][MAX_IIS_DATA_TRANSMIT_SIZE] = {0};

auto SPI_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Spi>(SX1262_MOSI, SX1262_SCLK, SX1262_MISO, NRF_SPIM3, 0);
auto IIS_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iis>(-1, SPEAKER_DATA, SPEAKER_WS_LRCK, SPEAKER_BCLK, -1);

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

                        if (Sx1262_Send_Flag == true)
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

    PDM.setGain(60);

    while (1)
    {
        if (Sx1262_Send_Flag == true)
        {
            if (Codec2_Encode_Buffer.size() < MAX_CODEC2_ENCODE_BUFFER_COUNT)
            {
                if (Pdm_Stream.size() >= Sample_16khz_Int8_t_Size / sizeof(int16_t))
                {
                    auto pcm_buffer = std::make_unique<int16_t[]>(Sample_16khz_Int8_t_Size / sizeof(int16_t));

                    memcpy(pcm_buffer.get(), Pdm_Stream.data(), Sample_16khz_Int8_t_Size);

                    Pdm_Stream.erase(Pdm_Stream.begin(), Pdm_Stream.begin() + (Sample_16khz_Int8_t_Size / sizeof(int16_t)));

                    Codec2_Encode_Buffer.push_back(std::move(pcm_buffer));
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

    // 在16khz CODEC2_MODE_3200下Iis_Data_Transmit_Size为160个32位数据
    // Iis_Data_Transmit_Size = Sample_16khz_Int8_t_Size / sizeof(uint32_t);
    // 在8khz CODEC2_MODE_3200下Iis_Data_Transmit_Size为80个32位数据
    Iis_Data_Transmit_Size = Sample_8khz_Int8_t_Size / sizeof(uint32_t);

    printf("Sample_8khz_Int8_t_Size: %d\n", Sample_8khz_Int8_t_Size);
    printf("Sample_16khz_Int8_t_Size: %d\n", Sample_16khz_Int8_t_Size);
    printf("Codec2_Encode_Size: %d\n", Codec2_Encode_Size);
    printf("Iis_Data_Transmit_Size: %d\n", Iis_Data_Transmit_Size);

    Codec2_Init_Flag = true;

    while (1)
    {
        if (Sx1262_Send_Flag == true)
        {
            if (Codec2_Send_Buffer.size() < MAX_CODEC2_TRANSMIT_BUFFER_COUNT * MAX_SX126X_TRANSMIT_BUFFER_SIZE)
            {
                if (Codec2_Encode_Buffer.empty() == false)
                {
                    // 降采样：从16kHz降到8kHz
                    auto downsampled_buffer = std::make_unique<int16_t[]>(Sample_8khz_Int16_t_Size);

                    // 调用降采样函数
                    downsample_16k_to_8k(Codec2_Encode_Buffer.front().get(), downsampled_buffer.get(), Sample_8khz_Int16_t_Size);
                    Codec2_Encode_Buffer.erase(Codec2_Encode_Buffer.begin());

                    auto codec2_encode_buffer = std::make_unique<uint8_t[]>(Codec2_Encode_Size);

                    // 编码 (压缩) - 使用8kHz数据
                    codec2_encode(Codec2_Status, codec2_encode_buffer.get(), downsampled_buffer.get());

                    Codec2_Send_Buffer.insert(Codec2_Send_Buffer.end(), codec2_encode_buffer.get(), codec2_encode_buffer.get() + Codec2_Encode_Size);

                    // Codec2_Decode_Buffer.push_back(std::move(downsampled_buffer));
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
        if (Codec2_Decode_Buffer.size() < MAX_CODEC2_DECODE_BUFFER_COUNT)
        {
            if (Codec2_Receive_Buffer.size() >= Codec2_Encode_Size)
            {
                auto codec2_decode_buffer = std::make_unique<int16_t[]>(Sample_8khz_Int16_t_Size);

                // 解码 (还原) - 得到8kHz数据
                codec2_decode(Codec2_Status, codec2_decode_buffer.get(), Codec2_Receive_Buffer.data());
                Codec2_Receive_Buffer.erase(Codec2_Receive_Buffer.begin(), Codec2_Receive_Buffer.begin() + Codec2_Encode_Size);

                // // 升采样：从8kHz升到16kHz
                // auto upsampled_buffer = std::make_unique<int16_t[]>(Sample_16khz_Int8_t_Size / sizeof(int16_t));

                // 调用升采样函数
                // upsample_8k_to_16k(codec2_decode_buffer.get(), upsampled_buffer.get(), Sample_8khz_Int16_t_Size);
                // upsample_8k_to_16k(downsampled_buffer.get(), upsampled_buffer.get(), Sample_8khz_Int16_t_Size);

                // printf("codec2_encode_decode finish\n");

                // Iis_Stream.insert(Iis_Stream.end(), downsampled_buffer.get(), downsampled_buffer.get() + Sample_8khz_Int16_t_Size);
                // Iis_Stream.insert(Iis_Stream.end(), codec2_decode_buffer.get(), codec2_decode_buffer.get() + Sample_8khz_Int16_t_Size);
                Codec2_Decode_Buffer.push_back(std::move(codec2_decode_buffer));
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
        if (Codec2_Decode_Buffer.empty() == false)
        {
            for (uint8_t i = 0; i < MAX_IIS_TX_BUFFER_COUNT; i++)
            {
                if (Iis_Tx_Buffer_Full_Flag[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] == false)
                {
                    if (Iis_Tx_Buffer_Count[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] < Iis_Data_Transmit_Size * MAX_IIS_DATA_TRANSMIT_COUNT)
                    {
                        Iis_Data_Convert(Codec2_Decode_Buffer.front().get(),
                                         &Iis_Tx_Buffer[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT][Iis_Tx_Buffer_Count[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT]],
                                         0, Sample_8khz_Int8_t_Size);

                        Serial.printf("Iis_Tx_Buffer_Count[%d]: %d\n",
                                      (Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT, Iis_Tx_Buffer_Count[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT]);

                        Codec2_Decode_Buffer.erase(Codec2_Decode_Buffer.begin());

                        Iis_Tx_Buffer_Count[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] += (Sample_8khz_Int8_t_Size / sizeof(uint32_t));
                    }
                    else
                    {
                        Iis_Tx_Buffer_Full_Flag[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] = true;

                        Iis_Tx_Buffer_Count[(Current_Iis_Tx_Buffer_Count + i) % MAX_IIS_TX_BUFFER_COUNT] = 0;
                    }

                    break;
                }
            }
        }

        delay(10);
    }
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

    while (1)
    {
        if (Sx1262_Send_Flag == true)
        {
            if (Codec2_Send_Buffer.size() >= MAX_SX126X_TRANSMIT_BUFFER_SIZE)
            {
                // 设置发送模式，发送完成后进入快速切换模式（FS模式）
                Sx1262->start_gfsk_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::TX, 0, Cpp_Bus_Driver::Sx126x::Fallback_Mode::FS);
                Sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE,
                                         Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
                                         Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE);
                Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);

                printf("Sx1262 send start\n");
                uint16_t timeout_count = 0;
                Set_Sx1262_Rf_Switch(Sx1262_Rf_Switch_Status::SEND);
                if (Sx1262->send_data(Codec2_Send_Buffer.data(), MAX_SX126X_TRANSMIT_BUFFER_SIZE) == true)
                {
                    Codec2_Send_Buffer.erase(Codec2_Send_Buffer.begin(), Codec2_Send_Buffer.begin() + MAX_SX126X_TRANSMIT_BUFFER_SIZE);

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

                Set_Sx1262_Rf_Switch(Sx1262_Rf_Switch_Status::RECEIVE);

                // 还原接收模式
                Sx1262->start_gfsk_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
                Sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE,
                                         Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
                                         Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE);
                Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
            }
        }
        else
        {
            if (digitalRead(SX1262_INT) == 1) // 接收完成中断
            {
                // 检查中断
                Cpp_Bus_Driver::Sx126x::Irq_Status is;
                // 判断中断正确性
                if (Sx1262->parse_irq_status(Sx1262->get_irq_flag(), is) == false)
                {
                    printf("parse_irq_status fail\n");
                }
                else
                {
                    if (is.all_flag.tx_rx_timeout == true)
                    {
                        printf("receive timeout\n");
                        Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TIMEOUT);
                    }
                    else if (is.all_flag.crc_error == true)
                    {
                        printf("receive crc error\n");
                        Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::CRC_ERROR);
                    }
                    else
                    {
                        // 判断传输包正确性
                        Cpp_Bus_Driver::Sx126x::Gfsk_Packet_Status gps;
                        uint32_t packet_buffer = Sx1262->get_gfsk_packet_status();
                        if (Sx1262->parse_gfsk_packet_status(packet_buffer, gps) == false)
                        {
                            printf("parse_gfsk_packet_status fail\n");
                        }
                        else
                        {
                            if (gps.abort_error_flag == true) // 中止错误
                            {
                                printf("receive abort_error_flag error\n");
                            }
                            else if (gps.length_error_flag == true) // 长度错误
                            {
                                printf("receive length_error_flag error\n");
                            }
                            else if (gps.crc_error_flag == true) // CRC校验错误
                            {
                                printf("receive crc_error_flag error\n");
                            }
                            else if (gps.address_error_flag == true) // 地址错误
                            {
                                printf("receive address_error_flag error\n");
                            }
                            else if (gps.sync_word_flag == true) // 同步错误
                            {
                                printf("receive sync_word_flag error\n");
                            }
                            else if (gps.preamble_error_flag == true) // 前导错误
                            {
                                printf("receive preamble_error_flag error\n");
                            }
                            else
                            {
                                auto codec2_receive_buffer = std::make_unique<uint8_t[]>(MAX_SX126X_TRANSMIT_BUFFER_SIZE);

                                uint8_t length_buffer = Sx1262->receive_data(codec2_receive_buffer.get(), MAX_SX126X_TRANSMIT_BUFFER_SIZE);
                                if (length_buffer == 0)
                                {
                                    printf("Sx1262 receive fail (error assert: %d)\n", Sx1262->_assert);
                                }
                                else
                                {
                                    Cpp_Bus_Driver::Sx126x::Packet_Metrics pm;
                                    Sx1262->parse_gfsk_packet_metrics(packet_buffer, pm);

                                    printf("Sx1262 receive rssi_average: %.01f rssi_sync: %.01f\n", pm.gfsk.rssi_average, pm.gfsk.rssi_sync);

                                    // for (uint8_t i = 0; i < length_buffer; i++)
                                    // {
                                    //     printf("get Sx1262 data[%d]: %d\n", i, codec2_receive_buffer[i]);
                                    // }

                                    if (Codec2_Receive_Buffer.size() < MAX_CODEC2_TRANSMIT_BUFFER_COUNT * MAX_SX126X_TRANSMIT_BUFFER_SIZE)
                                    {
                                        Codec2_Receive_Buffer.insert(Codec2_Receive_Buffer.end(), codec2_receive_buffer.get(), codec2_receive_buffer.get() + length_buffer);
                                    }
                                }
                            }
                        }
                    }
                }

                Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
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

    // 3.3V Power ON
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);
    delay(100);
    digitalWrite(RT9080_EN, LOW);
    delay(100);
    digitalWrite(RT9080_EN, HIGH);
    delay(100);

    pinMode(SPEAKER_EN, OUTPUT);
    digitalWrite(SPEAKER_EN, HIGH);
    pinMode(SPEAKER_EN_2, OUTPUT);
    digitalWrite(SPEAKER_EN_2, HIGH);

    pinMode(nRF52840_BOOT, INPUT_PULLUP);
    pinMode(SX1262_INT, INPUT);

    pinMode(SX1262_RF_VC1, OUTPUT);
    pinMode(SX1262_RF_VC2, OUTPUT);
    delay(500);

    IIS_Bus->begin(nrf_i2s_ratio_t::NRF_I2S_RATIO_128X, IIS_SAMPLE_RATE, nrf_i2s_swidth_t::NRF_I2S_SWIDTH_16BIT, nrf_i2s_channels_t::NRF_I2S_CHANNELS_LEFT);

    xTaskCreate(&Codec2_Encode_Task, "Codec2_Encode_Task", 15 * 1024, NULL, 3, NULL);

    while (1)
    {
        if (Codec2_Init_Flag == true)
        {
            if (IIS_Bus->start_transmit(Iis_Tx_Buffer[Current_Iis_Tx_Buffer_Count], nullptr, Iis_Data_Transmit_Size * MAX_IIS_DATA_TRANSMIT_COUNT) == true)
            {
                Iis_Tx_Buffer_Full_Flag[Current_Iis_Tx_Buffer_Count] = false;
                Current_Iis_Tx_Buffer_Count = (Current_Iis_Tx_Buffer_Count + 1) % MAX_IIS_TX_BUFFER_COUNT;
                printf("start_transmit success\n");
            }
            else
            {
                printf("start_transmit fail\n");
            }

            break;
        }
    }

    xTaskCreate(&Codec2_Decode_Task, "Codec2_Decode_Task", 15 * 1024, NULL, 3, NULL);

    xTaskCreate(&Iis_Tx_Handle, "Iis_Tx_Handle", 1 * 1024, NULL, 5, NULL);

    xTaskCreate(&Pdm_Task, "Pdm_Task", 1 * 1024, NULL, 3, NULL);

    xTaskCreate(&Sx1262_Task, "Sx1262_Task", 1 * 1024, NULL, 3, NULL);
}

void loop()
{
    if (IIS_Bus->get_write_event_flag() == true)
    {
        if (Iis_Tx_Buffer_Full_Flag[Current_Iis_Tx_Buffer_Count] == true)
        {
            IIS_Bus->set_next_write_data(Iis_Tx_Buffer[Current_Iis_Tx_Buffer_Count]);
            Iis_Tx_Buffer_Full_Flag[Current_Iis_Tx_Buffer_Count] = false;
        }
    }

    if (digitalRead(nRF52840_BOOT) == LOW)
    {
        delay(300);

        Sx1262_Send_Flag = !Sx1262_Send_Flag;
        if (Sx1262_Send_Flag == true)
        {
            IIS_Bus->stop_transmit();

            Pdm_Stream.clear();
            Codec2_Encode_Buffer.clear();
            Codec2_Send_Buffer.clear();

            printf("sx1262 send start\n");
        }
        else
        {
            if (IIS_Bus->start_transmit(Iis_Tx_Buffer[Current_Iis_Tx_Buffer_Count], nullptr, Iis_Data_Transmit_Size * MAX_IIS_DATA_TRANSMIT_COUNT) == true)
            {
                Iis_Tx_Buffer_Full_Flag[Current_Iis_Tx_Buffer_Count] = false;
                Current_Iis_Tx_Buffer_Count = (Current_Iis_Tx_Buffer_Count + 1) % MAX_IIS_TX_BUFFER_COUNT;
                printf("start_transmit success\n");
            }
            else
            {
                printf("start_transmit fail\n");
            }

            Codec2_Decode_Buffer.clear();
            Codec2_Receive_Buffer.clear();

            printf("sx1262 receive start\n");
        }
    }

    if (millis() > Cycle_Time)
    {
        printf("Pdm_Stream size: %d\n", Pdm_Stream.size());
        printf("Codec2_Encode_Buffer: %d\n", Codec2_Encode_Buffer.size());
        printf("Codec2_Decode_Buffer: %d\n", Codec2_Decode_Buffer.size());
        printf("Codec2_Send_Buffer: %d\n", Codec2_Send_Buffer.size());
        printf("Codec2_Receive_Buffer: %d\n", Codec2_Receive_Buffer.size());

        printf("Iis_Tx_Buffer: %d\n", static_cast<int16_t>(Iis_Tx_Buffer[Current_Iis_Tx_Buffer_Count][0] >> 16));

        // printf("\n");

        // printf("Sx1262 ID: %s\n", Sx1262->get_device_id().c_str());

        // printf("Sx1262 get current limit: %d\n", Sx1262->get_current_limit());

        // switch (Sx1262->get_packet_type())
        // {
        // case Cpp_Bus_Driver::Sx126x::Packet_Type::GFSK:
        //     printf("Sx1262 packet type: GFSK\n");
        //     break;
        // case Cpp_Bus_Driver::Sx126x::Packet_Type::LORA:
        //     printf("Sx1262 packet type: LORA\n");
        //     break;
        // case Cpp_Bus_Driver::Sx126x::Packet_Type::LR_FHSS:
        //     printf("Sx1262 packet type: LR_FHSS\n");
        //     break;

        // default:
        //     break;
        // }

        // switch (Sx1262->parse_chip_mode_status(Sx1262->get_status()))
        // {
        // case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::STBY_RC:
        //     printf("Sx1262 chip mode status: STBY_RC\n");
        //     break;
        // case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::STBY_XOSC:
        //     printf("Sx1262 chip mode status: STBY_XOSC\n");
        //     break;
        // case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::FS:
        //     printf("Sx1262 chip mode status: FS\n");
        //     break;
        // case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::RX:
        //     printf("Sx1262 chip mode status: RX\n");
        //     break;
        // case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::TX:
        //     printf("Sx1262 chip mode status: TX\n");
        //     break;

        // default:
        //     break;
        // }

        Cycle_Time = millis() + 1000;
    }

    delay(10);
}