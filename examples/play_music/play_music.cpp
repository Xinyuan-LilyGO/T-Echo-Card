/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-08-25 16:09:08
 * @LastEditTime: 2025-09-23 09:24:18
 * @License: GPL 3.0
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TinyUSB.h>
#include "t_echo_card_config.h"
#include "cpp_bus_driver_library.h"
#include "c2_b16_s44100_2.h"

#define SAMPLE_RATE 44100
#define MAX_IIS_DATA_TRANSMIT_SIZE 512

uint32_t Iis_Tx_Buffer[MAX_IIS_DATA_TRANSMIT_SIZE] = {0};

// 已经发送的数据大小
size_t Iis_Send_Data_Size = 0;
bool Iis_Transmit_Flag = false;

bool Iis_Data_Convert_Wait = false;

size_t Play_Count = 0;

auto IIS_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iis>(-1, SPEAKER_DATA, SPEAKER_WS_LRCK, SPEAKER_BCLK, -1);

// void Iis_Data_Convert(const uint16_t *input_data, uint32_t *out_buffer, size_t input_data_start_index, size_t length)
// {
//     for (size_t i = 0; i < length; i++)
//     {
//         uint16_t sample_l = input_data[input_data_start_index + i * 2];
//         uint16_t sample_r = input_data[input_data_start_index + i * 2 + 1];

//         // 小端序：低位存储左声道，高位存储右声道
//         out_buffer[i] = (sample_r << 16) | sample_l;
//     }
// }

// void Iis_Data_Convert(const uint16_t *input_data, uint32_t *out_buffer, size_t input_data_start_index, size_t out_data_length)
// {
//     const uint16_t *input_ptr = input_data + input_data_start_index;
//     uint32_t *out_ptr = out_buffer;

//     for (size_t i = 0; i < out_data_length; ++i)
//     {
//         // 使用 memcpy 直接将两个 uint16_t 复制到 uint32_t 中，并进行字节序调整
//         memcpy(out_ptr, input_ptr, 2);                      // 复制 sample_l
//         memcpy(((uint8_t *)out_ptr) + 2, input_ptr + 1, 2); // 复制 sample_r 到高位

//         out_ptr++;
//         input_ptr += 2;
//     }
// }

// void Iis_Data_Convert(const uint16_t *input_data, uint32_t *out_buffer, size_t input_data_start_index, size_t out_data_length)
// {
//     const uint8_t *input_ptr = (const uint8_t *)(input_data + input_data_start_index);

//     for (size_t i = 0; i < out_data_length; i++)
//     {
//         // 明确复制字节，避免字节序混淆
//         memcpy(&out_buffer[i], input_ptr, 4);
//         input_ptr += 4;
//     }
// }

void Iis_Data_Convert(const void *input_data, void *out_buffer, size_t input_data_start_index, size_t byte)
{
    const uint8_t *input_ptr = (const uint8_t *)input_data + input_data_start_index;
    uint8_t *out_ptr = (uint8_t *)out_buffer;

    memcpy(out_ptr, input_ptr, byte);
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
    printf("Ciallo\n");

    // 3.3V Power ON
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    pinMode(SPEAKER_EN, OUTPUT);
    digitalWrite(SPEAKER_EN, HIGH);

    pinMode(SPEAKER_EN_2, OUTPUT);
    digitalWrite(SPEAKER_EN_2, HIGH);

    // 必须加上电延时
    delay(500);

    IIS_Bus->begin(nrf_i2s_ratio_t ::NRF_I2S_RATIO_32X, SAMPLE_RATE, nrf_i2s_swidth_t::NRF_I2S_SWIDTH_16BIT, nrf_i2s_channels_t::NRF_I2S_CHANNELS_STEREO);

    pinMode(nRF52840_BOOT, INPUT_PULLUP);
}

void loop()
{
    // Iic_Scan();
    // delay(1000);

    if (digitalRead(nRF52840_BOOT) == LOW)
    {
        delay(300);
        // Iic_Scan();
        // ES8311->set_adc_data_to_dac(true);
        // uint8_t buffer = 0;
        // for (size_t i = 0; i < 256; i++)
        // {
        //     IIC_Bus_0->Bus_Iic_Guide::read((uint8_t)i, &buffer);
        //     printf("es8311 register[%d]: %#X\n", i, buffer);
        // }

        // 播放音乐测试
        printf("music play start\n");

        Iis_Send_Data_Size = 0;
        Iis_Data_Convert(c2_b16_s44100_2, Iis_Tx_Buffer, 0, MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t));

        if (IIS_Bus->start_transmit(Iis_Tx_Buffer, nullptr, MAX_IIS_DATA_TRANSMIT_SIZE) == true)
        {
            Iis_Send_Data_Size += MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t);
            Iis_Transmit_Flag = true;
        }
        else
        {
            Iis_Transmit_Flag = false;
            printf("music play fail (ES8311->start_transmit fail)\n");
        }

        Play_Count++;
        printf("play_count: %d\n", Play_Count);
    }

    if (Iis_Transmit_Flag == true)
    {
        if (Iis_Send_Data_Size >= sizeof(c2_b16_s44100_2))
        {
            printf("music play finish iis_send_data_size: %d\n", Iis_Send_Data_Size);
            IIS_Bus->stop_transmit();

            Iis_Data_Convert_Wait = false;
            Iis_Transmit_Flag = false;
        }
        else
        {
            if (Iis_Data_Convert_Wait == false)
            {
                size_t buffer_length = min(MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t), sizeof(c2_b16_s44100_2) - Iis_Send_Data_Size);

                // printf("iis_send_data_size: %d\n", Iis_Send_Data_Size);
                // printf("iis send data length: %d\n", buffer_length);

                if (buffer_length != MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t))
                {
                    memset(Iis_Tx_Buffer, 0, MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t));
                }
                Iis_Data_Convert(c2_b16_s44100_2, Iis_Tx_Buffer, Iis_Send_Data_Size, buffer_length);

                Iis_Send_Data_Size += buffer_length;

                Iis_Data_Convert_Wait = true;
            }
        }

        if (IIS_Bus->get_write_event_flag() == true)
        {
            if (Iis_Data_Convert_Wait == true)
            {
                IIS_Bus->set_next_write_data(Iis_Tx_Buffer);
                Iis_Data_Convert_Wait = false;
            }
        }
    }
}
