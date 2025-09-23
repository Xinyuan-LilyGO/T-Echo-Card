/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-09-02 09:33:15
 * @LastEditTime: 2025-09-23 10:04:35
 * @License: GPL 3.0
 */
#include "PDM.h"
#include "t_echo_card_config.h"
#include <vector>

#define MAX_PDM_DATA_TRANSMIT_SIZE 256
#define MAX_PDM_DATA_TRANSMIT_MULTIPLE 2

// 用于存储rdm接收流的容器
std::vector<int16_t> Pdm_Rx_Stream;

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

    PDM.setPins(MICROPHONE_DATA, MICROPHONE_SCLK, -1);
    // Pdm_Callback中断回调里面不能放printf
    PDM.onReceive([]()
                    {
                        if (PDM.available() >= MAX_PDM_DATA_TRANSMIT_SIZE)
                        {
                            int16_t pdm_rx_buffer[MAX_PDM_DATA_TRANSMIT_SIZE];
                            int32_t buffer_length = PDM.read(pdm_rx_buffer, MAX_PDM_DATA_TRANSMIT_SIZE * sizeof(int16_t));

                            if (Pdm_Rx_Stream.size() < (MAX_PDM_DATA_TRANSMIT_SIZE * MAX_PDM_DATA_TRANSMIT_MULTIPLE))
                            {
                                Pdm_Rx_Stream.insert(Pdm_Rx_Stream.end(), pdm_rx_buffer, pdm_rx_buffer + (buffer_length / sizeof(int16_t)));
                            }
                        } });

    // // optionally set the gain, defaults to 20
    // // PDM.setGain(30);

    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (PDM.begin(2, 16000) == false)
    {
        printf("failed to start pdm\n");
        delay(100);
    }
}

void loop()
{
    if (Pdm_Rx_Stream.size() >= 2)
    {
        printf("Pdm_Rx_Stream size: %d\n", Pdm_Rx_Stream.size());

        // 输出左声道数据
        printf("left: %d\n", Pdm_Rx_Stream[0]);

        // 输出右声道数据
        printf("right: %d\n", Pdm_Rx_Stream[1]);

        Pdm_Rx_Stream.erase(Pdm_Rx_Stream.begin(), Pdm_Rx_Stream.begin() + MAX_PDM_DATA_TRANSMIT_SIZE);
    }
}