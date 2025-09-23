/*
 * @Description: GPS test
 * @Author: LILYGO_L
 * @Date: 2024-10-25 17:57:30
 * @LastEditTime: 2025-09-23 11:22:22
 * @License: GPL 3.0
 */
#include "cpp_bus_driver_library.h"
#include "t_echo_card_config.h"
#include <Adafruit_TinyUSB.h>

size_t CycleTime = 0;

auto Nrf52840_Gnss = std::make_shared<Cpp_Bus_Driver::Gnss>();

void setup()
{
    Serial.begin(115200);
    // while (!Serial)
    // {
    //     delay(100); // wait for native usb
    // }
    Serial.println("Ciallo");

    // 3.3V Power ON
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    pinMode(GPS_WAKE_UP, OUTPUT);
    digitalWrite(GPS_WAKE_UP, HIGH);

    Serial2.setPins(GPS_UART_TX, GPS_UART_RX);
    Serial2.begin(9600);

    pinMode(GPS_1PPS, INPUT);
}

void loop()
{
    if (millis() > CycleTime)
    {
        // 检查Serial2是否有可用数据
        if (Serial2.available())
        {
            // 读取Serial2所有可用数据到缓冲区
            const size_t bufferSize = 1024;

            std::unique_ptr<uint8_t[]> buffer(new uint8_t[bufferSize]);
            size_t bytesRead = 0;

            while (Serial2.available() && bytesRead < bufferSize)
            {
                buffer[bytesRead++] = Serial2.read();
            }

            buffer[bytesRead] = '\0';

            // 打印RMC的相关信息
            printf("---begin---\n%s \n---end---\n", buffer.get());

            printf("------------RMC------------\n");

            // 创建Rmc对象用于存储解析结果
            Cpp_Bus_Driver::Gnss::Rmc rmc;
            // 调用parse_rmc_info进行解码
            if (Nrf52840_Gnss->parse_rmc_info(buffer.get(), bytesRead, rmc) == true)
            {
                printf("location status: %s\n", (rmc.location_status).c_str());

                if (rmc.data.update_flag == true)
                {
                    printf("utc data: %d/%d/%d\n", rmc.data.year + 2000, rmc.data.month, rmc.data.day);
                    rmc.data.update_flag = false;
                }
                if (rmc.utc.update_flag == true)
                {
                    printf("utc time: %d:%d:%.03f\n", rmc.utc.hour, rmc.utc.minute, rmc.utc.second);
                    printf("china time: %d:%d:%.03f\n", (rmc.utc.hour + 8 + 24) % 24, rmc.utc.minute, rmc.utc.second);
                    rmc.utc.update_flag = false;
                }

                if ((rmc.location.lat.update_flag == true) && (rmc.location.lat.direction_update_flag == true))
                {
                    printf("location lat degrees: %d \nlocation lat minutes: %.10lf \nlocation lat degrees_minutes: %.10lf \nlocation lat direction: %s\n",
                           rmc.location.lat.degrees, rmc.location.lat.minutes, rmc.location.lat.degrees_minutes, (rmc.location.lat.direction).c_str());
                    rmc.location.lat.update_flag = false;
                    rmc.location.lat.direction_update_flag = false;
                }
                if ((rmc.location.lon.update_flag == true) && (rmc.location.lon.direction_update_flag == true))
                {
                    printf("location lon degrees: %d \nlocation lon minutes: %.10lf \nlocation lon degrees_minutes: %.10lf \nlocation lon direction: %s\n",
                           rmc.location.lon.degrees, rmc.location.lon.minutes, rmc.location.lon.degrees_minutes, (rmc.location.lon.direction).c_str());
                    rmc.location.lon.update_flag = false;
                    rmc.location.lon.direction_update_flag = false;
                }
            }
        }

        CycleTime = millis() + 1000;
    }
}