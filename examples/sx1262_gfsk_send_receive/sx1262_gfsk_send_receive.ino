/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-01-05 11:55:57
 * @LastEditTime: 2026-01-08 17:48:11
 * @License: GPL 3.0
 */

#include "t_echo_card_config.h"
#include "cpp_bus_driver_library.h"

enum class Sx1262_Rf_Switch_Status
{
    SEND,
    RECEIVE,
};

uint8_t Receive_Package[255] = {0};

uint8_t Send_Package[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

size_t Cycle_Time = 0;

auto SPI_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Spi>(SX1262_MOSI, SX1262_SCLK, SX1262_MISO, NRF_SPIM3, 0);

auto Sx1262 = std::make_unique<Cpp_Bus_Driver::Sx126x>(SPI_Bus, Cpp_Bus_Driver::Sx126x::Chip_Type::SX1262, SX1262_BUSY,
                                                       SX1262_CS, SX1262_RST);

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
    delay(500);

    pinMode(nRF52840_BOOT, INPUT_PULLUP);
    pinMode(SX1262_INT, INPUT);

    pinMode(SX1262_RF_VC1, OUTPUT);
    pinMode(SX1262_RF_VC2, OUTPUT);

    Set_Sx1262_Rf_Switch(Sx1262_Rf_Switch_Status::RECEIVE);

    Sx1262->begin(10000000);
    Sx1262->config_gfsk_params(850.0, 200.0, Cpp_Bus_Driver::Sx126x::Gfsk_Bw::BW_467000HZ, 140, 22);
    Sx1262->clear_buffer();

    Sx1262->start_gfsk_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
    Sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE,
                             Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
                             Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE);
    Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);

    printf("Sx1262 start gfsk transmit\n");
}

void loop()
{
    if (millis() > Cycle_Time)
    {
        printf("Sx1262 ID: %s\n", Sx1262->get_device_id().c_str());

        printf("Sx1262 get current limit: %d\n", Sx1262->get_current_limit());

        switch (Sx1262->get_packet_type())
        {
        case Cpp_Bus_Driver::Sx126x::Packet_Type::GFSK:
            printf("Sx1262 packet type: GFSK\n");
            break;
        case Cpp_Bus_Driver::Sx126x::Packet_Type::LORA:
            printf("Sx1262 packet type: LORA\n");
            break;
        case Cpp_Bus_Driver::Sx126x::Packet_Type::LR_FHSS:
            printf("Sx1262 packet type: LR_FHSS\n");
            break;

        default:
            break;
        }

        switch (Sx1262->parse_chip_mode_status(Sx1262->get_status()))
        {
        case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::STBY_RC:
            printf("Sx1262 chip mode status: STBY_RC\n");
            break;
        case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::STBY_XOSC:
            printf("Sx1262 chip mode status: STBY_XOSC\n");
            break;
        case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::FS:
            printf("Sx1262 chip mode status: FS\n");
            break;
        case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::RX:
            printf("Sx1262 chip mode status: RX\n");
            break;
        case Cpp_Bus_Driver::Sx126x::Chip_Mode_Status::TX:
            printf("Sx1262 chip mode status: TX\n");
            break;

        default:
            break;
        }

        Cycle_Time = millis() + 1000;
    }

    if (digitalRead(nRF52840_BOOT) == 0)
    {
        delay(1000);

        // 设置发送模式，发送完成后进入快速切换模式（FS模式）
        Sx1262->start_gfsk_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::TX, 0, Cpp_Bus_Driver::Sx126x::Fallback_Mode::FS);
        Sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE,
                                 Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
                                 Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE);
        Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);

        printf("Sx1262 send start\n");
        uint16_t timeout_count = 0;
        Set_Sx1262_Rf_Switch(Sx1262_Rf_Switch_Status::SEND);
        if (Sx1262->send_data(Send_Package, sizeof(Send_Package)) == true)
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

        Set_Sx1262_Rf_Switch(Sx1262_Rf_Switch_Status::RECEIVE);

        // 还原接收模式
        Sx1262->start_gfsk_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
        Sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE,
                                 Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
                                 Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE);
        Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
    }

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
                        memset(Receive_Package, 0, 255);
                        uint8_t length_buffer = Sx1262->receive_data(Receive_Package);
                        if (length_buffer == 0)
                        {
                            printf("Sx1262 receive fail (error assert: %d)\n", Sx1262->_assert);
                        }
                        else
                        {
                            Cpp_Bus_Driver::Sx126x::Packet_Metrics pm;
                            Sx1262->parse_gfsk_packet_metrics(packet_buffer, pm);

                            printf("Sx1262 receive rssi_average: %.01f rssi_sync: %.01f\n", pm.gfsk.rssi_average, pm.gfsk.rssi_sync);

                            for (uint8_t i = 0; i < length_buffer; i++)
                            {
                                printf("get Sx1262 data[%d]: %d\n", i, Receive_Package[i]);
                            }

                            // 接收完成中断清除需要成功接收到数据后才能清除
                            // 不能放在其他接收到错误数据的位置上
                            Sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
                        }
                    }
                }
            }
        }
    }

    delay(10);
}
