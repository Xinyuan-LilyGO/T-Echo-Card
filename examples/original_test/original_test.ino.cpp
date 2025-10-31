# 1 "C:\\Users\\XK\\AppData\\Local\\Temp\\tmpujv7pf63"
#include <Arduino.h>
# 1 "D:/Information/visual_studio_code/gitHub/T-Echo-Card/examples/original_test/original_test.ino"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "t_echo_card_config.h"
#include "Display_Fonts.h"
#include <bluefruit.h>
#include "Adafruit_SPIFlash.h"
#include "cpp_bus_driver_library.h"
#include "wiring.h"
#include "Adafruit_QMC5883P.h"
#include "Adafruit_NeoPixel.h"
#include "c2_b16_s44100_2.h"
#include "c2_b16_s44100_3.h"
#include "PDM.h"
#include <vector>

#define SOFTWARE_NAME "original_test"
#define SOFTWARE_LASTEDITTIME "202510311640"
#define BOARD_VERSION "v1.0"

#define NUM_LEDS 1

#define SAMPLE_RATE 44100
#define MAX_IIS_DATA_TRANSMIT_SIZE 512

#define MAX_PDM_DATA_TRANSMIT_SIZE 256
#define MAX_PDM_DATA_TRANSMIT_MULTIPLE 2

#define MAX_UART_RX_BUFFER_SIZE 1024

enum class System_Window
{
    HOME = 0,
    FLASH_TEST,
    BATTERY_TEST,
    IMU_TEST,
    GPS_TEST,
    MICROPHONE_TEST,

    END,
};

static const uint32_t Local_MAC[2] =
    {
        NRF_FICR->DEVICEID[0],
        NRF_FICR->DEVICEID[1],
};

SPIFlash_Device_t ZD25WQ32C =
    {
        total_size : (1UL << 22),
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

struct BLE_Uart_Operator
{
    using state = enum {
        UNCONNECTED,
        CONNECTED,
    };

    struct
    {
        bool state_flag = state::UNCONNECTED;
        char device_name[32] = {'\0'};
    } connection;

    struct
    {
        uint8_t receive_data[100] = {'\0'};
    } transmission;

    bool initialization_flag = false;
};

struct System_Operator
{
    size_t time = 0;

    uint8_t sleep_count = 0;

    struct
    {
        bool screen = false;
        bool flash = false;
        bool imu = false;

    } init_flag;
};

struct Button_Triggered_Operator
{
    using gesture = enum {
        NOT_ACTIVE,
        SINGLE_CLICK,
        DOUBLE_CLICK,
        LONG_PRESS,
    };
    const uint32_t button_number = nRF52840_BOOT;

    bool trigger_level = LOW;

    size_t cycletime_1 = 0;
    size_t cycletime_2 = 0;

    uint8_t current_state = gesture::NOT_ACTIVE;
    bool trigger_start_flag = false;
    bool trigger_flag = false;
    bool timing_flag = false;
    int8_t paragraph_triggered_level = -1;
    uint8_t high_triggered_count = 0;
    uint8_t low_triggered_count = 0;
    uint8_t paragraph_triggered_count = 0;

    volatile bool Interrupt_Flag = false;
};

uint8_t Current_Window_Count = 0;
System_Window Current_Window = System_Window::HOME;

bool Battery_Control_Switch = false;

size_t CycleTime = 0;
size_t CycleTime_2 = 0;

bool Gps_Positioning_Flag = false;
size_t Gps_Positioning_Time = 0;

uint32_t Iis_Tx_Buffer[MAX_IIS_DATA_TRANSMIT_SIZE] = {0};


size_t Iis_Send_Data_Size = 0;
bool Iis_Transmit_Flag = false;

bool Iis_Data_Convert_Wait = false;


std::vector<int16_t> Pdm_Rx_Stream;

uint8_t Uart_Rx_Buffer[MAX_UART_RX_BUFFER_SIZE] = {0};
size_t Uart_Rx_Count = 0;







BLEDfu bledfu;
BLEDis bledis;
BLEUart bleuart;

BLE_Uart_Operator BLE_Uart_Op;
System_Operator System_Op;
Button_Triggered_Operator Button_Triggered_OP;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, SCREEN_RST);






Adafruit_FlashTransport_QSPI flashTransport(ZD25WQ32C_SCLK, ZD25WQ32C_CS,
                                            ZD25WQ32C_IO0, ZD25WQ32C_IO1,
                                            ZD25WQ32C_IO2, ZD25WQ32C_IO3);

Adafruit_SPIFlash flash(&flashTransport);

Adafruit_QMC5883P qmc;

auto Nrf52840_Gnss = std::make_unique<Cpp_Bus_Driver::Gnss>();

auto IIS_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iis>(-1, SPEAKER_DATA, SPEAKER_WS_LRCK, SPEAKER_BCLK, -1);

Adafruit_NeoPixel Led_1(NUM_LEDS, WS2812_DATA_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel Led_2(NUM_LEDS, WS2812_DATA_2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel Led_3(NUM_LEDS, WS2812_DATA_3, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel *Led[] =
    {
        &Led_1,
        &Led_2,
        &Led_3,
};
void log_printf(const char *fmt, ...);
void Buzzer_Trigger(uint32_t delay_ms);
bool Key_Scanning(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void startAdv(void);
bool BLE_Uart_Initialization(void);
void Window_Init(System_Window Window);
void Window_End(System_Window Window);
void System_Sleep(bool mode);
void Iis_Data_Convert(const void *input_data, void *out_buffer, size_t input_data_start_index, size_t byte);
void music_play_welcome();
void setup();
void loop();
#line 200 "D:/Information/visual_studio_code/gitHub/T-Echo-Card/examples/original_test/original_test.ino"
void log_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), fmt, args);

    Serial.print(buffer);
    if ((BLE_Uart_Op.initialization_flag == true) && (BLE_Uart_Op.connection.state_flag == BLE_Uart_Op.state::CONNECTED))
    {
        bleuart.print(buffer);
    }

    va_end(args);
}

void Buzzer_Trigger(uint32_t delay_ms)
{
    tone(BUZZER_DATA, 2000, delay_ms);
}

bool Key_Scanning(void)
{
    if (Button_Triggered_OP.trigger_start_flag == false)
    {
        if (digitalRead(Button_Triggered_OP.button_number) == Button_Triggered_OP.trigger_level)
        {
            Button_Triggered_OP.trigger_start_flag = true;

            log_printf("press button to trigger start");
            Button_Triggered_OP.high_triggered_count = 0;
            Button_Triggered_OP.low_triggered_count = 0;
            Button_Triggered_OP.paragraph_triggered_count = 0;
            Button_Triggered_OP.paragraph_triggered_level = -1;
            Button_Triggered_OP.trigger_flag = true;
            Button_Triggered_OP.timing_flag = true;
        }
    }
    if (Button_Triggered_OP.timing_flag == true)
    {
        Button_Triggered_OP.cycletime_2 = millis() + 1000;
        Button_Triggered_OP.timing_flag = false;
    }
    if (Button_Triggered_OP.trigger_flag == true)
    {
        if (millis() > Button_Triggered_OP.cycletime_1)
        {
            if (digitalRead(Button_Triggered_OP.button_number) == HIGH)
            {
                Button_Triggered_OP.high_triggered_count++;
                if (Button_Triggered_OP.paragraph_triggered_level != HIGH)
                {
                    Button_Triggered_OP.paragraph_triggered_count++;
                    Button_Triggered_OP.paragraph_triggered_level = HIGH;
                }
            }
            else if (digitalRead(Button_Triggered_OP.button_number) == LOW)
            {
                Button_Triggered_OP.low_triggered_count++;
                if (Button_Triggered_OP.paragraph_triggered_level != LOW)
                {
                    Button_Triggered_OP.paragraph_triggered_count++;
                    Button_Triggered_OP.paragraph_triggered_level = LOW;
                }
            }
            Button_Triggered_OP.cycletime_1 = millis() + 50;
        }

        if (Button_Triggered_OP.timing_flag == false)
        {
            if (millis() > Button_Triggered_OP.cycletime_2)
            {
                log_printf("end\n");
                log_printf("high_triggered_count: %d\n", Button_Triggered_OP.high_triggered_count);
                log_printf("low_triggered_count: %d\n", Button_Triggered_OP.low_triggered_count);
                log_printf("paragraph_triggered_count: %d\n", Button_Triggered_OP.paragraph_triggered_count);

                Button_Triggered_OP.trigger_flag = false;
                Button_Triggered_OP.trigger_start_flag = false;

                if ((Button_Triggered_OP.paragraph_triggered_count == 2))
                {
                    Button_Triggered_OP.current_state = Button_Triggered_OP.gesture::SINGLE_CLICK;
                    log_printf("key triggered: SINGLE_CLICK\n");
                    Buzzer_Trigger(150);
                    delay(500);
                    return true;
                }
                else if ((Button_Triggered_OP.paragraph_triggered_count == 4))
                {
                    Button_Triggered_OP.current_state = Button_Triggered_OP.gesture::DOUBLE_CLICK;
                    log_printf("key triggered: DOUBLE_CLICK\n");
                    Buzzer_Trigger(150);
                    delay(300);
                    Buzzer_Trigger(150);
                    delay(500);
                    return true;
                }
                else if ((Button_Triggered_OP.paragraph_triggered_count == 1))
                {
                    Button_Triggered_OP.current_state = Button_Triggered_OP.gesture::LONG_PRESS;
                    log_printf("key triggered: LONG_PRESS\n");
                    Buzzer_Trigger(500);
                    delay(500);
                    return true;
                }
            }
        }
    }

    return false;
}


void connect_callback(uint16_t conn_handle)
{

    BLEConnection *connection = Bluefruit.Connection(conn_handle);

    char central_name[32] = {0};
    connection->getPeerName(central_name, sizeof(central_name));

    Serial.print("Connected to ");
    Serial.println(central_name);

    strcpy(BLE_Uart_Op.connection.device_name, central_name);

    BLE_Uart_Op.connection.state_flag = BLE_Uart_Op.state::CONNECTED;
}






void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
    (void)conn_handle;
    (void)reason;

    Serial.println();
    Serial.print("Disconnected, reason = 0x");
    Serial.println(reason, HEX);
    BLE_Uart_Op.connection.state_flag = BLE_Uart_Op.state::UNCONNECTED;
}

void startAdv(void)
{

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();


    Bluefruit.Advertising.addService(bleuart);



    Bluefruit.ScanResponse.addName();
# 369 "D:/Information/visual_studio_code/gitHub/T-Echo-Card/examples/original_test/original_test.ino"
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
}

bool BLE_Uart_Initialization(void)
{
# 385 "D:/Information/visual_studio_code/gitHub/T-Echo-Card/examples/original_test/original_test.ino"
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

    if (Bluefruit.begin() == false)
    {
        Serial.println("BLE initialization failed");
        return false;
    }
    Serial.println("BLE initialization successful");

    Bluefruit.setTxPower(8);

    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);


    bledfu.begin();


    bledis.setManufacturer("LILYGO Industries");
    bledis.setModel("T-Impulse-Plus");
    bledis.begin();


    bleuart.begin();


    startAdv();

    Serial.println("Please use the BLE debugging tool to connect to the development board.");
    Serial.println("Once connected, enter character(s) that you wish to send");

    return true;
}

void Window_Init(System_Window Window)
{
    switch (Window)
    {
    case System_Window::HOME:
        System_Op.sleep_count = 0;
        break;
    case System_Window::FLASH_TEST:


        flash.begin();
        flashTransport.setClockSpeed(32000000UL, 0);
        flashTransport.runCommand(0xAB);

        if (flash.begin(&ZD25WQ32C) == false)
        {
            log_printf("flash init fail\n");
            System_Op.init_flag.flash = false;
        }
        else
        {
            log_printf("flash init successful\n");
            System_Op.init_flag.flash = true;
        }

        break;

    case System_Window::BATTERY_TEST:

        pinMode(BATTERY_ADC_DATA, INPUT);
        pinMode(BATTERY_MEASUREMENT_CONTROL, OUTPUT);
        digitalWrite(BATTERY_MEASUREMENT_CONTROL, HIGH);

        Battery_Control_Switch = true;


        analogReference(AR_INTERNAL_3_0);

        analogReadResolution(12);

        break;

    case System_Window::IMU_TEST:
        Wire.setPins(QMC5883P_SDA, QMC5883P_SCL);
        if (qmc.begin() == false)
        {
            log_printf("qmc5883p init fail\n");
            System_Op.init_flag.imu = false;
        }
        else
        {
            log_printf("qmc5883p init success\n");
            System_Op.init_flag.imu = true;


            qmc.setMode(QMC5883P_MODE_NORMAL);

            qmc5883p_mode_t currentMode = qmc.getMode();
            log_printf("Mode: ");
            switch (currentMode)
            {
            case QMC5883P_MODE_SUSPEND:
                log_printf("Suspend\n");
                break;
            case QMC5883P_MODE_NORMAL:
                log_printf("Normal\n");
                break;
            case QMC5883P_MODE_SINGLE:
                log_printf("Single\n");
                break;
            case QMC5883P_MODE_CONTINUOUS:
                log_printf("Continuous\n");
                break;
            default:
                log_printf("Unknown\n");
                break;
            }


            qmc.setODR(QMC5883P_ODR_50HZ);
            qmc5883p_odr_t currentODR = qmc.getODR();
            log_printf("ODR (Output Data Rate): ");
            switch (currentODR)
            {
            case QMC5883P_ODR_10HZ:
                log_printf("10Hz\n");
                break;
            case QMC5883P_ODR_50HZ:
                log_printf("50Hz\n");
                break;
            case QMC5883P_ODR_100HZ:
                log_printf("100Hz\n");
                break;
            case QMC5883P_ODR_200HZ:
                log_printf("200Hz\n");
                break;
            default:
                log_printf("Unknown\n");
                break;
            }


            qmc.setOSR(QMC5883P_OSR_4);
            qmc5883p_osr_t currentOSR = qmc.getOSR();
            log_printf("OSR (Over Sample Ratio): ");
            switch (currentOSR)
            {
            case QMC5883P_OSR_8:
                log_printf("8\n");
                break;
            case QMC5883P_OSR_4:
                log_printf("4\n");
                break;
            case QMC5883P_OSR_2:
                log_printf("2\n");
                break;
            case QMC5883P_OSR_1:
                log_printf("1\n");
                break;
            default:
                log_printf("Unknown\n");
                break;
            }


            qmc.setDSR(QMC5883P_DSR_2);
            qmc5883p_dsr_t currentDSR = qmc.getDSR();
            log_printf("DSR (Downsample Ratio): ");
            switch (currentDSR)
            {
            case QMC5883P_DSR_1:
                log_printf("1\n");
                break;
            case QMC5883P_DSR_2:
                log_printf("2\n");
                break;
            case QMC5883P_DSR_4:
                log_printf("4\n");
                break;
            case QMC5883P_DSR_8:
                log_printf("8\n");
                break;
            default:
                log_printf("Unknown");
                break;
            }


            qmc.setRange(QMC5883P_RANGE_8G);
            qmc5883p_range_t currentRange = qmc.getRange();
            log_printf("Range: ");
            switch (currentRange)
            {
            case QMC5883P_RANGE_30G:
                log_printf("±30G\n");
                break;
            case QMC5883P_RANGE_12G:
                log_printf("±12G\n");
                break;
            case QMC5883P_RANGE_8G:
                log_printf("±8G\n");
                break;
            case QMC5883P_RANGE_2G:
                log_printf("±2G\n");
                break;
            default:
                log_printf("Unknown\n");
                break;
            }


            qmc.setSetResetMode(QMC5883P_SETRESET_ON);
            qmc5883p_setreset_t currentSetReset = qmc.getSetResetMode();
            log_printf("Set/Reset Mode: ");
            switch (currentSetReset)
            {
            case QMC5883P_SETRESET_ON:
                log_printf("Set and Reset On\n");
                break;
            case QMC5883P_SETRESET_SETONLY:
                log_printf("Set Only On\n");
                break;
            case QMC5883P_SETRESET_OFF:
                log_printf("Set and Reset Off\n");
                break;
            default:
                log_printf("Unknown\n");
                break;
            }
        }

        break;

    case System_Window::GPS_TEST:
        Serial2.setPins(GPS_UART_TX, GPS_UART_RX);
        Serial2.begin(9600);

        digitalWrite(GPS_WAKE_UP, HIGH);
        digitalWrite(GPS_RF_EN, HIGH);

        Gps_Positioning_Flag = false;
        Gps_Positioning_Time = 0;

        break;

    case System_Window::MICROPHONE_TEST:
        Pdm_Rx_Stream.clear();
        PDM.setPins(MICROPHONE_DATA, MICROPHONE_SCLK, -1);

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







        if (PDM.begin(2, 16000) == false)
        {
            printf("failed to start pdm\n");
            delay(100);
        }
        break;

    default:
        break;
    }
}

void Window_End(System_Window Window)
{
    switch (Window)
    {
    case System_Window::HOME:
        CycleTime = 0;
        break;
    case System_Window::FLASH_TEST:
        flashTransport.runCommand(0xB9);
        flash.end();
        pinMode(ZD25WQ32C_SCLK, INPUT);
        pinMode(ZD25WQ32C_CS, INPUT);
        pinMode(ZD25WQ32C_IO0, INPUT);
        pinMode(ZD25WQ32C_IO1, INPUT);
        pinMode(ZD25WQ32C_IO2, INPUT);
        pinMode(ZD25WQ32C_IO3, INPUT);
        CycleTime = 0;
        break;
    case System_Window::BATTERY_TEST:
        digitalWrite(BATTERY_MEASUREMENT_CONTROL, LOW);
        CycleTime = 0;
        break;
    case System_Window::IMU_TEST:
        pinMode(QMC5883P_SDA, INPUT);
        pinMode(QMC5883P_SCL, INPUT);
        CycleTime = 0;
        break;
    case System_Window::GPS_TEST:
        Serial2.end();
        digitalWrite(GPS_WAKE_UP, LOW);
        digitalWrite(GPS_RF_EN, LOW);
        CycleTime = 0;
        break;
    case System_Window::MICROPHONE_TEST:
        PDM.end();

        pinMode(MICROPHONE_DATA, INPUT);
        pinMode(MICROPHONE_SCLK, INPUT);
        CycleTime = 0;
        break;

    default:
        break;
    }
}

void System_Sleep(bool mode)
{
    if (mode == true)
    {
        Serial.end();

        pinMode(SCREEN_SDA, INPUT);
        pinMode(SCREEN_SCL, INPUT);
        Wire.end();

        pinMode(BUZZER_DATA, INPUT);

        pinMode(GPS_WAKE_UP, INPUT);
        pinMode(GPS_RF_EN, INPUT);

        pinMode(nRF52840_BOOT, INPUT);

        for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
        {
            Led[i]->setPixelColor(0, Led[i]->Color(0, 0, 0));
            Led[i]->show();
        }

        IIS_Bus->end();

        pinMode(SPEAKER_EN, INPUT);
        pinMode(SPEAKER_EN_2, INPUT);

        digitalWrite(RT9080_EN, LOW);
        pinMode(RT9080_EN, INPUT_PULLDOWN);


        Bluefruit.Advertising.stop();
    }
    else
    {

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

        IIS_Bus->begin(nrf_i2s_ratio_t ::NRF_I2S_RATIO_32X, SAMPLE_RATE, nrf_i2s_swidth_t::NRF_I2S_SWIDTH_16BIT, nrf_i2s_channels_t::NRF_I2S_CHANNELS_STEREO);

        pinMode(nRF52840_BOOT, INPUT_PULLUP);

        Serial.begin(115200);

        Bluefruit.Advertising.start(0);

        pinMode(BUZZER_DATA, OUTPUT);
        digitalWrite(BUZZER_DATA, LOW);

        pinMode(GPS_WAKE_UP, OUTPUT);
        digitalWrite(GPS_WAKE_UP, HIGH);
        pinMode(GPS_RF_EN, OUTPUT);
        digitalWrite(GPS_RF_EN, HIGH);

        Wire.setPins(SCREEN_SDA, SCREEN_SCL);
        Wire.begin();
        if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS) == false)
        {
            log_printf("ssd1315 init fail\n");
            System_Op.init_flag.screen = false;
        }
        else
        {
            log_printf("ssd1315 init successful\n");
            System_Op.init_flag.screen = true;
        }
    }
}

void Iis_Data_Convert(const void *input_data, void *out_buffer, size_t input_data_start_index, size_t byte)
{
    const uint8_t *input_ptr = (const uint8_t *)input_data + input_data_start_index;
    uint8_t *out_ptr = (uint8_t *)out_buffer;

    memcpy(out_ptr, input_ptr, byte);
}

void music_play_welcome()
{

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

    while (1)
    {
        if (Iis_Transmit_Flag == true)
        {
            if (Iis_Send_Data_Size >= sizeof(c2_b16_s44100_2))
            {
                printf("music play finish iis_send_data_size: %d\n", Iis_Send_Data_Size);
                IIS_Bus->stop_transmit();

                Iis_Data_Convert_Wait = false;
                Iis_Transmit_Flag = false;
                break;
            }
            else
            {
                if (Iis_Data_Convert_Wait == false)
                {
                    size_t buffer_length = min(MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t), sizeof(c2_b16_s44100_2) - Iis_Send_Data_Size);




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
}

void setup()
{
    Serial.begin(115200);


    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);
    delay(100);
    digitalWrite(RT9080_EN, LOW);
    delay(100);
    digitalWrite(RT9080_EN, HIGH);
    delay(100);

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->begin();
        Led[i]->show();
        Led[i]->setBrightness(50);
    }

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(255, 0, 0));
        Led[i]->show();
    }
    delay(1000);

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(0, 255, 0));
        Led[i]->show();
    }
    delay(1000);

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(0, 0, 255));
        Led[i]->show();
    }
    delay(1000);

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(255, 255, 255));
        Led[i]->show();
    }

    Wire.setPins(SCREEN_SDA, SCREEN_SCL);
    Wire.begin();

    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS) == false)
    {
        log_printf("ssd1306 init fail\n");
        System_Op.init_flag.screen = false;
    }
    else
    {
        log_printf("ssd1306 init successful\n");
        System_Op.init_flag.screen = true;
    }

    display.setOffsetCursor(28, 24);
    display.clearDisplay();
    display.fillScreen(WHITE);
    display.setTextColor(BLACK);
    display.setCursor(15, 10);
    display.printf("LILYGO");
    display.display();

    display.setTextColor(WHITE);

    delay(1000);

    uint8_t serial_init_count = 0;
    while (!Serial)
    {
        delay(100);
        serial_init_count++;
        if (serial_init_count > 30)
        {
            break;
        }
    }
    Serial.println("[T-Echo-Card_" + (String)BOARD_VERSION "][" + (String)SOFTWARE_NAME +
                   "]_firmware_" + (String)SOFTWARE_LASTEDITTIME);

    pinMode(KEY_1, INPUT_PULLUP);
    pinMode(nRF52840_BOOT, INPUT_PULLUP);

    pinMode(GPS_WAKE_UP, OUTPUT);
    digitalWrite(GPS_WAKE_UP, HIGH);
    pinMode(GPS_RF_EN, OUTPUT);
    digitalWrite(GPS_RF_EN, HIGH);

    pinMode(BUZZER_DATA, OUTPUT);
    digitalWrite(BUZZER_DATA, LOW);

    if (BLE_Uart_Initialization() == true)
    {
        BLE_Uart_Op.initialization_flag = true;
        log_printf("BLE_Uart_Initialization successful\n");
    }
    else
    {
        BLE_Uart_Op.initialization_flag = false;
        log_printf("BLE_Uart_Initialization fail\n");
    }

    flash.begin();
    flashTransport.setClockSpeed(32000000UL, 0);
    flashTransport.runCommand(0xAB);
    if (flash.begin(&ZD25WQ32C) == false)
    {
        log_printf("flash init fail\n");
        System_Op.init_flag.flash = false;
    }
    else
    {
        log_printf("flash init successful\n");
        System_Op.init_flag.flash = true;
    }
    flashTransport.runCommand(0xB9);
    flash.end();
    pinMode(ZD25WQ32C_SCLK, INPUT);
    pinMode(ZD25WQ32C_CS, INPUT);
    pinMode(ZD25WQ32C_IO0, INPUT);
    pinMode(ZD25WQ32C_IO1, INPUT);
    pinMode(ZD25WQ32C_IO2, INPUT);
    pinMode(ZD25WQ32C_IO3, INPUT);

    Wire.setPins(QMC5883P_SDA, QMC5883P_SCL);
    if (qmc.begin() == false)
    {
        log_printf("qmc5883p init fail\n");
        System_Op.init_flag.imu = false;
    }
    else
    {
        log_printf("qmc5883p init success\n");
        System_Op.init_flag.imu = true;


        qmc.setMode(QMC5883P_MODE_NORMAL);

        qmc5883p_mode_t currentMode = qmc.getMode();
        log_printf("Mode: ");
        switch (currentMode)
        {
        case QMC5883P_MODE_SUSPEND:
            log_printf("Suspend\n");
            break;
        case QMC5883P_MODE_NORMAL:
            log_printf("Normal\n");
            break;
        case QMC5883P_MODE_SINGLE:
            log_printf("Single\n");
            break;
        case QMC5883P_MODE_CONTINUOUS:
            log_printf("Continuous\n");
            break;
        default:
            log_printf("Unknown\n");
            break;
        }


        qmc.setODR(QMC5883P_ODR_50HZ);
        qmc5883p_odr_t currentODR = qmc.getODR();
        log_printf("ODR (Output Data Rate): ");
        switch (currentODR)
        {
        case QMC5883P_ODR_10HZ:
            log_printf("10Hz\n");
            break;
        case QMC5883P_ODR_50HZ:
            log_printf("50Hz\n");
            break;
        case QMC5883P_ODR_100HZ:
            log_printf("100Hz\n");
            break;
        case QMC5883P_ODR_200HZ:
            log_printf("200Hz\n");
            break;
        default:
            log_printf("Unknown\n");
            break;
        }


        qmc.setOSR(QMC5883P_OSR_4);
        qmc5883p_osr_t currentOSR = qmc.getOSR();
        log_printf("OSR (Over Sample Ratio): ");
        switch (currentOSR)
        {
        case QMC5883P_OSR_8:
            log_printf("8\n");
            break;
        case QMC5883P_OSR_4:
            log_printf("4\n");
            break;
        case QMC5883P_OSR_2:
            log_printf("2\n");
            break;
        case QMC5883P_OSR_1:
            log_printf("1\n");
            break;
        default:
            log_printf("Unknown\n");
            break;
        }


        qmc.setDSR(QMC5883P_DSR_2);
        qmc5883p_dsr_t currentDSR = qmc.getDSR();
        log_printf("DSR (Downsample Ratio): ");
        switch (currentDSR)
        {
        case QMC5883P_DSR_1:
            log_printf("1\n");
            break;
        case QMC5883P_DSR_2:
            log_printf("2\n");
            break;
        case QMC5883P_DSR_4:
            log_printf("4\n");
            break;
        case QMC5883P_DSR_8:
            log_printf("8\n");
            break;
        default:
            log_printf("Unknown");
            break;
        }


        qmc.setRange(QMC5883P_RANGE_8G);
        qmc5883p_range_t currentRange = qmc.getRange();
        log_printf("Range: ");
        switch (currentRange)
        {
        case QMC5883P_RANGE_30G:
            log_printf("±30G\n");
            break;
        case QMC5883P_RANGE_12G:
            log_printf("±12G\n");
            break;
        case QMC5883P_RANGE_8G:
            log_printf("±8G\n");
            break;
        case QMC5883P_RANGE_2G:
            log_printf("±2G\n");
            break;
        default:
            log_printf("Unknown\n");
            break;
        }


        qmc.setSetResetMode(QMC5883P_SETRESET_ON);
        qmc5883p_setreset_t currentSetReset = qmc.getSetResetMode();
        log_printf("Set/Reset Mode: ");
        switch (currentSetReset)
        {
        case QMC5883P_SETRESET_ON:
            log_printf("Set and Reset On\n");
            break;
        case QMC5883P_SETRESET_SETONLY:
            log_printf("Set Only On\n");
            break;
        case QMC5883P_SETRESET_OFF:
            log_printf("Set and Reset Off\n");
            break;
        default:
            log_printf("Unknown\n");
            break;
        }
    }

    pinMode(QMC5883P_SDA, INPUT);
    pinMode(QMC5883P_SCL, INPUT);

    pinMode(SPEAKER_EN, OUTPUT);
    digitalWrite(SPEAKER_EN, HIGH);
    pinMode(SPEAKER_EN_2, OUTPUT);
    digitalWrite(SPEAKER_EN_2, HIGH);

    IIS_Bus->begin(nrf_i2s_ratio_t ::NRF_I2S_RATIO_32X, SAMPLE_RATE, nrf_i2s_swidth_t::NRF_I2S_SWIDTH_16BIT, nrf_i2s_channels_t::NRF_I2S_CHANNELS_STEREO);

    Window_Init(Current_Window);

    music_play_welcome();

    Buzzer_Trigger(150);

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        Led[i]->setPixelColor(0, Led[i]->Color(0, 0, 0));
        Led[i]->show();
    }
}

void loop()
{
    static uint16_t hue = 0;

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {

        uint32_t color = Led[i]->ColorHSV(hue, 255, 50);
        Led[i]->setPixelColor(0, color);
        Led[i]->show();
    }


    hue += 256;
    if (hue >= 65535)
    {
        hue = 0;
    }

    if (digitalRead(KEY_1) == LOW)
    {
        if (Iis_Transmit_Flag == false)
        {
            delay(300);

            printf("music play start\n");

            Iis_Send_Data_Size = 0;
            Iis_Data_Convert(c2_b16_s44100_3, Iis_Tx_Buffer, 0, MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t));

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
        }

        switch (Current_Window)
        {
        case System_Window::HOME:
            System_Op.sleep_count = 0;
            break;
        case System_Window::BATTERY_TEST:


            Battery_Control_Switch = !Battery_Control_Switch;
            if (Battery_Control_Switch == true)
            {
                digitalWrite(BATTERY_MEASUREMENT_CONTROL, HIGH);
                log_printf("turn on battery voltage measurement\n");
            }
            else
            {
                digitalWrite(BATTERY_MEASUREMENT_CONTROL, LOW);
                log_printf("turn off battery voltage measurement\n");
            }
            break;

        default:
            break;
        }
    }

    if (Iis_Transmit_Flag == true)
    {
        if (Iis_Send_Data_Size >= sizeof(c2_b16_s44100_3))
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
                size_t buffer_length = min(MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t), sizeof(c2_b16_s44100_3) - Iis_Send_Data_Size);




                if (buffer_length != MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t))
                {
                    memset(Iis_Tx_Buffer, 0, MAX_IIS_DATA_TRANSMIT_SIZE * sizeof(uint32_t));
                }
                Iis_Data_Convert(c2_b16_s44100_3, Iis_Tx_Buffer, Iis_Send_Data_Size, buffer_length);

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

    if (Key_Scanning() == true)
    {
        switch (Button_Triggered_OP.current_state)
        {
        case Button_Triggered_OP.gesture::SINGLE_CLICK:

            switch (Current_Window)
            {
            case System_Window::HOME:
                System_Op.sleep_count = 0;
                break;

            default:
                break;
            }

            break;
        case Button_Triggered_OP.gesture::DOUBLE_CLICK:
            if (Current_Window == System_Window::HOME)
            {
                display.clearDisplay();
                display.setCursor(0, 10);

                display.printf("power off");
                display.display();


                log_printf("power off\n");

                delay(1000);

                System_Sleep(true);

                systemOff(KEY_1, LOW);
            }
            break;
        case Button_Triggered_OP.gesture::LONG_PRESS:
            Window_End(Current_Window);

            Current_Window_Count++;
            if (Current_Window_Count >= (uint8_t)System_Window::END)
            {
                Current_Window_Count = 0;
            }

            Current_Window = (System_Window)Current_Window_Count;

            Window_Init(Current_Window);
            break;

        default:
            break;
        }
    }

    switch (Current_Window)
    {
    case System_Window::HOME:
    {
        if (millis() > CycleTime)
        {
            System_Op.time = millis();

            display.clearDisplay();
            display.setCursor(20, 0);
            display.printf("T-E-C");
            log_printf("T-Echo-Card\n");

            display.setCursor(10, 15);
            uint32_t total_seconds = System_Op.time / 1000;
            uint8_t hours = total_seconds / 3600;
            uint8_t minutes = (total_seconds % 3600) / 60;
            uint8_t seconds = total_seconds % 60;
            display.printf("%02d:%02d:%02d", hours, minutes, seconds);
            log_printf("system time: %02d:%02d:%02d\n", hours, minutes, seconds);

            display.display();

            System_Op.sleep_count++;
            if (System_Op.sleep_count > 10)
            {
                display.clearDisplay();
                display.setCursor(0, 10);
                display.printf("light sleep");
                display.display();

                log_printf("light sleep\n");

                delay(1000);

                System_Sleep(true);

                while (1)
                {
                    waitForEvent();

                    delay(1000);

                    if (digitalRead(KEY_1) == LOW)
                    {
                        System_Sleep(false);

                        display.clearDisplay();
                        display.setCursor(10, 10);
                        display.printf("wake up");
                        display.display();

                        log_printf("wake up\n");

                        Window_Init(Current_Window);

                        Buzzer_Trigger(150);

                        delay(1000);
                        break;
                    }
                }
            }

            waitForEvent();

            CycleTime = millis() + 1000;
        }
        break;
    }
    case System_Window::FLASH_TEST:
        if (millis() > CycleTime)
        {
            display.clearDisplay();
            display.setCursor(0, 0);

            if (System_Op.init_flag.flash == true)
            {
                display.printf("Flash Y");
                display.setCursor(0, 10);
                display.printf("%#X", flash.getJEDECID());
                display.setCursor(0, 20);
                display.printf("%dkbytes", flash.size() / 1024);

                log_printf("flash init successful\n");
                log_printf("id: %#X\n", flash.getJEDECID());
                log_printf("size: %d kbytes\n", flash.size() / 1024);
            }
            else
            {
                display.printf("Flash N");

                log_printf("flash init fail\n");
            }

            display.display();

            CycleTime = millis() + 1000;
        }
        break;

    case System_Window::BATTERY_TEST:

        if (millis() > CycleTime)
        {
            float battery_voltage = (((float)analogRead(BATTERY_ADC_DATA) * ((3000.0 / 4096.0))) / 1000.0) * 2.0;

            log_printf("---battery---\n");

            display.clearDisplay();
            display.setCursor(0, 0);
            display.printf("Battery");
            display.setCursor(0, 10);
            if (Battery_Control_Switch == true)
            {
                display.printf("Switch:ON");
                log_printf("battery switch: on\n");
            }
            else
            {
                display.printf("Switch:OFF");
                log_printf("battery switch: off\n");
            }
            display.setCursor(0, 20);
            display.printf("%.03fv", battery_voltage);

            display.display();

            log_printf("battery voltage: %.03f v\n", battery_voltage);

            CycleTime = millis() + 1000;
        }

        break;
    case System_Window::IMU_TEST:
        if (millis() > CycleTime)
        {
            if (System_Op.init_flag.imu == true)
            {
                float gx, gy, gz;

                if (qmc.getGaussField(&gx, &gy, &gz))
                {
                    display.clearDisplay();
                    display.setCursor(0, 0);
                    display.printf("Imu Y");
                    display.setCursor(0, 10);







                    float B_horizontal_sq = (gx * gx) + (gy * gy);



                    float I_rad = atan2(gz, sqrt(B_horizontal_sq));



                    float dip_angle = I_rad * (180.0 / PI);


                    display.setCursor(0, 10);
                    display.printf("angle:%.1f deg", dip_angle);

                    display.display();

                    log_printf("imu init successful\n");
                    log_printf("gx = %.6f  |  gy = %.6f  |  gz = %.6f | dip angle:%.1f deg\n", gx, gy, gz, dip_angle);
                }
                else
                {
                    display.clearDisplay();
                    display.setCursor(0, 0);
                    display.printf("Imu E");

                    display.display();

                    log_printf("imu get data fail\n");
                }

                CycleTime = millis() + 500;
            }
            else
            {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.printf("Imu N");

                display.display();

                log_printf("imu init fail\n");

                CycleTime = millis() + 1000;
            }
        }
        break;

    case System_Window::GPS_TEST:

        while (Serial2.available() && Uart_Rx_Count < MAX_UART_RX_BUFFER_SIZE)
        {
            Uart_Rx_Buffer[Uart_Rx_Count++] = Serial2.read();
        }

        if (millis() > CycleTime)
        {
            if (Gps_Positioning_Flag == false)
            {
                Gps_Positioning_Time++;
            }


            if (Uart_Rx_Count >= MAX_UART_RX_BUFFER_SIZE)
            {
                Uart_Rx_Buffer[Uart_Rx_Count] = '\0';


                log_printf("---begin---\n%s \n---end---\n", Uart_Rx_Buffer);

                log_printf("---RMC---\n");


                Cpp_Bus_Driver::Gnss::Rmc rmc;

                display.clearDisplay();
                display.setCursor(0, 0);

                if (Gps_Positioning_Flag == false)
                {
                    display.printf("Gps N:%ds", Gps_Positioning_Time);
                    log_printf("Gps N:%ds\n", Gps_Positioning_Time);
                }
                else
                {
                    display.printf("Gps Y:%d s", Gps_Positioning_Time);
                    log_printf("Gps Y:%d s\n", Gps_Positioning_Time);
                }


                if (Nrf52840_Gnss->parse_rmc_info(Uart_Rx_Buffer, Uart_Rx_Count, rmc) == true)
                {
                    log_printf("location status: %s\n", (rmc.location_status).c_str());

                    if (rmc.data.update_flag == true)
                    {
                        log_printf("utc data: %d/%d/%d\n", rmc.data.year + 2000, rmc.data.month, rmc.data.day);
                        rmc.data.update_flag = false;
                    }
                    if (rmc.utc.update_flag == true)
                    {
                        log_printf("utc time: %d:%d:%.03f\n", rmc.utc.hour, rmc.utc.minute, rmc.utc.second);
                        log_printf("china time: %d:%d:%.03f\n", (rmc.utc.hour + 8 + 24) % 24, rmc.utc.minute, rmc.utc.second);
                        rmc.utc.update_flag = false;
                    }

                    if ((rmc.location.lat.update_flag == true) && (rmc.location.lat.direction_update_flag == true))
                    {
                        log_printf("location lat degrees: %d \nlocation lat minutes: %.10lf \nlocation lat degrees_minutes: %.10lf \nlocation lat direction: %s\n",
                                   rmc.location.lat.degrees, rmc.location.lat.minutes, rmc.location.lat.degrees_minutes, (rmc.location.lat.direction).c_str());
                        rmc.location.lat.update_flag = false;
                        rmc.location.lat.direction_update_flag = false;

                        display.setCursor(0, 10);
                        display.printf("%.6f", rmc.location.lat.degrees_minutes);

                        Gps_Positioning_Flag = true;
                    }
                    if ((rmc.location.lon.update_flag == true) && (rmc.location.lon.direction_update_flag == true))
                    {
                        log_printf("location lon degrees: %d \nlocation lon minutes: %.10lf \nlocation lon degrees_minutes: %.10lf \nlocation lon direction: %s\n",
                                   rmc.location.lon.degrees, rmc.location.lon.minutes, rmc.location.lon.degrees_minutes, (rmc.location.lon.direction).c_str());
                        rmc.location.lon.update_flag = false;
                        rmc.location.lon.direction_update_flag = false;

                        display.setCursor(0, 20);
                        display.printf("%.6f", rmc.location.lon.degrees_minutes);

                        Gps_Positioning_Flag = true;
                    }

                    display.display();
                }
                else
                {
                    display.printf("invalid");
                    display.setCursor(0, 20);
                    display.printf("invalid");
                    display.display();

                    log_printf("gps data: read fail\n");
                }

                Uart_Rx_Count = 0;
            }
            else
            {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.printf("Gps E:%ds", Gps_Positioning_Time);
                display.display();

                log_printf("Gps E:%ds\n", Gps_Positioning_Time);
            }

            CycleTime = millis() + 3000;
        }

        break;

    case System_Window::MICROPHONE_TEST:
        if (millis() > CycleTime)
        {
            if (Pdm_Rx_Stream.size() >= 2)
            {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.printf("Mic Y");
                display.setCursor(0, 10);
                display.printf("size: %d", Pdm_Rx_Stream.size());
                display.setCursor(0, 20);
                display.printf("left: %d", Pdm_Rx_Stream[0]);
                display.setCursor(0, 30);
                display.printf("right: %d", Pdm_Rx_Stream[1]);

                display.display();

                log_printf("Pdm_Rx_Stream size: %d\n", Pdm_Rx_Stream.size());

                log_printf("left: %d\n", Pdm_Rx_Stream[0]);

                log_printf("right: %d\n", Pdm_Rx_Stream[1]);

                Pdm_Rx_Stream.erase(Pdm_Rx_Stream.begin(), Pdm_Rx_Stream.begin() + MAX_PDM_DATA_TRANSMIT_SIZE);
            }

            CycleTime = millis() + 500;
        }
        break;

    default:
        break;
    }

    delay(10);
}