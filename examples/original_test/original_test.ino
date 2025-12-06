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
#include "Adafruit_NeoPixel.h"
#include "c2_b16_s44100_2.h"
#include "c2_b16_s44100_3.h"
#include "PDM.h"
#include <vector>
#include "ICM20948_WE.h"

#define SOFTWARE_NAME "original_test"
#define SOFTWARE_LASTEDITTIME "202512061012"
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

struct BLE_Uart_Operator
{
    using state = enum {
        UNCONNECTED, // not connected
        CONNECTED,   // connected already
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
        NOT_ACTIVE,   // not active
        SINGLE_CLICK, // single click
        DOUBLE_CLICK, // double click
        LONG_PRESS,   // long press
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

// 已经发送的数据大小
size_t Iis_Send_Data_Size = 0;
bool Iis_Transmit_Flag = false;

bool Iis_Data_Convert_Wait = false;

// 用于存储rdm接收流的容器
std::vector<int16_t> Pdm_Rx_Stream;

uint8_t Uart_Rx_Buffer[MAX_UART_RX_BUFFER_SIZE] = {0};
size_t Uart_Rx_Count = 0;

/* UART Serivce: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * UART RXD    : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 * UART TXD    : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 */

// BLE Service
BLEDfu bledfu;   // OTA DFU service
BLEDis bledis;   // device information
BLEUart bleuart; // uart over ble

BLE_Uart_Operator BLE_Uart_Op;
System_Operator System_Op;
Button_Triggered_Operator Button_Triggered_OP;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, SCREEN_RST);

// // SPI
// SPIClass Custom_SPI(NRF_SPIM3, ZD25WQ32C_MISO, ZD25WQ32C_SCLK, ZD25WQ32C_MOSI);
// Adafruit_FlashTransport_SPI flashTransport(ZD25WQ32C_CS, Custom_SPI);

// QSPI
Adafruit_FlashTransport_QSPI flashTransport(ZD25WQ32C_SCLK, ZD25WQ32C_CS,
                                            ZD25WQ32C_IO0, ZD25WQ32C_IO1,
                                            ZD25WQ32C_IO2, ZD25WQ32C_IO3);

Adafruit_SPIFlash flash(&flashTransport);

ICM20948_WE myIMU = ICM20948_WE(&Wire, ICM20948_ADDRESS);

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
        Button_Triggered_OP.cycletime_2 = millis() + 1000; // timing 1000ms off
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

                if ((Button_Triggered_OP.paragraph_triggered_count == 2)) // single click
                {
                    Button_Triggered_OP.current_state = Button_Triggered_OP.gesture::SINGLE_CLICK;
                    log_printf("key triggered: SINGLE_CLICK\n");
                    Buzzer_Trigger(150);
                    delay(500);
                    return true;
                }
                else if ((Button_Triggered_OP.paragraph_triggered_count == 4)) // double click
                {
                    Button_Triggered_OP.current_state = Button_Triggered_OP.gesture::DOUBLE_CLICK;
                    log_printf("key triggered: DOUBLE_CLICK\n");
                    Buzzer_Trigger(150);
                    delay(300);
                    Buzzer_Trigger(150);
                    delay(500);
                    return true;
                }
                else if ((Button_Triggered_OP.paragraph_triggered_count == 1)) // long press
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

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
    // Get the reference to current connection
    BLEConnection *connection = Bluefruit.Connection(conn_handle);

    char central_name[32] = {0};
    connection->getPeerName(central_name, sizeof(central_name));

    Serial.print("Connected to ");
    Serial.println(central_name);

    strcpy(BLE_Uart_Op.connection.device_name, central_name);

    BLE_Uart_Op.connection.state_flag = BLE_Uart_Op.state::CONNECTED;
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
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
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    // Include bleuart 128-bit uuid
    Bluefruit.Advertising.addService(bleuart);

    // Secondary Scan Response packet (optional)
    // Since there is no room for 'Name' in Advertising packet
    Bluefruit.ScanResponse.addName();

    /* Start Advertising
     * - Enable auto advertising if disconnected
     * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     * - Timeout for fast mode is 30 seconds
     * - Start(timeout) with timeout = 0 will advertise forever (until connected)
     *
     * For recommended advertising interval
     * https://developer.apple.com/library/content/qa/qa1931/_index.html
     */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
    Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

bool BLE_Uart_Initialization(void)
{
    // Setup the BLE LED to be enabled on CONNECT
    // Note: This is actually the default behavior, but provided
    // here in case you want to control this LED manually via PIN 19
    // Bluefruit.autoConnLed(true);

    // Config the peripheral connection with maximum bandwidth
    // more SRAM required by SoftDevice
    // Note: All config***() function must be called before begin()
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

    if (Bluefruit.begin() == false)
    {
        Serial.println("BLE initialization failed");
        return false;
    }
    Serial.println("BLE initialization successful");

    Bluefruit.setTxPower(8); // Check bluefruit.h for supported values
    // Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // To be consistent OTA DFU should be added first if it exists
    bledfu.begin();

    // Configure and Start Device Information Service
    bledis.setManufacturer("LILYGO Industries");
    bledis.setModel("T-Impulse-Plus");
    bledis.begin();

    // Configure and Start BLE Uart Service
    bleuart.begin();

    // Set up and start advertising
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
        // Custom_SPI.setClockDivider(SPI_CLOCK_DIV2); // dual frequency 32MHz

        flash.begin();
        flashTransport.setClockSpeed(32000000UL, 0);
        flashTransport.runCommand(0xAB); // Exit deep sleep mode

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
        // Measure battery
        pinMode(BATTERY_ADC_DATA, INPUT);
        pinMode(BATTERY_MEASUREMENT_CONTROL, OUTPUT);
        digitalWrite(BATTERY_MEASUREMENT_CONTROL, HIGH); // Turn on battery voltage measurement

        Battery_Control_Switch = true;

        // Set the analog reference to 3.0V (default = 3.6V)
        analogReference(AR_INTERNAL_3_0);
        // Set the resolution to 12-bit (0..4095)
        analogReadResolution(12); // Can be 8, 10, 12 or 14

        break;

    case System_Window::IMU_TEST:
        Wire.setPins(ICM20948_SDA, ICM20948_SCL);

        if (myIMU.init() == false)
        {
            log_printf("ICM20948 AG init fail\n");
            System_Op.init_flag.imu = false;
        }
        else
        {
            myIMU.sleep(false);
            if (myIMU.initMagnetometer() == false)
            {
                log_printf("ICM20948 M init fail\n");
                System_Op.init_flag.imu = false;
            }
            else
            {
                log_printf("ICM20948 init successful\n");
                System_Op.init_flag.imu = true;

                /*  This is a method to calibrate. You have to determine the minimum and maximum
                 *  raw acceleration values of the axes determined in the range +/- 2 g.
                 *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
                 *  The parameters are floats.
                 *  The calibration changes the slope / ratio of raw acceleration vs g. The zero point
                 *  is set as (min + max)/2.
                 */
                // myIMU.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16520.0, 16690.0);

                /* The starting point, if you position the ICM20948 flat, is not necessarily 0g/0g/1g for x/y/z.
                 * The autoOffset function measures offset. It assumes your ICM20948 is positioned flat with its
                 * x,y-plane. The more you deviate from this, the less accurate will be your results.
                 * It overwrites the zero points of setAccOffsets, but keeps the correction of the slope.
                 * The function also measures the offset of the gyroscope data. The gyroscope offset does not
                 * depend on the positioning.
                 * This function needs to be called after setAccsOffsets but before other settings since it will
                 * overwrite settings!
                 * You can query the offsets with the functions:
                 * xyzFloat getAccOffsets() and xyzFloat getGyrOffsets()
                 * You can apply the offsets using:
                 * setAccOffsets(xyzFloat yourOffsets) and setGyrOffsets(xyzFloat yourOffsets)
                 */
                log_printf("position your icm20948 flat and don't move it - calibrating...\n");
                delay(1000);
                myIMU.autoOffsets();
                log_printf("done!\n");

                /* enables or disables the acceleration sensor, default: enabled */
                // myIMU.enableAcc(true);

                /*  ICM20948_ACC_RANGE_2G      2 g   (default)
                 *  ICM20948_ACC_RANGE_4G      4 g
                 *  ICM20948_ACC_RANGE_8G      8 g
                 *  ICM20948_ACC_RANGE_16G    16 g
                 */
                myIMU.setAccRange(ICM20948_ACC_RANGE_2G);

                /*  Choose a level for the Digital Low Pass Filter or switch it off.
                 *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF
                 *
                 *  IMPORTANT: This needs to be ICM20948_DLPF_7 if DLPF is used in cycle mode!
                 *
                 *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
                 *    0              246.0               1125/(1+ASRD)
                 *    1              246.0               1125/(1+ASRD)
                 *    2              111.4               1125/(1+ASRD)
                 *    3               50.4               1125/(1+ASRD)
                 *    4               23.9               1125/(1+ASRD)
                 *    5               11.5               1125/(1+ASRD)
                 *    6                5.7               1125/(1+ASRD)
                 *    7              473.0               1125/(1+ASRD)
                 *    OFF           1209.0               4500
                 *
                 *    ASRD = Accelerometer Sample Rate Divider (0...4095)
                 *    You achieve lowest noise using level 6
                 */
                myIMU.setAccDLPF(ICM20948_DLPF_6);

                /*  Acceleration sample rate divider divides the output rate of the accelerometer.
                 *  Sample rate = Basic sample rate / (1 + divider)
                 *  It can only be applied if the corresponding DLPF is not off!
                 *  Divider is a number 0...4095 (different range compared to gyroscope)
                 *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
                 *  sample rate has priority.
                 */
                // myIMU.setAccSampleRateDivider(10);

                /* You can set the following modes for the magnetometer:
                 * AK09916_PWR_DOWN          Power down to save energy
                 * AK09916_TRIGGER_MODE      Measurements on request, a measurement is triggered by
                 *                           calling setMagOpMode(AK09916_TRIGGER_MODE)
                 * AK09916_CONT_MODE_10HZ    Continuous measurements, 10 Hz rate
                 * AK09916_CONT_MODE_20HZ    Continuous measurements, 20 Hz rate
                 * AK09916_CONT_MODE_50HZ    Continuous measurements, 50 Hz rate
                 * AK09916_CONT_MODE_100HZ   Continuous measurements, 100 Hz rate (default)
                 */
                myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
            }
        }

        break;

    case System_Window::GPS_TEST:
        Serial2.setPins(GPS_UART_TX, GPS_UART_RX);
        Serial2.begin(9600);

        digitalWrite(GPS_WAKE_UP, HIGH); // gps打开
        digitalWrite(GPS_RF_EN, HIGH);

        Gps_Positioning_Flag = false;
        Gps_Positioning_Time = -5;

        break;

    case System_Window::MICROPHONE_TEST:
        Pdm_Rx_Stream.clear();
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
        flashTransport.runCommand(0xB9); // Flash Deep Sleep
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
        digitalWrite(BATTERY_MEASUREMENT_CONTROL, LOW); // Turn off battery voltage measurement
        CycleTime = 0;
        break;
    case System_Window::IMU_TEST:
        myIMU.sleep(true);
        pinMode(ICM20948_SDA, INPUT);
        pinMode(ICM20948_SCL, INPUT);
        CycleTime = 0;
        break;
    case System_Window::GPS_TEST:
        Serial2.end();
        digitalWrite(GPS_WAKE_UP, LOW); // gps关闭
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

        // 停止广播
        Bluefruit.Advertising.stop();
    }
    else
    {
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

        IIS_Bus->begin(nrf_i2s_ratio_t ::NRF_I2S_RATIO_32X, SAMPLE_RATE, nrf_i2s_swidth_t::NRF_I2S_SWIDTH_16BIT, nrf_i2s_channels_t::NRF_I2S_CHANNELS_STEREO);

        pinMode(nRF52840_BOOT, INPUT_PULLUP);

        Serial.begin(115200);

        Bluefruit.Advertising.start(0); // 0 = Don't stop advertising after n seconds

        pinMode(BUZZER_DATA, OUTPUT);
        digitalWrite(BUZZER_DATA, LOW);

        pinMode(GPS_WAKE_UP, OUTPUT);
        digitalWrite(GPS_WAKE_UP, HIGH); // gps开启
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
}

void setup()
{
    Serial.begin(115200);

    // 3.3V Power ON
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
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
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
        delay(100); // wait for native usb
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
    digitalWrite(GPS_WAKE_UP, HIGH); // gps开启
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
    flashTransport.runCommand(0xAB); // Exit deep sleep mode
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
    flashTransport.runCommand(0xB9); // Flash Deep Sleep
    flash.end();
    pinMode(ZD25WQ32C_SCLK, INPUT);
    pinMode(ZD25WQ32C_CS, INPUT);
    pinMode(ZD25WQ32C_IO0, INPUT);
    pinMode(ZD25WQ32C_IO1, INPUT);
    pinMode(ZD25WQ32C_IO2, INPUT);
    pinMode(ZD25WQ32C_IO3, INPUT);

    Wire.setPins(ICM20948_SDA, ICM20948_SCL);
    if (myIMU.init() == false)
    {
        log_printf("ICM20948 AG init fail\n");
        System_Op.init_flag.imu = false;
    }
    else
    {
        myIMU.sleep(false);
        if (myIMU.initMagnetometer() == false)
        {
            log_printf("ICM20948 M init fail\n");
            System_Op.init_flag.imu = false;
        }
        else
        {
            log_printf("ICM20948 init successful\n");
            System_Op.init_flag.imu = true;

            /*  This is a method to calibrate. You have to determine the minimum and maximum
             *  raw acceleration values of the axes determined in the range +/- 2 g.
             *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
             *  The parameters are floats.
             *  The calibration changes the slope / ratio of raw acceleration vs g. The zero point
             *  is set as (min + max)/2.
             */
            // myIMU.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16520.0, 16690.0);

            /* The starting point, if you position the ICM20948 flat, is not necessarily 0g/0g/1g for x/y/z.
             * The autoOffset function measures offset. It assumes your ICM20948 is positioned flat with its
             * x,y-plane. The more you deviate from this, the less accurate will be your results.
             * It overwrites the zero points of setAccOffsets, but keeps the correction of the slope.
             * The function also measures the offset of the gyroscope data. The gyroscope offset does not
             * depend on the positioning.
             * This function needs to be called after setAccsOffsets but before other settings since it will
             * overwrite settings!
             * You can query the offsets with the functions:
             * xyzFloat getAccOffsets() and xyzFloat getGyrOffsets()
             * You can apply the offsets using:
             * setAccOffsets(xyzFloat yourOffsets) and setGyrOffsets(xyzFloat yourOffsets)
             */
            log_printf("position your icm20948 flat and don't move it - calibrating...\n");
            delay(1000);
            myIMU.autoOffsets();
            log_printf("done!\n");

            /* enables or disables the acceleration sensor, default: enabled */
            // myIMU.enableAcc(true);

            /*  ICM20948_ACC_RANGE_2G      2 g   (default)
             *  ICM20948_ACC_RANGE_4G      4 g
             *  ICM20948_ACC_RANGE_8G      8 g
             *  ICM20948_ACC_RANGE_16G    16 g
             */
            myIMU.setAccRange(ICM20948_ACC_RANGE_2G);

            /*  Choose a level for the Digital Low Pass Filter or switch it off.
             *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF
             *
             *  IMPORTANT: This needs to be ICM20948_DLPF_7 if DLPF is used in cycle mode!
             *
             *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
             *    0              246.0               1125/(1+ASRD)
             *    1              246.0               1125/(1+ASRD)
             *    2              111.4               1125/(1+ASRD)
             *    3               50.4               1125/(1+ASRD)
             *    4               23.9               1125/(1+ASRD)
             *    5               11.5               1125/(1+ASRD)
             *    6                5.7               1125/(1+ASRD)
             *    7              473.0               1125/(1+ASRD)
             *    OFF           1209.0               4500
             *
             *    ASRD = Accelerometer Sample Rate Divider (0...4095)
             *    You achieve lowest noise using level 6
             */
            myIMU.setAccDLPF(ICM20948_DLPF_6);

            /*  Acceleration sample rate divider divides the output rate of the accelerometer.
             *  Sample rate = Basic sample rate / (1 + divider)
             *  It can only be applied if the corresponding DLPF is not off!
             *  Divider is a number 0...4095 (different range compared to gyroscope)
             *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
             *  sample rate has priority.
             */
            // myIMU.setAccSampleRateDivider(10);

            /* You can set the following modes for the magnetometer:
             * AK09916_PWR_DOWN          Power down to save energy
             * AK09916_TRIGGER_MODE      Measurements on request, a measurement is triggered by
             *                           calling setMagOpMode(AK09916_TRIGGER_MODE)
             * AK09916_CONT_MODE_10HZ    Continuous measurements, 10 Hz rate
             * AK09916_CONT_MODE_20HZ    Continuous measurements, 20 Hz rate
             * AK09916_CONT_MODE_50HZ    Continuous measurements, 50 Hz rate
             * AK09916_CONT_MODE_100HZ   Continuous measurements, 100 Hz rate (default)
             */
            myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
        }
    }
    myIMU.sleep(true);
    pinMode(ICM20948_SDA, INPUT);
    pinMode(ICM20948_SCL, INPUT);

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
    static uint16_t hue = 0; // HSV色调值（0-65535）

    for (uint8_t i = 0; i < sizeof(Led) / sizeof(*Led); i++)
    {
        // 使用HSV颜色模式，亮度固定为50（val=50）
        uint32_t color = Led[i]->ColorHSV(hue, 255, 50);
        Led[i]->setPixelColor(0, color);
        Led[i]->show();
    }

    // 每次增加色调值，实现颜色渐变
    hue += 256; // 调整这个值可以控制颜色变化速度
    if (hue >= 65535)
    {
        hue = 0; // 循环色调值
    }

    if (digitalRead(KEY_1) == LOW)
    {
        if (Iis_Transmit_Flag == false)
        {
            delay(300);
            // 播放音乐测试
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
            // delay(300);

            Battery_Control_Switch = !Battery_Control_Switch;
            if (Battery_Control_Switch == true)
            {
                digitalWrite(BATTERY_MEASUREMENT_CONTROL, HIGH); // Enable battery voltage measurement
                log_printf("turn on battery voltage measurement\n");
            }
            else
            {
                digitalWrite(BATTERY_MEASUREMENT_CONTROL, LOW); // Turn off battery voltage measurement
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

                // printf("iis_send_data_size: %d\n", Iis_Send_Data_Size);
                // printf("iis send data length: %d\n", buffer_length);

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
                // display.printf("deep sleep");
                display.printf("power off");
                display.display();

                // log_printf("deep sleep\n");
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

            log_printf(("[T-Echo-Card_" + (String)BOARD_VERSION "][" + (String)SOFTWARE_NAME +
                        "]_firmware_" + (String)SOFTWARE_LASTEDITTIME + "\n")
                           .c_str());

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

                while (1) // 开始睡眠
                {
                    waitForEvent();
                    // systemOff(KEY_1, LOW);
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
                myIMU.readSensor();
                xyzFloat gValue;
                myIMU.getGValues(&gValue);
                xyzFloat angle;
                myIMU.getAngles(&angle);
                float pitch = myIMU.getPitch();
                float roll = myIMU.getRoll();

                // obtain the x and y values of the magnetometer to calculate the heading angle (Yaw)
                xyzFloat magValues;
                myIMU.getMagValues(&magValues);
                float yaw = atan2(magValues.y, magValues.x) * (180.0 / M_PI); // calculate heading angle

                display.clearDisplay();
                display.setCursor(0, 0);
                display.printf("Imu Y p:%.01f", pitch);
                display.setCursor(0, 10);
                display.printf("r:%.01f", roll);
                display.setCursor(0, 20);
                display.printf("y:%.01f", yaw);

                display.display();

                log_printf("imu init successful\n");
                log_printf("pitch = %.6f  |  roll = %.6f  |  yaw = %.6f\n", pitch, roll, yaw);

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
            Uart_Rx_Buffer[Uart_Rx_Count] = Serial2.read();
            Uart_Rx_Count++;
        }

        if (millis() > CycleTime)
        {
            if (Gps_Positioning_Flag == false)
            {
                Gps_Positioning_Time += 5;
            }

            // 检查Serial2是否有可用数据
            if (Uart_Rx_Count >= MAX_UART_RX_BUFFER_SIZE)
            {
                Uart_Rx_Buffer[MAX_UART_RX_BUFFER_SIZE - 1] = '\0';

                // 打印RMC的相关信息
                log_printf("---begin---\n%s \n---end---\n", Uart_Rx_Buffer);

                log_printf("---RMC---\n");

                // 创建Rmc对象用于存储解析结果
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

                // 调用parse_rmc_info进行解码
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

            CycleTime = millis() + 5000;
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
                // 输出左声道数据
                log_printf("left: %d\n", Pdm_Rx_Stream[0]);
                // 输出右声道数据
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
