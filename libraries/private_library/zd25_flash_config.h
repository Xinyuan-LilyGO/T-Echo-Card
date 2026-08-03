#pragma once

#include "Adafruit_SPIFlash.h"

SPIFlash_Device_t ZD25WQ32_DEVICES[] =
{
    // ZD25WQ32C, JEDEC ID: BA 60 16
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
    },
    // ZD25Q32D, JEDEC ID: BA 40 16
    {
        total_size : (1UL << 22), /* 4 MiB */
        start_up_time_us : 12000,
        manufacturer_id : 0xBA,
        memory_type : 0x40,
        capacity : 0x16,
        max_clock_speed_mhz : 133,
        quad_enable_bit_mask : 0x02,
        has_sector_protection : false,
        supports_fast_read : true,
        supports_qspi : true,
        supports_qspi_writes : true,
        write_status_register_split : true,
        single_status_byte : false,
        is_fram : false,
    },
};

constexpr size_t ZD25WQ32_DEVICE_COUNT =
    sizeof(ZD25WQ32_DEVICES) / sizeof(ZD25WQ32_DEVICES[0]);

