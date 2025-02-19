/**
 * @file common.hpp
 * @brief Header file containing common definitions and configurations.
 */
#ifndef common_hpp
#define common_hpp

// ====================================================== //
// ================= Generic inclusions ================= //
// ====================================================== //

#include <Arduino.h>
#include <numeric>

// ====================================================== //
// =============== Shared objects headers =============== //
// ====================================================== //
// Include header that contains definitions of global objects, shared between multiple files.

#include "libDM_timer_tool.hpp"
#include "libDM_stream_logger.hpp"

// ##################################################################### //
// ####################### Variables declarations ###################### //
// ##################################################################### //

// ====================================================== //
// =================== Global objects =================== //
// ====================================================== //
// Place here global objects that are shared between multiple files.

extern timerTool myTimer;
extern streamLogger streamObj;

// ====================================================== //
// ================== Pins definitions ================== //
// ====================================================== //
// Define here pins assignements that are used in multiple files.

/**
 * @brief Configuration for pin assignments.
 */
struct PIN_CONFIG
{
    uint8_t pinSS    = SS;
    uint8_t pinMmcSS = 11U;
    uint8_t pinLpsSS = 9U;
    uint8_t pinMosi  = 12U;
    uint8_t pinMiso  = 13U;
    uint8_t pinClk   = 14U;

    uint8_t pinSdioClk  = 38U;
    uint8_t pinSdioDat0 = 39U;
    uint8_t pinSdioDat1 = 42U;
    uint8_t pinSdioDat2 = 21U;
    uint8_t pinSdioDat3 = 41U;
    uint8_t pinSdioCmd  = 40U;

    uint8_t pinGpsRx = 8;
    uint8_t pinGpsTx = 18;

    uint8_t pinVn200Rx = 17;
    uint8_t pinVn200Tx = 16;

    gpio_num_t pinCanRx = GPIO_NUM_1;
    gpio_num_t pinCanTx = GPIO_NUM_2;

    uint8_t pinProtocolRx = 5;
    uint8_t pinProtocolTx = 4;
};

extern const PIN_CONFIG pinConf;

// ====================================================== //
// =================== Configurations =================== //
// ====================================================== //
// Place here common configurations that are used in multiple files.

/**
 * @brief Configuration parameters for the application.
 */
struct GENERAL_CONFIG
{
    uint32_t sdioClock         = 40000U; // [kHz]
    const char* sdioMountpoint = "/sdcard";

    uint32_t icmSpiClock = 24000000U; // [Hz]

    uint32_t mmcSpiClock = 1000000U; // [Hz]

    uint32_t vn200Baudrate = 921600U; // [bps]

    uint32_t gpsBaudrate = 230400U; // [bps]
};

extern const GENERAL_CONFIG generalConf;

#endif
