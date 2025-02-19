#ifndef lps22hb_regs_enums_hpp
#define lps22hb_regs_enums_hpp
#include <Arduino.h>

// ====================================================== //
// ======================== ENUMS ======================= //
// ====================================================== //
namespace LPS22HB_ENUM
{
enum class ODR : uint8_t
{
    _75HZ = 75,
    _50HZ = 50,
    _25HZ = 25,
    _10HZ = 10,
    _1HZ  = 1,
};

enum class FILTER_BW : uint8_t
{
    ODR_DIV_2  = 2,  // BW=ODR/2
    ODR_DIV_9  = 9,  // BW=ODR/9
    ODR_DIV_20 = 20, // BW=ODR/20
};

enum class STATUS : uint8_t
{
    TEMP_READY = 0,
    PRESSURE_READY,
    ALL_READY,
    NO_MEAS
};

enum class FIFO_MODE : uint8_t
{
    BYPASS = 0,
    FIFO,
    STREAM,
    DYNAMIC_STREAM,
    STREAM_TO_FIFO,
    BYPASS_TO_STREAM,
    BYPASS_TO_FIFO,
    FIFO_OFF
};
} // namespace LPS22HB_ENUM

// ====================================================== //
// ====================== REGISTERS ===================== //
// ====================================================== //
struct LPS22HB_REGISTERS
{
    uint8_t READ_FLAG = 0x80;

    uint8_t INTERRUPT_CFG = 0x0B;
    uint8_t THS_P_L       = 0x0C;
    uint8_t THS_P_H       = 0x0D;
    uint8_t WHO_AM_I      = 0x0F;
    uint8_t CTRL_REG1     = 0x10;
    uint8_t CTRL_REG2     = 0x11;
    uint8_t CTRL_REG3     = 0x12;
    uint8_t FIFO_CTRL     = 0x14;
    uint8_t REF_P_XL      = 0x15;
    uint8_t REF_P_L       = 0x16;
    uint8_t REF_P_H       = 0x17;
    uint8_t RPDS_L        = 0x18;
    uint8_t RPDS_H        = 0x19;
    uint8_t RES_CONF      = 0x1A;
    uint8_t INT_SOURCE    = 0x25;
    uint8_t FIFO_STATUS   = 0x26;
    uint8_t STATUS        = 0x27;
    uint8_t PRESS_OUT_XL  = 0x28;
    uint8_t PRESS_OUT_L   = 0x29;
    uint8_t PRESS_OUT_H   = 0x2A;
    uint8_t TEMP_OUT_L    = 0x2B;
    uint8_t TEMP_OUT_H    = 0x2C;
    uint8_t LPFP_RES      = 0x33; // A read on this register before generating pressure measurements
                                  // causes the content of the internal filter to be discarded
};

// ====================================================== //
// ================== REGISTERS VALUES ================== //
// ====================================================== //
struct LPS22HB_REGISTER_VALUES
{
    uint8_t ID = 0xB1;

    struct
    {
        uint8_t ODR_POWER_DOWN_ONE_SHOT = 0b00000000U;
        uint8_t ODR_1HZ                 = 0b00010000U;
        uint8_t ODR_10HZ                = 0b00100000U;
        uint8_t ODR_25HZ                = 0b00110000U;
        uint8_t ODR_50HZ                = 0b01000000U;
        uint8_t ODR_75HZ                = 0b01010000U;
        uint8_t ODR_MASK                = 0b10001111U;
        uint8_t LPF_DISABLE             = 0b00000000U;
        uint8_t LPF_ODR_DIV_9           = 0b00001000U;
        uint8_t LPF_ODR_DIV_20          = 0b00001100U;
        uint8_t LPF_MASK                = 0b11110011U;
    } CTRL_REG1;

    struct
    {
        uint8_t FIFO_ENABLE      = 0b01000000U;
        uint8_t FIFO_ENABLE_MASK = 0b10111111U;
        uint8_t SW_RESET         = 0b00000100U;
        uint8_t SW_RESET_MASK    = 0b11111011U;
    } CTRL_REG2;

    struct
    {
        uint8_t FIFO_MODE_BYPASS           = 0b00000000U;
        uint8_t FIFO_MODE_FIFO             = 0b00100000U;
        uint8_t FIFO_MODE_STREAM           = 0b01000000U;
        uint8_t FIFO_MODE_STREAM_TO_FIFO   = 0b01100000U;
        uint8_t FIFO_MODE_BYPASS_TO_STREAM = 0b10000000U;
        uint8_t FIFO_MODE_DYNAMIC_STREAM   = 0b11000000U;
        uint8_t FIFO_MODE_BYPASS_TO_FIFO   = 0b11100000U;
        uint8_t FIFO_MODE_MASK             = 0b00011111U;
    } FIFO_CTRL;

    struct
    {
        uint8_t FIFO_SIZE_MASK = 0b00111111U;
    } FIFO_STATUS;

    struct
    {
        uint8_t TEMPERATURE_AVAILABLE = 0b00000010U;
        uint8_t PRESSURE_AVAILABLE    = 0b00000001U;
        uint8_t ALL_AVAILABLE         = 0b00000011U;
    } STATUS; // sensor readings status
};

#endif