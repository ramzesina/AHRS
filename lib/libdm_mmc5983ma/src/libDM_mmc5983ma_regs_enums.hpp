#ifndef mmc5983ma_regs_enums_hpp
#define mmc5983ma_regs_enums_hpp
#include <Arduino.h>

// ====================================================== //
// ======================== ENUMS ======================= //
// ====================================================== //
namespace MMC5983MA_ENUM
{
// NOLINTBEGIN(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
enum class ODR : uint16_t
{
    _OFF    = 0,
    _1HZ    = 1,
    _10HZ   = 10,
    _20HZ   = 20,
    _50HZ   = 50,
    _100HZ  = 100,
    _200HZ  = 200,
    _1000HZ = 1000
};

enum class PERIODIC_SET_TIME : uint16_t
{
    _OFF  = 0,
    _1    = 1,
    _25   = 25,
    _75   = 75,
    _100  = 100,
    _250  = 250,
    _500  = 500,
    _1000 = 1000,
    _2000 = 2000
};
// NOLINTEND(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)

enum class BW : uint16_t
{
    _100HZ = 100,
    _200HZ = 200,
    _400HZ = 400,
    _800HZ = 800
};

enum class STATUS : uint8_t
{
    TEMP_READY = 0,
    MAG_READY,
    ALL_READY,
    NO_MEAS
};
} // namespace MMC5983MA_ENUM

// ====================================================== //
// ====================== REGISTERS ===================== //
// ====================================================== //
struct MMC5983MA_REGISTERS
{
    uint8_t READ_FLAG = 0x80;

    uint8_t Xout0      = 0x00;
    uint8_t Xout1      = 0x01;
    uint8_t Yout0      = 0x02;
    uint8_t Yout1      = 0x03;
    uint8_t Zout0      = 0x04;
    uint8_t Zout1      = 0x05;
    uint8_t XYZoutComp = 0x06;
    uint8_t Temp       = 0x07;
    uint8_t Status     = 0x08;
    uint8_t control_0  = 0x09;
    uint8_t control_1  = 0x0A;
    uint8_t control_2  = 0x0B;
    uint8_t control_3  = 0x0C;
    uint8_t ID         = 0x2F;
};

// ====================================================== //
// ================== REGISTERS VALUES ================== //
// ====================================================== //
struct MMC5983MA_REGISTER_VALUES
{
    uint8_t MMC_ID = 0x30;

    struct
    {
        uint8_t MEAS_M_DONE        = 0b00000001U;
        uint8_t MEAS_M_DONE_MASK   = 0b11111110U;
        uint8_t MEAS_T_DONE        = 0b00000010U;
        uint8_t MEAS_T_DONE_MASK   = 0b11111101U;
        uint8_t MEAS_ALL_DONE      = 0b00000011U;
        uint8_t MEAS_ALL_DONE_MASK = 0b11111100U;
        uint8_t OTP_RD_DONE        = 0b00010000U;
        uint8_t OTP_RD_DONE_MASK   = 0b11101111U;
    } STATUS;
    struct
    {
        uint8_t TAKE_MAG_MEAS          = 0b00000001U;
        uint8_t TAKE_MAG_MEAS_MASK     = 0b11111110U;
        uint8_t TAKE_TEMP_MEAS         = 0b00000010U;
        uint8_t TAKE_TEMP_MEAS_MASK    = 0b11111101U;
        uint8_t ENABLE_INT_MEAS        = 0b00000100U;
        uint8_t ENABLE_INT_MEAS_MASK   = 0b11111011U;
        uint8_t SET_CURRENT            = 0b00001000U;
        uint8_t SET_CURRENT_MASK       = 0b11110111U;
        uint8_t RESET_CURRENT          = 0b00010000U;
        uint8_t RESET_CURRENT_MASK     = 0b11101111U;
        uint8_t ENABLE_AUTO_SET_RESET  = 0b00100000U;
        uint8_t DISABLE_AUTO_SET_RESET = 0b00000000U;
        uint8_t AUTO_SET_RESET_MASK    = 0b11011111U;
    } CONTROL_0;

    struct
    {
        uint8_t BW_100HZ                 = 0b00000000U;
        uint8_t BW_200HZ                 = 0b00000001U;
        uint8_t BW_400HZ                 = 0b00000010U;
        uint8_t BW_800HZ                 = 0b00000011U;
        uint8_t BW_MASK                  = 0b11111100U;
        uint8_t DISABLE_X_CHANNEL        = 0b00000100U;
        uint8_t DISABLE_X_CHANNEL_MASK   = 0b00000100U;
        uint8_t DISABLE_XY_CHANNELS      = 0b00011000U;
        uint8_t DISABLE_XY_CHANNELS_MASK = 0b11100111U;
        uint8_t SW_RESET                 = 0b10000000U;
        uint8_t SW_RESET_MASK            = 0b01111111U;
    } CONTROL_1;

    struct
    {
        uint8_t MEAS_FREQ_0HZ                = 0b00000000U;
        uint8_t MEAS_FREQ_1HZ                = 0b00000001U;
        uint8_t MEAS_FREQ_10HZ               = 0b00000010U;
        uint8_t MEAS_FREQ_20HZ               = 0b00000011U;
        uint8_t MEAS_FREQ_50HZ               = 0b00000100U;
        uint8_t MEAS_FREQ_100HZ              = 0b00000101U;
        uint8_t MEAS_FREQ_200HZ              = 0b00000110U;
        uint8_t MEAS_FREQ_1000HZ             = 0b00000111U;
        uint8_t MEAS_FREQ_MASK               = 0b11111000U;
        uint8_t ENABLE_CONTINUOUS_MEAS       = 0b00001000U;
        uint8_t DISABLE_CONTINUOUS_MEAS      = 0b00000000U;
        uint8_t CONTINUOUS_MEAS_MASK         = 0b11110111U;
        uint8_t PERIODIC_SET_1               = 0b00000000U;
        uint8_t PERIODIC_SET_25              = 0b00010000U;
        uint8_t PERIODIC_SET_75              = 0b00100000U;
        uint8_t PERIODIC_SET_100             = 0b00110000U;
        uint8_t PERIODIC_SET_250             = 0b01000000U;
        uint8_t PERIODIC_SET_500             = 0b01010000U;
        uint8_t PERIODIC_SET_1000            = 0b01100000U;
        uint8_t PERIODIC_SET_2000            = 0b01110000U;
        uint8_t PERIODIC_SET_MASK            = 0b10001111U;
        uint8_t ENABLE_PERIODIC_SET          = 0b10000000U;
        uint8_t DISABLE_PERIODIC_SET         = 0b00000000U;
        uint8_t ACTIVATION_PERIODIC_SET_MASK = 0b01111111U;
    } CONTROL_2;

    struct
    {
        uint8_t DESAT_SENSOR_POS      = 0b00000010U;
        uint8_t DESAT_SENSOR_POS_MASK = 0b11111101U;
        uint8_t DESAT_SENSOR_NEG      = 0b00000100U;
        uint8_t DESAT_SENSOR_NEG_MASK = 0b11111011U;
    } CONTROL_3;
};

struct MMC5983MA_REGISTERS_MEMORY
{
    uint8_t Status    = 0U;
    uint8_t control_0 = 0U;
    uint8_t control_1 = 0U;
    uint8_t control_2 = 0U;
    uint8_t control_3 = 0U;
    void setZero()
    {
        Status    = 0U;
        control_0 = 0U;
        control_1 = 0U;
        control_2 = 0U;
        control_3 = 0U;
    }
    uint8_t updateVal(uint8_t oldVal, uint8_t value, uint8_t mask)
    {
        uint8_t curVal = oldVal;
        curVal &= mask;
        curVal |= value;
        return curVal;
    }
};

#endif