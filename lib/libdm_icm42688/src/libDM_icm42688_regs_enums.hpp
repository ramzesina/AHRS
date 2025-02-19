#ifndef icm42688_regs_enums_hpp
#define icm42688_regs_enums_hpp
#include <Arduino.h>

// ====================================================== //
// ======================== ENUMS ======================= //
// ====================================================== //
namespace ICM42688_ENUM
{
enum class ODR : uint16_t
{
    _32kHz  = 32000,
    _16kHz  = 16000,
    _8kHz   = 8000,
    _4kHz   = 4000,
    _2kHz   = 2000,
    _1kHz   = 1000,
    _500Hz  = 500,
    _200Hz  = 200,
    _100Hz  = 100,
    _50Hz   = 50,
    _25Hz   = 25,
    _12_5Hz = 12
};

enum class GYRO_RANGE : uint16_t
{
    _2000dps   = 2000,
    _1000dps   = 1000,
    _500dps    = 500,
    _250dps    = 250,
    _125dps    = 125,
    _62_5dps   = 62,
    _31_25dps  = 31,
    _15_625dps = 15
};

enum class ACC_RANGE : uint8_t
{
    _16g = 16,
    _8g  = 8,
    _4g  = 4,
    _2g  = 2
};

enum class FILTER_ORDER : uint8_t
{
    _1st_ORDER = 1,
    _2nd_ORDER = 2,
    _3rd_ORDER = 3,
    _DISABLED  = 0 // NOLINT
};

enum class TEMP_DLPF_BW : uint16_t
{
    _4000HZ = 4000, // DLPF BW = 4000Hz; DLPF Latency = 0.125ms (default)
    _170HZ  = 170,  // DLPF BW = 170Hz; DLPF Latency = 1ms
    _82HZ   = 82,   // DLPF BW = 82Hz; DLPF Latency = 2ms
    _40HZ   = 40,   // DLPF BW = 40Hz; DLPF Latency = 4ms
    _20HZ   = 20,   // DLPF BW = 20Hz; DLPF Latency = 8ms
    _10HZ   = 10,   // DLPF BW = 10Hz; DLPF Latency = 16ms
    _5HZ    = 5     // DLPF BW = 5Hz; DLPF Latency = 32ms
};

enum class FILTER_BW : uint16_t // -3db
{
    ODR_DIV_2         = 2,   // BW=ODR/2
    ODR_DIV_4         = 4,   // BW=max(400Hz, ODR)/4 (par défaut)
    ODR_DIV_5         = 5,   // BW=max(400Hz, ODR)/5
    ODR_DIV_8         = 8,   // BW=max(400Hz, ODR)/8
    ODR_DIV_10        = 10,  // BW=max(400Hz, ODR)/10
    ODR_DIV_16        = 16,  // BW=max(400Hz, ODR)/16
    ODR_DIV_20        = 20,  // BW=max(400Hz, ODR)/20
    ODR_DIV_40        = 40,  // BW=max(400Hz, ODR)/40
    LOW_LATENCY_ODR   = 400, // BW=max(400Hz, ODR)
    LOW_LATENCY_8_ODR = 200  // BW=max(200Hz, 8*ODR)
};

enum class APEX_WOM_INT_MODE : uint8_t
{
    WOM_INT_MODE_OR  = 0, // Motion detected if any threshold is exceeded on any axes
    WOM_INT_MODE_AND = 1, // Motion detected if all thresholds are exceeded on all axes
};

enum class APEX_WOM_MODE : uint8_t
{
    WOM_MODE_ABS   = 0, // Motion detected with comparing absolute acceleration values
    WOM_MODE_DELTA = 1, // Motion detected with comparing delta acceleration values
};

enum class APEX_SMD_MODE : uint8_t
{
    SMD_DISABLED = 0, // SMD disabled
    SMD_RESERVED = 1, // if WOM is enabled without SMD, this bit should be set to 1
    SMD_SHORT    = 2, // SMD short (1 sec wait) An SMD event is detected when two WOM are detected 1
                      // sec apart ODR_DIV_8 = 8
    SMD_LONG = 3, // SMD long (3 sec wait) An SMD event is detected when two WOM are detected 3 sec
                  // apart
};

enum class INTERRUPT_MODE : uint8_t
{
    PULSED  = 0, // Impulse when event is detected
    LATCHED = 1, // Keep state until cleared
};

enum class INTERRUPT_DRIVE_MODE : uint8_t
{
    INT_OPEN_DRAIN = 0, // Can drive with external pull-up
    INT_PUSH_PULL  = 1, // Can drive in both directions (high of low)
};

enum class INTERRUPT_POLARITY : uint8_t
{
    ACTIVE_LOW  = 0, // Can drive with external pull-up
    ACTIVE_HIGH = 1, // Can drive in both directions (high of low)
};

} // namespace ICM42688_ENUM
namespace ICM42688_STRUCTS
{
struct APEX_STATUS
{
    bool SMD_DETECTED        = false; // SMD has been detected
    bool WOM_X_DET           = false; // WOM_X has been detected
    bool WOM_Y_DET           = false; // WOM_Y has been detected
    bool WOM_Z_DET           = false; // WOM_Z has been detected
    bool STEP_DETECTED       = false; // Step has been detected
    bool STEP_COUNT_OVERFLOW = false; // Step count overflow
    bool TILT_DETECTED       = false; // Tilt event has been detected
    bool WAKE_DETECTED       = false; // Wake event has been detected
    bool SLEEP_DETECTED      = false; // Sleep event has been detected
    bool TAP_DETECTED        = false; // Tap event has been detected
};
} // namespace ICM42688_STRUCTS

// ====================================================== //
// ====================== REGISTERS ===================== //
// ====================================================== //
struct ICM42688_REGISTERS
{
    uint8_t READ_FLAG = 0x80;
    uint8_t SENSOR_ID = 0x47;
    struct
    {
        uint8_t DEVICE_CONFIG      = 0x11;
        uint8_t DRIVE_CONFIG       = 0x13;
        uint8_t INT_CONFIG         = 0x14;
        uint8_t FIFO_CONFIG        = 0x16;
        uint8_t TEMP_DATA1         = 0x1D;
        uint8_t TEMP_DATA0         = 0x1E;
        uint8_t ACCEL_DATA_X1      = 0x1F;
        uint8_t ACCEL_DATA_X0      = 0x20;
        uint8_t ACCEL_DATA_Y1      = 0x21;
        uint8_t ACCEL_DATA_Y0      = 0x22;
        uint8_t ACCEL_DATA_Z1      = 0x23;
        uint8_t ACCEL_DATA_Z0      = 0x24;
        uint8_t GYRO_DATA_X1       = 0x25;
        uint8_t GYRO_DATA_X0       = 0x26;
        uint8_t GYRO_DATA_Y1       = 0x27;
        uint8_t GYRO_DATA_Y0       = 0x28;
        uint8_t GYRO_DATA_Z1       = 0x29;
        uint8_t GYRO_DATA_Z0       = 0x2A;
        uint8_t TMST_FSYNCH        = 0x2B;
        uint8_t TMST_FSYNCL        = 0x2C;
        uint8_t INT_STATUS         = 0x2D;
        uint8_t FIFO_COUNTH        = 0x2E;
        uint8_t FIFO_COUNTL        = 0x2F;
        uint8_t FIFO_DATA          = 0x30;
        uint8_t APEX_DATA0         = 0x31;
        uint8_t APEX_DATA1         = 0x32;
        uint8_t APEX_DATA2         = 0x33;
        uint8_t APEX_DATA3         = 0x34;
        uint8_t APEX_DATA4         = 0x35;
        uint8_t APEX_DATA5         = 0x36;
        uint8_t INT_STATUS2        = 0x37;
        uint8_t INT_STATUS3        = 0x38;
        uint8_t SIGNAL_PATH_RESET  = 0x4B;
        uint8_t INTF_CONFIG0       = 0x4C;
        uint8_t INTF_CONFIG1       = 0x4D;
        uint8_t PWR_MGMT0          = 0x4E;
        uint8_t GYRO_CONFIG0       = 0x4F;
        uint8_t ACCEL_CONFIG0      = 0x50;
        uint8_t GYRO_CONFIG1       = 0x51;
        uint8_t GYRO_ACCEL_CONFIG0 = 0x52;
        uint8_t ACCEL_CONFIG1      = 0x53;
        uint8_t TMST_CONFIG        = 0x54;
        uint8_t APEX_CONFIG0       = 0x56;
        uint8_t SMD_CONFIG         = 0x57;
        uint8_t FIFO_CONFIG1       = 0x5F;
        uint8_t FIFO_CONFIG2       = 0x60;
        uint8_t FIFO_CONFIG3       = 0x61;
        uint8_t FSYNC_CONFIG       = 0x62;
        uint8_t INT_CONFIG0        = 0x63;
        uint8_t INT_CONFIG1        = 0x64;
        uint8_t INT_SOURCE0        = 0x65;
        uint8_t INT_SOURCE1        = 0x66;
        uint8_t INT_SOURCE3        = 0x68;
        uint8_t INT_SOURCE4        = 0x69;
        uint8_t FIFO_LOST_PKT0     = 0x6C;
        uint8_t FIFO_LOST_PKT1     = 0x6D;
        uint8_t SELF_TEST_CONFIG   = 0x70;
        uint8_t WHO_AM_I           = 0x75; // should return =0x47
        uint8_t REG_BANK_SEL       = 0x76;
    } BANK0;

    struct
    {
        uint8_t SENSOR_CONFIG0       = 0x03;
        uint8_t GYRO_CONFIG_STATIC2  = 0x0B;
        uint8_t GYRO_CONFIG_STATIC3  = 0x0C;
        uint8_t GYRO_CONFIG_STATIC4  = 0x0D;
        uint8_t GYRO_CONFIG_STATIC5  = 0x0E;
        uint8_t GYRO_CONFIG_STATIC6  = 0x0F;
        uint8_t GYRO_CONFIG_STATIC7  = 0x10;
        uint8_t GYRO_CONFIG_STATIC8  = 0x11;
        uint8_t GYRO_CONFIG_STATIC9  = 0x12;
        uint8_t GYRO_CONFIG_STATIC10 = 0x13;
        uint8_t XG_ST_DATA           = 0x5F;
        uint8_t YG_ST_DATA           = 0x60;
        uint8_t ZG_ST_DATA           = 0x61;
        uint8_t TMSTAL0              = 0x63;
        uint8_t TMSTAL1              = 0x64;
        uint8_t TMSTAL2              = 0x62;
        uint8_t INTF_CONFIG4         = 0x7A;
        uint8_t INTF_CONFIG5         = 0x7B;
        uint8_t INTF_CONFIG6         = 0x7C;
    } BANK1;

    struct
    {
        uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
        uint8_t ACCEL_CONFIG_STATIC3 = 0x04;
        uint8_t ACCEL_CONFIG_STATIC4 = 0x05;
        uint8_t XA_ST_DATA           = 0x3B;
        uint8_t YA_ST_DATA           = 0x3C;
        uint8_t ZA_ST_DATA           = 0x3D;
    } BANK2;

    struct
    {
        uint8_t APEX_CONFIG1    = 0x40;
        uint8_t APEX_CONFIG2    = 0x41;
        uint8_t APEX_CONFIG3    = 0x42;
        uint8_t APEX_CONFIG4    = 0x43;
        uint8_t APEX_CONFIG5    = 0x44;
        uint8_t APEX_CONFIG6    = 0x45;
        uint8_t APEX_CONFIG7    = 0x46;
        uint8_t APEX_CONFIG8    = 0x47;
        uint8_t APEX_CONFIG9    = 0x48;
        uint8_t ACCEL_WOM_X_THR = 0x4A;
        uint8_t ACCEL_WOM_Y_THR = 0x4B;
        uint8_t ACCEL_WOM_Z_THR = 0x4C;
        uint8_t INT_SOURCE6     = 0x4D;
        uint8_t INT_SOURCE7     = 0x4E;
        uint8_t INT_SOURCE8     = 0x4F;
        uint8_t INT_SOURCE9     = 0x50;
        uint8_t INT_SOURCE10    = 0x51;
        uint8_t OFFSET_USER0    = 0x77;
        uint8_t OFFSET_USER1    = 0x78;
        uint8_t OFFSET_USER2    = 0x79;
        uint8_t OFFSET_USER3    = 0x7A;
        uint8_t OFFSET_USER4    = 0x7B;
        uint8_t OFFSET_USER5    = 0x7C;
        uint8_t OFFSET_USER6    = 0x7D;
        uint8_t OFFSET_USER7    = 0x7E;
        uint8_t OFFSET_USER8    = 0x7F;
    } BANK4;
};

// ====================================================== //
// ================== REGISTERS VALUES ================== //
// ====================================================== //
struct ICM42688_REGISTER_VALUES
{
    struct
    {
        uint8_t BANK0     = 0U;
        uint8_t BANK1     = 1U;
        uint8_t BANK2     = 2U;
        uint8_t BANK3     = 3U;
        uint8_t BANK4     = 4U;
        uint8_t BANK_MASK = 0b11111000;
    } BANK;
    struct
    {
        uint8_t ACCEL_LOW_NOISE = 0b1100;
        uint8_t GYRO_LOW_NOISE  = 0b11;
    } PWR_MGMT0;
    struct
    {
        uint8_t FIFO_HOLD_LAST_DATA_EN       = 0b10000000;
        uint8_t FIFO_HOLD_LAST_DATA_EN_MASK  = 0b01111111;
        uint8_t FIFO_COUNT_REC_IN_COUNT      = 0b01000000;
        uint8_t FIFO_COUNT_REC_IN_COUNT_MASK = 0b10111111;
    } INTF_CONFIG0;

    struct
    {
        uint8_t RTC_ENABLE = 0b00001101;
        uint8_t RTC_MASK   = 0b11110000;
    } INTF_CONFIG1;
    struct
    {
        uint8_t PIN9_CLKIN = 0b100;
        uint8_t PIN9_FSYNC = 0b10;
        uint8_t PIN9_INT2  = 0;
        uint8_t PIN9_MASK  = 0b11111001;
    } INTF_CONFIG5;
    struct
    {
        uint8_t _32kHz  = 0b0001;
        uint8_t _16kHz  = 0b0010;
        uint8_t _8kHz   = 0b0011;
        uint8_t _4kHz   = 0b0100;
        uint8_t _2kHz   = 0b0101;
        uint8_t _1kHz   = 0b0110; // default
        uint8_t _500Hz  = 0b1111;
        uint8_t _200Hz  = 0b0111;
        uint8_t _100Hz  = 0b1000;
        uint8_t _50Hz   = 0b1001;
        uint8_t _25Hz   = 0b1010;
        uint8_t _12_5Hz = 0b1011;
    } GYRO_ODR;
    struct
    {
        uint8_t _2000dps   = 0b000; // Will be used by default
        uint8_t _1000dps   = 0b001;
        uint8_t _500dps    = 0b010;
        uint8_t _250dps    = 0b011;
        uint8_t _125dps    = 0b100;
        uint8_t _62_5dps   = 0b101;
        uint8_t _31_25dps  = 0b110;
        uint8_t _15_625dps = 0b111;
    } GYRO_RANGE;
    struct
    {
        uint8_t _32kHz  = 0b0001;
        uint8_t _16kHz  = 0b0010;
        uint8_t _8kHz   = 0b0011;
        uint8_t _4kHz   = 0b0100;
        uint8_t _2kHz   = 0b0101;
        uint8_t _1kHz   = 0b0110; // default
        uint8_t _500Hz  = 0b1111;
        uint8_t _200Hz  = 0b0111;
        uint8_t _100Hz  = 0b1000;
        uint8_t _50Hz   = 0b1001;
        uint8_t _25Hz   = 0b1010;
        uint8_t _12_5Hz = 0b1011;
    } ACC_ODR;
    struct
    {
        uint8_t _16g = 0b000; // Will be used by default
        uint8_t _8g  = 0b001;
        uint8_t _4g  = 0b010;
        uint8_t _2g  = 0b011;
    } ACC_RANGE;
    struct
    {
        uint8_t _4000HZ           = 0b000; // DLPF BW = 4000Hz; DLPF Latency = 0.125ms (default)
        uint8_t _170HZ            = 0b001; // DLPF BW = 170Hz; DLPF Latency = 1ms
        uint8_t _82HZ             = 0b010; // DLPF BW = 82Hz; DLPF Latency = 2ms
        uint8_t _40HZ             = 0b011; // DLPF BW = 40Hz; DLPF Latency = 4ms
        uint8_t _20HZ             = 0b100; // DLPF BW = 20Hz; DLPF Latency = 8ms
        uint8_t _10HZ             = 0b101; // DLPF BW = 10Hz; DLPF Latency = 16ms
        uint8_t _5HZ              = 0b111; // DLPF BW = 5Hz; DLPF Latency = 32ms
        uint8_t TEMP_FILT_BW_MASK = 0b00011111;
    } TEMP_FILT_BW;
    struct
    {
        uint8_t ODR_DIV_2       = 0;    // 0 BW=ODR/2
        uint8_t ODR_DIV_4       = 1;    // 1 BW=max(400Hz, ODR)/4 (par défaut)
        uint8_t ODR_DIV_5       = 2;    // 2 BW=max(400Hz, ODR)/5
        uint8_t ODR_DIV_8       = 3;    // 3 BW=max(400Hz, ODR)/8
        uint8_t ODR_DIV_10      = 4;    // 4 BW=max(400Hz, ODR)/10
        uint8_t ODR_DIV_16      = 5;    // 5 BW=max(400Hz, ODR)/16
        uint8_t ODR_DIV_20      = 6;    // 6 BW=max(400Hz, ODR)/20
        uint8_t ODR_DIV_40      = 7;    // 7 BW=max(400Hz, ODR)/40
        uint8_t LOW_LATENCY_ODR = 14;   // 14 Low Latency option: Trivial decimation @ ODR of Dec2
                                        // filter output. Dec2 runs at max(400Hz, ODR)
        uint8_t LOW_LATENCY_8_ODR = 15; // 15 Low Latency option: Trivial decimation @ ODR of
                                        // Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)
    } ACC_GYRO_FILTER_BW;
    struct
    {
        uint8_t _1st_ORDER = 0;
        uint8_t _2nd_ORDER = 1;
        uint8_t _3rd_ORDER = 2;
        uint8_t _DISABLED  = 3; // NOLINT
        uint8_t GYRO_MASK  = 0b11110011;
        uint8_t ACC_MASK   = 0b11100111;
    } ACC_GYRO_FILTER_ORDER;
    struct
    {
        uint8_t HIGH_RES_FULL_FIFO_PACKET_4 = 0b10111;
        uint8_t FIFO_PACKET_3               = 0b00111;
        uint8_t FIFO_PACKET_MASK            = 0b10000000;
    } FIFO_CONFIG1;
    struct
    {
        uint8_t STREAM_2_FIFO  = 0b01000000;
        uint8_t FIFO_MODE_MASK = 0b00111111;
    } FIFO_CONFIG;
    struct
    {
        uint8_t FIFO_FLUSH_TRUE = 0b00000010;
        uint8_t FIFO_FLUSH_MASK = 0b11111101;
    } SIGNAL_PATH_RESET;
    struct
    {
        uint8_t WOM_INT_MODE_OR   = 0b00000000;
        uint8_t WOM_INT_MODE_AND  = 0b00001000;
        uint8_t WOM_INT_MODE_MASK = 0b11110111;
        uint8_t WOM_MODE_ABS      = 0b00000000;
        uint8_t WOM_MODE_DELTA    = 0b00000100;
        uint8_t WOM_MODE_MASK     = 0b11111011;
        uint8_t SMD_MODE_DISABLED = 0b00000000;
        uint8_t SMD_MODE_RESERVED = 0b00000001;
        uint8_t SMD_MODE_SHORT    = 0b00000010;
        uint8_t SMD_MODE_LONG     = 0b00000011;
        uint8_t SMD_MODE_MASK     = 0b11111100;
    } APEX_WOM_CONF;
    struct
    {
        uint8_t WOM_TO_INT1_EN   = 0b00000111;
        uint8_t WOM_TO_INT1_DIS  = 0b00000000;
        uint8_t WOM_TO_INT1_MASK = 0b11111000;
        uint8_t SMD_TO_INT1_EN   = 0b00001000;
        uint8_t SMD_TO_INT1_DIS  = 0b00000000;
        uint8_t SMD_TO_INT1_MASK = 0b11110111;
    } INT_SOURCE1;
    struct
    {
        uint8_t INT1_POLARITY_ACTIVE_LOW  = 0b00000000;
        uint8_t INT1_POLARITY_ACTIVE_HIGH = 0b00000001;
        uint8_t INT1_POLARITY_MASK        = 0b11111110;
        uint8_t INT1_DRIVE_OPEN_DRAIN     = 0b00000000;
        uint8_t INT1_DRIVE_PUSH_PULL      = 0b00000010;
        uint8_t INT1_DRIVE_MASK           = 0b11111101;
        uint8_t INT1_MODE_PULSED          = 0b00000000;
        uint8_t INT1_MODE_LATCHED         = 0b00000100;
        uint8_t INT1_MODE_MASK            = 0b11111011;
        uint8_t INT1_CONF_MASK            = 0b11111000;
        // TODO: Idem for int2
    } INT_CONFIG;
    struct
    {
        uint8_t INT_ASYNC_RESET = 0b00000000;
        uint8_t INT_ASYNC_MASK  = 0b11101111;
    } INT_CONFIG1;
};

#endif