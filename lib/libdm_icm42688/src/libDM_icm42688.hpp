#ifndef icm42688_hpp
#define icm42688_hpp

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

#include "libDM_abstract_sensor_protocol.hpp"
#include "libDM_abstract_imu.hpp"
#include "libDM_icm42688_regs_enums.hpp"
#include <map>

class ICM42688_SPI : public ABSTRACT_SENSOR_SPI, public ABSTRACT_IMU
{
public:
    ICM42688_SPI(uint32_t spiClockSpeed,
                 uint8_t spiSS,
                 streamLogger* streamObj,
                 timerTool* timerToolObj,
                 SPIClass* spiObj,
                 SPI_CONF spiConfAtEndTransaction,
                 bool useFifo,
                 bool useExternalClock) :
        mUseFifo(useFifo),
        mUseExternalClock(useExternalClock),
        ABSTRACT_SENSOR_SPI(spiSS,
                            streamObj,
                            timerToolObj,
                            spiObj,
                            SPI_CONF(spiClockSpeed, MSBFIRST, SPI_MODE0),
                            spiConfAtEndTransaction),
        ABSTRACT_IMU(streamObj, timerToolObj)
    {
        // These definitions are provided by ABSTRACT_SENSOR_SPI and ABSTRACT_IMU, so we specify the
        // one to choose. In the end it is the same object from ABSTRACT_SENSOR
        mStreamObj = ABSTRACT_IMU::mStreamObj;
        mTimerObj  = ABSTRACT_IMU::mTimerObj;
    };

    virtual ~ICM42688_SPI() = default;
    // Copy constructor
    ICM42688_SPI(const ICM42688_SPI& other) = default;
    // Copy assignment operator
    ICM42688_SPI& operator=(const ICM42688_SPI& other) = default;
    // Move constructor
    ICM42688_SPI(ICM42688_SPI&& other) noexcept = default;
    // Move assignment operator
    ICM42688_SPI& operator=(ICM42688_SPI&& other) noexcept = default;

    // ~~~~~~ Redefined virtual methods ~~~~~~ //
    bool begin() override;
    void readTemp() override;
    void readAcc() override;
    void readGyro() override;
    void readAccGyro() override;
    void readAllImu() override;
    uint8_t whoAmI() override;

    // ~~~~~~~~~ Icm specific methods ~~~~~~~~ //

    /**
     * @brief Read the fifo and fill the output struct
     * @details This method will read the fifo, extract gyrometer, accelerometer values and
     * temperature values, and apply corrections to them.
     */
    void readFifo();

    /**
     * @brief Reads data from the FIFO buffer without checking status
     * @details This function reads data from the FIFO buffer of the ICM42688 without checking fifo
     * status and without going for last value stored. The function will read the same values as
     * readFifo but will be faster, with the possiblity of reading older values.
     */
    void readFifoFast();

    /**
     * @brief Sets the gyroscope output data rate (ODR) and range.
     * @param freq The desired gyroscope output data rate.
     * @param range The desired gyroscope range.
     */
    void setGyroOdrRange(ICM42688_ENUM::ODR freq, ICM42688_ENUM::GYRO_RANGE range);
    /**
     * @brief Set the accelerometer output data rate (ODR) and range.
     * @param freq The desired accelerometer output data rate.
     * @param range The desired accelerometer range.
     */
    void setAccOdrRange(ICM42688_ENUM::ODR freq, ICM42688_ENUM::ACC_RANGE range);

    /**
     * @brief Sets the filter parameters for the gyroscope and accelerometer.
     * @param gyroBW The bandwidth of the gyroscope filter.
     * @param accBW The bandwidth of the accelerometer filter.
     * @param gyroFiltOrder The filter order for the gyroscope.
     * @param accFiltOrder The filter order for the accelerometer.
     */
    void setGyroAccelFilter(ICM42688_ENUM::FILTER_BW gyroBW,
                            ICM42688_ENUM::FILTER_BW accBW,
                            ICM42688_ENUM::FILTER_ORDER gyroFiltOrder,
                            ICM42688_ENUM::FILTER_ORDER accFiltOrder);

    /**
     * @brief Set the temperature filter bandwidth.
     * @param tempBw The bandwidth of the temperature filter.
     */
    void setTempFilter(ICM42688_ENUM::TEMP_DLPF_BW tempBw);

    /**
     * @brief Resets the sensor to its default state.
     * @details This function performs a software reset on the sensor, restoring it to its default
     * configuration. After the reset, the sensor will be in a known state and ready for further
     * operations.
     */
    void softResetSensor();

    /**
     * @brief Flushes the FIFO buffer.
     * @param printStatus If true, prints the status of the flush operation.
     */
    void flushFifo(bool printStatus);

    /**
     * @brief Sets the Apex Wake-on-Motion (WOM) parameters.
     * @details Example:
     * @code {.cpp}
     * icm.setApexWOM(3,
     *                3,
     *                3,
     *                ICM42688_ENUM::APEX_WOM_INT_MODE::WOM_INT_MODE_OR,
     *                ICM42688_ENUM::APEX_WOM_MODE::WOM_MODE_DELTA,
     *                ICM42688_ENUM::APEX_SMD_MODE::SMD_RESERVED);
     * @endcode
     *
     * @param xThreshold The threshold value for the X-axis -> X /256. If X=256, the threshold is
     * 1g.
     * @param yThreshold The threshold value for the Y-axis -> Y /256. If Y=256, the threshold is
     * 1g.
     * @param zThreshold The threshold value for the Z-axis -> Z /256. If Z=256, the threshold is
     * 1g.
     * @param womIntMode The WOM interrupt mode: 0 for OR, 1 for AND.
     * @param womMode The WOM mode: 0 is absolute accelerometer threshold, 1 is relative
     * accelerometer values from the previous sample.
     * @param smdMode The SMD mode.
     */
    void setApexWOM(uint8_t xThreshold,
                    uint8_t yThreshold,
                    uint8_t zThreshold,
                    ICM42688_ENUM::APEX_WOM_INT_MODE womIntMode,
                    ICM42688_ENUM::APEX_WOM_MODE womMode,
                    ICM42688_ENUM::APEX_SMD_MODE smdMode);

    /**
     * @brief Retrieves the Apex status of the ICM42688 sensor.
     * @return The Apex status of the ICM42688 sensor.
     */
    const ICM42688_STRUCTS::APEX_STATUS& getApexStatus();

private:
    // Definitions coming from multiples objects and ambiguous
    streamLogger* mStreamObj = NULL;
    timerTool* mTimerObj     = NULL;
    using ABSTRACT_SENSOR_SPI::readRegister;
    using ABSTRACT_SENSOR_SPI::readRegisterBurst;
    using ABSTRACT_SENSOR_SPI::writeRegister;
    using ABSTRACT_SENSOR_SPI::writeRegisterWithMask;
    // Internal values
    bool mUseFifo                              = false;
    bool mUseExternalClock                     = true;
    float _16bits_range                        = 32768.F;
    float _20bits_range                        = 524288.F;
    float mGyroFactor                          = 2000.F / _16bits_range;
    float mAccFactor                           = 16.F / _16bits_range;
    float mGyroFactor20Bits                    = 2000.F / _20bits_range;
    float mAccFactor20bits                     = 16.F / _20bits_range;
    ICM42688_ENUM::ODR mGyroFreq               = ICM42688_ENUM::ODR::_1kHz;
    ICM42688_ENUM::GYRO_RANGE mGyroRange       = ICM42688_ENUM::GYRO_RANGE::_2000dps;
    ICM42688_ENUM::ODR mAccFreq                = ICM42688_ENUM::ODR::_1kHz;
    ICM42688_ENUM::ACC_RANGE mAccRange         = ICM42688_ENUM::ACC_RANGE::_16g;
    ICM42688_ENUM::TEMP_DLPF_BW mTempBW        = ICM42688_ENUM::TEMP_DLPF_BW::_4000HZ;
    ICM42688_ENUM::FILTER_BW mGyroBW           = ICM42688_ENUM::FILTER_BW::ODR_DIV_4;
    ICM42688_ENUM::FILTER_BW mAccBW            = ICM42688_ENUM::FILTER_BW::ODR_DIV_4;
    ICM42688_ENUM::FILTER_ORDER mGyroFiltOrder = ICM42688_ENUM::FILTER_ORDER::_DISABLED;
    ICM42688_ENUM::FILTER_ORDER mAccFiltOrder  = ICM42688_ENUM::FILTER_ORDER::_DISABLED;
    uint8_t mFifoStatus                        = 0;
    // As the last timestamp is sometimes corrupted, we need to store the previous one, compare fifo
    // timestamps et keep the last valid one
    uint32_t mFifoTimestampDiffThreshold =
            1000000U / static_cast<uint32_t>(ICM42688_ENUM::ODR::_1kHz); // in us
    uint32_t mFifoTimestampDiffMargin = mFifoTimestampDiffThreshold / 2;

    ICM42688_REGISTERS reg;
    ICM42688_REGISTER_VALUES regVal;

    // APEX
    ICM42688_STRUCTS::APEX_STATUS mApexStatus = ICM42688_STRUCTS::APEX_STATUS();

    // ~~~~~~~~~~~ Private methods ~~~~~~~~~~~ //
    bool tryStart();
    void powerOnSensors(); // activate accel/gyro
    void activateExtClock();
    void updateGyroFactor(ICM42688_ENUM::GYRO_RANGE range);
    void updateAccFactor(ICM42688_ENUM::ACC_RANGE range);
    void activateFifo();
    uint16_t getFifoSize();
    void fillOutputStruct(const int16_t rawGyro[3],
                          const int16_t rawAcc[3],
                          int16_t rawTemp,
                          float gyroFactor,
                          float accFactor,
                          uint32_t timestamp);
    void fillOutputStruct(const int16_t rawGyro[3],
                          const int16_t rawAcc[3],
                          float gyroFactor,
                          float accFactor,
                          uint32_t timestamp);
    void fillOutputStruct(const int32_t rawGyro[3],
                          const int32_t rawAcc[3],
                          int16_t rawTemp,
                          float gyroFactor,
                          float accFactor,
                          uint32_t timestamp);

    void configureInterrupt1(ICM42688_ENUM::INTERRUPT_DRIVE_MODE driveMode,
                             ICM42688_ENUM::INTERRUPT_POLARITY polarity,
                             ICM42688_ENUM::INTERRUPT_MODE mode);

    // ~~~~~~~~~~~ Association maps ~~~~~~~~~~ //
    std::map<ICM42688_ENUM::ODR, uint8_t> enum_odr_to_gyro_odr_map = {
            {ICM42688_ENUM::ODR::_32kHz, regVal.GYRO_ODR._32kHz},
            {ICM42688_ENUM::ODR::_16kHz, regVal.GYRO_ODR._16kHz},
            {ICM42688_ENUM::ODR::_8kHz, regVal.GYRO_ODR._8kHz},
            {ICM42688_ENUM::ODR::_4kHz, regVal.GYRO_ODR._4kHz},
            {ICM42688_ENUM::ODR::_2kHz, regVal.GYRO_ODR._2kHz},
            {ICM42688_ENUM::ODR::_1kHz, regVal.GYRO_ODR._1kHz},
            {ICM42688_ENUM::ODR::_500Hz, regVal.GYRO_ODR._500Hz},
            {ICM42688_ENUM::ODR::_200Hz, regVal.GYRO_ODR._200Hz},
            {ICM42688_ENUM::ODR::_100Hz, regVal.GYRO_ODR._100Hz},
            {ICM42688_ENUM::ODR::_50Hz, regVal.GYRO_ODR._50Hz},
            {ICM42688_ENUM::ODR::_25Hz, regVal.GYRO_ODR._25Hz},
            {ICM42688_ENUM::ODR::_12_5Hz, regVal.GYRO_ODR._12_5Hz}};
    std::map<ICM42688_ENUM::ODR, uint8_t> enum_odr_to_acc_odr_map = {
            {ICM42688_ENUM::ODR::_32kHz, regVal.ACC_ODR._32kHz},
            {ICM42688_ENUM::ODR::_16kHz, regVal.ACC_ODR._16kHz},
            {ICM42688_ENUM::ODR::_8kHz, regVal.ACC_ODR._8kHz},
            {ICM42688_ENUM::ODR::_4kHz, regVal.ACC_ODR._4kHz},
            {ICM42688_ENUM::ODR::_2kHz, regVal.ACC_ODR._2kHz},
            {ICM42688_ENUM::ODR::_1kHz, regVal.ACC_ODR._1kHz},
            {ICM42688_ENUM::ODR::_500Hz, regVal.ACC_ODR._500Hz},
            {ICM42688_ENUM::ODR::_200Hz, regVal.ACC_ODR._200Hz},
            {ICM42688_ENUM::ODR::_100Hz, regVal.ACC_ODR._100Hz},
            {ICM42688_ENUM::ODR::_50Hz, regVal.ACC_ODR._50Hz},
            {ICM42688_ENUM::ODR::_25Hz, regVal.ACC_ODR._25Hz},
            {ICM42688_ENUM::ODR::_12_5Hz, regVal.ACC_ODR._12_5Hz}};
    std::map<ICM42688_ENUM::GYRO_RANGE, std::pair<uint8_t, float>>
            enum_range_to_gyro_range_caracs_map = {
                    {ICM42688_ENUM::GYRO_RANGE::_2000dps,
                     {regVal.GYRO_RANGE._2000dps, 2000.F / _16bits_range}},
                    {ICM42688_ENUM::GYRO_RANGE::_1000dps,
                     {regVal.GYRO_RANGE._1000dps, 1000.F / _16bits_range}},
                    {ICM42688_ENUM::GYRO_RANGE::_500dps,
                     {regVal.GYRO_RANGE._500dps, 500.F / _16bits_range}},
                    {ICM42688_ENUM::GYRO_RANGE::_250dps,
                     {regVal.GYRO_RANGE._250dps, 250.F / _16bits_range}},
                    {ICM42688_ENUM::GYRO_RANGE::_125dps,
                     {regVal.GYRO_RANGE._125dps, 125.F / _16bits_range}},
                    {ICM42688_ENUM::GYRO_RANGE::_62_5dps,
                     {regVal.GYRO_RANGE._62_5dps, 62.5F / _16bits_range}},
                    {ICM42688_ENUM::GYRO_RANGE::_31_25dps,
                     {regVal.GYRO_RANGE._31_25dps, 31.25F / _16bits_range}},
                    {ICM42688_ENUM::GYRO_RANGE::_15_625dps,
                     {regVal.GYRO_RANGE._15_625dps, 15.625F / _16bits_range}}};
    std::map<ICM42688_ENUM::ACC_RANGE, std::pair<uint8_t, float>>
            enum_range_to_acc_range_caracs_map = {
                    {ICM42688_ENUM::ACC_RANGE::_16g, {regVal.ACC_RANGE._16g, 16.F / _16bits_range}},
                    {ICM42688_ENUM::ACC_RANGE::_8g, {regVal.ACC_RANGE._8g, 8.F / _16bits_range}},
                    {ICM42688_ENUM::ACC_RANGE::_4g, {regVal.ACC_RANGE._4g, 4.F / _16bits_range}},
                    {ICM42688_ENUM::ACC_RANGE::_2g, {regVal.ACC_RANGE._2g, 2.F / _16bits_range}}};
    std::map<ICM42688_ENUM::TEMP_DLPF_BW, uint8_t> enum_dlpf_bw_to_temp_dlpf_map = {
            {ICM42688_ENUM::TEMP_DLPF_BW::_4000HZ, regVal.TEMP_FILT_BW._4000HZ},
            {ICM42688_ENUM::TEMP_DLPF_BW::_170HZ, regVal.TEMP_FILT_BW._170HZ},
            {ICM42688_ENUM::TEMP_DLPF_BW::_82HZ, regVal.TEMP_FILT_BW._82HZ},
            {ICM42688_ENUM::TEMP_DLPF_BW::_40HZ, regVal.TEMP_FILT_BW._40HZ},
            {ICM42688_ENUM::TEMP_DLPF_BW::_20HZ, regVal.TEMP_FILT_BW._20HZ},
            {ICM42688_ENUM::TEMP_DLPF_BW::_10HZ, regVal.TEMP_FILT_BW._10HZ},
            {ICM42688_ENUM::TEMP_DLPF_BW::_5HZ, regVal.TEMP_FILT_BW._5HZ}};
    std::map<ICM42688_ENUM::FILTER_BW, uint8_t> enum_filter_bw_acc_gyro_filter_conf_map = {
            {ICM42688_ENUM::FILTER_BW::ODR_DIV_2, regVal.ACC_GYRO_FILTER_BW.ODR_DIV_2},
            {ICM42688_ENUM::FILTER_BW::ODR_DIV_4, regVal.ACC_GYRO_FILTER_BW.ODR_DIV_4},
            {ICM42688_ENUM::FILTER_BW::ODR_DIV_5, regVal.ACC_GYRO_FILTER_BW.ODR_DIV_5},
            {ICM42688_ENUM::FILTER_BW::ODR_DIV_8, regVal.ACC_GYRO_FILTER_BW.ODR_DIV_8},
            {ICM42688_ENUM::FILTER_BW::ODR_DIV_10, regVal.ACC_GYRO_FILTER_BW.ODR_DIV_10},
            {ICM42688_ENUM::FILTER_BW::ODR_DIV_16, regVal.ACC_GYRO_FILTER_BW.ODR_DIV_16},
            {ICM42688_ENUM::FILTER_BW::ODR_DIV_20, regVal.ACC_GYRO_FILTER_BW.ODR_DIV_20},
            {ICM42688_ENUM::FILTER_BW::ODR_DIV_40, regVal.ACC_GYRO_FILTER_BW.ODR_DIV_40},
            {ICM42688_ENUM::FILTER_BW::LOW_LATENCY_ODR, regVal.ACC_GYRO_FILTER_BW.LOW_LATENCY_ODR},
            {ICM42688_ENUM::FILTER_BW::LOW_LATENCY_8_ODR,
             regVal.ACC_GYRO_FILTER_BW.LOW_LATENCY_8_ODR}};
    std::map<ICM42688_ENUM::FILTER_ORDER, uint8_t> enum_filter_order_to_acc_gyro_filter_order_map =
            {{ICM42688_ENUM::FILTER_ORDER::_1st_ORDER, regVal.ACC_GYRO_FILTER_ORDER._1st_ORDER},
             {ICM42688_ENUM::FILTER_ORDER::_2nd_ORDER, regVal.ACC_GYRO_FILTER_ORDER._2nd_ORDER},
             {ICM42688_ENUM::FILTER_ORDER::_3rd_ORDER, regVal.ACC_GYRO_FILTER_ORDER._3rd_ORDER},
             {ICM42688_ENUM::FILTER_ORDER::_DISABLED, regVal.ACC_GYRO_FILTER_ORDER._DISABLED}};

    // APEX WOM
    std::map<ICM42688_ENUM::APEX_WOM_MODE, uint8_t> enum_apex_wom_map = {
            {ICM42688_ENUM::APEX_WOM_MODE::WOM_MODE_ABS, regVal.APEX_WOM_CONF.WOM_MODE_ABS},
            {ICM42688_ENUM::APEX_WOM_MODE::WOM_MODE_DELTA, regVal.APEX_WOM_CONF.WOM_MODE_DELTA}};
    std::map<ICM42688_ENUM::APEX_WOM_INT_MODE, uint8_t> enum_apex_wom_int_map = {
            {ICM42688_ENUM::APEX_WOM_INT_MODE::WOM_INT_MODE_AND,
             regVal.APEX_WOM_CONF.WOM_INT_MODE_AND},
            {ICM42688_ENUM::APEX_WOM_INT_MODE::WOM_INT_MODE_OR,
             regVal.APEX_WOM_CONF.WOM_INT_MODE_OR}};
    std::map<ICM42688_ENUM::APEX_SMD_MODE, uint8_t> enum_apex_smd_map = {
            {ICM42688_ENUM::APEX_SMD_MODE::SMD_DISABLED, regVal.APEX_WOM_CONF.SMD_MODE_DISABLED},
            {ICM42688_ENUM::APEX_SMD_MODE::SMD_RESERVED, regVal.APEX_WOM_CONF.SMD_MODE_RESERVED},
            {ICM42688_ENUM::APEX_SMD_MODE::SMD_SHORT, regVal.APEX_WOM_CONF.SMD_MODE_SHORT},
            {ICM42688_ENUM::APEX_SMD_MODE::SMD_LONG, regVal.APEX_WOM_CONF.SMD_MODE_LONG}};

    // Interrupts
    std::map<ICM42688_ENUM::INTERRUPT_DRIVE_MODE, uint8_t> enum_int_drive_mode_map = {
            {ICM42688_ENUM::INTERRUPT_DRIVE_MODE::INT_OPEN_DRAIN,
             regVal.INT_CONFIG.INT1_DRIVE_OPEN_DRAIN},
            {ICM42688_ENUM::INTERRUPT_DRIVE_MODE::INT_PUSH_PULL,
             regVal.INT_CONFIG.INT1_DRIVE_PUSH_PULL}};
    std::map<ICM42688_ENUM::INTERRUPT_POLARITY, uint8_t> enum_int_polarity_map = {
            {ICM42688_ENUM::INTERRUPT_POLARITY::ACTIVE_LOW,
             regVal.INT_CONFIG.INT1_POLARITY_ACTIVE_LOW},
            {ICM42688_ENUM::INTERRUPT_POLARITY::ACTIVE_HIGH,
             regVal.INT_CONFIG.INT1_POLARITY_ACTIVE_HIGH}};
    std::map<ICM42688_ENUM::INTERRUPT_MODE, uint8_t> enum_int_mode_map = {
            {ICM42688_ENUM::INTERRUPT_MODE::PULSED, regVal.INT_CONFIG.INT1_MODE_PULSED},
            {ICM42688_ENUM::INTERRUPT_MODE::LATCHED, regVal.INT_CONFIG.INT1_MODE_LATCHED}};

    // ~~~~~~~~~~~~~~ Class name ~~~~~~~~~~~~~ //
    String mClassName = "ICM42688_SPI";
};

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

#endif