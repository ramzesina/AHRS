#ifndef mmc5983ma_hpp
#define mmc5983ma_hpp

// Warning: A lot of registers are not readable!! So we have to store a local copy of what each
// register should contain

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

#include "libDM_abstract_sensor_protocol.hpp"
#include "libDM_abstract_mag.hpp"
#include "libDM_mmc5983ma_regs_enums.hpp"
#include <map>

class MMC5983MA_SPI : public ABSTRACT_SENSOR_SPI, public ABSTRACT_MAG
{
public:
    MMC5983MA_SPI(uint32_t spiClockSpeed,
                  uint8_t spiSS,
                  streamLogger* streamObj,
                  timerTool* timerToolObj,
                  SPIClass* spiObj,
                  SPI_CONF spiConfAtEndTransaction) :
        ABSTRACT_SENSOR_SPI(spiSS,
                            streamObj,
                            timerToolObj,
                            spiObj,
                            SPI_CONF(spiClockSpeed, MSBFIRST, SPI_MODE0),
                            spiConfAtEndTransaction),
        ABSTRACT_MAG(streamObj, timerToolObj)
    {
        // These definitions are provided by ABSTRACT_SENSOR_SPI and ABSTRACT_MAG, so we specify the
        // one to choose. In the end it is the same object from ABSTRACT_SENSOR
        mStreamObj = ABSTRACT_MAG::mStreamObj;
        mTimerObj  = ABSTRACT_MAG::mTimerObj;
    };

    virtual ~MMC5983MA_SPI() = default;
    // Copy constructor
    MMC5983MA_SPI(const MMC5983MA_SPI& other) = default;
    // Copy assignment operator
    MMC5983MA_SPI& operator=(const MMC5983MA_SPI& other) = default;
    // Move constructor
    MMC5983MA_SPI(MMC5983MA_SPI&& other) noexcept = default;
    // Move assignment operator
    MMC5983MA_SPI& operator=(MMC5983MA_SPI&& other) noexcept = default;

    // ~~~~~~ Redefined virtual methods ~~~~~~ //
    bool begin() override;
    void readTemp() override;
    void readMag() override;
    uint8_t whoAmI() override;

    // ~~~~~~~~~ Mmc specific methods ~~~~~~~~ //

    /**
     * @brief Sets the magnetometer output data rate (ODR) and bandwidth (BW).
     * @param magFreq The desired magnetometer output data rate.
     * @param magBw The desired magnetometer bandwidth.
     */
    void setMagOdrBw(MMC5983MA_ENUM::ODR magFreq, MMC5983MA_ENUM::BW magBw);

    /**
     * @brief Sets the periodic set time for the MMC5983MA sensor.
     * @param time The periodic set time to be set.
     */
    void setPeriodicSetTime(MMC5983MA_ENUM::PERIODIC_SET_TIME time);

    /**
     * @brief Will enable or disable the periodic set feature of the MMC5983MA sensor.
     * @param state The desired state of the periodic setting.
     */
    void setPeriodicSetState(bool state);

    /**
     * @brief Enable the automatic set/reset feauture of the MMC5983MA sensor.
     * @param state The desired state of the automatic set/reset feature.
     */
    void setAutoSetResetState(bool state);

    /**
     * @brief Performs a soft reset of the sensor.
     */
    void softResetSensor();

    /**
     * @brief Manually apply set current degaussing
     */
    void setCurrent();

    /**
     * @brief Manually reset current degaussing
     */
    void resetCurrent();

    /**
     * @brief Asks for one magnetic measurement.
     */
    void askForOneMagMeasure();

    /**
     * @brief Asks for one temperature measurement.
     */
    void askForOneTempMeasure();

    /**
     * @brief Activates or deactivates the specified axes.
     * @param activeX Flag indicating whether to activate or deactivate the X-axis.
     * @param activeYZ Flag indicating whether to activate or deactivate the Y and Z axes.
     */
    void activateAxis(bool activeX, bool activeYZ);

    /**
     * @brief Updates the bridge offset.
     * @details The bridge offset is the value that is added to the raw magnetic measurements. The
     * idea is to force the internal magnetization in the direction of SET field, do a measure and
     * repeat the maneuver in the opposite direction with a RESET. The substraction of the two
     * measurements gives the offset.
     */
    void updateBridgeOffset();

    /**
     * @brief Sets the measurement check states for the magnetometer and temperature sensor.
     * @param magCheck Flag indicating whether to verify magnetometer measurement status before
     * doing a measurement.
     * @param tempCheck Flag indicating whether to verify temperature measurement status before
     * doing a measurement.
     */
    void setMeasCheckStates(bool magCheck, bool tempCheck);

private:
    // Definitions coming from multiples objects and ambiguous
    streamLogger* mStreamObj = NULL;
    timerTool* mTimerObj     = NULL;
    using ABSTRACT_SENSOR_SPI::readRegister;
    using ABSTRACT_SENSOR_SPI::readRegisterBurst;
    using ABSTRACT_SENSOR_SPI::writeRegister;
    using ABSTRACT_SENSOR_SPI::writeRegisterWithMask;

    // Internal values
    MMC5983MA_ENUM::ODR mMagFreq                          = MMC5983MA_ENUM::ODR::_100HZ;
    MMC5983MA_ENUM::BW mMagBW                             = MMC5983MA_ENUM::BW::_100HZ;
    MMC5983MA_ENUM::PERIODIC_SET_TIME mMagPeriodicSetTime = MMC5983MA_ENUM::PERIODIC_SET_TIME::_1;
    float mBridgeOffset[3]                                = {0.F, 0.F, 0.F};
    struct
    {
        bool temp = true;
        bool mag  = true;
    } mChecks;

    double mNsTimeOutMeasReading = 100000; // [ns] 100us timeout for meas reading
    // NOLINTBEGIN(altera-struct-pack-align)
    struct
    {
        float tempLow   = -75.F;
        float tempHigh  = 125.F;
        float tempRange = 255.F;
        float tempSlope = (tempHigh - tempLow) / tempRange;
        float magLow    = -8.F;
        float magHigh   = 8.F;
        float magRange  = 262144.F; // 18bits
        float magSlope  = (magHigh - magLow) / magRange;
    } mConversions;
    // NOLINTEND(altera-struct-pack-align)

    MMC5983MA_REGISTERS reg;
    MMC5983MA_REGISTER_VALUES regVal;
    MMC5983MA_REGISTERS_MEMORY regMem;

    // ~~~~~~~~~~~ Private methods ~~~~~~~~~~~ //
    bool tryStart();
    void powerOnSensors(); // activate mag
    void fillOutputStruct(const int32_t rawMag[3], int16_t rawTemp, uint32_t timestamp);
    void setContinuousModeState(bool state);
    MMC5983MA_ENUM::STATUS getMeasuresStatus();

    // ~~~~~~~~~~~ Association maps ~~~~~~~~~~ //
    std::map<MMC5983MA_ENUM::ODR, uint8_t> enum_odr_to_mag_odr_map = {
            {MMC5983MA_ENUM::ODR::_1000HZ, regVal.CONTROL_2.MEAS_FREQ_1000HZ},
            {MMC5983MA_ENUM::ODR::_200HZ, regVal.CONTROL_2.MEAS_FREQ_200HZ},
            {MMC5983MA_ENUM::ODR::_100HZ, regVal.CONTROL_2.MEAS_FREQ_100HZ},
            {MMC5983MA_ENUM::ODR::_50HZ, regVal.CONTROL_2.MEAS_FREQ_50HZ},
            {MMC5983MA_ENUM::ODR::_20HZ, regVal.CONTROL_2.MEAS_FREQ_20HZ},
            {MMC5983MA_ENUM::ODR::_10HZ, regVal.CONTROL_2.MEAS_FREQ_10HZ},
            {MMC5983MA_ENUM::ODR::_1HZ, regVal.CONTROL_2.MEAS_FREQ_1HZ},
            {MMC5983MA_ENUM::ODR::_OFF, regVal.CONTROL_2.DISABLE_CONTINUOUS_MEAS}};

    std::map<MMC5983MA_ENUM::PERIODIC_SET_TIME, uint8_t> enum_periodic_set_to_mag_periodic_map = {
            {MMC5983MA_ENUM::PERIODIC_SET_TIME::_2000, regVal.CONTROL_2.PERIODIC_SET_2000},
            {MMC5983MA_ENUM::PERIODIC_SET_TIME::_1000, regVal.CONTROL_2.PERIODIC_SET_1000},
            {MMC5983MA_ENUM::PERIODIC_SET_TIME::_500, regVal.CONTROL_2.PERIODIC_SET_500},
            {MMC5983MA_ENUM::PERIODIC_SET_TIME::_250, regVal.CONTROL_2.PERIODIC_SET_250},
            {MMC5983MA_ENUM::PERIODIC_SET_TIME::_100, regVal.CONTROL_2.PERIODIC_SET_250},
            {MMC5983MA_ENUM::PERIODIC_SET_TIME::_75, regVal.CONTROL_2.PERIODIC_SET_75},
            {MMC5983MA_ENUM::PERIODIC_SET_TIME::_25, regVal.CONTROL_2.PERIODIC_SET_25},
            {MMC5983MA_ENUM::PERIODIC_SET_TIME::_1, regVal.CONTROL_2.PERIODIC_SET_1},
            {MMC5983MA_ENUM::PERIODIC_SET_TIME::_OFF, regVal.CONTROL_2.DISABLE_PERIODIC_SET}};

    std::map<MMC5983MA_ENUM::BW, uint8_t> enum_bw_to_mag_bw_map = {
            {MMC5983MA_ENUM::BW::_800HZ, regVal.CONTROL_1.BW_800HZ},
            {MMC5983MA_ENUM::BW::_400HZ, regVal.CONTROL_1.BW_400HZ},
            {MMC5983MA_ENUM::BW::_200HZ, regVal.CONTROL_1.BW_200HZ},
            {MMC5983MA_ENUM::BW::_100HZ, regVal.CONTROL_1.BW_100HZ}};

    // ~~~~~~~~~~~~~~ Class name ~~~~~~~~~~~~~ //
    String mClassName = "MMC5983MA_SPI";
};

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

#endif