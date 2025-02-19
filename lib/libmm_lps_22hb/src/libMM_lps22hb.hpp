#ifndef lps22hb_hpp
#define lps22hb_hpp

#include "libDM_abstract_sensor_protocol.hpp"
#include "libDM_abstract_baro.hpp"
#include "libMM_lps22hb_regs_enums.hpp"
#include <map>

class streamLogger;

class LPS22HB_SPI : public ABSTRACT_SENSOR_SPI, public ABSTRACT_BARO
{
public:
    // NOLINTBEGIN(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    LPS22HB_SPI(uint32_t spiClockSpeed,
                uint8_t spiSS,
                bool useFifo,
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
        ABSTRACT_BARO(streamObj, timerToolObj),
        mUseFifo(useFifo)
    {
        // These definitions are provided by ABSTRACT_SENSOR_SPI and ABSTRACT_BARO, so we specify
        // the one to choose. In the end it is the same object from ABSTRACT_SENSOR
        mStreamObj = ABSTRACT_BARO::mStreamObj;
        mTimerObj  = ABSTRACT_BARO::mTimerObj;
    }
    // NOLINTEND(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    // ~~~~~~ Redefined virtual methods ~~~~~~ //
    bool begin() override;
    void readTemperature() override;
    void readPressure() override;
    void readTemperaturePressureAltitude() override;
    uint8_t whoAmI() override;

    // ~~~~~~~~~ Lps specific methods ~~~~~~~~ //

    /**
     * @brief Set the barometer ODR
     * @param odr the ODR value to set
     */
    void setOdr(LPS22HB_ENUM::ODR odr);

    /**
     * @brief Set the barometer bandwidth
     * @param bandwidth the bandwidth value to set, as an ODR ratio
     */
    void setLowPassFilterState(LPS22HB_ENUM::FILTER_BW bandwidth);

    /**
     * @brief Set the measurement check states.
     * @details This function allows you to set the check states for pressure and temperature
     * measurements. If set to false, the data availablility will ne be verified before reading the
     * data. It will be faster but less safe.
     * @param pressureCheck A boolean value indicating whether to check pressure measurements.
     * @param tempCheck A boolean value indicating whether to check temperature measurements.
     */
    void setMeasCheckStates(bool pressureCheck, bool tempCheck);

    /**
     * @brief Set the FIFO mode (BYPASS, FIFO, STREAM ...)
     * @param mode The FIFO mode to set
     */
    void setFifoMode(LPS22HB_ENUM::FIFO_MODE mode);

    /**
     * @brief Reads data from the FIFO buffer.
     * @note This function will be called automatically with readTemperaturePressureAltitude() if
     * the object has been constructed with the useFifo parameter set to true.
     */
    void readFifo();

    /**
     * @brief Set the One Point Calibration object
     * @param hPaDeltaPressureMeanRef delta between mean pressure measured by sensor and reference
     * pressure, as meanPressureMeasured - referencePressure in hPa
     */
    void setOnePointCalibration(float hPaDeltaPressureMeanRef);

    /**
     * @brief Reset the One Point Calibration object
     */
    void resetOnePointCalibration();

    /**
     * @brief Get the one point calibration from the sensor registers
     * @return int16_t The one point calibration value
     */
    int16_t getOnePointCalibration();

private:
    // Definitions coming from multiples objects and ambiguous
    streamLogger* mStreamObj = NULL; // NOLINT
    timerTool* mTimerObj     = NULL;
    using ABSTRACT_SENSOR_SPI::readRegister;
    using ABSTRACT_SENSOR_SPI::readRegisterBurst;
    using ABSTRACT_SENSOR_SPI::writeRegister;
    using ABSTRACT_SENSOR_SPI::writeRegisterWithMask;

    // Internal values
    LPS22HB_ENUM::ODR mBaroOdr      = LPS22HB_ENUM::ODR::_75HZ;
    LPS22HB_ENUM::FILTER_BW mBaroBw = LPS22HB_ENUM::FILTER_BW::ODR_DIV_2;

    struct // NOLINT
    {
        bool temp     = true;
        bool pressure = true;
    } mChecks;

    double mNsTimeOutMeasReading = 100000.0; // [ns] 100us timeout for meas reading //NOLINT
    bool mUseFifo                = false;
    float mDeltaPressureToOpcVal =
            17.F; // Do not know for this gain value, found experimentally... // NOLINT
    LPS22HB_ENUM::FIFO_MODE mFifoMode = LPS22HB_ENUM::FIFO_MODE::FIFO_OFF;

    LPS22HB_REGISTERS reg;
    LPS22HB_REGISTER_VALUES regVal;

    // ~~~~~~~~~~~ Private methods ~~~~~~~~~~~ //
    bool tryStart();
    LPS22HB_ENUM::STATUS getMeasuresStatus();
    void setFifoActivationState(bool state);
    uint8_t getFifoSize();

    // ~~~~~~~~~~~ Association maps ~~~~~~~~~~ //
    std::map<LPS22HB_ENUM::ODR, uint8_t> enum_odr_to_baro_odr_map = {
            {LPS22HB_ENUM::ODR::_75HZ, regVal.CTRL_REG1.ODR_75HZ},
            {LPS22HB_ENUM::ODR::_50HZ, regVal.CTRL_REG1.ODR_50HZ},
            {LPS22HB_ENUM::ODR::_25HZ, regVal.CTRL_REG1.ODR_25HZ},
            {LPS22HB_ENUM::ODR::_10HZ, regVal.CTRL_REG1.ODR_10HZ},
            {LPS22HB_ENUM::ODR::_1HZ, regVal.CTRL_REG1.ODR_1HZ}};

    std::map<LPS22HB_ENUM::FILTER_BW, uint8_t> enum_filter_bw_to_baro_bw_map = {
            {LPS22HB_ENUM::FILTER_BW::ODR_DIV_20, regVal.CTRL_REG1.LPF_ODR_DIV_20},
            {LPS22HB_ENUM::FILTER_BW::ODR_DIV_9, regVal.CTRL_REG1.LPF_ODR_DIV_9},
            {LPS22HB_ENUM::FILTER_BW::ODR_DIV_2, regVal.CTRL_REG1.LPF_DISABLE}};

    std::map<LPS22HB_ENUM::FIFO_MODE, uint8_t> enum_fifo_mode_to_fifo_mode_map = {
            {LPS22HB_ENUM::FIFO_MODE::BYPASS, regVal.FIFO_CTRL.FIFO_MODE_BYPASS},
            {LPS22HB_ENUM::FIFO_MODE::FIFO, regVal.FIFO_CTRL.FIFO_MODE_FIFO},
            {LPS22HB_ENUM::FIFO_MODE::STREAM, regVal.FIFO_CTRL.FIFO_MODE_STREAM},
            {LPS22HB_ENUM::FIFO_MODE::STREAM_TO_FIFO, regVal.FIFO_CTRL.FIFO_MODE_STREAM_TO_FIFO},
            {LPS22HB_ENUM::FIFO_MODE::BYPASS_TO_STREAM,
             regVal.FIFO_CTRL.FIFO_MODE_BYPASS_TO_STREAM},
            {LPS22HB_ENUM::FIFO_MODE::DYNAMIC_STREAM, regVal.FIFO_CTRL.FIFO_MODE_DYNAMIC_STREAM},
            {LPS22HB_ENUM::FIFO_MODE::BYPASS_TO_FIFO, regVal.FIFO_CTRL.FIFO_MODE_BYPASS_TO_FIFO}};

    // ~~~~~~~~~~~~~~ Class name ~~~~~~~~~~~~~ //
    String mClassName = "LPS22HB_SPI";
};

#endif