#ifndef abstract_baro_hpp
#define abstract_baro_hpp

#include "libDM_abstract_sensors.hpp"

namespace ABSTRACT_BARO_ENUM

{
/**
 * @brief This file contains basic definitions and methods specific to barometric sensors.
 * @details The BARO_VALUES struct provides constants and utility functions for working with
 * barometric sensor data. It defines default values, invalid thresholds, and validation functions
 * for pressure, temperature, and altitude.
 */
struct BARO_VALUES
{
    static constexpr float DEFAULT_SEA_LEVEL_PRESSURE     = 1013.25F; // [hPa]
    static constexpr float DEFAULT_SEA_LEVEL_TEMPERATURE  = 15.F;     // [deg Celsius]
    static constexpr float INVALID_PRESSURE_THRESHOLD     = -10000.F;
    static constexpr float INVALID_TEMPERATURE_THRESHOLD  = -10000.F;
    static constexpr float INVALID_ALTITUDE_THRESHOLD     = -10000.F;
    static constexpr float INVALID_PRESSURE               = -100000.F;
    static constexpr float INVALID_TEMPERATURE            = -100000.F;
    static constexpr float INVALID_ALTITUDE               = -100000.F;
    static constexpr uint16_t DEFAULT_NB_AVERAGING_VALUES = 50U; // Typical measure number necessary
                                                                 // to get stable values

    static constexpr uint16_t DISCARDING_VALUES_NB =
            30U; // First values read from the sensor can be wrong in some cases, we discard them

    /**
     * @brief Check if the pressure value is valid.
     * @param pressure The pressure value to check.
     * @return true if the pressure is valid, false otherwise.
     */
    static bool isPressureValid(float pressure)
    {
        return pressure > INVALID_PRESSURE_THRESHOLD;
    }

    /**
     * @brief Check if the temperature value is valid.
     * @param temperature The temperature value to check.
     * @return true if the temperature is valid, false otherwise.
     */
    static bool isTemperatureValid(float temperature)
    {
        return temperature > INVALID_TEMPERATURE_THRESHOLD;
    }

    /**
     * @brief Check if the altitude value is valid.
     * @param altitude The altitude value to check.
     * @return true if the altitude is valid, false otherwise.
     */
    static bool isAltitudeValid(float altitude)
    {
        return altitude > INVALID_ALTITUDE_THRESHOLD;
    }
};
} // namespace ABSTRACT_BARO_ENUM

class ABSTRACT_BARO : public ABSTRACT_SENSOR
{
public:
    /**
     * @brief Structure representing the output of the barometric sensor.
     *
     * This structure contains the following information:
     * - hPaPressure: The pressure in hPa.
     * - degTemperature: The temperature in degrees Celsius.
     * - mAbsoluteAltitude: The altitude in meters compared to sea level.
     * - mRelativeAltitude: The altitude in meters compared to ground level (assuming ground
     * pressure reference has been set).
     * - usPressureTimestamp: The timestamp of the pressure measurement in microseconds.
     * - usTemperatureTimestamp: The timestamp of the temperature measurement in microseconds.
     */
    struct BARO_OUTPUT
    {
        float hPaPressure       = 0.F; // [hPa]
        float degTemperature    = 0.F; // [deg Celsius]
        float mAbsoluteAltitude = 0.F; // [m] compared to sea level
        float mRelativeAltitude = 0.F; // [m] compared to ground level (assuming ground pressure
                                       // reference has been set)
        uint32_t usPressureTimestamp    = 0U;
        uint32_t usTemperatureTimestamp = 0U;
    } mBaroOut;

    ABSTRACT_BARO(streamLogger* streamObj, timerTool* timerToolObj) :
        ABSTRACT_SENSOR(streamObj, timerToolObj) {};
    virtual ~ABSTRACT_BARO() = default;
    // Copy constructor
    ABSTRACT_BARO(const ABSTRACT_BARO& other) = default;
    // Copy assignment operator
    ABSTRACT_BARO& operator=(const ABSTRACT_BARO& other) = default;
    // Move constructor
    ABSTRACT_BARO(ABSTRACT_BARO&& other) noexcept = default;
    // Move assignment operator
    ABSTRACT_BARO& operator=(ABSTRACT_BARO&& other) noexcept = default;

    // ====================================================== //
    // ================= Struct definitions ================= //
    // ====================================================== //
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)

    /**
     * @brief Configuration settings for the abstract barometer sensor.
     * @details Correction values for BARO + reference pressure and temperature for altitude
     * computation
     */
    struct ABSTRACT_BARO_CFG
    {
        SENSOR_CORRECTIONS mBaroCorrection;
        float referencePressureAtSeaLevel =
                ABSTRACT_BARO_ENUM::BARO_VALUES::DEFAULT_SEA_LEVEL_PRESSURE; // [hPa] pressure at
                                                                             // sea level
        float referenceGroundPressure =
                ABSTRACT_BARO_ENUM::BARO_VALUES::DEFAULT_SEA_LEVEL_PRESSURE; // [hPa] pressure at
                                                                             // ground level
        float offsetGroundToSeaLevel = 0.F; // [m] offset between sea level and ground
        float referenceTemperature =
                ABSTRACT_BARO_ENUM::BARO_VALUES::DEFAULT_SEA_LEVEL_TEMPERATURE; // [deg Celsius]
        float opcDeltaPressure = 0.F; // [hPa] pressure difference between measured pressure and
                                      // reference measured pressure (meas-ref)
    } mCfg;

    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)

    // ====================================================== //
    // =================== Public methods =================== //
    // ====================================================== //
public:
    /**
     * @brief Sets the configuration (corrections) for the abstract barometer sensor.
     * @details This function allows you to set the configuration for the abstract barometer sensor.
     * The configuration is specified by the ABSTRACT_BARO_CFG structure.
     * @param correction The configuration to be set.
     */
    void setConfig(const ABSTRACT_BARO_CFG& correction);

    /**
     * @brief Get the configuration (corrections) of the abstract barometer.
     * @return const ABSTRACT_BARO_CFG& The configuration of the abstract barometer.
     */
    const ABSTRACT_BARO_CFG& getConfig();

    /**
     * @brief Get the output of the barometric sensor.
     * @return const BARO_OUTPUT& The output of the barometric sensor.
     */
    const BARO_OUTPUT& getOutput();

    // ~~~~~~~~~~~~~ Calibrations ~~~~~~~~~~~~ //

    bool calibBaro();
    std::array<float, 3> getBaroBias(uint32_t averagingNumber);
    void getBaroScaleFactor(uint32_t averagingNumber);

    /**
     * @brief Set the Reference Pressure and temperature for precise absolute measures
     * @details The altitude will be calculated with this reference pressure and temperature. If
     * some parameters are not set, default parameters will be used.
     * @param referenceGroundPressure ground pressure at ground level
     * @param referenceGroundAltitude altitude at ground level if available from external source
     * (map, GPS...). Parameters (groundOffset, referencePressureAtSeaLevel) will be updated
     * accordingly. Otherwise, if not specified, we will consider referenceSeaPressure as 1013.25
     * and groundPressure will be converted to altitude with this value, which will give a
     * groundOffset.
     */
    void configureReferenceParameters(
            float referenceGroundPressure    = ABSTRACT_BARO_ENUM::BARO_VALUES::INVALID_PRESSURE,
            float referenceGroundTemperature = ABSTRACT_BARO_ENUM::BARO_VALUES::INVALID_TEMPERATURE,
            float referenceGroundAltitude    = ABSTRACT_BARO_ENUM::BARO_VALUES::INVALID_ALTITUDE,
            uint16_t nbAveragingSamples =
                    ABSTRACT_BARO_ENUM::BARO_VALUES::DEFAULT_NB_AVERAGING_VALUES);

    // Diag
    bool DiagBaroRead(uint8_t Mode);

    // ====================================================== //
    // ================== Protected methods ================= //
    // ====================================================== //
protected:
    // ~~~~~~~~~~~~ Virtual method ~~~~~~~~~~~ //

    /**
     * @brief Returns the identification number of the sensor.
     * @return The identification number of the sensor.
     */
    virtual uint8_t whoAmI();

    /**
     * @brief Reads the temperature from the barometer sensor.
     */
    virtual void readTemperature();

    /**
     * @brief Reads the pressure from the barometer sensor.
     */
    virtual void readPressure();

    /**
     * @brief Reads the temperature, pressure, and altitude from the sensor.
     */
    virtual void readTemperaturePressureAltitude();

    /**
     * @brief Convert pressure to altitude with using reference pressure
     * @note This method is influenced by configuration set with configureReferenceParameters
     */
    virtual void readAltitude();

    /**
     * @brief Corrects the pressure measurements.
     * @details This method corrects the pressure measurements using the sensor's corrections
     * specified with setConfig.
     */
    virtual void correctPressureMeasures();

    /**
     * @brief Initializes the drift timer.
     * @details This function is responsible for initializing the drift timer used as axis for
     * temporal drift correction configuration
     */
    virtual void initDriftTimer();

    /**
     * @brief Updates the drift timer.
     * @details This function is responsible for updating the drift timer used as axis for temporal
     * drift correction configuration
     */
    virtual void updateDriftTimer();

private:
    CTRL_DM mCtrl;
    float mDriftTimerValue     = 0.F; // [s]
    float mDriftTimerValuePrev = 0.F; // [s]
    String mClassName          = "ABSTRACT_BARO";
};

#endif
