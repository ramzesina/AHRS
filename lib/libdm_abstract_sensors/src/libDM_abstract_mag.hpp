#ifndef abstract_mag_hpp
#define abstract_mag_hpp

#include "libDM_abstract_sensors.hpp"

class ABSTRACT_MAG : public ABSTRACT_SENSOR
{
public:
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
    /**
     * @brief Structure representing the output of the magnetometer.
     * @details This structure contains the magnetometer data, IMU temperature, and timestamps.
     */
    struct MAG_OUTPUT
    {
        std::array<float, 3> GMagData{{0.F, 0.F, 0.F}}; // [Gauss]
        float degImuTemperature  = 0.F;                 // [deg]
        uint32_t usMagTimestamp  = 0;
        uint32_t usTempTimestamp = 0;
    } mMagOut;

    /**
     * @brief Structure representing the axes of a magnetometer.
     */
    struct AXIS
    {
        uint8_t X = 0;
        uint8_t Y = 1;
        uint8_t Z = 2;
    } mag_axis;
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
    ABSTRACT_MAG(streamLogger* streamObj, timerTool* timerToolObj) :
        ABSTRACT_SENSOR(streamObj, timerToolObj) {};
    virtual ~ABSTRACT_MAG() = default;
    // Copy constructor
    ABSTRACT_MAG(const ABSTRACT_MAG& other) = default;
    // Copy assignment operator
    ABSTRACT_MAG& operator=(const ABSTRACT_MAG& other) = default;
    // Move constructor
    ABSTRACT_MAG(ABSTRACT_MAG&& other) noexcept = default;
    // Move assignment operator
    ABSTRACT_MAG& operator=(ABSTRACT_MAG&& other) noexcept = default;

    // ====================================================== //
    // ================= Struct definitions ================= //
    // ====================================================== //
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
protected:
    SENSOR_CORRECTIONS mMagCorrection;

    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)

    // ====================================================== //
    // =================== Public methods =================== //
    // ====================================================== //
public:
    /**
     * @brief Sets the configuration for the sensor.
     * @details This method sets the corrections (offsets, gains...) for the magnetometer.
     * @note This method has to be called to apply the corections factors obtained after
     * magnetometer calibration.
     * @param correction The SENSOR_CORRECTIONS object containing the desired configuration.
     */
    void setConfig(const SENSOR_CORRECTIONS& correction);

    /**
     * @brief Returns the configuration (corrections) struct used by the sensor.
     * @return const SENSOR_CORRECTIONS& The reference to the sensor configuration.
     */
    const SENSOR_CORRECTIONS& getConfig();

    /**
     * @brief Returns the output of the magnetometer.
     * @return const MAG_OUTPUT& The reference to the magnetometer output.
     */
    const MAG_OUTPUT& getOutput();

    /**
     * @brief Peudo-calibrates the magnetometer and outputs the corrections results.
     * @details This method calibrates the magnetometer and stores the corrections in the
     * mMagCorrection struct. It also print the correction in Serial monitor and log it to SD card.
     * @param msDuration The duration of the calibration in milliseconds.
     * @return bool True if the calibration was successful, false otherwise.
     */
    bool calibMag(uint32_t msDuration);

    // Diag
    bool DiagMagRead(uint8_t Mode);

    // ====================================================== //
    // ================== Protected methods ================= //
    // ====================================================== //
protected:
    // ~~~~~~~~~~~~ Virtual method ~~~~~~~~~~~ //

    /**
     * @brief Reads the temperature from the sensor and store it in the mMagOut struct.
     */
    virtual void readTemp();

    /**
     * @brief Reads the magnetometer data from the sensor and store it in the mMagOut struct.
     */
    virtual void readMag();

    /**
     * @brief Returns the identification number of the sensor.
     * @return The identification number of the sensor.
     */
    virtual uint8_t whoAmI();

    // ====================================================== //
    // ================== Protected values ================== //
    // ====================================================== //

private:
    bool resizeMagSphere(uint32_t msDuration);

    uint32_t maxErrorsTolerated                = 50U;
    float maxGDeltaCoherency                   = 0.2F; //[G] at 100Hz
    float noGainComplementaryMinMaxComputation = 0.2F;

    String mClassName = "ABSTRACT_MAG";
};

#endif
