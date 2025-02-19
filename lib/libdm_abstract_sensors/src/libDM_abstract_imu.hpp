#ifndef abstract_imu_hpp
#define abstract_imu_hpp

#include "libDM_abstract_sensors.hpp"

constexpr float MAX_DPS_FOR_GYRO_BIAS = 5.F;

class ABSTRACT_IMU : public ABSTRACT_SENSOR
{
public:
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
    /**
     * @brief Structure representing the output data from the IMU sensor.
     * @details This structure contains the following data:
     * - gAccelData: Acceleration data in units of g.
     * - dpsGyroData: Gyroscope data in units of degrees per second.
     * - utMagData: Magnetometer data in units of microtesla.
     * - degImuTemperature: IMU temperature in degrees.
     * - usMagTimestamp: Timestamp for magnetometer data.
     * - usGyroTimestamp: Timestamp for gyroscope data.
     * - usAccelTimestamp: Timestamp for acceleration data.
     * - usTempTimestamp: Timestamp for IMU temperature data.
     */
    struct IMU_OUTPUT
    {
        std::array<float, 3> gAccelData{{0.F, 0.F, 0.F}};  // [g]
        std::array<float, 3> dpsGyroData{{0.F, 0.F, 0.F}}; // [dps]
        std::array<float, 3> utMagData{{0.F, 0.F, 0.F}};   // [uT]
        float degImuTemperature   = 0.F;                   // [deg]
        uint32_t usMagTimestamp   = 0;
        uint32_t usGyroTimestamp  = 0;
        uint32_t usAccelTimestamp = 0;
        uint32_t usTempTimestamp  = 0;
    } mImuOut;
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
    ABSTRACT_IMU(streamLogger* streamObj, timerTool* timerToolObj) :
        ABSTRACT_SENSOR(streamObj, timerToolObj) {};
    virtual ~ABSTRACT_IMU() = default;
    // Copy constructor
    ABSTRACT_IMU(const ABSTRACT_IMU& other) = default;
    // Copy assignment operator
    ABSTRACT_IMU& operator=(const ABSTRACT_IMU& other) = default;
    // Move constructor
    ABSTRACT_IMU(ABSTRACT_IMU&& other) noexcept = default;
    // Move assignment operator
    ABSTRACT_IMU& operator=(ABSTRACT_IMU&& other) noexcept = default;

    // ====================================================== //
    // ================= Struct definitions ================= //
    // ====================================================== //
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)

    /**
     * @brief Configuration structure for the IMU sensor until 9 axis (including magnetometer)
     * @details This structure holds the sensor corrections for the accelerometer, gyroscope, and
     * magnetometer.
     */
    struct ABSTRACT_IMU_CFG
    {
        SENSOR_CORRECTIONS acc;
        SENSOR_CORRECTIONS gyro;
        SENSOR_CORRECTIONS mag;
    } mImuCorrection;

    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)

    // ====================================================== //
    // =================== Public methods =================== //
    // ====================================================== //
public:
    enum class SENSOR_TYPE : uint8_t
    {
        ACC  = 0,
        GYRO = 1,
        MAG  = 2
    };
    struct AXIS
    {
        uint8_t X = 0;
        uint8_t Y = 1;
        uint8_t Z = 2;
    } imu_axis;

    /**
     * @brief Sets the configuration for the IMU sensor.
     * @details This function allows you to set the configuration for the IMU sensor (accelerometer,
     * gyroscope, and magnetometer).
     * @param type The type of sensor (accelerometer, gyroscope, or magnetometer)
     * @param correction The sensor corrections to be applied.
     */
    void setConfig(SENSOR_TYPE type, const SENSOR_CORRECTIONS& correction);

    /**
     * @brief Retrieves the configuration for a specific sensor type.
     * @param type The type of sensor (accelerometer, gyroscope, or magnetometer)
     * @return The configuration for the specified sensor type.
     */
    const SENSOR_CORRECTIONS& getConfig(SENSOR_TYPE type);

    /**
     * @brief Retrieves the output of the IMU sensor.
     * @return The output of the IMU sensor.
     */
    const IMU_OUTPUT& getOutput();

    // ~~~~~~~~~~~~~ Calibrations ~~~~~~~~~~~~ //

    /**
     * @brief Calibrates the gyroscope by performing averaging over a specified number of samples.
     * @warning This method should be called before using the gyroscope, with keeping the IMU for
     * some time without moving. The calibration will be interrupted if the IMU is moved. You can
     * call checkImmobility before calling this method
     * @param averagingNumber The number of samples to average over for gyroscope calibration.
     */
    void calibGyro(uint32_t averagingNumber);

    void calibAcc();
    void calibMag();

    // Diag
    bool DiagImuRead(uint8_t Mode);

    // ~~~~~~~~~~~~~ helpers ~~~~~~~~~~~~ //

    /**
     * @brief Checks if the IMU sensor is immobile for a certain duration.
     * @details This function checks if the IMU sensor is immobile by comparing the change in
     * angular velocity (dps) with a given threshold. The duration of immobility is determined by
     * the msDurationThreshold parameter. The msImuPeriod parameter specifies the period at which
     * the IMU sensor readings are obtained. The msTimeOut parameter specifies the maximum time
     * allowed for the immobility check.
     * @example checkImmobility(0.3F, 200U, 1U, 1000U)
     * @note This method can be called as a condition to start a gyro calibration.
     * @param dpsThreshold The threshold for change in angular velocity (dps) to determine
     * immobility.
     * @param msDurationThreshold The duration of immobility required to consider the sensor as
     * immobile.
     * @param msImuPeriod The period in ms at which the IMU sensor readings are obtained (poling
     * rate)
     * @param msTimeOut The maximum time allowed for the immobility check.
     * @return true if the IMU sensor is immobile for the specified duration, false otherwise.
     */
    bool checkImmobility(float dpsThreshold,
                         uint32_t msDurationThreshold,
                         uint32_t msImuPeriod,
                         uint32_t msTimeOut);

    // ====================================================== //
    // ================== Protected methods ================= //
    // ====================================================== //
protected:
    // ~~~~~~~~~~~~ Virtual method ~~~~~~~~~~~ //

    /**
     * @brief Reads the temperature from the IMU sensor and stores it in the mImuOut struct.
     */
    virtual void readTemp();

    /**
     * @brief Reads the acceleration data from the IMU sensor and stores it in the mImuOut struct.
     */
    virtual void readAcc();

    /**
     * @brief Reads the gyroscope data from the IMU sensor and stores it in the mImuOut struct.
     */
    virtual void readGyro();

    /**
     * @brief Reads the magnetometer data from the IMU sensor and stores it in the mImuOut struct.
     */
    virtual void readMag();

    /**
     * @brief Reads all the data from the IMU sensor and stores it in the mImuOut struct.
     */
    virtual void readAllImu();

    /**
     * @brief Reads the acceleration and gyroscope data from the IMU sensor and stores it in the
     * mImuOut struct.
     */
    virtual void readAccGyro();

    /**
     * @brief Returns the identification number of the IMU sensor.
     * @return The identification number of the IMU sensor.
     */
    virtual uint8_t whoAmI();

    // ====================================================== //
    // ================== Protected values ================== //
    // ====================================================== //

private:
    // ====================================================== //
    // =================== Private methods ================== //
    // ====================================================== //
    std::array<float, 3> getGyroBias(uint32_t averagingNumber, float dpsInterruptThreshold);
    void getGyroScaleFactor(uint32_t averagingNumber);
    std::array<float, 3> getAccBias(uint32_t averagingNumber);
    void getAccScaleFactor(uint32_t averagingNumber);

    // ====================================================== //
    // =================== Private values =================== //
    // ====================================================== //

    String mClassName = "ABSTRACT_IMU";
};

#endif
