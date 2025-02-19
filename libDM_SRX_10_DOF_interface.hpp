#ifndef LIBDM_SRX_10_DOF_INTERFACE
#define LIBDM_SRX_10_DOF_INTERFACE

#include <iostream>
#include <array>
#include <memory>

class SRX_INS_10_DOF;
class SRX_MODEL_INS_10_DOF
{
public:
    // Local structures and enums
    /**
     * @brief Enumeration representing the different states of the heading through magnetic field
     * estimation
     */
    enum class INT_HEADING_STATES : uint32_t
    {
        unknown = 1,              /**< Unknown heading state */
        disturbed_dead_reckoning, /**< Magnetic field and hard irons have been correctly estimated
                                     but magnetometer is actually considered unreliable  */
        unprecise_with_wmm, /**< World magnetic model reference has been set but hard irons have not
                              been correctly estimated and magnetometer is considered unreliable. */
        precise_no_wmm, /**< Heading state is considered reliable because of correct magnetometer
                           measurements thresholds have been set without World Magnetic Model (WMM)
                         */
        precise_wmm,    /**< Heading state is considered reliable because of correct magnetometer
                           measurements thresholds have been set with World Magnetic    Model (WMM) and
                           are more precise than the automatically one sets */
        precise_wmm_gps /**< Heading state is considered reliable because of correct magnetometer
                           measurements thresholds have been set with World Magnetic Model (WMM) and
                           are more precise than the automatically one sets . Heading is GPS aided*/
    };

    SRX_MODEL_INS_10_DOF();

    // ====================================================== //
    // ================= Structs definitions ================ //
    // ====================================================== //

    /**
     * @brief This structure contains the inputs of the model
     * @details This structure has to be at least partially filled and passed to the model at each
     * step.
     */
    struct modelInputs
    {
        /**
         * @brief Gyro xyz rates in deg/s
         */
        std::array<float, 3> gyroDpsRate{{0.F, 0.F, 0.F}};

        /**
         * @brief Accelerometer xyz in G
         */
        std::array<float, 3> acceleroG{{0.F, 0.F, 0.F}};

        /**
         * @brief Magnetometer xyz in Gauss
         */
        std::array<float, 3> magGauss{{0.F, 0.F, 0.F}};

        /**
         * @brief Barometer altitude above mean sea level in meters
         */
        float barometerAltitudeAmsl = 0.F;

        /**
         * @brief Barometer altitude relative to the ground in meters
         */
        float barometerAltitudeRel = 0.F;

        /**
         * @brief Timestamp of the barometer in microseconds
         */
        uint64_t barometerTimestampUs = 0U;

        /**
         * @brief Timestamp of the magnetometer in microseconds
         */
        uint64_t magTimestampUs = 0U;

        // ~~~~~~~~~~~~~~ GPS inputs ~~~~~~~~~~~~~ //
        /**
         * @brief Heading accuracy of the GPS in radians
         */
        float gpsHeadingAccuracy = 0.F;

        /**
         * @brief Motion direction (motion heading) in radians
         */
        float gpsMotionHeading = 0.F;

        /**
         * @brief Validity of the GPS date
         */
        uint8_t gpsDateValid = 0U;

        /**
         * @brief Latitude of the GPS location in degrees
         */
        double degGpsLatitude = 0.0;

        /**
         * @brief Longitude of the GPS location in degrees
         */
        double degGpsLongitude = 0.0;

        /**
         * @brief Height of the GPS location above sea level in meters
         */
        float gpsHeightAboveSea = 0.F;

        /**
         * @brief Horizontal accuracy of the GPS location
         */
        float gpsHorizontalAccuracy = 0.F;

        /**
         * @brief Vertical accuracy of the GPS location
         */
        float gpsVerticalAccuracy = 0.F;

        /**
         * @brief Type of GPS fix
         * @details 0: no fix, 1: Dead reckoning, 2: 2D fix, 3: 3D fix, 4: GNSS + dead reckoning
         */
        uint8_t gpsFixType = 0U;

        /**
         * @brief Current hour according to the GPS time
         */
        uint8_t gpsHour = 0U;

        /**
         * @brief Current minute according to the GPS time
         */
        uint8_t gpsMinutes = 0U;

        /**
         * @brief Current month according to the GPS time
         */
        uint8_t gpsMonth = 0U;

        /**
         * @brief Current nanoseconds according to the GPS time
         */
        int32_t gpsNanoSeconds = 0;

        /**
         * @brief Number of satellites the GPS is currently connected to
         */
        uint8_t gpsNumSatellites = 0U;

        /**
         * @brief Current second according to the GPS time
         */
        uint8_t gpsSeconds = 0U;

        /**
         * @brief Current year according to the GPS time
         */
        uint16_t gpsYear = 0U;

        /**
         * @brief Current day according to the GPS time
         */
        uint8_t gpsDay = 0U;

        /**
         * @brief Accuracy of the GPS speed in m/s
         */
        float gpsSpeedAccuracy = 0.F;

        /**
         * @brief Ground speed of the GPS in m/s
         */
        float gpsSpeedGround = 0.F;

        /**
         * @brief North-East-Down (NED) speed of the GPS in m/s
         */
        std::array<float, 3> gpsSpeedNED{{0.F, 0.F, 0.F}};

        /**
         * @brief Timestamp of the GPS in microseconds
         */
        uint64_t gpsTimestampUs = 0U;

        /**
         * @brief Timestamp of the caling processor in microseconds
         */
        uint64_t timestampUs = 0U;

        /**
         * @brief Reset the model (set to true for one step)
         */
        bool reset = false;
    };

    /**
     * @brief This struct contains the outputs of the model.
     */
    struct modelOutputs
    {
        // ~~~~~~~~~~~~ Processed data ~~~~~~~~~~~ //

        /**
         * @brief Quaternion discribing body orientation relative to local frame
         * @note The quaternion is not globally aligned with NED with the magnetometer. It is
         * indeed much more reliable than the one from localToGlobalQuat.
         */
        const float* localToBodyQuat = nullptr;

        /**
         * @brief Quaternion discribing body orientation relative to global frame (NED)
         * @warning The quaternion is globally aligned with NED with the magnetometer. It is
         * subject to magnetometer disturbances and noise, especially if the sensor has not been
         * calibrated properly.
         */
        const float* nedToBodyQuat = nullptr;

        /**
         * @brief Quaternion linking local frame to global frame (NED) with the relation
         * quatGlobal = localToGlobalQuat x quatLocal
         * @details The quaternion is globally aligned with NED with the magnetometer. It is
         * subject to magnetometer disturbances and noise, especially if the sensor has not been
         * calibrated properly.
         */
        const float* localToGlobalQuat = nullptr;

        /**
         * @brief Yaw, pitch and roll in radians in local frame (yaw initialized to 0 at startup)
         * @details  This information is obtained from the decomposition of localToBodyQuat using
         * ZYX decomposition:
         * - yaw is defined between -pi and pi
         * - pitch is defined between -pi/2 and pi/2
         * - roll is defined between -pi and pi.
         * @note Yaw is not aligned with magnetometer and only correct slightly from it. The
         * informations from this fields are indeed much more reliable than the ones from
         * yawPitchRollGlobal.
         */
        const float* yawPitchRollLocal = nullptr;

        /**
         * @brief Yaw, pitch and roll in radians in global frame (NED)
         * @details Yaw is aligned with magnetometer and does not include declination that has to be
         * added to get a correct alignment with north. This information is obtained from the
         * decomposition of nedToBodyQuat using ZYX decomposition:
         * - yaw is defined between -pi and pi
         * - pitch is defined between -pi/2 and pi/2
         * - roll is defined between -pi and pi.
         * @warning As the magnetometer is used as main source for heading, the yaw component is
         * subject to magnetometer disturbances and noise, especially if the sensor has not been
         * calibrated properly.
         */
        const float* yawPitchRollGlobal = nullptr;

        /**
         * @brief Tilt angle in radians
         */
        const float* tiltAngle = nullptr;

        /**
         * @brief Yaw angle offsetted to match global yaw (oriented to north) in radians
         */
        const float* yawLocalOffsetted = nullptr;

        /**
         * @brief Yaw angle unwrapped in radians. Dimension size is 2 :
         * [yawLocalUnwrapped yawLocalOffsettedUnwrapped]
         */
        const float* agYawLocalUnwrapped = nullptr;

        /**
         * @brief Altitude above ground in meters
         */
        const float* dstAltitude = nullptr;

        /**
         * @brief Up speed in m/s
         */
        const float* spdUp = nullptr;

        // ~~~~~~~~ Corrected sensors data ~~~~~~~ //

        /**
         * @brief Body rates (unbiased gyro rates) in radians/s
         */
        const float* bodyRates = nullptr;

        /**
         * @brief Gyro bias in radians/s
         */
        const float* gyroBias = nullptr;

        /**
         * @brief Accelerometer unbiased in G
         */
        const float* accelerometerUnbiased = nullptr;

        /**
         * @brief Accelerometer bias in G
         */
        const float* accelerometerZBias = nullptr;

        /**
         * @brief Linear acceleration in m/s^2 in local frame
         */
        const float* linearAccelerationLocal = nullptr;

        /**
         * @brief Barometer altitude corrected in meters
         */
        const float* barometerAltitudeCorrected = nullptr;

        /**
         * @brief Barometer bias in meters
         */
        const float* barometerBias = nullptr;

        // ~~~~~~~~~~ Supervisor outputs ~~~~~~~~~ //

        /**
         * @brief 1 if the INS is immobile
         */
        const uint8_t* immobility = nullptr;

        /**
         * @brief 1 if the INS has been reset
         */
        const uint8_t* reset = nullptr;

        /**
         * @brief Timer of the model in seconds
         */
        const float* modelTimer = nullptr;

        /**
         * @brief Timer of the model in seconds since the last reset
         */
        const float* modelTimerFromReset = nullptr;

        // ~~ Magnetic field estimation outputs ~~ //

        /**
         * @brief Magnetic field estimator states (x6)
         * @details The states are [magFieldX magFieldY magFieldZ hardIronX hardIronY hardIronZ]
         */
        const float* klmMagStates = nullptr;

        /**
         * @brief Raw Magnetic Field Filtered Norm in Gauss. Dimension size is 2 :
         * [normMagHorizontal normMagVertical]
         */
        const float* rawMagNormFiltered = nullptr;

        /**
         * @brief Current vertical magnetic field norm acceptation threshold. Dimension is 2 :[min
         * max]
         */
        const float* vertMagNormThreshold = nullptr;

        /**
         * @brief Current horizontal magnetic field norm acceptation threshold. Dimension is 2 :[min
         * max]
         */
        const float* horizMagNormThreshold = nullptr;

        /**
         * @brief Current magnetic field estimation state
         */
        INT_HEADING_STATES headingState = INT_HEADING_STATES::unknown;

        /**
         * @brief Current estimated magnetic field horizontal norm
         */
        const float* gaussMagHorizNorm = nullptr;

        /**
         * @brief Current estimated magnetic field vertical norm
         */
        const float* gaussMagVertNorm = nullptr;

        // ~~~~~ World Magnetic model outputs ~~~~ //
        /**
         * @brief Word Magnetic Model informations has been set at least one time and can be used to
         * reject incorrect magnetic measurements
         */
        const uint8_t* localWmmInfosSet = nullptr;

        /**
         * @brief World Magnetic Model informations updated
         */
        const uint8_t* wmmUpdate = nullptr;

        /**
         * @brief World Magnetic Model reference horizontal norm for the current position
         */
        const float* gaussWmmMagRefHorizNorm = nullptr;

        /**
         * @brief World Magnetic Model reference norm for the current position
         */
        const float* gaussWmmMagRefNorm = nullptr;

        /**
         * @brief World Magnetic Model reference magnetic field values in Gauss for the current
         * position
         * @details The values are [North East Down] and are given in gauss
         */
        const float* gaussWmmMagRefNorthEastDown = nullptr;

        /**
         * @brief World Magnetic Model reference vertical norm for the current position
         */
        const float* gaussWmmMagRefVertNorm = nullptr;

        /**
         * @brief Computed decimal year from GPS date
         */
        const float* gpsDyear = nullptr;

        /**
         * @brief Magnetic declination in radians
         */
        const float* wmmDeclination = nullptr;

        /**
         * @brief Magnetic inclination in radians
         */
        const float* wmmInclination = nullptr;
    };

    /**
     * @brief This struct contains the parameters of the model.
     * @details This structure has to be filled before calling the initialize function.
     */
    struct modelParameters
    {
        // ~~~~~~~~ Sensors preprocessing ~~~~~~~~ //
        // Accelerometer ======================================================
        /**
         * @brief Accelerometer low pass filter denominator coefficients (6th order, 7
         * coefficients)
         */
        float* ACCPROC_lowPassFilterDen = nullptr;

        /**
         * @brief Accelerometer low pass filter numerator coefficients (6th order, 7
         * coefficients)
         */
        float* ACCPROC_lowPassFilterNum = nullptr;

        /**
         * @brief Accelerometer low pass filter initial gain (initGain = 1 / sum(den))
         */
        float* ACCPROC_lowPassFilterInitGain = nullptr;

        // Gyroscope ======================================================
        /**
         * @brief Gyroscope low pass filter denominator coefficients (6th order, 7 coefficients)
         */
        float* GYROPROC_lowPassFilterDen = nullptr;

        /**
         * @brief Gyroscope low pass filter numerator coefficients (6th order, 7 coefficients)
         */
        float* GYROPROC_lowPassFilterNum = nullptr;

        /**
         * @brief Gyroscope low pass filter initial gain (initGain = 1 / sum(den))
         */
        float* GYROPROC_lowPassFilterInitGain = nullptr;

        // Magnetometer ======================================================
        /**
         * @brief Magnetometer low pass filter denominator coefficients (4th order, 5
         * coefficients)
         */
        float* MAGPROC_lowPassFilterNum = nullptr;

        /**
         * @brief Magnetometer low pass filter numerator coefficients (4th order, 5
         * coefficients)
         */
        float* MAGPROC_lowPassFilterDen = nullptr;

        /**
         * @brief Magnetometer low pass filter initial gain (initGain = 1 / sum(den))
         */
        float* MAGPROC_lowPassFilterInitGain = nullptr;

        // Barometer ======================================================
        /**
         * @brief Barometer low pass filter numerator coefficients (4th order, 5 coefficients)
         */
        float* BAROPROC_lowPassFilterNum = nullptr;

        /**
         * @brief Barometer low pass filter denominator coefficients (4th order, 5 coefficients)
         */
        float* BAROPROC_lowPassFilterDen = nullptr;

        /**
         * @brief Barometer low pass filter initial gain (initGain = 1 / sum(den))
         */
        float* BAROPROC_lowPassFilterInitGain = nullptr;

        /**
         * @brief Barometer glitch low pass filter denominator coefficients (1st order, 2
         * coefficients)
         */
        float* BAROPROC_glitchLPFilterDen = nullptr;

        /**
         * @brief Barometer glitch low pass filter numerator coefficients (1st order, 2
         * coefficients)
         */
        float* BAROPROC_glitchLPFilterNum = nullptr;

        /**
         * @brief Barometer glitch low pass filter initial gain (initGain = 1 / sum(den))
         */
        float* BAROPROC_glitchLPFInitGain = nullptr;

        /**
         * @brief Barometer max threshold noise in meters.
         * @details If abs(baro-baro_glitch_filtered) > dstMaxThresholdNoise, the barometer is
         * considered as glitched and rejected
         */
        float* BAROPROC_dstMaxThresholdNoise = nullptr;

        // ~~~~~~~~~~~~ INS detectors ~~~~~~~~~~~~ //
        // Immobility detector ======================================================
        // A 10 samples window is used to detect immobility.The difference between the max and min
        // gyro rate is computed.

        /**
         * @brief Above this gyro rate difference, the immobility is set to false
         */
        float* DET_dpsImmoDisableThreshold = nullptr;

        /**
         * @brief Above this absolute gyro rate on any axis, the immobility is set to false
         */
        float* DET_dpsImmoDisableAbsGyro = nullptr;

        /**
         * @brief Below this gyro rate difference, the immobility is set to true after some time
         */
        float* DET_dpsFiltImmoEnableThreshold = nullptr;

        /**
         * @brief Time constant of the low pass filter used to compute the gyro rate difference
         */
        float* DET_immoGyroLowPassTimeConstant = nullptr;

        /**
         * @brief Time during which the gyro rate difference has to be below
         * DET_dpsFiltImmoEnableThreshold to set the immobility to true
         */
        float* DET_immoTimeConfirmation = nullptr;

        // ~~~~~~~ Kalman filter parameters ~~~~~~ //
        // States are
        // thetaErrorVectorErrBody (x3)
        // Bias gyro (x3)
        // DstZNed
        // SpdZNed

        // Sensors considered are
        // Accelerometer (x3)
        // BiasMeasures (in case of immobility) (x3)
        // Magnetometer (x3)
        // Barometer

        // Kalman filter main matrix ======================================================

        /**
         * @brief Initial covariance matrix (8x8)
         */
        float* KLM_P0mat = nullptr;

        /**
         * @brief Measurement noise covariance matrix (7x7)
         */
        float* KLM_Rmat = nullptr;

        /**
         * @brief Process noise covariance matrix (8x8)
         */
        float* KLM_Qmat = nullptr;

        // Barometer kalman special tuning ======================================================

        /**
         * @brief Gain applied to thetaErrorVector state and biases when in baro only mode
         */
        float* KLM_baroOnlyGainAngles = nullptr;

        /**
         * @brief Covariance gain for Q(SpdZNed, SpdZNed) when in baro only mode
         */
        float* KLM_baroOnlySpdCovGain = nullptr;

        /**
         * @brief Covariance gain for Rmat(baro) when in baro only mode
         */
        float* KLM_baroOnlyBaroNoiseCovGain = nullptr;

        /**
         * @brief Desensibilization table for Qmat and Rmat when in baro only mode (3x3 table)
         * @details
         * <table>
         * <caption id="multi_row">Default parameters</caption>
         * <tr><th>0<th>0.5<th>1<td> % Table axis: norm for [Wx Wy] body rate in rad/s
         * <tr><td>5<td>1<td>0.1<td> % gain for Rmat(baro)
         * <tr><td>1<td>10<td>100<td> % gain for Qmat(spdZ, spdZ)
         * </table>
         */
        float* KLM_baroOnlyRotMoveDesensibilizationTable = nullptr;

        /**
         * @brief Gain used for barometer bias estimation in immobility mode
         */
        float* KLM_gainBiasBarometer = nullptr;

        // Accelerometer kalman special tuning
        // ====================================================== At initialization, reinforce angle
        // adjustment on inclinometer

        /**
         * @brief Duration of the angle adjustment after reset (or initialization)
         */
        float* KLM_sAngleInitDuration = nullptr;

        /**
         * @brief Inclinometer noise covariance gain applied to Rmat for inclinometer corrections
         */
        float* KLM_angleInitCovInclinoNoiseGain = nullptr;

        /**
         * @brief Model angle covariance gain applied to Qmat for thetaErrorVector states
         */
        float* KLM_angleInitCovAngleGain = nullptr;

        /**
         * @brief If abs(1-norm(accelero)) is above this threshold, the inclino is rejected
         */
        float* KLM_gLinAccelLevelRejectionInclino = nullptr;

        // clang-format off
        /**
         * @brief Desensibilization table for Qmat and Rmat when in presence of non gravitational acceleration (5x3 table)
         * @details
         * <table>
         * <caption id="multi_row">Default parameters</caption>
         * <tr><th>0<th>0.2<th>0.75<td> % Table Axis: Accelerometer noise (1-norm(g))
         * <tr><td>1<td>400000*2<td>115000000<td> % Rmat, inclino noise desensibilization
         * <tr><td>1<td>1<td>2.85<td> % Qmat, theta angle errors desensibilization
         * <tr><td>1<td>0.002<td>0.1<td> % Qmat, bias desensibilization
         * <tr><td>1<td>0.01<td>0.02<td> % Increase baro angles/bias corrections (Qmat(spdZ, spdZ), Qmat(dstZ, dstZ)/gain)
         * <tr><td>1<td>1<td>1<td> % Reduce barometer noise to increase its corrections on angles
         * <tr><td>1<td>0.1<td>0.01<td> % With high level of noise, barometer will have more weight on angle and bias correction.
         * We can reduce the correction on the bias with this parameter
         * </table>
         */
        // clang-format on
        float* KLM_gNoiseDesensibilizationTable = nullptr;

        /**
         * @brief Gain applied to thetaErrorVector states covariances when in immobility mode
         */
        float* KLM_immobilityInclinoCovAngleGain = nullptr;

        /**
         * @brief Gain applied to accelerometer Rmat covariances when in immobility mode
         */
        float* KLM_immobilityInclinoNoiseCovGain = nullptr;

        /**
         * @brief Time constant of the low pass filter used to compute the inclinometer noise
         * (abs(1-norm(accelero)))
         */
        float* KLM_inclinoNoiseTau = nullptr;

        /**
         * @brief Gain applied for the accelerometer bias Z estimation state when in immobility mode
         */
        float* KLM_gainBiasZ = nullptr;

        // Magnetometer kalman special tuning ======================================================

        /**
         * @brief Gain used to compute global yaw (directed to north)
         */
        float* KLM_gainGlobalMagAngleCorrection = nullptr;

        /**
         * @brief Gain used to correct local yaw with magnetometer
         * @note The correction level should be low or zero to avoid magnetometer noise impact on
         * local estimation
         */
        float* KLM_gainLocalMagAngleCorrection = nullptr;

        /**
         * @brief If the difference between the current global yaw and new magnetometer heading is
         * above this threshold, a jump will be applied to the global yaw
         */
        float* KLM_headingJumpVal = nullptr;

        /**
         * @brief Maximum value of the local yaw angle correction
         */
        float* KLM_localMagAngleMaxVal = nullptr;

        /**
         * @brief This gain is used to adjust the local yaw offseted the global yaw.
         * @details The higher the gain, the faster the local yaw offsetted will be adjusted to the
         * global yaw. The default value is 1e-4
         */
        float* KLM_yawLocalOffsetAdjustmentGain = nullptr;

        // ~~~~~~~ Magnetic field Kalman filter parameters ~~~~~~ //
        // States are
        // Estimated Magnetic Field (x3)
        // Hard Iron (x3)

        // Sensors considered are
        // Magnetometer (x3)

        /**
         * @brief True if the magnetic field estimation is used
         *
         */
        uint8_t* KLMAG_bUseMagEst = nullptr;

        // Magnetic Field Estimation Kalman filter main matrix
        // ======================================================

        /**
         * @brief Initial covariance matrix (6x8)
         */
        float* KLMAG_P0mat = nullptr;

        /**
         * @brief Measurement noise covariance matrix (3x3)
         */
        float* KLMAG_Rmat = nullptr;

        /**
         * @brief Process noise covariance matrix (6x6)
         */
        float* KLMAG_Qmat = nullptr;

        // Kalman filter special tuning ======================================================

        /**
         * @brief Hard Iron Estimate Covariance threshold to consider it reliable
         */
        float* KLMAG_covHardIronReliable = nullptr;

        /**
         * @brief Magnetic Field Estimate Covariance threshold to consider it reliable
         */
        float* KLMAG_covMagFieldReliable = nullptr;

        /**
         * @brief Initial magnetic field horizontal norm threshold (min/max) to consider the
         * magnetic field as reliable (size 2 for [min max])
         */
        float* KLMAG_magHorizNormMinMax = nullptr;

        /**
         * @brief Initial magnetic field vertical norm threshold (min/max) to consider the magnetic
         * field as reliable (size 2 for [min max])
         */
        float* KLMAG_magVertNormMinMax = nullptr;

        /**
         * @brief Magnetic field horizontal norm tolerance threshold to consider the magnetic field
         * as reliable. The vector if of size 2 for a comparison vs [mean WorldMagneticModel]
         * @details Two modes are available to fix magnetic field thresholds. If World Magnetic
         * Model is available (given by valid GPS position), the thresholds are computed as follow:
         * threshold = mean WorldMagneticModel +/- tolerance. If World Magnetic Model is not
         * available, the thresholds are computed as follow: threshold = mean magField +/-
         * tolerance, with the meanMagField being considered after Kalman has converged. The
         * tolerance should be smaller if world magnetic model is available as the information is
         * more reliable than using the mean of estimated magnetic field
         */
        float* KLMAG_magVertNormThresold = nullptr;

        /**
         * @brief Magnetic field vertical norm tolerance threshold to consider the magnetic field as
         * reliable. The vector if of size 2 for a comparison vs [mean WorldMagneticModel]
         * @details Two modes are available to fix magnetic field thresholds. If World Magnetic
         * Model is available (given by valid GPS position), the thresholds are computed as follow:
         * threshold = mean WorldMagneticModel +/- tolerance. If World Magnetic Model is not
         * available, the thresholds are computed as follow: threshold = mean magField +/-
         * tolerance, with the meanMagField being considered after Kalman has converged. The
         * tolerance should be smaller if world magnetic model is available as the information is
         * more reliable than using the mean of estimated magnetic field
         */
        float* KLMAG_magHorizNormThresold = nullptr;

        /**
         * @brief For magnetic field validation, we use filtered norm. This parameter is used as
         * follow: magNormFiltered = KLMAG_magNormFilterUpdateRate * rawMagNorm +
         * (1-KLMAG_magNormFilterUpdateRate) * magNormFiltered. At 100Hz, a value of 0.1 is a good
         * compromise between reactivity and noise rejection
         */
        float* KLMAG_magNormFilterUpdateRate = nullptr;

        /**
         * @brief When magnetometer is rejected because of disturbance detection, we consider it to
         * become valid again after the count of valid values reaches this threshold. The default
         * value is 500 which is 5 seconds at 100Hz
         */
        uint32_t* KLMAG_magReliableCounterThreshold = nullptr;

        // World Magnetic Model ======================================================
        /**
         * @brief World Magnetic Model time threshold for update in seconds
         * @example If the model is updated every 10s, the value should be 10
         */
        float* WMMPROC_updateTimeThreshold = nullptr;
    };

    // ====================================================== //
    // =================== Public methods =================== //
    // ====================================================== //

    /**
     * @brief Run one step of the model
     * @details This function has to be called at the rate of the model (0.001s)
     */
    void step();

    /**
     * @brief Initialize the model and interfaces
     */
    void initialize();

    /**
     * @brief Set the Inputs used by the model
     * @param inputs is the struct containing the inputs
     */
    void setInputs(const modelInputs& inputs);

    /**
     * @brief Get the Outputs of the model
     * @return the struct containing the outputs
     */
    const modelOutputs& getOutputs();

    /**
     * @brief Get the Parameters of the model
     * @return the struct containing the references to parameters
     */
    inline modelParameters& getParameters()
    {
        return mModelParameters;
    }

private:
    modelOutputs mModelOuputs              = modelOutputs();
    modelParameters mModelParameters       = modelParameters();
    std::shared_ptr<SRX_INS_10_DOF> mRtObj = nullptr;
};

#endif // LIBDM_SRX_10_DOF_INTERFACE