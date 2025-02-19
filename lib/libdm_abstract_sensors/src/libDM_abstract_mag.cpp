#define ERR_INVALID 255

#include "libDM_abstract_mag.hpp"
#pragma push_macro("abs")
#undef abs

void ABSTRACT_MAG::setConfig(const SENSOR_CORRECTIONS& correction)
{
    mMagCorrection = correction;
}

const ABSTRACT_MAG::SENSOR_CORRECTIONS& ABSTRACT_MAG::getConfig()
{
    return mMagCorrection;
}

const ABSTRACT_MAG::MAG_OUTPUT& ABSTRACT_MAG::getOutput()
{
    return mMagOut;
}

bool ABSTRACT_MAG::calibMag(uint32_t msDuration)
{
    if (!checkBeginState())
        return false;

    return resizeMagSphere(msDuration);
}

bool ABSTRACT_MAG::resizeMagSphere(uint32_t msDuration)
{
    if (!checkBeginState())
        return false;

    std::array<float, 3> magMin{{30000.F, 30000.F, 30000.F}};          // [G]
    std::array<float, 3> magMax{{-30000.F, -30000.F, -30000.F}};       // [G]
    std::array<float, 3> lastValidMag{{-30000.F, -30000.F, -30000.F}}; //[G]
    uint32_t samplesFreq  = 100U;
    uint32_t nsSampleTime = 100000000 / 100;
    uint32_t samplesCount = 0U;

    // Copy calibration data to restore if failure during calibration
    SENSOR_CORRECTIONS oldMagCorrection = mMagCorrection;
    mMagCorrection                      = SENSOR_CORRECTIONS();

    // Consider 100Hz frequency
    samplesCount = msDuration / 1000U * samplesFreq;

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": Starting magnetometer calibration in 5s, "
                                    + "rotation following each axe during "
                                    + String(msDuration / 1000U) + "s");

    delay(5000);

    mStreamObj->printlnUlog(true, true, mClassName + ": Starting magnetometer calibration now");

    // shoot for ~fifteen seconds of mag data
    uint32_t count         = 0U;
    uint32_t errors        = 0U;
    uint32_t lastTimestamp = 0U;
    mTimerObj->startTiming(0);
    while (count < samplesCount)
    {
        if (mTimerObj->stopTiming(0) > nsSampleTime)
        {
            mTimerObj->startTiming(0);
            readMag();

            if (mMagOut.usMagTimestamp != lastTimestamp)
            {
                lastTimestamp = mMagOut.usMagTimestamp;

                for (uint8_t j = 0; j < 3; j++)
                {
                    if (std::abs(mMagOut.GMagData[j]) < 7.9F)
                    {
                        // Check coherency
                        if (count == 0 || lastValidMag[j] < -20000.F)
                        {
                            // Init
                            lastValidMag[j] = mMagOut.GMagData[j];
                            magMax[j]       = mMagOut.GMagData[j];
                            magMin[j]       = mMagOut.GMagData[j];
                        }
                        else
                        {
                            if (std::abs(lastValidMag[j] - mMagOut.GMagData[j])
                                < maxGDeltaCoherency)
                            {
                                lastValidMag[j] = mMagOut.GMagData[j];

                                if (mMagOut.GMagData[j] > magMax[j])
                                    magMax[j] += noGainComplementaryMinMaxComputation
                                                 * (mMagOut.GMagData[j] - magMax[j]);
                                if (mMagOut.GMagData[j] < magMin[j])
                                    magMin[j] += noGainComplementaryMinMaxComputation
                                                 * (mMagOut.GMagData[j] - magMin[j]);
                            }
                            else
                            {
                                errors++;
                            }
                        }
                    }
                    else
                    {
                        errors++;
                    }
                }
            }
            count++;
            delay(10);
        }
    }

    if (errors > maxErrorsTolerated)
    {
        mMagCorrection = oldMagCorrection;
        mStreamObj->printlnUlog(true,
                                true,
                                mClassName + ": calibration failed, too much errors ("
                                        + String(errors) + "), old calibration has been restored");
        return false;
    }

    // Get hard iron correction
    mMagCorrection.xyzBias[0] = (magMax[0] + magMin[0]) / 2.F; // get average x mag bias
    mMagCorrection.xyzBias[1] = (magMax[1] + magMin[1]) / 2.F; // get average y mag bias
    mMagCorrection.xyzBias[2] = (magMax[2] + magMin[2]) / 2.F; // get average z mag bias

    String curString = mClassName + ": Magnetometer hard iron bias = ["
                       + String(mMagCorrection.xyzBias[0], 4) + ", "
                       + String(mMagCorrection.xyzBias[1], 4) + ", "
                       + String(mMagCorrection.xyzBias[2], 4) + "]";
    mStreamObj->printlnUlog(true, true, curString);

    // Get soft iron correction
    mMagCorrection.xyzGain[0] =
            (magMax[0] - magMin[0]) / 2.F; // get average x axis max chord length
    mMagCorrection.xyzGain[1] =
            (magMax[1] - magMin[1]) / 2.F; // get average y axis max chord length
    mMagCorrection.xyzGain[2] =
            (magMax[2] - magMin[2]) / 2.F; // get average z axis max chord length

    // Normalize sphere
    float avgGain =
            (mMagCorrection.xyzGain[0] + mMagCorrection.xyzGain[1] + mMagCorrection.xyzGain[2])
            / 3.F;

    mMagCorrection.xyzGain[0] = avgGain / mMagCorrection.xyzGain[0];
    mMagCorrection.xyzGain[1] = avgGain / mMagCorrection.xyzGain[1];
    mMagCorrection.xyzGain[2] = avgGain / mMagCorrection.xyzGain[2];

    curString = mClassName + ": magnetometer soft iron = [" + String(mMagCorrection.xyzGain[0], 4)
                + ", " + String(mMagCorrection.xyzGain[1], 4) + ", "
                + String(mMagCorrection.xyzGain[2], 4) + "]";
    mStreamObj->printlnUlog(true, true, curString);

    mStreamObj->printlnUlog(true, true, mClassName + ": magnetometer calibration successful");

    return true;
}

// ====================================================== //
// ================= Default definitions ================ //
// ====================================================== //
void ABSTRACT_MAG::readTemp()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readTemp() method not implemented in this context");
}
void ABSTRACT_MAG::readMag()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readMag() method not implemented in this context");
}

uint8_t ABSTRACT_MAG::whoAmI()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": whoAmI() method not implemented in this context");
    return ERR_INVALID;
}

#pragma pop_macro("abs")