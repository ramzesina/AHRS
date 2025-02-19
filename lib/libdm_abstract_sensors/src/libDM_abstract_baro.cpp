#define ERR_INVALID 255

#include "libDM_abstract_baro.hpp"

void ABSTRACT_BARO::setConfig(const ABSTRACT_BARO_CFG& cfg)
{
    mCfg = cfg;
}

const ABSTRACT_BARO::ABSTRACT_BARO_CFG& ABSTRACT_BARO::getConfig()
{
    return mCfg;
}

const ABSTRACT_BARO::BARO_OUTPUT& ABSTRACT_BARO::getOutput()
{
    return mBaroOut;
}

void ABSTRACT_BARO::initDriftTimer()
{
    mDriftTimerValue     = 0.F;
    mDriftTimerValuePrev = mTimerObj->returnSystemCounterSecondsFloat();
}

void ABSTRACT_BARO::updateDriftTimer()
{
    mDriftTimerValue += (mTimerObj->returnSystemCounterSecondsFloat() - mDriftTimerValuePrev);
    mDriftTimerValuePrev = mDriftTimerValue;
}

bool ABSTRACT_BARO::calibBaro()
{
    // TODO: implement calibration method
    return false;
}

void ABSTRACT_BARO::correctPressureMeasures()
{
    float temporalDrift    = 0.0F;
    float temperatureDrift = 0.0F;

    // Generic bias and scale factor correction
    mBaroOut.hPaPressure = (mBaroOut.hPaPressure - mCfg.mBaroCorrection.xyzBias[2])
                           * mCfg.mBaroCorrection.xyzGain[2];

    // Temperature drift correction
    temperatureDrift =
            mCtrl.Carto1D_DM(mCfg.mBaroCorrection.TEMPERATURE_DRIFT.degTemperatureAxis.data(),
                             mCfg.mBaroCorrection.TEMPERATURE_DRIFT.driftValue.data(),
                             mBaroOut.degTemperature,
                             mCfg.mBaroCorrection.TEMPERATURE_DRIFT.degTemperatureAxis.size());

    // Temporal drift correction
    updateDriftTimer();
    temporalDrift = mCtrl.Carto1D_DM(mCfg.mBaroCorrection.TEMPORAL_DRIFT.sTimeAxis.data(),
                                     mCfg.mBaroCorrection.TEMPORAL_DRIFT.driftValue.data(),
                                     mDriftTimerValue,
                                     mCfg.mBaroCorrection.TEMPORAL_DRIFT.sTimeAxis.size());

    mBaroOut.hPaPressure -= (temperatureDrift + temporalDrift);
}

// ====================================================== //
// ================= Default definitions ================ //
// ====================================================== //
void ABSTRACT_BARO::readTemperature()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readTemp() method not implemented in this context");
}

void ABSTRACT_BARO::readPressure()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readPressure() method not implemented in this context");
}

void ABSTRACT_BARO::readTemperaturePressureAltitude()
{
    mStreamObj->printlnUlog(
            false,
            false,
            mClassName
                    + ": readTemperaturePressureAltitude() method not implemented in this context");
}

void ABSTRACT_BARO::readAltitude()
{
    if (!checkBeginState())
        return;

    if (mBaroOut.hPaPressure == 0.0F)
    {
        mBaroOut.mAbsoluteAltitude = 0.0F;
        mBaroOut.mRelativeAltitude = 0.0F;
        return;
    }

    // Barometric formula
    // mBaroOut.mAbsoluteAltitude =
    //         44330.0F
    //         * (1.0F
    //            - std::pow(mBaroOut.hPaPressure / mCfg.referencePressureAtSeaLevel,
    //                       0.190294957183635F));

    // Hypsometric formula
    mBaroOut.mAbsoluteAltitude =
            (mCfg.referenceTemperature + 273.15F) * 153.8461538461538F
            * (std::pow(mCfg.referencePressureAtSeaLevel / mBaroOut.hPaPressure, 0.190294957183635F)
               - 1.F);
    mBaroOut.mRelativeAltitude = mBaroOut.mAbsoluteAltitude - mCfg.offsetGroundToSeaLevel;
}

void ABSTRACT_BARO::configureReferenceParameters(float referenceGroundPressure,
                                                 float referenceGroundTemperature,
                                                 float referenceGroundAltitude,
                                                 uint16_t nbAveragingSamples)
{
    if (!checkBeginState())
        return;

    float measPressure    = 0.0F;
    float measTemperature = 0.0F;
    float measAltitude    = 0.0F;

    // Update measures to complete unspecified reference parameters
    mCfg.referencePressureAtSeaLevel = ABSTRACT_BARO_ENUM::BARO_VALUES::DEFAULT_SEA_LEVEL_PRESSURE;
    uint16_t samplesCounter          = 0U;
    uint16_t discardedValuesCounter  = 0U;
    uint64_t curTimeStamp            = 0U;

    while (samplesCounter < nbAveragingSamples)
    {
        readTemperaturePressureAltitude();

        if (mBaroOut.usPressureTimestamp != curTimeStamp)
        {
            curTimeStamp = mBaroOut.usPressureTimestamp;

            if (discardedValuesCounter < ABSTRACT_BARO_ENUM::BARO_VALUES::DISCARDING_VALUES_NB)
            {
                discardedValuesCounter++;
                delay(50);
                continue;
            }

            measPressure += mBaroOut.hPaPressure;
            measTemperature += mBaroOut.degTemperature;
            measAltitude += mBaroOut.mAbsoluteAltitude;

            samplesCounter++;
        }

        delay(50);
    }
    measPressure /= nbAveragingSamples;
    measTemperature /= nbAveragingSamples;
    measAltitude /= nbAveragingSamples;

    if (ABSTRACT_BARO_ENUM::BARO_VALUES::isTemperatureValid(referenceGroundTemperature))
        mCfg.referenceTemperature = referenceGroundTemperature;
    else
        mCfg.referenceTemperature = measTemperature;

    if (ABSTRACT_BARO_ENUM::BARO_VALUES::isAltitudeValid(referenceGroundAltitude))
    {
        mCfg.offsetGroundToSeaLevel = referenceGroundAltitude;

        if (ABSTRACT_BARO_ENUM::BARO_VALUES::isPressureValid(referenceGroundPressure))
            mCfg.referenceGroundPressure = referenceGroundPressure;
        else
            mCfg.referenceGroundPressure = measPressure;

        // Reproject pressure to sea level
        mCfg.referencePressureAtSeaLevel =
                mCfg.referenceGroundPressure
                * std::pow((1.F
                            + (0.0065F * mCfg.offsetGroundToSeaLevel)
                                      / (mCfg.referenceTemperature + 273.15F)),
                           5.255F);
    }
    else
    {
        mCfg.offsetGroundToSeaLevel  = measAltitude;
        mCfg.referenceGroundPressure = measPressure;
    }

    mStreamObj->printlnUlog(false,
                            true,
                            mClassName + ":\nreference ground pressure has been updated to: "
                                    + String(mCfg.referenceGroundPressure)
                                    + " hPa \nreference sea pressure update to "
                                    + String(mCfg.referencePressureAtSeaLevel)
                                    + " hPa \noffset ground to sea level update to "
                                    + String(mCfg.offsetGroundToSeaLevel)
                                    + " m \nreference temperature updated to "
                                    + String(mCfg.referenceTemperature) + " °C");
}

uint8_t ABSTRACT_BARO::whoAmI()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": whoAmI() method not implemented in this context");
    return ERR_INVALID;
}
