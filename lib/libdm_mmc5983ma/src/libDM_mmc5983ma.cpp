#include "libDM_mmc5983ma.hpp"

// Warning: A lot of registers are not readable!! So we have to store a local copy of what each
// register should contain

// ##################################################################### //
// ########################### INIT FUNCTIONS ########################## //
// ##################################################################### //

bool MMC5983MA_SPI::begin()
{
    if (!tryStart())
    {
        mStreamObj->printlnUlog(
                true,
                true,
                mClassName
                        + ": begin() error timeout, no response from sensor, will try so soft reset device");
        softResetSensor();

        if (!tryStart())
        {
            mStreamObj->printlnUlog(
                    true,
                    true,
                    mClassName
                            + ": begin() error timeout, still no response from sensor after soft reset, aborting");
            ABSTRACT_MAG::setBeginState(false);
            return false;
        }
    }

    mStreamObj->printlnUlog(
            true, true, mClassName + ": begin() success, sensor ID = " + String(reg.ID));

    softResetSensor();
    delay(100); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    ABSTRACT_MAG::setBeginState(true);
    return true;
}

bool MMC5983MA_SPI::tryStart()
{
    mTimerObj->startTiming(0);
#pragma unroll
    while (whoAmI() != regVal.MMC_ID)
    {
        if (mTimerObj->stopTiming(0) > mNsOperationTimeout)
            return false;

        delay(10); // NOLINT
    }

    return true;
}

// ##################################################################### //
// ######################## UTILITIES FUNCTIONS ######################## //
// ##################################################################### //

uint8_t MMC5983MA_SPI::whoAmI()
{
    return readRegister(0U, reg.ID | reg.READ_FLAG, 0U);
}

void MMC5983MA_SPI::softResetSensor()
{
    writeRegister(0U, reg.control_1, regVal.CONTROL_1.SW_RESET);
    mStreamObj->printlnUlog(true, true, mClassName + ": soft reset sensor");
    regMem.setZero();
    delay(10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

// ##################################################################### //
// ########################## CONFIG FUNCTIONS ######################### //
// ##################################################################### //

void MMC5983MA_SPI::activateAxis(bool activeX, bool activeYZ)
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte       = 0U;
    uint8_t activeXValue  = 0U;
    uint8_t activeYZValue = 0U;

    if (activeX)
        activeXValue = regVal.CONTROL_1.DISABLE_X_CHANNEL;
    if (activeYZ)
        activeYZValue = regVal.CONTROL_1.DISABLE_XY_CHANNELS;

    curByte = regMem.control_1;
    writeRegisterWithMask(
            0U, reg.control_1, curByte, activeXValue, regVal.CONTROL_1.DISABLE_X_CHANNEL_MASK);
    regMem.control_1 = regMem.updateVal(
            regMem.control_1, activeXValue, regVal.CONTROL_1.DISABLE_X_CHANNEL_MASK);

    curByte = regMem.control_1;
    writeRegisterWithMask(
            0U, reg.control_1, curByte, activeYZValue, regVal.CONTROL_1.DISABLE_XY_CHANNELS_MASK);
    regMem.control_1 = regMem.updateVal(
            regMem.control_1, activeYZValue, regVal.CONTROL_1.DISABLE_XY_CHANNELS_MASK);

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName
                                    + ": mag conf changed, X activation state = " + String(activeX)
                                    + ", XY activation state = " + String(activeYZ));
}

void MMC5983MA_SPI::setMagOdrBw(MMC5983MA_ENUM::ODR magFreq, MMC5983MA_ENUM::BW magBw)
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte   = 0U;
    uint8_t freqValue = 0U;
    uint8_t bwValue   = 0U;
    freqValue         = enum_odr_to_mag_odr_map.at(magFreq);
    bwValue           = enum_bw_to_mag_bw_map.at(magBw);

    // Set freq
    curByte = regMem.control_2;
    writeRegisterWithMask(0U, reg.control_2, curByte, freqValue, regVal.CONTROL_2.MEAS_FREQ_MASK);
    regMem.control_2 =
            regMem.updateVal(regMem.control_2, freqValue, regVal.CONTROL_2.MEAS_FREQ_MASK);

    curByte = readRegister(0U, reg.control_2 | reg.READ_FLAG, 0U);

    // Enable/disable continuous measurement
    if (magFreq != MMC5983MA_ENUM::ODR::_OFF)
        setContinuousModeState(true);
    else
        setContinuousModeState(false);

    // Set BW (no need to use mask, we do not wan any of the other options)
    writeRegister(0U, reg.control_1, bwValue);
    regMem.control_1 = bwValue;

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": mag conf updated, ODR = "
                                    + String(static_cast<uint16_t>(magFreq))
                                    + "Hz, BW = " + String(static_cast<uint16_t>(magBw)) + "Hz");
}

void MMC5983MA_SPI::setAutoSetResetState(bool state)
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte    = 0U;
    uint8_t stateValue = 0U;

    stateValue = state ? regVal.CONTROL_0.ENABLE_AUTO_SET_RESET :
                         regVal.CONTROL_0.DISABLE_AUTO_SET_RESET;

    curByte = regMem.control_0;
    writeRegisterWithMask(
            0U, reg.control_0, curByte, stateValue, regVal.CONTROL_0.AUTO_SET_RESET_MASK);
    regMem.control_0 =
            regMem.updateVal(regMem.control_0, stateValue, regVal.CONTROL_0.AUTO_SET_RESET_MASK);

    mStreamObj->printlnUlog(
            true,
            true,
            mClassName + ": mag conf updated, auto set/reset mode state = " + String(state));
    delay(1);
}

void MMC5983MA_SPI::setContinuousModeState(bool state)
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte    = 0U;
    uint8_t stateValue = 0U;

    stateValue = state ? regVal.CONTROL_2.ENABLE_CONTINUOUS_MEAS :
                         regVal.CONTROL_2.DISABLE_CONTINUOUS_MEAS;

    curByte = regMem.control_2;
    writeRegisterWithMask(
            0U, reg.control_2, curByte, stateValue, regVal.CONTROL_2.CONTINUOUS_MEAS_MASK);
    regMem.control_2 =
            regMem.updateVal(regMem.control_2, stateValue, regVal.CONTROL_2.CONTINUOUS_MEAS_MASK);

    mStreamObj->printlnUlog(
            true,
            true,
            mClassName + ": mag conf updated, continuous mode state = " + String(state));
    delay(1);
}

// TODO: periodic degauss??
void MMC5983MA_SPI::setPeriodicSetTime(MMC5983MA_ENUM::PERIODIC_SET_TIME time)
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte   = 0U;
    uint8_t timeValue = 0U;
    timeValue         = enum_periodic_set_to_mag_periodic_map.at(time);

    curByte = regMem.control_2;
    writeRegisterWithMask(
            0U, reg.control_2, curByte, timeValue, regVal.CONTROL_2.PERIODIC_SET_MASK);
    regMem.control_2 =
            regMem.updateVal(regMem.control_2, timeValue, regVal.CONTROL_2.PERIODIC_SET_MASK);

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": mag conf updated, periodic set time = "
                                    + String(static_cast<uint16_t>(time)));
}

// TODO: create method for posCurent/negCurrent

// TODO: seems to be hard degauss system, à renommer
void MMC5983MA_SPI::setPeriodicSetState(bool state)
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte    = 0U;
    uint8_t stateValue = 0U;

    stateValue =
            state ? regVal.CONTROL_2.ENABLE_PERIODIC_SET : regVal.CONTROL_2.DISABLE_PERIODIC_SET;

    curByte = regMem.control_2;

    writeRegisterWithMask(
            0U, reg.control_2, curByte, stateValue, regVal.CONTROL_2.ACTIVATION_PERIODIC_SET_MASK);
    regMem.control_2 = regMem.updateVal(
            regMem.control_2, stateValue, regVal.CONTROL_2.ACTIVATION_PERIODIC_SET_MASK);

    // This goes with two more registers to activate
    if (state)
    {
        setContinuousModeState(true);
        setAutoSetResetState(true);
    }

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": mag conf updated, periodic set state = " + String(state)
                                    + " continuous meas/ auto set-reset as well if true");
}

void MMC5983MA_SPI::setMeasCheckStates(bool magCheck, bool tempCheck)
{
    mChecks.mag  = magCheck;
    mChecks.temp = tempCheck;
}

// ##################################################################### //
// ####################### Calibration functions ####################### //
// ##################################################################### //

// TODO: à creuser... ça ne semble pas fonctionner
void MMC5983MA_SPI::updateBridgeOffset()
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t buf[6]          = {0U, 0U, 0U, 0U, 0U, 0U};
    uint16_t rawMagSet[3]   = {0U, 0U, 0U};
    uint16_t rawMagReset[3] = {0U, 0U, 0U};

    writeRegister(0U, reg.control_2, 0U);

    setCurrent();
    askForOneMagMeasure();
    delay(10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    readRegisterBurst(0U, reg.Xout0, buf, 6, 0U);
    rawMagSet[0] = ((uint16_t)buf[0] << 8) | buf[1];
    rawMagSet[1] = ((uint16_t)buf[2] << 8) | buf[3];
    rawMagSet[2] = ((uint16_t)buf[4] << 8) | buf[5];

    resetCurrent();
    askForOneMagMeasure();
    delay(10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    readRegisterBurst(0U, reg.Xout0, buf, 6, 0U);
    rawMagReset[0] = ((uint16_t)buf[0] << 8) | buf[1];
    rawMagReset[1] = ((uint16_t)buf[2] << 8) | buf[3];
    rawMagReset[2] = ((uint16_t)buf[4] << 8) | buf[5];

#pragma unroll
    for (size_t i = 0; i < 3; i++)
        mBridgeOffset[i] =
                (static_cast<float>(rawMagSet[i]) - static_cast<float>(rawMagReset[i])) * 0.5f;
}

// ##################################################################### //
// ################## SENSOR MANUAL READING FUNCTIONS ################## //
// ##################################################################### //

void MMC5983MA_SPI::setCurrent()
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte = 0U;

    curByte = regMem.control_0;
    writeRegisterWithMask(0U,
                          reg.control_0,
                          curByte,
                          regVal.CONTROL_0.SET_CURRENT,
                          regVal.CONTROL_0.SET_CURRENT_MASK);
    delayMicroseconds(1);
}

void MMC5983MA_SPI::resetCurrent()
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte = 0U;

    curByte = regMem.control_0;
    writeRegisterWithMask(0U,
                          reg.control_0,
                          curByte,
                          regVal.CONTROL_0.RESET_CURRENT,
                          regVal.CONTROL_0.RESET_CURRENT_MASK);
    delayMicroseconds(1);
}

void MMC5983MA_SPI::askForOneMagMeasure()
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte = 0U;

    curByte = regMem.control_0;
    writeRegisterWithMask(0U,
                          reg.control_0,
                          curByte,
                          regVal.CONTROL_0.TAKE_MAG_MEAS,
                          regVal.CONTROL_0.TAKE_MAG_MEAS_MASK);
    delayMicroseconds(
            10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

void MMC5983MA_SPI::askForOneTempMeasure()
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t curByte = 0U;

    curByte = regMem.control_0;
    writeRegisterWithMask(0U,
                          reg.control_0,
                          curByte,
                          regVal.CONTROL_0.TAKE_TEMP_MEAS,
                          regVal.CONTROL_0.TAKE_TEMP_MEAS_MASK);

    delayMicroseconds(
            10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

// ##################################################################### //
// ################### SENSOR AUTO READING FUNCTIONS ################### //
// ##################################################################### //

// Measures can be obtained in continuous of one by one:
// Continuous :
// // setAutoSetResetState(true);
// // setContinuousModeState(true);
// // readMag(), readTemp()

// One by one:
// // setCurrent() or resetCurrent() (not forced)
// // askForOneMagMeasure(), askForOneTempMeasure()
// // readMag(), readTemp()

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

MMC5983MA_ENUM::STATUS MMC5983MA_SPI::getMeasuresStatus() // 17us at 10MHz
{
    if (!ABSTRACT_MAG::checkBeginState())
        return MMC5983MA_ENUM::STATUS::NO_MEAS;

    uint8_t curByte = 0U;

    curByte = readRegister(0U, reg.Status | reg.READ_FLAG, 0U) & 0b11;
    curByte &= 0b11;

    if (curByte == regVal.STATUS.MEAS_ALL_DONE)
        return MMC5983MA_ENUM::STATUS::ALL_READY;

    if (curByte == regVal.STATUS.MEAS_M_DONE)
        return MMC5983MA_ENUM::STATUS::MAG_READY;

    if (curByte == regVal.STATUS.MEAS_T_DONE)
        return MMC5983MA_ENUM::STATUS::TEMP_READY;

    return MMC5983MA_ENUM::STATUS::NO_MEAS;
}

// askForOneTempMeasure() has to be called before.
void MMC5983MA_SPI::readTemp() // 110us at 10mHz, 77us without checks
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t temp = 0U;

    askForOneTempMeasure();

    if (mChecks.temp)
    {
        MMC5983MA_ENUM::STATUS status = getMeasuresStatus();

        mTimerObj->startTiming(0);
#pragma unroll
        while (status != MMC5983MA_ENUM::STATUS::ALL_READY
               && status != MMC5983MA_ENUM::STATUS::TEMP_READY)
        {
            if (mTimerObj->stopTiming(0) > mNsTimeOutMeasReading)
            {
                mStreamObj->printlnUlog(true, true, mClassName + ": temp measurement timeout");
                return;
            }

            delayMicroseconds(1);
            status = getMeasuresStatus();
        }
    }

    temp = readRegister(0U, reg.Temp | reg.READ_FLAG, 0U);

    mMagOut.degImuTemperature =
            mConversions.tempLow + static_cast<float>(temp) * mConversions.tempSlope;
    mMagOut.usTempTimestamp = mTimerObj->returnSystemTimestampUs();
}

// If not in continuous mode, need a askForOneMagMeasure() before
void MMC5983MA_SPI::readMag() // 116us at 10MHz, 84us without status check
{
    if (!ABSTRACT_MAG::checkBeginState())
        return;

    uint8_t buf[7]     = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint32_t rawMag[3] = {0U, 0U, 0U};
    uint8_t compMag    = 0U;

    if (mChecks.mag)
    {
        MMC5983MA_ENUM::STATUS status = getMeasuresStatus();
        mTimerObj->startTiming(0);
#pragma unroll
        while (status != MMC5983MA_ENUM::STATUS::ALL_READY
               && status != MMC5983MA_ENUM::STATUS::MAG_READY)
        {
            if (mTimerObj->stopTiming(0) > mNsTimeOutMeasReading)
            {
                mStreamObj->printlnUlog(true, true, mClassName + ": mag measurement timeout");
                return;
            }

            delayMicroseconds(1);
            status = getMeasuresStatus();
        }
    }

    readRegisterBurst(0U, reg.Xout0 | reg.READ_FLAG, buf, 7, 0U);

    compMag = (buf[6] >> 2) & 0b111111;
    rawMag[mag_axis.X] =
            ((uint32_t)buf[0] << 10) | (uint32_t)buf[1] << 2 | ((uint32_t)compMag >> 4 & 0b11);
    rawMag[mag_axis.Y] =
            ((uint32_t)buf[2] << 10) | (uint32_t)buf[3] << 2 | ((uint32_t)compMag >> 2 & 0b11);
    rawMag[mag_axis.Z] =
            ((uint32_t)buf[4] << 10) | (uint32_t)buf[5] << 2 | ((uint32_t)compMag & 0b11);

    // Convert to uncalibrated physical values
    mMagOut.GMagData[mag_axis.X] =
            mConversions.magLow + static_cast<float>(rawMag[mag_axis.X]) * mConversions.magSlope;
    mMagOut.GMagData[mag_axis.Y] =
            mConversions.magLow + static_cast<float>(rawMag[mag_axis.Y]) * mConversions.magSlope;
    mMagOut.GMagData[mag_axis.Z] =
            mConversions.magLow + static_cast<float>(rawMag[mag_axis.Z]) * mConversions.magSlope;

    // Apply calibration
    mMagOut.GMagData[mag_axis.X] =
            (mMagOut.GMagData[mag_axis.X] - mMagCorrection.xyzBias[mag_axis.X])
            * mMagCorrection.xyzGain[mag_axis.X];
    mMagOut.GMagData[mag_axis.Y] =
            (mMagOut.GMagData[mag_axis.Y] - mMagCorrection.xyzBias[mag_axis.Y])
            * mMagCorrection.xyzGain[mag_axis.Y];
    mMagOut.GMagData[mag_axis.Z] =
            (mMagOut.GMagData[mag_axis.Z] - mMagCorrection.xyzBias[mag_axis.Z])
            * mMagCorrection.xyzGain[mag_axis.Z];

    mMagOut.usMagTimestamp = mTimerObj->returnSystemTimestampUs();
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)