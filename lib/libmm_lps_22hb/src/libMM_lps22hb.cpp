#include "libMM_lps22hb.hpp"

// ##################################################################### //
// ########################### INIT FUNCTIONS ########################## //
// ##################################################################### //

bool LPS22HB_SPI::begin()
{
    if (!tryStart())
    {
        mStreamObj->printlnUlog(
                true,
                true,
                mClassName
                        + ": begin() error timeout, no response from sensor, will try so soft reset device");
        // TODO: RESET sensor

        if (!tryStart())
        {
            mStreamObj->printlnUlog(
                    true,
                    true,
                    mClassName
                            + ": begin() error timeout, still no response from sensor after soft reset, aborting");
            ABSTRACT_BARO::setBeginState(false);
            return false;
        }
    }

    mStreamObj->printlnUlog(
            true, true, mClassName + ": begin() success, sensor ID = " + String(regVal.ID));

    delay(100); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    ABSTRACT_BARO::setBeginState(true);
    initDriftTimer();
    return true;
}

bool LPS22HB_SPI::tryStart()
{
    mTimerObj->startTiming(0);
#pragma unroll
    while (whoAmI() != regVal.ID)
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

uint8_t LPS22HB_SPI::whoAmI()
{
    return readRegister(0U, reg.WHO_AM_I | reg.READ_FLAG, 0U);
}

void LPS22HB_SPI::setOdr(LPS22HB_ENUM::ODR odr)
{
    if (!ABSTRACT_BARO::checkBeginState())
        return;

    uint8_t odrValue = 0U;
    uint8_t curByte  = 0U;

    odrValue = enum_odr_to_baro_odr_map.at(odr);

    curByte = readRegister(0U, reg.CTRL_REG1 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U, reg.CTRL_REG1, curByte, odrValue, regVal.CTRL_REG1.ODR_MASK);
    mBaroOdr = odr;

    mStreamObj->printlnUlog(
            true,
            true,
            mClassName + ": baro conf updated, ODR = " + String(static_cast<uint8_t>(odr)));
}

void LPS22HB_SPI::setLowPassFilterState(LPS22HB_ENUM::FILTER_BW bandwidth)
{
    if (!ABSTRACT_BARO::checkBeginState())
        return;

    uint8_t bwValue = 0U;
    uint8_t curByte = 0U;

    mBaroBw = bandwidth;

    curByte = readRegister(0U, reg.CTRL_REG1 | reg.READ_FLAG, 0U);

    if (bandwidth == LPS22HB_ENUM::FILTER_BW::ODR_DIV_2)
    {
        writeRegisterWithMask(0U,
                              reg.CTRL_REG1,
                              curByte,
                              regVal.CTRL_REG1.LPF_DISABLE,
                              regVal.CTRL_REG1.LPF_MASK);

        mStreamObj->printlnUlog(true, true, mClassName + ": baro filter disabled");
        return;
    }

    bwValue = enum_filter_bw_to_baro_bw_map.at(bandwidth);
    writeRegisterWithMask(0U, reg.CTRL_REG1, curByte, bwValue, regVal.CTRL_REG1.LPF_MASK);

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": baro conf updated, BW = "
                                    + String(static_cast<uint8_t>(mBaroOdr)) + "/"
                                    + String(static_cast<uint8_t>(bandwidth)));
}

void LPS22HB_SPI::setMeasCheckStates(bool pressureCheck, bool tempCheck)
{
    mChecks.pressure = pressureCheck;
    mChecks.temp     = tempCheck;
}

void LPS22HB_SPI::setFifoMode(LPS22HB_ENUM::FIFO_MODE mode)
{
    if (!ABSTRACT_BARO::checkBeginState())
        return;

    uint8_t modeValue = 0U;
    uint8_t curByte   = 0U;

    mFifoMode = mode;

    curByte = readRegister(0U, reg.FIFO_CTRL | reg.READ_FLAG, 0U);

    if (mode == LPS22HB_ENUM::FIFO_MODE::FIFO_OFF)
    {
        setFifoActivationState(false);
        mStreamObj->printlnUlog(true, true, mClassName + ": baro fifo mode set to FIFO_OFF");
        return;
    }

    setFifoActivationState(true);

    modeValue = enum_fifo_mode_to_fifo_mode_map.at(mode);
    writeRegisterWithMask(0U, reg.FIFO_CTRL, curByte, modeValue, regVal.FIFO_CTRL.FIFO_MODE_MASK);

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName
                                    + ": baro conf updated, fifo mode activated with FIFO MODE = "
                                    + String(static_cast<uint8_t>(mFifoMode)));
}

void LPS22HB_SPI::setFifoActivationState(bool state)
{
    if (!ABSTRACT_BARO::checkBeginState())
        return;

    uint8_t curByte = 0U;
    uint8_t newByte = 0U;

    curByte = readRegister(0U, reg.CTRL_REG2 | reg.READ_FLAG, 0U);
    if (state)
        newByte = regVal.CTRL_REG2.FIFO_ENABLE;

    writeRegisterWithMask(0U, reg.CTRL_REG2, curByte, newByte, regVal.CTRL_REG2.FIFO_ENABLE_MASK);
    mUseFifo = state;

    mStreamObj->printlnUlog(
            true,
            true,
            mClassName + ": baro conf updated, FIFO activation state = " + String(state));
}

void LPS22HB_SPI::setOnePointCalibration(float hPaDeltaPressureMeanRef)
{
    if (!ABSTRACT_BARO::checkBeginState())
        return;

    uint8_t lsbByte       = 0U;
    uint8_t msbByte       = 0U;
    int16_t deltaPressure = 0;

    mCfg.opcDeltaPressure = hPaDeltaPressureMeanRef;

    deltaPressure = static_cast<int16_t>(hPaDeltaPressureMeanRef * mDeltaPressureToOpcVal);
    lsbByte       = (uint8_t)((uint16_t)deltaPressure & 0xFF);        // NOLINT
    msbByte       = (uint8_t)(((uint16_t)deltaPressure >> 8) & 0xFF); // NOLINT
    writeRegister(0U, reg.RPDS_L, lsbByte);
    writeRegister(0U, reg.RPDS_H, msbByte);

    mStreamObj->printlnUlog(
            true,
            true,
            mClassName + ": baro conf updated, one point calibration updated with delta = "
                    + String(hPaDeltaPressureMeanRef) + ", value = " + String(deltaPressure));
}

int16_t LPS22HB_SPI::getOnePointCalibration()
{
    if (!ABSTRACT_BARO::checkBeginState())
        return 0;

    uint8_t buf[2] = {0U, 0U};

    readRegisterBurst(0U, reg.RPDS_L | reg.READ_FLAG, buf, 2, 0U);

    return ((int16_t)buf[1] << 8) | (int16_t)buf[0];
}

void LPS22HB_SPI::resetOnePointCalibration()
{
    if (!ABSTRACT_BARO::checkBeginState())
        return;

    uint8_t lsbByte = 0U;
    uint8_t msbByte = 0U;

    writeRegister(0U, reg.RPDS_L, lsbByte);
    writeRegister(0U, reg.RPDS_H, msbByte);

    mStreamObj->printlnUlog(
            true, true, mClassName + ": baro conf updated, one point calibration resetted to 0");
}

// ##################################################################### //
// ################## SENSOR MANUAL READING FUNCTIONS ################## //
// ##################################################################### //

LPS22HB_ENUM::STATUS LPS22HB_SPI::getMeasuresStatus()
{
    if (!ABSTRACT_BARO::checkBeginState())
        return LPS22HB_ENUM::STATUS::NO_MEAS;

    uint8_t curByte = 0U;

    curByte = readRegister(0U, reg.STATUS | reg.READ_FLAG, 0U) & 0b11;

    if (curByte == regVal.STATUS.ALL_AVAILABLE)
        return LPS22HB_ENUM::STATUS::ALL_READY;

    if (curByte == regVal.STATUS.PRESSURE_AVAILABLE)
        return LPS22HB_ENUM::STATUS::PRESSURE_READY;

    if (curByte == regVal.STATUS.TEMPERATURE_AVAILABLE)
        return LPS22HB_ENUM::STATUS::TEMP_READY;

    return LPS22HB_ENUM::STATUS::NO_MEAS;
}

void LPS22HB_SPI::readTemperature() // 36us with checks, 19 without
{
    if (!ABSTRACT_BARO::checkBeginState())
        return;

    if (mUseFifo)
    {
        readFifo();
        return;
    }

    int16_t temperature = 0;
    uint8_t buf[2]      = {0U, 0U};
    // cppcheck-suppress unreadVariable
    LPS22HB_ENUM::STATUS status = LPS22HB_ENUM::STATUS::NO_MEAS;

    if (mChecks.temp)
    {
        status = getMeasuresStatus();

        mTimerObj->startTiming(0);
#pragma unroll
        while (status != LPS22HB_ENUM::STATUS::ALL_READY
               && status != LPS22HB_ENUM::STATUS::TEMP_READY)
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

    readRegisterBurst(0U, reg.TEMP_OUT_L | reg.READ_FLAG, buf, 2, 0U);

    temperature = ((int16_t)buf[1] << 8) | (int16_t)buf[0];

    mBaroOut.degTemperature = (float)temperature * 0.01F; // divide by 100

    mBaroOut.usTemperatureTimestamp = mTimerObj->returnSystemTimestampUs();
}

void LPS22HB_SPI::readPressure() // 39us with checks, 22us without
{
    if (!ABSTRACT_BARO::checkBeginState())
        return;

    if (mUseFifo)
    {
        readFifo();
        return;
    }

    int32_t pressure     = 0;
    int16_t pressureMsbs = 0;
    uint8_t buf[3]       = {0U, 0U, 0U};
    // cppcheck-suppress unreadVariable
    LPS22HB_ENUM::STATUS status = LPS22HB_ENUM::STATUS::NO_MEAS;

    if (mChecks.pressure)
    {
        status = getMeasuresStatus();

        mTimerObj->startTiming(0);
#pragma unroll
        while (status != LPS22HB_ENUM::STATUS::ALL_READY
               && status != LPS22HB_ENUM::STATUS::PRESSURE_READY)
        {
            if (mTimerObj->stopTiming(0) > mNsTimeOutMeasReading)
            {
                mStreamObj->printlnUlog(true, true, mClassName + ": pressure measurement timeout");
                return;
            }

            delayMicroseconds(1);
            status = getMeasuresStatus();
        }
    }

    readRegisterBurst(0U, reg.PRESS_OUT_XL | reg.READ_FLAG, buf, 3, 0U);

    // As the number is supposed SIGNED 24 bits (2s complement), we will force arithmetic shift with
    // using 16 bits number and then shift
    pressureMsbs = ((int16_t)buf[2] << 8) | (int16_t)buf[1];
    pressure     = ((int32_t)pressureMsbs << 8) | (int32_t)buf[0];

    mBaroOut.hPaPressure = (float)pressure * 0.000244140625f; // divide by 4096
    correctPressureMeasures();

    mBaroOut.usPressureTimestamp = mTimerObj->returnSystemTimestampUs();
}

void LPS22HB_SPI::readTemperaturePressureAltitude() // 45us with check, 26 without
{
    if (!ABSTRACT_BARO::checkBeginState())
        return;

    if (mUseFifo)
    {
        readFifo();
        return;
    }

    // Temperature is not as important as pressure, so we do not return if no temp is available
    int32_t pressure            = 0;
    int16_t pressureMsbs        = 0;
    uint8_t buf[5]              = {0U, 0U, 0U, 0U, 0U};
    uint32_t measureTimestamp   = 0U;
    LPS22HB_ENUM::STATUS status = LPS22HB_ENUM::STATUS::NO_MEAS;

    if (mChecks.pressure)
    {
        status = getMeasuresStatus();

        mTimerObj->startTiming(0);
#pragma unroll
        while (status != LPS22HB_ENUM::STATUS::ALL_READY
               && status != LPS22HB_ENUM::STATUS::PRESSURE_READY)
        {
            if (mTimerObj->stopTiming(0) > mNsTimeOutMeasReading)
            {
                mStreamObj->printlnUlog(true, true, mClassName + ": pressure measurement timeout");
                return;
            }

            delayMicroseconds(1);
            status = getMeasuresStatus();
        }
    }

    readRegisterBurst(0U, reg.PRESS_OUT_XL | reg.READ_FLAG, buf, 5, 0U);
    measureTimestamp = mTimerObj->returnSystemTimestampUs();
    if (status == LPS22HB_ENUM::STATUS::ALL_READY)
    {
        int32_t temperature = 0;

        // All measures are available

        // As the number is supposed SIGNED 24 bits (2s complement), we will force arithmetic shift
        // with using 16 bits number and then shift
        pressureMsbs         = ((int16_t)buf[2] << 8) | (int16_t)buf[1];
        pressure             = ((int32_t)pressureMsbs << 8) | (int32_t)buf[0];
        mBaroOut.hPaPressure = (float)pressure * 0.000244140625f; // divide by 4096
        correctPressureMeasures();
        mBaroOut.usPressureTimestamp = measureTimestamp;
        readAltitude();

        temperature                     = ((uint32_t)buf[4] << 8) | (uint32_t)buf[3];
        mBaroOut.degTemperature         = (float)temperature * 0.01F; // divide by 100
        mBaroOut.usTemperatureTimestamp = measureTimestamp;
        return;
    }

    // As the number is supposed SIGNED 24 bits (2s complement), we will force arithmetic shift
    // with using 16 bits number and then shift
    pressureMsbs         = ((int16_t)buf[2] << 8) | (int16_t)buf[1];
    pressure             = ((int32_t)pressureMsbs << 8) | (int32_t)buf[0];
    mBaroOut.hPaPressure = (float)pressure * 0.000244140625f; // divide by 4096
    correctPressureMeasures();
    mBaroOut.usPressureTimestamp = measureTimestamp;
    readAltitude();
}

uint8_t LPS22HB_SPI::getFifoSize()
{
    if (!ABSTRACT_BARO::checkBeginState())
        return 0U;

    return readRegister(0U, reg.FIFO_STATUS | reg.READ_FLAG, 0U)
           & regVal.FIFO_STATUS.FIFO_SIZE_MASK;
}

void LPS22HB_SPI::readFifo()
{
    if (!ABSTRACT_BARO::checkBeginState() || !mUseFifo)
        return;

    // Temperature is not as important as pressure, so we do not return if no temp is available
    int32_t pressure          = 0;
    int16_t pressureMsbs      = 0;
    int32_t temperature       = 0;
    uint8_t buf[5]            = {0U, 0U, 0U, 0U, 0U};
    uint32_t measureTimestamp = 0U;
    uint8_t fifoSize          = 0U;
    // cppcheck-suppress unreadVariable
    LPS22HB_ENUM::STATUS status = LPS22HB_ENUM::STATUS::NO_MEAS; // NOLINT

    fifoSize = getFifoSize();
    if (fifoSize < 1)
    {
        mStreamObj->printlnUlog(
                false,
                true,
                mClassName + ": no byte in fifo, structs left unchanged"); // TODO: In sd card
                                                                           // only once debug
                                                                           // is finished
        return;
    }

    // Keep only most recent sample, we could use buffer to store results. So the best way to use
    // fifo is in bypass mode at the moment
    for (uint8_t i = 0; i < fifoSize; i++)
        readRegisterBurst(0U, reg.PRESS_OUT_XL | reg.READ_FLAG, buf, 5, 0U);

    measureTimestamp = mTimerObj->returnSystemTimestampUs();

    // As the number is supposed SIGNED 24 bits (2s complement), we will force arithmetic shift
    // with using 16 bits number and then shift
    pressureMsbs         = ((int16_t)buf[2] << 8) | (int16_t)buf[1];
    pressure             = ((int32_t)pressureMsbs << 8) | (int32_t)buf[0];
    mBaroOut.hPaPressure = (float)pressure * 0.000244140625f; // divide by 4096
    correctPressureMeasures();
    mBaroOut.usPressureTimestamp = measureTimestamp;
    readAltitude();

    temperature                     = ((uint32_t)buf[4] << 8) | (uint32_t)buf[3];
    mBaroOut.degTemperature         = (float)temperature * 0.01F; // divide by 100
    mBaroOut.usTemperatureTimestamp = measureTimestamp;
}