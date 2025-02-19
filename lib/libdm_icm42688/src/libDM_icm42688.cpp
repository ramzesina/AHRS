#include "libDM_icm42688.hpp"

#include "libDM_abstract_imu.hpp"
#pragma push_macro("abs")
#undef abs

// ##################################################################### //
// ########################### INIT FUNCTIONS ########################## //
// ##################################################################### //

bool ICM42688_SPI::begin()
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
            ABSTRACT_IMU::setBeginState(false);
            return false;
        }
    }

    mStreamObj->printlnUlog(
            true, true, mClassName + ": begin() success, sensor ID = " + String(reg.SENSOR_ID));

    powerOnSensors();

    if (mUseExternalClock)
        activateExtClock();

    if (mUseFifo)
        activateFifo();

    ABSTRACT_IMU::setBeginState(true);
    return true;
}

bool ICM42688_SPI::tryStart()
{
    mTimerObj->startTiming(0);
#pragma unroll
    while (whoAmI() != reg.SENSOR_ID)
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

uint8_t ICM42688_SPI::whoAmI()
{
    return readRegister(0U, reg.BANK0.WHO_AM_I | reg.READ_FLAG, 0U);
}

void ICM42688_SPI::softResetSensor()
{
    uint8_t res = writeRegister(0U, reg.BANK0.DEVICE_CONFIG, 0b1);
    mStreamObj->printlnUlog(true, true, mClassName + ": soft reset sensor = " + String(res == 0b1));
    delay(1);
}

void ICM42688_SPI::powerOnSensors()
{
    writeRegister(0U, reg.BANK0.REG_BANK_SEL, regVal.BANK.BANK0);
    // Accel in low noise mode, Gyro in low noise mode
    writeRegister(0U,
                  reg.BANK0.PWR_MGMT0,
                  regVal.PWR_MGMT0.ACCEL_LOW_NOISE | regVal.PWR_MGMT0.GYRO_LOW_NOISE);

    mStreamObj->printlnUlog(true, true, mClassName + ": acc and gyro powered on");
}

void ICM42688_SPI::activateExtClock()
{
    uint8_t curByte = 0U;

    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK0, regVal.BANK.BANK_MASK);

    curByte = readRegister(0U, reg.BANK0.INTF_CONFIG1 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.INTF_CONFIG1,
                          curByte,
                          regVal.INTF_CONFIG1.RTC_ENABLE,
                          regVal.INTF_CONFIG1.RTC_MASK);

    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK1, regVal.BANK.BANK_MASK);

    curByte = readRegister(0U, reg.BANK1.INTF_CONFIG5 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK1.INTF_CONFIG5,
                          curByte,
                          regVal.INTF_CONFIG5.PIN9_CLKIN,
                          regVal.INTF_CONFIG5.PIN9_MASK);

    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK0, regVal.BANK.BANK_MASK);

    mStreamObj->printlnUlog(true, true, mClassName + ": ext clock has been activated");
}

void ICM42688_SPI::activateFifo()
{
    uint8_t curByte = 0;

    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK0, regVal.BANK.BANK_MASK);

    // Activate Fifo
    curByte = readRegister(0U, reg.BANK0.FIFO_CONFIG1 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.FIFO_CONFIG1,
                          curByte,
                          regVal.FIFO_CONFIG1.HIGH_RES_FULL_FIFO_PACKET_4,
                          regVal.FIFO_CONFIG1.FIFO_PACKET_MASK);

    // Stream to Fifo
    curByte = readRegister(0U, reg.BANK0.FIFO_CONFIG | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.FIFO_CONFIG,
                          curByte,
                          regVal.FIFO_CONFIG.STREAM_2_FIFO,
                          regVal.FIFO_CONFIG.FIFO_MODE_MASK);

    // Set fifo to hold last valid byte
    curByte = readRegister(0U, reg.BANK0.INTF_CONFIG0 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.INTF_CONFIG0,
                          curByte,
                          regVal.INTF_CONFIG0.FIFO_HOLD_LAST_DATA_EN,
                          regVal.INTF_CONFIG0.FIFO_HOLD_LAST_DATA_EN_MASK);

    // Return fifo size in buffer count instead of bytes
    curByte = readRegister(0U, reg.BANK0.INTF_CONFIG0 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.INTF_CONFIG0,
                          curByte,
                          regVal.INTF_CONFIG0.FIFO_COUNT_REC_IN_COUNT,
                          regVal.INTF_CONFIG0.FIFO_COUNT_REC_IN_COUNT_MASK);

    mStreamObj->printlnUlog(
            true,
            true,
            mClassName
                    + ": fifo has been activated, FIFO will not output invalid datas but will hold "
                    + "last valid byte. Fifo size will be reported in counts");
}

void ICM42688_SPI::flushFifo(bool printStatus)
{
    uint8_t curByte = 0U;

    curByte = readRegister(0U, reg.BANK0.SIGNAL_PATH_RESET | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.SIGNAL_PATH_RESET,
                          curByte,
                          regVal.SIGNAL_PATH_RESET.FIFO_FLUSH_TRUE,
                          regVal.SIGNAL_PATH_RESET.FIFO_FLUSH_MASK);

    if (printStatus)
        mStreamObj->printlnUlog(true, true, mClassName + ": fifo has been flushed");
}

// ##################################################################### //
// ########################## CONFIG FUNCTIONS ######################### //
// ##################################################################### //

void ICM42688_SPI::setGyroOdrRange(ICM42688_ENUM::ODR freq, ICM42688_ENUM::GYRO_RANGE range)
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    uint8_t curByte    = 0U;
    uint8_t freqValue  = 0;
    uint8_t rangeValue = 0;
    freqValue          = enum_odr_to_gyro_odr_map.at(freq);
    if (mUseFifo)
    {
        mStreamObj->printlnUlog(
                true,
                true,
                mClassName
                        + ": fifo is activated (packet 4) so specified gyro range will not be used");
        rangeValue = enum_range_to_gyro_range_caracs_map.at(ICM42688_ENUM::GYRO_RANGE::_2000dps)
                             .first; // TODO: if fifo disabled, perhaps letting rangeValue/factor
                                     // memorized to reuse after (fifo use 20bits factor)
    }
    else
    {
        rangeValue = enum_range_to_gyro_range_caracs_map.at(range).first;
    }

    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK0, regVal.BANK.BANK_MASK);

    writeRegister(0U, reg.BANK0.GYRO_CONFIG0, rangeValue << 5 | freqValue);

    mGyroFreq  = freq;
    mGyroRange = range;

    updateGyroFactor(range);

    mStreamObj->printlnUlog(
            true,
            true,
            mClassName + ": gyro conf updated, ODR = " + String(static_cast<uint16_t>(freq))
                    + "Hz, range = " + String(static_cast<uint16_t>(range)) + "dps");
}

void ICM42688_SPI::setAccOdrRange(ICM42688_ENUM::ODR freq, ICM42688_ENUM::ACC_RANGE range)
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    uint8_t curByte    = 0U;
    uint8_t freqValue  = 0;
    uint8_t rangeValue = 0;
    freqValue          = enum_odr_to_acc_odr_map.at(freq);
    if (mUseFifo)
    {
        mStreamObj->printlnUlog(
                true,
                true,
                mClassName
                        + ": fifo (packet 4) is activated so specified acc range will not be used");
        rangeValue = enum_range_to_acc_range_caracs_map.at(ICM42688_ENUM::ACC_RANGE::_16g)
                             .first; // TODO: if fifo disabled, perhaps letting rangeValue/factor
                                     // memorized to reuse after (fifo use 20bits factor)
    }
    else
    {
        rangeValue = enum_range_to_acc_range_caracs_map.at(range).first;
    }

    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK0, regVal.BANK.BANK_MASK);

    writeRegister(0U, reg.BANK0.ACCEL_CONFIG0, rangeValue << 5 | freqValue);

    mAccFreq  = freq;
    mAccRange = range;

    updateAccFactor(range);

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": acc conf updated, ODR = "
                                    + String(static_cast<uint16_t>(freq))
                                    + "Hz, range = " + String(static_cast<uint16_t>(range)) + "g");

    // Update fifo check parameters
    if (mUseFifo)
    {
        mFifoTimestampDiffThreshold = 1000000U / static_cast<uint32_t>(mAccFreq); // in us
        mFifoTimestampDiffMargin    = mFifoTimestampDiffThreshold / 2;
    }
}

void ICM42688_SPI::updateGyroFactor(ICM42688_ENUM::GYRO_RANGE range)
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    // TODO: if fifo disabled, perhaps letting rangeValue/factor memorized to reuse after (fifo use
    // 20bits factor)
    if (mUseFifo)
        mGyroFactor = mGyroFactor20Bits;
    else
        mGyroFactor = enum_range_to_gyro_range_caracs_map.at(range).second;
}

void ICM42688_SPI::updateAccFactor(ICM42688_ENUM::ACC_RANGE range)
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    // TODO: if fifo disabled, perhaps letting rangeValue/factor memorized to reuse after (fifo use
    // 20bits factor)
    if (mUseFifo)
        mAccFactor = mAccFactor20bits;
    else
        mAccFactor = enum_range_to_acc_range_caracs_map.at(range).second;
}

void ICM42688_SPI::setGyroAccelFilter(ICM42688_ENUM::FILTER_BW gyroBW,
                                      ICM42688_ENUM::FILTER_BW accBW,
                                      ICM42688_ENUM::FILTER_ORDER gyroFiltOrder,
                                      ICM42688_ENUM::FILTER_ORDER accFiltOrder)
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    uint8_t curByte = 0;

    uint8_t gyroBWValue        = 0;
    uint8_t accBWValue         = 0;
    uint8_t gyroFiltOrderValue = 0;
    uint8_t accFiltOrderValue  = 0;

    gyroBWValue        = enum_filter_bw_acc_gyro_filter_conf_map.at(gyroBW);
    accBWValue         = enum_filter_bw_acc_gyro_filter_conf_map.at(accBW);
    gyroFiltOrderValue = enum_filter_order_to_acc_gyro_filter_order_map.at(gyroFiltOrder);
    accFiltOrderValue  = enum_filter_order_to_acc_gyro_filter_order_map.at(accFiltOrder);

    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK0, regVal.BANK.BANK_MASK);

    // Write BW values
    writeRegister(0U, reg.BANK0.GYRO_ACCEL_CONFIG0, accBWValue << 4 | gyroBWValue);

    // Write filter order values
    // Gyro
    curByte = readRegister(0U, reg.BANK0.GYRO_CONFIG1 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.GYRO_CONFIG1,
                          curByte,
                          gyroFiltOrderValue << 2,
                          regVal.ACC_GYRO_FILTER_ORDER.GYRO_MASK);

    // Acc
    curByte = readRegister(0U, reg.BANK0.ACCEL_CONFIG1 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.ACCEL_CONFIG1,
                          curByte,
                          accFiltOrderValue << 3,
                          regVal.ACC_GYRO_FILTER_ORDER.ACC_MASK);

    mGyroBW        = gyroBW;
    mAccBW         = accBW;
    mGyroFiltOrder = gyroFiltOrder;
    mAccFiltOrder  = accFiltOrder;

    if (mGyroFreq < ICM42688_ENUM::ODR::_2kHz)
        mStreamObj->printlnUlog(true,
                                true,
                                mClassName + ": gyro filter BW has been changed, BW = "
                                        + String(static_cast<uint16_t>(mGyroBW)));
    else
        mStreamObj->printlnUlog(
                true,
                true,
                mClassName + ": gyro filter BW can not be changed, it is fixed to ODR/4 = "
                        + String(static_cast<uint16_t>(mGyroFreq) / 4U));

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": gyro filter order has been changed, order = "
                                    + String(static_cast<uint8_t>(mGyroFiltOrder)));

    if (mAccFreq < ICM42688_ENUM::ODR::_2kHz)
        mStreamObj->printlnUlog(true,
                                true,
                                mClassName + ": acc filter BW has been changed, BW = "
                                        + String(static_cast<uint16_t>(mAccBW)));
    else
        mStreamObj->printlnUlog(
                true,
                true,
                mClassName + ": acc filter BW can not be changed, it is fixed to ODR/4 = "
                        + String(static_cast<uint16_t>(mAccFreq) / 4U));

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": acc filter order has been changed, order = "
                                    + String(static_cast<uint8_t>(mAccFiltOrder)));
}

void ICM42688_SPI::setTempFilter(ICM42688_ENUM::TEMP_DLPF_BW tempBw)
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    uint8_t curByte = 0;

    uint8_t bwValue = 0;
    bwValue         = enum_dlpf_bw_to_temp_dlpf_map.at(tempBw);

    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK0, regVal.BANK.BANK_MASK);

    // Set filter bw
    curByte = readRegister(0U, reg.BANK0.GYRO_CONFIG1 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.GYRO_CONFIG1,
                          curByte,
                          bwValue << 5,
                          regVal.TEMP_FILT_BW.TEMP_FILT_BW_MASK);

    mTempBW = tempBw;
    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": Temperature filter has been changed, bw = "
                                    + String(static_cast<uint16_t>(tempBw)));
}

void ICM42688_SPI::setApexWOM(uint8_t xThreshold,
                              uint8_t yThreshold,
                              uint8_t zThreshold,
                              ICM42688_ENUM::APEX_WOM_INT_MODE womIntMode,
                              ICM42688_ENUM::APEX_WOM_MODE womMode,
                              ICM42688_ENUM::APEX_SMD_MODE smdMode)
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    uint8_t curByte       = 0U;
    uint8_t curWomIntMode = 0U;
    uint8_t curWomMode    = 0U;
    uint8_t curSmdMode    = 0U;

    curWomIntMode = enum_apex_wom_int_map.at(womIntMode);
    curWomMode    = enum_apex_wom_map.at(womMode);
    curSmdMode    = enum_apex_smd_map.at(smdMode);

    configureInterrupt1(ICM42688_ENUM::INTERRUPT_DRIVE_MODE::INT_PUSH_PULL,
                        ICM42688_ENUM::INTERRUPT_POLARITY::ACTIVE_HIGH,
                        ICM42688_ENUM::INTERRUPT_MODE::PULSED);

    // Go bank 4
    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK4, regVal.BANK.BANK_MASK);

    // Set thresholds
    writeRegister(0U, reg.BANK4.ACCEL_WOM_X_THR, xThreshold);
    writeRegister(0U, reg.BANK4.ACCEL_WOM_Y_THR, yThreshold);
    writeRegister(0U, reg.BANK4.ACCEL_WOM_Z_THR, zThreshold);

    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": WOM thresholds have been set, x = "
                                    + String(xThreshold * 3.9)
                                    + "mg , y = " + String(yThreshold * 3.9)
                                    + "mg , z = " + String(zThreshold * 3.9) + "mg");

    delay(1); // NOLINT

    // Go bank 0
    curByte = readRegister(0U, reg.BANK0.REG_BANK_SEL | reg.READ_FLAG, 0U);
    writeRegisterWithMask(
            0U, reg.BANK0.REG_BANK_SEL, curByte, regVal.BANK.BANK0, regVal.BANK.BANK_MASK);

    // Set INT1 to WOM
    curByte = readRegister(0U, reg.BANK0.INT_SOURCE1 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          reg.BANK0.INT_SOURCE1,
                          curByte,
                          regVal.INT_SOURCE1.WOM_TO_INT1_EN,
                          regVal.INT_SOURCE1.WOM_TO_INT1_MASK);

    mStreamObj->printlnUlog(true, true, mClassName + ": WOM has been set to INT1");

    delay(50); // NOLINT

    // Configure WOM INT MODE and WOM mode
    curByte = readRegister(0U, reg.BANK0.SMD_CONFIG | reg.READ_FLAG, 0U);
    curByte &= regVal.APEX_WOM_CONF.WOM_INT_MODE_MASK;
    curByte |= curWomIntMode;
    curByte &= regVal.APEX_WOM_CONF.WOM_MODE_MASK;
    curByte |= curWomMode;
    curByte &= regVal.APEX_WOM_CONF.SMD_MODE_MASK;
    curByte |= curSmdMode;
    writeRegister(0U, reg.BANK0.SMD_CONFIG, curByte);
}

void ICM42688_SPI::configureInterrupt1(ICM42688_ENUM::INTERRUPT_DRIVE_MODE driveMode,
                                       ICM42688_ENUM::INTERRUPT_POLARITY polarity,
                                       ICM42688_ENUM::INTERRUPT_MODE mode)
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    uint8_t curByte      = 0U;
    uint8_t curDriveMode = 0U;
    uint8_t curPolarity  = 0U;
    uint8_t curMode      = 0U;
    uint8_t int1Config   = 0U;

    curDriveMode = enum_int_drive_mode_map.at(driveMode);
    curPolarity  = enum_int_polarity_map.at(polarity);
    curMode      = enum_int_mode_map.at(mode);

    // Go bank0
    writeRegister(0U, reg.BANK0.REG_BANK_SEL, regVal.BANK.BANK0);

    // INT_CONFIG
    curByte    = readRegister(0U, reg.BANK0.INT_CONFIG | reg.READ_FLAG, 0U);
    int1Config = curDriveMode | curPolarity | curMode;
    writeRegisterWithMask(
            0U, reg.BANK0.INT_CONFIG, curByte, int1Config, regVal.INT_CONFIG.INT1_CONF_MASK);

    // Clear INT_CONFIG1 4th byte
    curByte = readRegister(0U, reg.BANK0.INT_CONFIG1 | reg.READ_FLAG, 0U);
    writeRegisterWithMask(0U,
                          0x64,
                          curByte,
                          regVal.INT_CONFIG1.INT_ASYNC_RESET,
                          regVal.INT_CONFIG1.INT_ASYNC_MASK);
}

// ##################################################################### //
// ###################### SENSOR READING FUNCTIONS ##################### //
// ##################################################################### //

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

void ICM42688_SPI::readTemp()
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    if (mUseFifo)
    {
        readFifo();
        return;
    }

    uint8_t buf[2]  = {0U, 0U};
    int16_t rawTemp = 0;

    readRegisterBurst(0U, reg.BANK0.TEMP_DATA1 | reg.READ_FLAG, buf, 2, 0U);
    rawTemp = ((int16_t)buf[0] << 8) | (int16_t)buf[1];

    mImuOut.usTempTimestamp   = mTimerObj->returnSystemTimestampUs();
    mImuOut.degImuTemperature = (float)rawTemp / 132.48F + 25.F; // NOLINT
}

void ICM42688_SPI::readAcc() // 20us at 24mHz spi
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    if (mUseFifo)
    {
        readFifo();
        return;
    }

    uint8_t buf[6]    = {0U, 0U, 0U, 0U, 0U, 0U};
    int16_t rawAcc[3] = {0, 0, 0};

    readRegisterBurst(0U, reg.BANK0.ACCEL_DATA_X1 | reg.READ_FLAG, buf, 6, 0U);
    rawAcc[imu_axis.X] = ((int16_t)buf[0] << 8) | (int16_t)buf[1];
    rawAcc[imu_axis.Y] = ((int16_t)buf[2] << 8) | (int16_t)buf[3];
    rawAcc[imu_axis.Z] = ((int16_t)buf[4] << 8) | (int16_t)buf[5];

    // TODO: apply calibration
    mImuOut.gAccelData[imu_axis.X] = static_cast<float>(rawAcc[imu_axis.X]) * mAccFactor;
    mImuOut.gAccelData[imu_axis.Y] = static_cast<float>(rawAcc[imu_axis.Y]) * mAccFactor;
    mImuOut.gAccelData[imu_axis.Z] = static_cast<float>(rawAcc[imu_axis.Z]) * mAccFactor;

    mImuOut.usAccelTimestamp = mTimerObj->returnSystemTimestampUs();
}

void ICM42688_SPI::readGyro() // 20us at 24mHz spi
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    if (mUseFifo)
    {
        readFifo();
        return;
    }

    uint8_t buf[6]     = {0U, 0U, 0U, 0U, 0U, 0U};
    int16_t rawGyro[3] = {0, 0, 0};

    readRegisterBurst(0U, reg.BANK0.GYRO_DATA_X1 | reg.READ_FLAG, buf, 6, 0U);
    rawGyro[imu_axis.X] = ((int16_t)buf[0] << 8) | (int16_t)buf[1];
    rawGyro[imu_axis.Y] = ((int16_t)buf[2] << 8) | (int16_t)buf[3];
    rawGyro[imu_axis.Z] = ((int16_t)buf[4] << 8) | (int16_t)buf[5];

    // TODO: apply complete calibration
    mImuOut.dpsGyroData[imu_axis.X] = static_cast<float>(rawGyro[imu_axis.X]) * mGyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.X];
    mImuOut.dpsGyroData[imu_axis.Y] = static_cast<float>(rawGyro[imu_axis.Y]) * mGyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.Y];
    mImuOut.dpsGyroData[imu_axis.Z] = static_cast<float>(rawGyro[imu_axis.Z]) * mGyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.Z];

    mImuOut.usGyroTimestamp = mTimerObj->returnSystemTimestampUs();
}

void ICM42688_SPI::readAccGyro() // 22us at 24mHz spi
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    if (mUseFifo)
    {
        readFifo();
        return;
    }

    uint8_t buf[12]    = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    int16_t rawGyro[3] = {0, 0, 0};
    int16_t rawAcc[3]  = {0, 0, 0};
    uint32_t timestamp = 0U;

    readRegisterBurst(0U, reg.BANK0.ACCEL_DATA_X1 | reg.READ_FLAG, buf, 12, 0U);
    rawAcc[imu_axis.X]  = ((int16_t)buf[0] << 8) | (int16_t)buf[1];
    rawAcc[imu_axis.Y]  = ((int16_t)buf[2] << 8) | (int16_t)buf[3];
    rawAcc[imu_axis.Z]  = ((int16_t)buf[4] << 8) | (int16_t)buf[5];
    rawGyro[imu_axis.X] = ((int16_t)buf[6] << 8) | (int16_t)buf[7];
    rawGyro[imu_axis.Y] = ((int16_t)buf[8] << 8) | (int16_t)buf[9];
    rawGyro[imu_axis.Z] = ((int16_t)buf[10] << 8) | (int16_t)buf[11];
    timestamp           = mTimerObj->returnSystemTimestampUs();

    fillOutputStruct(rawGyro, rawAcc, mGyroFactor, mAccFactor, timestamp);
}

void ICM42688_SPI::readAllImu()
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    if (mUseFifo)
    {
        readFifo();
        return;
    }
    readAccGyro();
    readTemp();
}

uint16_t ICM42688_SPI::getFifoSize()
{
    if (!ABSTRACT_IMU::checkBeginState())
        return 0U;

    uint8_t buf[2]    = {0U, 0U};
    uint16_t fifoSize = 0;

    readRegisterBurst(0U, reg.BANK0.FIFO_COUNTH | reg.READ_FLAG, buf, 2, 0U);
    fifoSize = (uint16_t)buf[0] | (uint16_t)buf[1];

    return fifoSize;
}

// 65us at 24mHz with 1-2 values in buffer, 2ms to read a complete fifo (102 values)
void ICM42688_SPI::readFifo()
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    // 2 buffers are used to get the data. One temporary buffer to store the data read through spi
    // bus and one to store valid buffers
    uint8_t buf1[20]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                         0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t buf2[20]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                         0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t* buf      = buf1;
    uint8_t* bufValid = buf2;

    int32_t rawGyro[3] = {0, 0, 0};
    int32_t rawAcc[3]  = {0, 0, 0};
    // To store 16 bits values, get the sign bit and THEN apply arithmetic shift (not logical) to
    // the signed value and add the complementary bits if 20 bits. This is very important as the
    // sign bit is lost if doing direct int32 concatenation and int20_t is not a type (with the msb
    // bit being the sign bit)!
    int16_t rawGyro16[3] = {0, 0, 0};
    int16_t rawAcc16[3]  = {0, 0, 0};
    int16_t rawTemp      = 0;
    uint32_t timestamp   = 0U;
    bool isFifoEmpty     = false;
    bool isHeader20      = false;
    uint8_t compAccelX   = 0U;
    uint8_t compAccelY   = 0U;
    uint8_t compAccelZ   = 0U;
    uint8_t compGyroX    = 0U;
    uint8_t compGyroY    = 0U;
    uint8_t compGyroZ    = 0U;
    uint16_t fifoSize    = 0U;

    fifoSize = getFifoSize();
    if (fifoSize < 1)
    {
        mStreamObj->printlnUlog(
                false, true, mClassName + ": no byte in fifo, structs left unchanged");
        return;
    }

    // Read all fifo values and take the last valid one
    uint16_t imuTimestampPrev = 0U;
#pragma unroll
    for (uint16_t i = 0; i < fifoSize; i++)
    {
        bool isBufferValid = true;

        readRegisterBurst(0U, reg.BANK0.FIFO_DATA | reg.READ_FLAG, buf, 20, 0U);
        uint16_t imuTimestamp = buf[15] << 8 | buf[16];

        if (i > 0)
        {
            int32_t timestampDiff = imuTimestamp - imuTimestampPrev;
            if (timestampDiff < 0)
                timestampDiff += 65536;
            int32_t timestampError =
                    std::abs(timestampDiff - static_cast<int32_t>(mFifoTimestampDiffThreshold));
            isBufferValid = timestampError <= mFifoTimestampDiffMargin;
        }

        if (isBufferValid)
        {
            // Swap buffers values to keep the last valid buffer
            uint8_t* tempAddress = buf;
            buf                  = bufValid;
            bufValid             = tempAddress;
        }
        else
        {
            mStreamObj->printlnUlog(
                    false,
                    true,
                    mClassName
                            + ": fifo timestamp diff is too high, buffer discarded, verify connexion quality");
        }

        imuTimestampPrev = imuTimestamp;
    }

    isFifoEmpty = static_cast<bool>(buf[0] >> 7 & 0b1);
    isHeader20  = static_cast<bool>(buf[0] >> 4 & 0b1);

    if (isFifoEmpty)
    {
        mStreamObj->printlnUlog(
                false, true, mClassName + ": no valid data in fifo, structs left unchanged");
        return;
    }

    rawAcc16[imu_axis.X]  = ((int16_t)bufValid[1] << 8) | (int16_t)bufValid[2];
    rawAcc16[imu_axis.Y]  = ((int16_t)bufValid[3] << 8) | (int16_t)bufValid[4];
    rawAcc16[imu_axis.Z]  = ((int16_t)bufValid[5] << 8) | (int16_t)bufValid[6];
    rawGyro16[imu_axis.X] = ((int16_t)bufValid[7] << 8) | (int16_t)bufValid[8];
    rawGyro16[imu_axis.Y] = ((int16_t)bufValid[9] << 8) | (int16_t)bufValid[10];
    rawGyro16[imu_axis.Z] = ((int16_t)bufValid[11] << 8) | (int16_t)bufValid[12];
    rawTemp               = ((int16_t)bufValid[13] << 8) | (int16_t)bufValid[14];
    timestamp             = mTimerObj->returnSystemTimestampUs();

    if (isHeader20)
    {
        compAccelX = bufValid[17] >> 4 & 0xF;
        compAccelY = bufValid[18] >> 4 & 0xF;
        compAccelZ = bufValid[19] >> 4 & 0xF;
        compGyroX  = bufValid[17] & 0xF;
        compGyroY  = bufValid[18] & 0xF;
        compGyroZ  = bufValid[19] & 0xF;
    }

    rawAcc[imu_axis.X]  = (int32_t)rawAcc16[imu_axis.X] << 4 | (int32_t)compAccelX;
    rawAcc[imu_axis.Y]  = (int32_t)rawAcc16[imu_axis.Y] << 4 | (int32_t)compAccelY;
    rawAcc[imu_axis.Z]  = (int32_t)rawAcc16[imu_axis.Z] << 4 | (int32_t)compAccelZ;
    rawGyro[imu_axis.X] = (int32_t)rawGyro16[imu_axis.X] << 4 | (int32_t)compGyroX;
    rawGyro[imu_axis.Y] = (int32_t)rawGyro16[imu_axis.Y] << 4 | (int32_t)compGyroY;
    rawGyro[imu_axis.Z] = (int32_t)rawGyro16[imu_axis.Z] << 4 | (int32_t)compGyroZ;

    fillOutputStruct(rawGyro, rawAcc, rawTemp, mGyroFactor20Bits, mAccFactor20bits, timestamp);
}

// 26us without going for last sample at 24mHz, 2ms to read a complete fifo (102 values)
void ICM42688_SPI::readFifoFast()
{
    if (!ABSTRACT_IMU::checkBeginState())
        return;

    uint8_t buf[20]    = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    int32_t rawGyro[3] = {0, 0, 0};
    int32_t rawAcc[3]  = {0, 0, 0};
    // To store 16 bits values, get the sign bit and THEN apply arithmetic shift (not logical) to
    // the signed value and add the complementary bits if 20 bits. This is very important as the
    // sign bit is lost if doing direct int32 concatenation and int20_t is not a type (with the msb
    // bit being the sign bit)!
    int16_t rawGyro16[3] = {0, 0, 0};
    int16_t rawAcc16[3]  = {0, 0, 0};
    int16_t rawTemp      = 0;
    uint32_t timestamp   = 0U;
    bool isFifoEmpty     = false;
    bool isHeader20      = false;
    uint8_t compAccelX   = 0U;
    uint8_t compAccelY   = 0U;
    uint8_t compAccelZ   = 0U;
    uint8_t compGyroX    = 0U;
    uint8_t compGyroY    = 0U;
    uint8_t compGyroZ    = 0U;

    readRegisterBurst(0U, reg.BANK0.FIFO_DATA | reg.READ_FLAG, buf, 20, 0U);

    isFifoEmpty = static_cast<bool>(buf[0] >> 7 & 0b1);
    isHeader20  = static_cast<bool>(buf[0] >> 4 & 0b1);

    if (isFifoEmpty)
    {
        mStreamObj->printlnUlog(
                false, true, mClassName + ": no valid data in fifo, structs left unchanged");
        return;
    }

    rawAcc16[imu_axis.X]  = ((int16_t)buf[1] << 8) | (int16_t)buf[2];
    rawAcc16[imu_axis.Y]  = ((int16_t)buf[3] << 8) | (int16_t)buf[4];
    rawAcc16[imu_axis.Z]  = ((int16_t)buf[5] << 8) | (int16_t)buf[6];
    rawGyro16[imu_axis.X] = ((int16_t)buf[7] << 8) | (int16_t)buf[8];
    rawGyro16[imu_axis.Y] = ((int16_t)buf[9] << 8) | (int16_t)buf[10];
    rawGyro16[imu_axis.Z] = ((int16_t)buf[11] << 8) | (int16_t)buf[12];
    rawTemp               = ((int16_t)buf[13] << 8) | (int16_t)buf[14];
    timestamp             = mTimerObj->returnSystemTimestampUs();

    if (isHeader20)
    {
        compAccelX = buf[17] >> 4 & 0xF;
        compAccelY = buf[18] >> 4 & 0xF;
        compAccelZ = buf[19] >> 4 & 0xF;
        compGyroX  = buf[17] & 0xF;
        compGyroY  = buf[18] & 0xF;
        compGyroZ  = buf[19] & 0xF;
    }

    rawAcc[imu_axis.X]  = (int32_t)rawAcc16[imu_axis.X] << 4 | (int32_t)compAccelX;
    rawAcc[imu_axis.Y]  = (int32_t)rawAcc16[imu_axis.Y] << 4 | (int32_t)compAccelY;
    rawAcc[imu_axis.Z]  = (int32_t)rawAcc16[imu_axis.Z] << 4 | (int32_t)compAccelZ;
    rawGyro[imu_axis.X] = (int32_t)rawGyro16[imu_axis.X] << 4 | (int32_t)compGyroX;
    rawGyro[imu_axis.Y] = (int32_t)rawGyro16[imu_axis.Y] << 4 | (int32_t)compGyroY;
    rawGyro[imu_axis.Z] = (int32_t)rawGyro16[imu_axis.Z] << 4 | (int32_t)compGyroZ;

    fillOutputStruct(rawGyro, rawAcc, rawTemp, mGyroFactor20Bits, mAccFactor20bits, timestamp);
}

void ICM42688_SPI::fillOutputStruct(const int16_t rawGyro[3],
                                    const int16_t rawAcc[3],
                                    const int16_t rawTemp,
                                    const float gyroFactor,
                                    const float accFactor,
                                    const uint32_t timestamp)
{ // TODO: apply complete calibration
    mImuOut.degImuTemperature      = (float)rawTemp / 132.48F + 25.F; // NOLINT
    mImuOut.gAccelData[imu_axis.X] = static_cast<float>(rawAcc[imu_axis.X]) * accFactor;
    mImuOut.gAccelData[imu_axis.Y] = static_cast<float>(rawAcc[imu_axis.Y]) * accFactor;
    mImuOut.gAccelData[imu_axis.Z] = static_cast<float>(rawAcc[imu_axis.Z]) * accFactor;

    mImuOut.dpsGyroData[imu_axis.X] = static_cast<float>(rawGyro[imu_axis.X]) * gyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.X];
    mImuOut.dpsGyroData[imu_axis.Y] = static_cast<float>(rawGyro[imu_axis.Y]) * gyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.Y];
    mImuOut.dpsGyroData[imu_axis.Z] = static_cast<float>(rawGyro[imu_axis.Z]) * gyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.Z];

    mImuOut.usGyroTimestamp  = timestamp;
    mImuOut.usAccelTimestamp = timestamp;
    mImuOut.usTempTimestamp  = timestamp;
}

void ICM42688_SPI::fillOutputStruct(const int16_t rawGyro[3],
                                    const int16_t rawAcc[3],
                                    const float gyroFactor,
                                    const float accFactor,
                                    const uint32_t timestamp)
{
    mImuOut.gAccelData[imu_axis.X] = static_cast<float>(rawAcc[imu_axis.X]) * accFactor;
    mImuOut.gAccelData[imu_axis.Y] = static_cast<float>(rawAcc[imu_axis.Y]) * accFactor;
    mImuOut.gAccelData[imu_axis.Z] = static_cast<float>(rawAcc[imu_axis.Z]) * accFactor;

    mImuOut.dpsGyroData[imu_axis.X] = static_cast<float>(rawGyro[imu_axis.X]) * gyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.X];
    mImuOut.dpsGyroData[imu_axis.Y] = static_cast<float>(rawGyro[imu_axis.Y]) * gyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.Y];
    mImuOut.dpsGyroData[imu_axis.Z] = static_cast<float>(rawGyro[imu_axis.Z]) * gyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.Z];

    mImuOut.usGyroTimestamp  = timestamp;
    mImuOut.usAccelTimestamp = timestamp;
}

void ICM42688_SPI::fillOutputStruct(const int32_t rawGyro[3],
                                    const int32_t rawAcc[3],
                                    const int16_t rawTemp,
                                    const float gyroFactor,
                                    const float accFactor,
                                    const uint32_t timestamp)
{
    mImuOut.degImuTemperature      = (float)rawTemp / 132.48F + 25.F; // NOLINT
    mImuOut.gAccelData[imu_axis.X] = static_cast<float>(rawAcc[imu_axis.X]) * accFactor;
    mImuOut.gAccelData[imu_axis.Y] = static_cast<float>(rawAcc[imu_axis.Y]) * accFactor;
    mImuOut.gAccelData[imu_axis.Z] = static_cast<float>(rawAcc[imu_axis.Z]) * accFactor;

    mImuOut.dpsGyroData[imu_axis.X] = static_cast<float>(rawGyro[imu_axis.X]) * gyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.X];
    mImuOut.dpsGyroData[imu_axis.Y] = static_cast<float>(rawGyro[imu_axis.Y]) * gyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.Y];
    mImuOut.dpsGyroData[imu_axis.Z] = static_cast<float>(rawGyro[imu_axis.Z]) * gyroFactor
                                      - mImuCorrection.gyro.xyzBias[imu_axis.Z];

    mImuOut.usGyroTimestamp  = timestamp;
    mImuOut.usAccelTimestamp = timestamp;
    mImuOut.usTempTimestamp  = timestamp;
}

const ICM42688_STRUCTS::APEX_STATUS& ICM42688_SPI::getApexStatus()
{
    mApexStatus = ICM42688_STRUCTS::APEX_STATUS();

    if (!ABSTRACT_IMU::checkBeginState())
        return mApexStatus;

    uint8_t curByte = 0U;

    // Status 2
    curByte = readRegister(0U, reg.BANK0.INT_STATUS2 | reg.READ_FLAG, 0U);

    mApexStatus.SMD_DETECTED = static_cast<bool>(curByte >> 3 & 0b1);
    mApexStatus.WOM_Z_DET    = static_cast<bool>(curByte >> 2 & 0b1);
    mApexStatus.WOM_Y_DET    = static_cast<bool>(curByte >> 1 & 0b1);
    mApexStatus.WOM_X_DET    = static_cast<bool>(curByte & 0b1);

    // Status 3
    curByte = readRegister(0U, reg.BANK0.INT_STATUS3 | reg.READ_FLAG, 0U);

    mApexStatus.STEP_DETECTED       = static_cast<bool>(curByte >> 5 & 0b1);
    mApexStatus.STEP_COUNT_OVERFLOW = static_cast<bool>(curByte >> 4 & 0b1);
    mApexStatus.TILT_DETECTED       = static_cast<bool>(curByte >> 3 & 0b1);
    mApexStatus.WAKE_DETECTED       = static_cast<bool>(curByte >> 2 & 0b1);
    mApexStatus.SLEEP_DETECTED      = static_cast<bool>(curByte >> 1 & 0b1);
    mApexStatus.TAP_DETECTED        = static_cast<bool>(curByte & 0b1);

    return mApexStatus;
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

#pragma pop_macro("abs")