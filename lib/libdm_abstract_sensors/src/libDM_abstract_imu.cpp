#define ERR_INVALID 255

#include "libDM_abstract_imu.hpp"
#pragma push_macro("abs")
#undef abs

void ABSTRACT_IMU::setConfig(SENSOR_TYPE type, const SENSOR_CORRECTIONS& correction)
{
    switch (type)
    {
    case SENSOR_TYPE::ACC:
        mImuCorrection.acc = correction;
        break;

    case SENSOR_TYPE::GYRO:
        mImuCorrection.gyro = correction;
        break;

    case SENSOR_TYPE::MAG:
        mImuCorrection.mag = correction;
        break;

    default:
        mStreamObj->printlnUlog(false, true, mClassName + ": Unknown config, nothing changed");
        break;
    }
}

const ABSTRACT_IMU::SENSOR_CORRECTIONS& ABSTRACT_IMU::getConfig(SENSOR_TYPE type)
{
    switch (type)
    {
    case SENSOR_TYPE::ACC:
        return mImuCorrection.acc;

    case SENSOR_TYPE::GYRO:
        return mImuCorrection.gyro;

    case SENSOR_TYPE::MAG:
        return mImuCorrection.mag;

    default:
        mStreamObj->printlnUlog(false, true, mClassName + ": Unknown config, acc struct return");
        return mImuCorrection.acc;
    }
}

const ABSTRACT_IMU::IMU_OUTPUT& ABSTRACT_IMU::getOutput()
{
    return mImuOut;
}

void ABSTRACT_IMU::calibGyro(uint32_t averagingNumber)
{
    if (!checkBeginState())
        return;

    String curString            = "";
    mImuCorrection.gyro.xyzBias = getGyroBias(averagingNumber, MAX_DPS_FOR_GYRO_BIAS);

    curString = mClassName + ": Gyrometer bias calibrated = ["
                + String(mImuCorrection.gyro.xyzBias[0], 4) + ", "
                + String(mImuCorrection.gyro.xyzBias[1], 4) + ", "
                + String(mImuCorrection.gyro.xyzBias[2], 4) + "]";
    mStreamObj->printlnUlog(true, true, curString);

    // TODO: scale factor
}

std::array<float, 3> ABSTRACT_IMU::getGyroBias(uint32_t averagingNumber,
                                               float dpsInterruptThreshold)
{
    std::array<float, 3> curGyro         = {0.F, 0.F, 0.F};
    std::array<float, 3> curGyroFiltered = {0.F, 0.F, 0.F};
    std::array<float, 3> gBias           = {0.F, 0.F, 0.F};

    CTRL_DM ctrl;

    for (uint16_t i = 0; i < averagingNumber; i++)
    {
        readGyro(); // TODO: Has to use fifo if activated
#pragma unroll
        for (uint8_t j = 0; j < 3; j++)
        {
            if (i == 0)
                ctrl.LPF1_DM(0.001F,
                             mImuOut.dpsGyroData[j],
                             true,
                             0.01F,
                             mImuOut.dpsGyroData[j],
                             &curGyroFiltered[j]);
            else
                ctrl.LPF1_DM(
                        0.001F, 0.F, false, 0.01F, mImuOut.dpsGyroData[j], &curGyroFiltered[j]);

            if (std::abs(curGyroFiltered[j]) >= dpsInterruptThreshold)
            {
                mStreamObj->printlnUlog(true,
                                        true,
                                        mClassName + ": Gyro bias calibration interrupted, "
                                                + String(curGyroFiltered[j], 4) + " dps > "
                                                + String(dpsInterruptThreshold, 4));
                return gBias;
            }
            curGyro[j] += curGyroFiltered[j];
        }
        delay(1);
    }

    for (uint8_t i = 0; i < 3; i++)
        gBias[i] = curGyro[i] / averagingNumber + mImuCorrection.gyro.xyzBias[i];

    return gBias;
}

bool ABSTRACT_IMU::checkImmobility(float dpsThreshold,
                                   uint32_t msDurationThreshold,
                                   uint32_t msImuPeriod,
                                   uint32_t msTimeOut)
{
    if (!checkBeginState())
        return false;

    // Use 10 values buffer to store gyro datas
    std::vector<std::array<float, 10>> gyroBuffer(
            3, {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});

    uint8_t bufferIdx                = 0U;
    uint8_t bufferSize               = 0U;
    uint32_t msCheckDuration         = 0U;
    uint32_t msImmobilityDetDuration = 0U;
    bool initFilterDone              = false;
    CTRL_DM ctrl;
    float curDiffFiltered[3] = {0.F, 0.F, 0.F};

    while (msCheckDuration < msTimeOut)
    {
        readGyro();
        for (size_t i = 0; i < 3; i++)
            gyroBuffer[i][bufferIdx] = mImuOut.dpsGyroData[i];

        bufferSize = std::min(bufferSize + 1, 10);
        ++bufferIdx;

        if (bufferIdx >= 10)
            bufferIdx = 0;

        if (bufferSize >= 10)
        {

            bool isUnderThreshold[3] = {false, false, false};

            // For each axis get min and max values
            for (size_t i = 0; i < 3; i++)
            {
                float curMax = -10000.F;
                float curMin = 10000.F;
                for (size_t j = 0; j < 10; j++)
                {
                    curMax = std::max(curMax, gyroBuffer[i][j]);
                    curMin = std::min(curMin, gyroBuffer[i][j]);
                }
                if (!initFilterDone)
                    ctrl.LPF1_DM((float)msImuPeriod * 0.001F,
                                 curMax - curMin,
                                 true,
                                 0.2F,
                                 curMax - curMin,
                                 &curDiffFiltered[i]);
                else
                    ctrl.LPF1_DM((float)msImuPeriod * 0.001F,
                                 0.F,
                                 false,
                                 0.2F,
                                 curMax - curMin,
                                 &curDiffFiltered[i]);
                isUnderThreshold[i] = curDiffFiltered[i] < dpsThreshold;
            }
            initFilterDone = true;

            // If all axis are under threshold, increment immobility duration
            if (isUnderThreshold[0] && isUnderThreshold[1] && isUnderThreshold[2])
            {
                msImmobilityDetDuration += msImuPeriod;
                if (msImmobilityDetDuration >= msDurationThreshold)
                {
                    mStreamObj->printlnUlog(true, true, mClassName + ": Immobility detected");
                    return true;
                }
            }
            else
            {
                msImmobilityDetDuration = 0U;
            }
        }

        msCheckDuration += msImuPeriod;
        delay(msImuPeriod);
    }

    mStreamObj->printlnUlog(true, true, mClassName + ": Immobility detection timeout");
    return false;
}

// ====================================================== //
// ================= Default definitions ================ //
// ====================================================== //
void ABSTRACT_IMU::readTemp()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readTemp() method not implemented in this context");
}
void ABSTRACT_IMU::readAcc()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readAcc() method not implemented in this context");
}
void ABSTRACT_IMU::readGyro()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readGyro() method not implemented in this context");
}

void ABSTRACT_IMU::readMag()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readTemp() method not implemented in this context");
}

void ABSTRACT_IMU::readAllImu()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readAllImu() method not implemented in this context");
}

void ABSTRACT_IMU::readAccGyro()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readAccGyro() method not implemented in this context");
}

uint8_t ABSTRACT_IMU::whoAmI()
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": whoAmI() method not implemented in this context");
    return ERR_INVALID;
}

#pragma pop_macro("abs")