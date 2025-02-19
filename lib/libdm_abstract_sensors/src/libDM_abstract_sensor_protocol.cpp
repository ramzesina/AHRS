#include "libDM_abstract_sensor_protocol.hpp"

uint8_t ABSTRACT_SENSOR_SPI::writeRegister(uint8_t sensorAdress, uint8_t address, uint8_t val)
{
    uint8_t res = 0; // NOLINT
    mSpiObj->beginTransaction(*mSpiConf);
    mTimerObj->delayNanosecondsAlt(100);

    OP::PIN_IO::digitalWriteFast(mSpiSS, LOW);
    mTimerObj->delayNanosecondsAlt(100);

    mSpiObj->transfer(address);
    res = mSpiObj->transfer(val);

    OP::PIN_IO::digitalWriteFast(mSpiSS, HIGH);

    mTimerObj->delayNanosecondsAlt(100);
    mSpiObj->endTransaction();

    return res;
}

uint8_t ABSTRACT_SENSOR_SPI::writeRegisterWithMask(uint8_t sensorAdress,
                                                   uint8_t address,
                                                   uint8_t oldRegVal,
                                                   uint8_t val,
                                                   uint8_t mask)
{
    uint8_t curByte = 0U;

    curByte = oldRegVal;
    curByte &= mask;
    curByte |= val;
    return writeRegister(0U, address, curByte);
}

uint8_t ABSTRACT_SENSOR_SPI::readRegister(uint8_t sensorAdress, uint8_t address, uint8_t val)
{
    uint8_t res = 0; // NOLINT
    mSpiObj->beginTransaction(*mSpiConf);
    mTimerObj->delayNanosecondsAlt(100);

    OP::PIN_IO::digitalWriteFast(mSpiSS, LOW);
    mTimerObj->delayNanosecondsAlt(100);

    mSpiObj->transfer(address);
    res = mSpiObj->transfer(val);

    OP::PIN_IO::digitalWriteFast(mSpiSS, HIGH);

    mTimerObj->delayNanosecondsAlt(100);
    mSpiObj->endTransaction();
    if (mSPIConfAtEndTransaction != NULL) // To avoid conflict between mode 0 and 3 for example
    {
        mSpiObj->beginTransaction(*mSPIConfAtEndTransaction);
        mSpiObj->endTransaction();
    }

    return res;
}

void ABSTRACT_SENSOR_SPI::readRegisterBurst(uint8_t sensorAdress,
                                            uint8_t startAddress,
                                            uint8_t* readBuf,
                                            uint8_t size,
                                            uint8_t val)
{
    mSpiObj->beginTransaction(*mSpiConf);
    mTimerObj->delayNanosecondsAlt(100);

    OP::PIN_IO::digitalWriteFast(mSpiSS, LOW);
    mTimerObj->delayNanosecondsAlt(100);

    mSpiObj->transfer(startAddress);
    mSpiObj->transfer(readBuf, size);

    OP::PIN_IO::digitalWriteFast(mSpiSS, HIGH);

    mTimerObj->delayNanosecondsAlt(100);
    mSpiObj->endTransaction();
    if (mSPIConfAtEndTransaction != NULL) // To avoid conflict between mode 0 and 3 for example
    {
        mSpiObj->beginTransaction(*mSPIConfAtEndTransaction);
        mSpiObj->endTransaction();
    }
}