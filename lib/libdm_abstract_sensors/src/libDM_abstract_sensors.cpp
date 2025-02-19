#include "libDM_abstract_sensors.hpp"

bool ABSTRACT_SENSOR::begin()
{
    mStreamObj->printlnUlog(
            true, true, mClassName + ": begin() method not implemented in this context");
    return false;
}

uint8_t ABSTRACT_SENSOR::readRegister(uint8_t sensorAdress, uint8_t address, uint8_t val)
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": readRegister() method not implemented in this context");
    return 255;
}

uint8_t ABSTRACT_SENSOR::writeRegister(uint8_t sensorAdress, uint8_t address, uint8_t val)
{
    mStreamObj->printlnUlog(
            false, false, mClassName + ": writeRegister() method not implemented in this context");
    return 255;
}

uint8_t ABSTRACT_SENSOR::writeRegisterWithMask(uint8_t sensorAdress,
                                               uint8_t address,
                                               uint8_t old_reg_val,
                                               uint8_t val,
                                               uint8_t mask)
{
    mStreamObj->printlnUlog(
            false,
            false,
            mClassName + ": writeRegisterWithMask() method not implemented in this context");
    return 255;
}

void ABSTRACT_SENSOR::readRegisterBurst(uint8_t sensorAdress,
                                        uint8_t startAddress,
                                        uint8_t* readBuf,
                                        uint8_t size,
                                        uint8_t val)
{
    mStreamObj->printlnUlog(
            false,
            false,
            mClassName + ": readRegisterBurst() method not implemented in this context");
}