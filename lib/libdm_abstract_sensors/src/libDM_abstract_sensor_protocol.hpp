#ifndef abstract_sensor_protocol_hpp
#define abstract_sensor_protocol_hpp

#include "libDM_abstract_sensors.hpp"

class ABSTRACT_SENSOR_SPI : public ABSTRACT_SENSOR
{
public:
    // Generic spi configuration agnostic of mcu used
    struct SPI_CONF
    {
        SPI_CONF() = default;
        explicit SPI_CONF(bool valid) :
            mIsValid(valid) {}; // explicit: To avoid compiler converting bool values to SPI_CONF if
                                // a function takes SPI_CONF as argument and is given a bool instead
        SPI_CONF(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) :
            mClock(clock), mBitOrder(bitOrder), mDataMode(dataMode), mIsValid(true) {};
        uint32_t mClock   = 4000000; // [Hz]
        uint8_t mBitOrder = MSBFIRST;
        uint8_t mDataMode = SPI_MODE0;
        bool mIsValid     = false;
    };

    ABSTRACT_SENSOR_SPI(uint8_t spiSS,
                        streamLogger* streamObj,
                        timerTool* timerToolObj,
                        SPIClass* spiObj,
                        SPI_CONF spiConf,
                        SPI_CONF spiConfAtEndTransaction) :
        ABSTRACT_SENSOR(streamObj, timerToolObj)
    {
        mSpiSS   = spiSS;
        mSpiConf = new SPISettings(spiConf.mClock, spiConf.mBitOrder, spiConf.mDataMode);
        if (spiConfAtEndTransaction.mIsValid)
            mSPIConfAtEndTransaction = new SPISettings(spiConfAtEndTransaction.mClock,
                                                       spiConfAtEndTransaction.mBitOrder,
                                                       spiConfAtEndTransaction.mDataMode);
        if (mSPIConfAtEndTransaction != NULL)
            mStreamObj->printlnUlog(
                    false,
                    true,
                    "ABSTRACT_SENSOR_SPI : will use specific spi configuration at the end of transaction");
        mSpiObj = spiObj;
    }
    virtual ~ABSTRACT_SENSOR_SPI() = default;
    // Copy constructor
    // ABSTRACT_SENSOR_SPI(const ABSTRACT_SENSOR_SPI& other) = default;
    ABSTRACT_SENSOR_SPI(const ABSTRACT_SENSOR_SPI& other) :
        ABSTRACT_SENSOR(other), mSpiSS(other.mSpiSS), mSpiObj(other.mSpiObj)
    {
        // Deep copy mSpiConf
        if (other.mSpiConf != NULL)
            mSpiConf = new SPISettings(*other.mSpiConf);
        else
            mSpiConf = NULL;

        // Deep copy mSPIConfAtEndTransaction
        if (other.mSPIConfAtEndTransaction != NULL)
            mSPIConfAtEndTransaction = new SPISettings(*other.mSPIConfAtEndTransaction);
        else
            mSPIConfAtEndTransaction = NULL;
    }
    // Copy assignment operator
    ABSTRACT_SENSOR_SPI& operator=(const ABSTRACT_SENSOR_SPI& other) = default;
    // Move constructor
    ABSTRACT_SENSOR_SPI(ABSTRACT_SENSOR_SPI&& other) noexcept = default;
    // // Move assignment operator
    // ABSTRACT_SENSOR_SPI& operator=(ABSTRACT_SENSOR_SPI&& other) noexcept = default;
    ABSTRACT_SENSOR_SPI& operator=(ABSTRACT_SENSOR_SPI&& other) noexcept
    {
        // Check for self-assignment
        if (this == &other)
            return *this;

        // Call base class operator= to copy base class members
        ABSTRACT_SENSOR::operator=(other);

        // Copy mSpiSS
        mSpiSS = other.mSpiSS;

        // Copy mSpiConf
        if (mSpiConf != NULL)
            delete mSpiConf;
        mSpiConf = new SPISettings(*other.mSpiConf);

        // Copy mSPIConfAtEndTransaction
        if (mSPIConfAtEndTransaction != NULL)
            delete mSPIConfAtEndTransaction;
        if (other.mSPIConfAtEndTransaction != NULL)
            mSPIConfAtEndTransaction = new SPISettings(*other.mSPIConfAtEndTransaction);
        else
            mSPIConfAtEndTransaction = NULL;

        // Copy mSpiObj
        mSpiObj = other.mSpiObj;

        return *this;
    }

protected:
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
    uint8_t mSpiSS                        = SS;
    SPIClass* mSpiObj                     = &SPI;
    double mNsOperationTimeout            = 1000000000; //[ns]
    SPISettings* mSpiConf                 = NULL;
    SPISettings* mSPIConfAtEndTransaction = NULL;

    // ~~~~~~~~~~ Overriden methods ~~~~~~~~~~ //
    uint8_t writeRegister(uint8_t sensorAdress, uint8_t address, uint8_t val) override;
    uint8_t writeRegisterWithMask(uint8_t sensorAdress,
                                  uint8_t address,
                                  uint8_t old_reg_val,
                                  uint8_t val,
                                  uint8_t mask) override;
    uint8_t readRegister(uint8_t sensorAdress, uint8_t address, uint8_t val) override;
    void readRegisterBurst(uint8_t sensorAdress,
                           uint8_t startAddress,
                           uint8_t* readBuf,
                           uint8_t size,
                           uint8_t val) override;
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
};

class ABSTRACT_SENSOR_I2C : public ABSTRACT_SENSOR
{
public:
    ABSTRACT_SENSOR_I2C(uint32_t i2cClockSpeed,
                        TwoWire* i2cObj,
                        streamLogger* streamObj,
                        timerTool* timerToolObj) :
        ABSTRACT_SENSOR(streamObj, timerToolObj)
    {
        mI2cClockSpeed = i2cClockSpeed;
        mI2cObj        = i2cObj;
    }
    ~ABSTRACT_SENSOR_I2C() override;
    // Copy constructor
    ABSTRACT_SENSOR_I2C(const ABSTRACT_SENSOR_I2C& other) = default;
    // Copy assignment operator
    ABSTRACT_SENSOR_I2C& operator=(const ABSTRACT_SENSOR_I2C& other) = default;
    // Move constructor
    ABSTRACT_SENSOR_I2C(ABSTRACT_SENSOR_I2C&& other) noexcept = default;
    // Move assignment operator
    ABSTRACT_SENSOR_I2C& operator=(ABSTRACT_SENSOR_I2C&& other) noexcept = default;

protected:
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
    TwoWire* mI2cObj           = &Wire;
    uint32_t mI2cClockSpeed    = 400000;    // [Hz]
    double mNsOperationTimeout = 100000000; //[ns]
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
};

#endif