#ifndef ABSTRACT_SENSOR_hpp
#define ABSTRACT_SENSOR_hpp

#include <array>
#include <vector>
#include "Arduino.h"
#include "libDM_ctrl.hpp"
#include "libDM_stream_logger.hpp"
#include "libDM_timer_tool.hpp"
#include "libDM_generic_operations.hpp"
#include "SPI.h"
#include "Wire.h"

class ABSTRACT_SENSOR
{
public:
    ABSTRACT_SENSOR(streamLogger* streamObj, timerTool* timerToolObj)
    {
        mStreamObj = streamObj;
        mTimerObj  = timerToolObj;
    }
    virtual ~ABSTRACT_SENSOR() = default;
    // Copy constructor
    ABSTRACT_SENSOR(const ABSTRACT_SENSOR& other) = default;
    // Copy assignment operator
    ABSTRACT_SENSOR& operator=(const ABSTRACT_SENSOR& other) = default;
    // Move constructor
    ABSTRACT_SENSOR(ABSTRACT_SENSOR&& other) noexcept = default;
    // Move assignment operator
    ABSTRACT_SENSOR& operator=(ABSTRACT_SENSOR&& other) noexcept = default;

    // ~~~~~~~~~~~~ shared methods ~~~~~~~~~~~ //
    /**
     * @brief return true if the sensor has been started correctly, false otherwise
     *
     */
    inline bool checkBeginState()
    {
        return mBegin;
    };

    // ====================================================== //
    // ================= Struct definitions ================= //
    // ====================================================== //
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
    /**
     * @brief Structure representing sensor corrections.
     */
    struct SENSOR_CORRECTIONS
    {
        std::array<float, 3> xyzBias{{0.F, 0.F, 0.F}}; /**< XYZ bias values. */
        std::array<float, 3> xyzGain{{1.F, 1.F, 1.F}}; /**< XYZ gain values. */

        struct // TODO: Carto 1D DM à templater et autoriser l'extrap
        {
            std::array<float, 5> degTemperatureAxis{
                    {0.F, 0.F, 0.F, 0.F, 0.F}}; /**< Temperature axis in degrees. */
            std::array<float, 5> driftValue{{0.F, 0.F, 0.F, 0.F, 0.F}}; /**< Drift values. */
        } TEMPERATURE_DRIFT;

        struct // TODO: Carto 1D DM à templater et autoriser l'extrap
        {
            std::array<float, 5> sTimeAxis{{0.F, 0.F, 0.F, 0.F, 0.F}}; /**< Time axis in seconds. */
            std::array<float, 5> driftValue{{0.F, 0.F, 0.F, 0.F, 0.F}}; /**< Drift values. */
        } TEMPORAL_DRIFT;
    };
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
protected:
    // ====================================================== //
    // ================== Protected methods ================= //
    // ====================================================== //
    // ~~~~~~~~ Pure virtual functions ~~~~~~~ //

    /**
     * @brief Initializes the sensor.
     * @warning This method has to be called for every sensor before any other method.
     * @return true if the sensor is successfully initialized, false otherwise.
     */
    virtual bool begin();

    /**
     * @brief Write to sensor with i2c/spi/i3c protocol... sensor adress is unused with spi
     *
     * @param sensorAdress sensor adress
     * @param address register adress in which modify values
     * @param val value to write at register adress
     * @return uint8_t protocol transfer return
     */
    virtual uint8_t writeRegister(uint8_t sensorAdress, uint8_t address, uint8_t val);

    /**
     * @brief Write to sensor with i2c/spi/i3c protocol with the value indicated, but without
     * affecting other registers bits. sensor adress is unused with spi.
     * @param sensorAdress sensor adress
     * @param address register adress in which modify values
     * @param old_reg_val a read of the actual register
     * @param val value to write at register adress
     * @param mask mask to apply to the register to erase bits at the position you want to write in.
     * A mask is an uint8_t with 1 at bits positions you want to keep, and 0 for bits positions you
     * want to erase. For example, 0b10101010 will erase bits at positions 6, 4, 2, 0
     * @return uint8_t protocol transfer return
     */
    virtual uint8_t writeRegisterWithMask(uint8_t sensorAdress,
                                          uint8_t address,
                                          uint8_t old_reg_val,
                                          uint8_t val,
                                          uint8_t mask);

    /**
     * @brief Read sensor register value with i2c/spi/i3c protocol... sensor adress is unused with
     * spi
     * @param sensorAdress sensor adress
     * @param address register adress in which read values
     * @param val value to write at register adress to ask for data transfer (usually 0)
     * @return uint8_t protocol transfer return
     */
    virtual uint8_t readRegister(uint8_t sensorAdress, uint8_t address, uint8_t val);

    /**
     * @brief Read sensor registers in burst mode. If the sensor is compatible, it will latch n
     * bytes with adressing only the starting adress. sensor adress is unused with spi
     * @param sensorAdress sensor adress
     * @param startAddress starting adress from where to start bursting read
     * @param readBuf pointer to the first adress of an array used to store data return by burst
     * read
     * @param size indicates the number of bytes read by the burst
     * @param val value to write at register adress to ask for data transfer (usually 0)
     */
    virtual void readRegisterBurst(uint8_t sensorAdress,
                                   uint8_t startAddress,
                                   uint8_t* readBuf,
                                   uint8_t size,
                                   uint8_t val);

    // ~~~~~~~~~~~~ shared methods ~~~~~~~~~~~ //
    /**
     * @brief Set the result of the begin method from derived class
     *
     * @param state begin state
     */
    inline void setBeginState(bool state)
    {
        mBegin = state;
    }

    // ====================================================== //
    // ================== Protected values ================== //
    // ====================================================== //
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)
    streamLogger* mStreamObj = NULL;
    timerTool* mTimerObj     = NULL;
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes,
    // misc-non-private-member-variables-in-classes)

private:
    bool mBegin       = false;
    String mClassName = "ABSTRACT_SENSOR";
};

#endif