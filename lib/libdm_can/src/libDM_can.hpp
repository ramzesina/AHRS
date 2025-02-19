/**
 * @file libDM_can.hpp
 * @brief Header file for the CAN_DRIVER class.
 * @details This file contains the declaration of the CAN_DRIVER class and its methods.
 * The CAN_DRIVER class provides an interface for communicating with a CAN bus.
 * It allows sending and receiving CAN frames, decoding signals and mapping variables to CAN frames.
 * The class is designed to work with the ESP32 microcontroller and the TWAI driver.
 */

#ifndef libDM_can_hpp
#define libDM_can_hpp

#include "Arduino.h"
#include <vector>
#include <array>
#include <algorithm>
#include <map>
#include "driver/twai.h"
#include "driver/gpio.h"
#include "libDM_stream_logger.hpp"
#include "libDM_timer_tool.hpp"
#include "libDM_can_regs_enums.hpp"
#include <type_traits>

constexpr uint32_t CAN_BUFFER_MAX_FRAME_NUMBER = 50U; // Buffer will be allocated for 50 frames
constexpr uint32_t CAN_DECODED_BUFFER_MAX_FRAME_NUMBER =
        20U; // Decoded frames buffer will be allocated for 20 frames
constexpr uint32_t CAN_MAX_RECOVER_NB = 100U; // Maximum number of effective restarts
constexpr uint8_t CAN_MAX_RETRY_ATTEMPTS_BEGIN =
        5U; // Maximum number of restarts attempts at begin init
constexpr uint8_t CAN_MAX_RESTARTS_ATTEMPTS  = 20U;  // Maximum number of restarts attempts
constexpr uint8_t CAN_MAX_TX_BUFFER_QUEUE    = 5U;   // Maximum TX queue clearing attempts.
constexpr uint8_t CAN_MAX_ERRORS_THRESHOLD   = 20U;  // Maximum number of errors before restart
constexpr uint8_t CAN_DIAGNOSTIC_RESERVED_ID = 0xFF; // This frame can not be used by the user
#define CAN_DIAGNOSTIC_RESERVED_ID_STR "0xFF"
constexpr uint8_t CAN_DIAGNOSTIC_DEFAULT_FRAME_SEND_NB = 5U;
constexpr uint8_t CAN_RECOVER_FRAME_SEND_NB            = 2U;

/**
 * @brief This class provides an interface for communicating with a CAN bus, simplifying both
 * emission/reception
 */
class CAN_DRIVER
{
public:
    CAN_DRIVER() = default;

    /**
     * @brief Constructs a new CAN_DRIVER object.
     * @details Complete constructor for the CAN_DRIVER class. Begin method can be called
     * immediately after
     * @param baudrate The baudrate of the CAN communication.
     * @param pinRX The RX pin of the CAN transceiver.
     * @param pinTX The TX pin of the CAN transceiver.
     * @param streamObj A pointer to the streamLogger object used for logging.
     * @param timerToolObj A pointer to the timerTool object used for timing.
     * @param loopbackMode Whether to enable or not loopback mode.
     */
    CAN_DRIVER(CAN_ENUM::BAUDRATE baudrate,
               uint8_t pinRX,
               uint8_t pinTX,
               streamLogger* streamObj,
               timerTool* timerToolObj,
               bool loopbackMode = true) :
        mPinRx((gpio_num_t)pinRX),
        mPinTx((gpio_num_t)pinTX),
        mBaudRate(baudrate),
        mStreamObj(streamObj),
        mTimerObj(timerToolObj),
        mUserIsLoopbackMode(loopbackMode)
    {
    }

    /**
     * @brief Constructs a new CAN_DRIVER object.
     * @details Partial constructor for the CAN_DRIVER class. setPins and setBaudrate methods have
     * to be called before calling begin method.
     * @param streamObj Pointer to a streamLogger object for logging messages.
     * @param timerToolObj Pointer to a timerTool object for timing operations.
     * @param loopbackMode Whether to enable or not loopback mode.
     */
    CAN_DRIVER(streamLogger* streamObj, timerTool* timerToolObj, bool loopbackMode = true) :
        mStreamObj(streamObj), mTimerObj(timerToolObj), mUserIsLoopbackMode(loopbackMode)
    {
    }

    /**
     * @brief Initializes the CAN bus communication.
     * @details Install and initialize CAN driver
     * @return true if the initialization was successful, false otherwise.
     */
    bool begin();

    /**
     * @brief Returns the current readiness status of the CAN driver.
     * @return true if the object is ready, false otherwise.
     */
    inline bool getIsReady() const
    {
        return mIsReady;
    }

    /**
     * @brief Sets the RX and TX pins for the CAN bus communication.
     * @param pinRX The GPIO pin number for the CAN bus RX pin.
     * @param pinTX The GPIO pin number for the CAN bus TX pin.
     */
    void setPins(gpio_num_t pinRX, gpio_num_t pinTX);

    /**
     * @brief Sets the baudrate of the CAN bus.
     * @param baudrate The desired baudrate to set.
     */
    void setBaudrate(CAN_ENUM::BAUDRATE baudrate);

    /**
     * @brief Sends all registered and enabled CAN frames.
     * @details This function sends all registered and enabled CAN frames that have been added to
     * the transmit buffer using the `addFrame` function.
     * @note After the use to addVariable function, the signal and eventually a frame are
     * automatically added to the transmit buffer. The user is supposed tu update the variable and
     * call this function to send the frame with the signal updated
     * @note This function implies synchronous sending. All enabled signals/frames will be sent at
     * the same time. If asynchronous sending is needed, the user can also keep a reference of each
     * frame, use serializeFrame then sendFrame manually. Another solution is to use these
     * references to enable/disable frames or signals and call this function.
     * @return `true` if frames were sent successfully, `false` otherwise.
     */
    bool sendAllFrames();

    /**
     * @brief Serializes and sends a CAN frame with the given ID.
     * @details This function serializes the data in the CAN frame and sends it over the
     * CAN bus. The frame ID is used to retrieve the frame from the frame vector. The user is
     * supposed to update the frames messages before calling this function.
     * @param frameId The ID of the CAN frame to send.
     * @return true if the frame was successfully sent, false otherwise.
     */
    bool serializeAndSendFrame(uint32_t frameId);
    /**
     * @brief Serializes a CAN frame and sends it over the bus.
     * @details Overload version of serializeAndSendFrame(uint32_t frameId). This function takes a
     * CAN frame as input, serializes it, and sends it over the CAN bus.
     * @param frame The CAN frame to be serialized and sent.
     * @return True if the frame was successfully sent, false otherwise.
     */
    bool serializeAndSendFrame(const CAN_DATATYPES::frameStruct* frame);

    /**
     * @brief Receives CAN frames from the bus.
     * @details This function receives CAN frames from the bus and stores them in
     * mRawMessageRxBuffer buffer.
     * @note also update all dysfunctional indicators
     * @return true if frames were successfully received, false otherwise.
     */
    bool receiveFrames();

    /**
     * @brief Decodes a signal from a CAN message frame.
     * @attention This function is not implemented yet.
     * @param message The CAN message frame to decode the signal from.
     * @param startBit The bit position of the signal within the message frame.
     * @param signalSize The size of the signal in bits.
     * @param coding The coding type used for the signal.
     * @param datatype The data type of the signal.
     * @param factor The scaling factor to apply to the decoded value.
     * @param offset The offset to apply to the decoded value.
     * @return The decoded value of the signal.
     */
    float decodeFrame(twai_message_t message,
                      uint8_t startBit,
                      uint8_t signalSize,
                      CAN_ENUM::CAN_CODING_TYPE coding,
                      CAN_ENUM::DATATYPE datatype,
                      float factor,
                      float offset) const;

    /**
     * @brief Decodes the raw receive buffer data.
     * @details This function decodes the raw receive buffer data and populates
     * mDecodedFramesRxBuffer with frameStruct
     * @note This function has to be called for any Id that needs to be decoded. Do not forget to
     * reset decodedBuffer at the beginning of the loop!
     * @param frameId The ID of the frame to decode.
     * @param frame The CAN frame structure to fill
     * @return true if the decoding was successful, false otherwise.
     */
    bool decodeRawRxBuffer(uint32_t frameId, CAN_DATATYPES::frameStruct& frame);

    /**
     * @brief Prints information about the CAN mapping (frames and signals)
     * @details This function prints information about the mapping to the serial monitor and/or on a
     * storage with streamLogger.
     * @param logSerialMonitor Whether to log the information to the serial monitor.
     * @param logStreamLogger Whether to log the information to a storage with streamLogger.
     */
    void printMappingInfo(bool logSerialMonitor, bool logStreamLogger);

    /**
     * @brief Prints information about a CAN signal.
     * @param signal The CAN signal to print information about.
     * @param logSerialMonitor Whether to log the information to the serial monitor.
     * @param logStreamLogger Whether to log the information to the stream logger.
     */
    void printSignalInfo(CAN_DATATYPES::signalStruct signal,
                         bool logSerialMonitor,
                         bool logStreamLogger);

    /**
     * @brief Adds a variable to the CAN signal list.
     * @details The function will append signal to existing frame or create a new one if necessary.
     * @tparam T The type of the variable to add.
     * @param variable A pointer to the variable to add.
     * @param name The name of the signal.
     * @param frame The frame ID of the signal.
     * @param frameName The name of the frame.
     * @param forceAdd Whether to force the addition of the variable to the signal list.
     * @return true if the variable was added successfully, false otherwise.
     */
    template <typename T>
    bool addVariable(T* variable,
                     String name,
                     uint32_t frameId,
                     String frameName,
                     bool forceAdd = false)
    {
        if (frameId == CAN_DIAGNOSTIC_RESERVED_ID && !forceAdd)
        {
            mStreamObj->printlnUlog(true,
                                    true,
                                    mClassName + ": frame ID " + String(frameId)
                                            + " is reserved for diagnostic purpose");
            return false;
        }

        CAN_ENUM::DATATYPE type            = CAN_ENUM::DATATYPE::UNDEF;
        CAN_DATATYPES::signalStruct signal = CAN_DATATYPES::signalStruct();
        uint8_t size                       = 0U;

        if (std::is_same<typename std::remove_pointer<T>::type, uint8_t>::value)
        {
            type = CAN_ENUM::DATATYPE::UINT8_T;
            size = CAN_CONSTANTS::SIZE_1_BYTE;
        }
        else if (std::is_same<typename std::remove_pointer<T>::type, uint16_t>::value)
        {
            type = CAN_ENUM::DATATYPE::UINT16_T;
            size = CAN_CONSTANTS::SIZE_2_BYTES;
        }
        else if (std::is_same<typename std::remove_pointer<T>::type, uint32_t>::value)
        {
            type = CAN_ENUM::DATATYPE::UINT32_T;
            size = CAN_CONSTANTS::SIZE_4_BYTES;
        }
        else if (std::is_same<typename std::remove_pointer<T>::type, uint64_t>::value)
        {
            type = CAN_ENUM::DATATYPE::UINT64_T;
            size = CAN_CONSTANTS::SIZE_8_BYTES;
        }
        else if (std::is_same<typename std::remove_pointer<T>::type, int8_t>::value)
        {
            type = CAN_ENUM::DATATYPE::INT8_T;
            size = CAN_CONSTANTS::SIZE_1_BYTE;
        }
        else if (std::is_same<typename std::remove_pointer<T>::type, int16_t>::value)
        {
            type = CAN_ENUM::DATATYPE::INT16_T;
            size = CAN_CONSTANTS::SIZE_2_BYTES;
        }
        else if (std::is_same<typename std::remove_pointer<T>::type, int32_t>::value)
        {
            type = CAN_ENUM::DATATYPE::INT32_T;
            size = CAN_CONSTANTS::SIZE_4_BYTES;
        }
        else if (std::is_same<typename std::remove_pointer<T>::type, int64_t>::value)
        {
            type = CAN_ENUM::DATATYPE::INT64_T;
            size = CAN_CONSTANTS::SIZE_8_BYTES;
        }
        else if (std::is_same<typename std::remove_pointer<T>::type, float>::value)
        {
            type = CAN_ENUM::DATATYPE::FLOAT;
            size = CAN_CONSTANTS::SIZE_4_BYTES;
        }
        else if (std::is_same<typename std::remove_pointer<T>::type, double>::value)
        {
            type = CAN_ENUM::DATATYPE::DOUBLE;
            size = CAN_CONSTANTS::SIZE_8_BYTES;
        }
        else
        {
            mStreamObj->printlnUlog(true, true, mClassName + ": unknown datatype");
            return false;
        }

        signal.dataType     = type;
        signal.frameId      = frameId;
        signal.frameName    = frameName;
        signal.signalAdress = static_cast<void*>(variable);
        signal.signalName   = name;

        // Search for frame info and create if it does not exist
        for (size_t idx = 0; idx < mFrameVector.size(); idx++)
        {
            if (frameId == mFrameVector[idx].frameId)
            {
                size += mFrameVector[idx].frameSize;
                if (size > CAN_MESSAGE_MAX_SIZE)
                {
                    mStreamObj->printlnUlog(true,
                                            true,
                                            mClassName + ": can not add signal in frame, size > 8, "
                                                    + signal.signalName + " has not been added");
                    return false;
                }
                signal.frameIdx = idx;
                mFrameVector[idx].signalsIdx.push_back(mSignalIdx);
                mSignalVector.push_back(signal);
                if (name.length() > mFrameVector[idx].maxNameStringLength)
                    mFrameVector[idx].maxNameStringLength = name.length();
                mFrameVector[idx].signalsNumber++;
                mFrameVector[idx].frameSize = size;
                mSignalIdx++;
                return true;
            }
        }

        // Create frame if it has not been found
        CAN_DATATYPES::frameStruct newFrame = CAN_DATATYPES::frameStruct(); // NOLINT
        newFrame.frameId                    = frameId;
        newFrame.frameName                  = frameName;
        newFrame.maxNameStringLength        = frameName.length();
        newFrame.frameSize                  = size;
        newFrame.signalsIdx.push_back(mSignalIdx);
        newFrame.signalsNumber        = 1;
        mFrameIDToFrameIndex[frameId] = mFrameVector.size();
        mFrameVector.push_back(newFrame);
        // Complete signal frame infos
        signal.frameIdx = mFrameVector.size() - 1;
        mSignalVector.push_back(signal);

        mSignalIdx++;
        return true;
    }

    /**
     * @brief Retrieves a CAN frame from the internal buffer based on its ID.
     * @param frameID The ID of the frame to retrieve.
     * @return A pointer to the retrieved frame, or nullptr if the frame was not found.
     */
    CAN_DATATYPES::frameStruct* getFrameFromID(uint32_t frameID)
    {
        if (getFrameNumber() == 0 || !isFrameInBuffer(frameID))
            return nullptr;

        return &mFrameVector.at(mFrameIDToFrameIndex.at(frameID));
    }

    /**
     * @brief Returns the frame number in the library frame vector
     * @return The frame number of the frame vector
     */
    inline uint32_t getFrameNumber() const
    {
        return mFrameVector.size();
    }

    /**
     * @brief Returns a pointer to the buffer containing received CAN messages.
     * @return A pointer to the reception buffer.
     */
    inline const std::array<twai_message_t, CAN_BUFFER_MAX_FRAME_NUMBER>* getRawRxBuffer() const
    {
        // cppcheck-suppress CastIntegerToAddressAtReturn
        return &mRawMessageRxBuffer;
    }

    /**
     * @brief Returns the size of the raw buffer used by this object.
     * @return The size of the raw buffer in bytes.
     */
    inline const uint32_t getRawBufferSize() const
    {
        return mRawMessageNumberRxBuffer;
    }

    /**
     * @brief Retrieves a signal structure from the given signal name.
     * @param signalName The name of the signal to retrieve.
     * @return A pointer to the signal structure, or nullptr if the signal was not found.
     */
    CAN_DATATYPES::signalStruct* getSignalFromName(String signalName)
    {
        for (size_t idx = 0; idx < mSignalVector.size(); idx++)
        {
            if (mSignalVector[idx].signalName.equals(signalName))
                return &mSignalVector[idx];
        }
        return nullptr;
    }

    /**
     * @brief Attempts to recover bus until the maximum number of attempts is reached.
     * @return true if the recovery was successful, false otherwise.
     */
    bool recoverBus();

    /**
     * @brief Monitor the CAN bus and take actions accordingly
     * @details This function checks the health of the CAN bus with checking errors and statuses. It
     * can disable the bus if too much errors are detected, restart it, and allow recover.
     * @note The user of the can driver is highly encouraged to call this function regularly to
     * avoid blockings and allow bus recovery
     */
    inline void monitorBus()
    {
        updateStatus();
        updateErrors();
    }

    /**
     * @brief Returns the current health status of the CAN driver.
     * @return The current health status of the CAN driver.
     */
    inline CAN_ENUM::HEALTH getHealth() const
    {
        return mHealth;
    }

private:
    /**
     * @brief Checks if the driver is ready to send and receive messages.
     * @param level The health level to check for.
     * @return true if the module is ready, false otherwise.
     */
    inline bool checkReady(CAN_ENUM::HEALTH level = CAN_ENUM::HEALTH::WARNING) const
    {
        uint8_t curHealth = static_cast<uint8_t>(mHealth);
        uint8_t reqHealth = static_cast<uint8_t>(level);
        return (mIsStarted & mIsReady & (curHealth < reqHealth));
    }

    /**
     * @brief Sends a CAN frame using the specified message.
     * @note also update all dysfunctional indicators
     * @param message The message to send.
     * @param forceSend Whether to force the sending of the frame, even if driver is not ready.
     * @return True if the frame was sent successfully, false otherwise.
     */
    bool sendFrame(twai_message_t message, bool forceSend = false);

    /**
     * @brief Checks if a CAN frame with the given ID is present in the buffer.
     * @param frameId The ID of the CAN frame to check for.
     * @return True if a frame with the given ID is present in the buffer, false otherwise.
     */
    inline bool isFrameInBuffer(uint32_t frameId) const
    {
        return mFrameIDToFrameIndex.find(frameId) != mFrameIDToFrameIndex.end();
    }

    /**
     * @brief Returns a string representation of the given CAN_ENUM::DATATYPE.
     * @param datatype The CAN_ENUM::DATATYPE to get a string representation for.
     * @return A string representation of the given datatype.
     */
    String getStringForDatatype(CAN_ENUM::DATATYPE datatype) const;

    /**
     * @brief Returns a string representation of the given CAN signal value.
     * @details This function takes a CAN signal struct and returns a string representation of its
     * value. The returned string is formatted according to the signal's data type.
     * @param signal The CAN signal to convert to a string.
     * @return A string representation of the given CAN signal value.
     */
    String getStringForValue(const CAN_DATATYPES::signalStruct& signal) const;

    /**
     * @brief Serializes a CAN frame into a buffer.
     * @brief This function takes a CAN frame and serializes it into a buffer. The buffer
     * can then be transmitted over a CAN bus. The length of the serialized frame
     * is returned in the `bufferLength` parameter.
     * @param frame Pointer to the CAN frame to serialize.
     * @param buffer A pointer to the buffer holding serialized frame. This buffer should at least
     * have a size of frame.frameSize, or 8 bytes for safety
     * @return `true` if the frame was successfully serialized, `false` otherwise.
     */
    bool serializeFrame(const CAN_DATATYPES::frameStruct* frame, uint8_t* frameBuffer) const;

    /**
     * @brief Update can driver status and take action if necessary
     */
    void updateStatus();

    /**
     * @brief Checks for errors counters and take action if necessary
     * @note This function should be called after updateStatus function
     * @note This function still runs if the error is only a warning (failed restart) to allow bus
     * recovery
     */
    void updateErrors();

    /**
     * @brief Checks the status of the CAN bus and returns true if it is functioning properly.
     * @details At CAN driver initialization,will send CAN frames in order to detect transceiver
     * correct communication.
     * @param nbFrameToSend The number of frames to send for the diagnosis.
     * @return true if the CAN bus is functioning properly, false otherwise.
     */
    bool diagnoseBus(size_t nbFrameToSend = CAN_DIAGNOSTIC_DEFAULT_FRAME_SEND_NB);

    /**
     * @brief Sets the loopback mode for can driver usage
     * @details When loopback mode is enabled, transmitted messages are also received by the module.
     * Also no ack is required to transmit messages. The driver will need complete reinstallation.
     * @param loopbackMode Whether to enable loopback mode.
     * @return True if the loopback mode was successfully set, false otherwise.
     */
    bool setLoopbackMode(bool loopbackMode);

    bool mIsReady                          = false;
    bool mIsStarted                        = false;
    bool mIsLoopbackMode                   = true;
    bool mUserIsLoopbackMode               = false;
    gpio_num_t mPinRx                      = GPIO_NUM_NC;
    gpio_num_t mPinTx                      = GPIO_NUM_NC;
    double mNsTimeout                      = 1000000000.0; // NOLINT // [s] 1s
    uint32_t mMsTimeoutFrame               = 0U;           // [ms] no need to wait as we use buffers
    uint32_t mMsBusRecoverTimeoutThreshold = 1000U;        // [ms]
    uint32_t mMsLastRestartTime            = 0U;           // [ms]
    uint32_t mBufferSizeRX                 = 2048U;
    uint32_t mBufferSizeTX                 = 2048U;
    CAN_ENUM::BAUDRATE mBaudRate           = CAN_ENUM::BAUDRATE::_UNSET;

    twai_general_config_t mConfig =
            TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_NC, GPIO_NUM_NC, TWAI_MODE_NO_ACK);
    twai_timing_config_t mTimingConfig = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t mFilterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    streamLogger* mStreamObj                               = nullptr;
    timerTool* mTimerObj                                   = nullptr;
    std::vector<CAN_DATATYPES::frameStruct> mFrameVector   = {};
    std::vector<CAN_DATATYPES::signalStruct> mSignalVector = {};
    uint16_t mSignalIdx                                    = 0U;

    // Monitoring variables
    twai_status_info_t mStatus = twai_status_info_t();
    uint32_t mConfiguredAlerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_ERR_PASS
                                 | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL
                                 | TWAI_ALERT_ABOVE_ERR_WARN;
    CAN_ENUM::HEALTH mHealth         = CAN_ENUM::HEALTH::OK;
    CAN_ENUM::HEALTH mHealthPrev     = CAN_ENUM::HEALTH::OK;
    uint32_t mRestartCounter         = 0U;
    uint8_t mRestartAttemptsCounter  = 0U;
    uint8_t mTxBufferClearingCounter = 0U;
    uint8_t mErrorsCounter           = 0U;

    std::map<uint32_t, uint32_t> mFrameIDToFrameIndex = {}; // Used to retrieve frame index from
    // frame ID.

    // Allocated variables
    CAN_DATATYPES::CONVERTERS* mConverters = new CAN_DATATYPES::CONVERTERS();
    uint8_t* mFrameBuffer                  = new uint8_t[CAN_MESSAGE_MAX_SIZE];

    // Save a buffer of the last received frames
    std::array<twai_message_t, CAN_BUFFER_MAX_FRAME_NUMBER> mRawMessageRxBuffer = {};
    uint32_t mRawMessageNumberRxBuffer                                          = 0U;

    // ~~~~~~~~~~~~~~ Class name ~~~~~~~~~~~~~ //
    String mClassName = "CAN_DRIVER";
};

#endif