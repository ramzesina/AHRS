/**
 * @file libDM_mavlink.hpp
 * @brief This file contains the definition of MAVLINK_UTILS class.
 */
#ifndef libdm_mavlink_hpp
#define libdm_mavlink_hpp

#include <Arduino.h>
#include <map>
#include <vector>
#include <deque>
#include <algorithm>

#include "libDM_stream_logger.hpp"
#include "libDM_mavlink_structs.hpp"

constexpr uint16_t MAVLINK_ESP32_BUFFER_SIZE = 2000U; // TODO: remove and do in msgHub
constexpr uint16_t MAVLINK_TEENSY_BUFFER_SIZE =
        3 * MAVLINK_MAX_PACKET_LEN; // TODO: remove and do in msgHub

/**
 * @brief A utility class for handling MAVLink messages.
 * @details This class provides functions to add messages to the monitoring list, add messages to
 * the sending list, add a MAVLink serial stream, update monitored messages, and send messages that
 * are ready to be sent. It also provides functions to get the MAVLINK_MSG_LIST of all monitored
 * messages and all messages to send.
 * @param streamObj A pointer to a streamLogger object.
 */
class MAVLINK_UTILS
{
public:
    MAVLINK_UTILS() = default;

    explicit MAVLINK_UTILS(streamLogger* streamObj) : mStreamObj(streamObj) {};

    /**
     * @brief Initiates the logging process of the sent/received messages.
     * @details This function initiates the logging process by setting up the necessary parameters
     * and starting the logging thread. Returns true if the logging process was successfully
     * initiated, false otherwise.
     * @warning This function should be called after all messages to monitor and send have been
     * added!
     * @return true if the logging process was successfully initiated, false otherwise.
     */
    bool initiateLogging();

    /**
     * @brief Adds a message to the monitoring list.
     * @details This function adds a message to the list of messages that are being monitored and
     * automatically retrived by the system.
     * @param msgID The ID of the message to add to the monitoring list.
     */
    void addMsgToMonitor(uint32_t msgID);

    /**
     * @brief Adds a message to the sending list.
     * @param msgID The ID of the message to add.
     * @param sendingInterfaces A vector of MAVLINK_SOURCE objects representing the interfaces to
     * send the message on.
     */
    void addMsgToSendingList(uint32_t msgID, const std::vector<MAVLINK_SOURCE>& sendingInterfaces);

    /**
     * @brief Adds a MAVLink serial stream for mavlink reception/emission.
     * @details This function adds a MAVLink serial stream to the library, which can be used to send
     * and receive MAVLink messages.
     * @param serialObj A pointer to the HardwareSerial object that represents the serial port to
     * use for the MAVLink stream.
     * @return true if the MAVLink stream was added successfully, false otherwise.
     */
    bool addMavlinkStream(HardwareSerial* serialObj);

    /**
     * @brief Updates all monitored message information.
     * @details This function updates all monitored messages information by checking for new
     * messages and updating the information for existing messages. It should be called periodically
     * to ensure that the monitored messages information is up-to-date. This function updates the
     * received messages using the buffer provided by the caller. This function does not directly
     * use hardware buffers to avoid pruning it if multiple protocols are used for the same
     * interface. For example if mavlink and protobuf are used on the same serial bus, the caller
     * has to copy the serial buffer into a temporary buffer and provide it to this function.
     * @param buffer A pointer to the buffer containing the received messages.
     * @param bufferLength The length of the buffer in bytes.
     */
    void updateMonitoredMessages(const uint8_t* buffer, size_t bufferLength);

    /**
     * @brief Sends messages that are ready to be sent.
     * @details This function sends messages that are ready to be sent. It should be called
     * periodically to ensure that all messages are sent in a timely manner.
     */
    void sendMessages();

    /**
     * @brief Sends message immediately on specified interface.
     * @details This function sends message immediately on specified interface. It can be called by
     * application to send the message on an interface even if the MAVLINK_MSG mSendInterfaces does
     * not contain it. Of course the the configured hardware interface has to be provided at least
     * one time with addMavlinkStream() for sendMessage() to succeed. doSend field is not updated by
     * this function.
     * @param source The MAVLink source to send the message from.
     * @param msg The MAVLink message to send.
     * @return True if the message was sent successfully, false otherwise.
     */
    bool sendMessage(MAVLINK_SOURCE source, const mavlink_message_t& msg);

    /**
     * @brief Sends message immediately on specified interfaces.
     * @param sources A vector of MAVLINK_SOURCE objects representing the sources to send the
     * message to.
     * @param msg The MAVLink message to send.
     * @return True if the message was sent successfully, false otherwise.
     */
    bool sendMessage(const std::vector<MAVLINK_SOURCE>& sources, const mavlink_message_t& msg);

    /**
     * @brief Returns a pointer to the list of all monitored MAVLink messages.
     * @warning This list should not be modified by the caller. Instead the caller should use the
     * MAVLINK_MSG_LIST::getSubset() function to get a subset of the messages it wants to follow.
     * @return A pointer to the list of monitored MAVLink messages.
     */
    const MAVLINK_MSG_LIST* getMonitoredMsgsList()
    {
        return &mMsgsMonitoredRefs;
    }

    /**
     * @brief Returns the MAVLINK_MSG_LIST of all messages to send.
     * @details This function returns a constant reference to the MAVLINK_MSG_LIST of all messages
     * that are currently queued to be sent.
     * @warning The returned reference should not be modified by the caller. Instead, the caller
     * should use the MAVLINK_MSG_LIST::getSubset() function to get a subset of the messages to send
     * and modify the messages in the subset.
     * @return A constant reference to the MAVLINK_MSG_LIST of all messages to send.
     */
    const MAVLINK_MSG_LIST* getMsgsToSendList()
    {
        return &mMsgsToSendRefs;
    }

private:
    /**
     * @brief Checks if the given MAVLINK_SOURCE is available as MAVLink stream.
     * @param source The MAVLINK_SOURCE to check.
     * @return true if the source is available, false otherwise.
     */
    inline bool isSource(
            const MAVLINK_SOURCE source) // Check is source is available for mavlink stream
    {
        return std::find(mSources.begin(), mSources.end(), source) != mSources.end();
    }

    streamLogger* mStreamObj   = NULL;
    HardwareSerial* mSerialObj = NULL;
    bool mIsLoggerReady        = false;

    /**
     * @brief List of all MAVLink sources used for MAVLink streams (serial, WiFi, BLE, etc.).
     */
    std::vector<MAVLINK_SOURCE> mSources = {};

    /**
     * @brief The status of the MAVLink communication link.
     */
    mavlink_status_t mStatus = mavlink_status_t();

    // ====================================================== //
    // ============= Mavlink messages references ============ //
    // ====================================================== //
    /**
     * @brief List of references of all monitored messages.
     * @details This list contains references to all monitored messages. It should not be
     * accessed directly, use the MAVLINK_MSG_LIST generated from it. You can do it with
     * getMonitoredMsgsList().getSubset(idx)
     */
    MAVLINK_MSG_LIST mMsgsMonitoredRefs = MAVLINK_MSG_LIST();

    /**
     * @brief List of references of all listed messages to send.
     * @details This list contains references to all messages to send. It should not be accessed
     * directly, use the MAVLINK_MSG_LIST generated from it. You can do it with
     * getMsgsToSendList().getSubset(idx)
     */
    MAVLINK_MSG_LIST mMsgsToSendRefs = MAVLINK_MSG_LIST();

    // ====================================================== //
    // ============== Mavlink messages storage ============== //
    // ====================================================== //

    // ~~~~~~~~~~ Messages monitored ~~~~~~~~~ //
    /**
     * @brief A pointer to a vector of MAVLINK_MSG objects that are being monitored.
     * @attention A container ensuring pointer validity with push_back() is required. A deque or a
     * list is adequate. A vector is not (or requires a reserve() before use).
     * @details Master list of all monitored messages, acting as a MAVLINK_MSG database. It should
     * not be accessed directly, use a subset of the MAVLINK_MSG_LIST mMsgsMonitoredRefs to access
     * messages
     */
    std::deque<MAVLINK_MSG>* mMsgsMonitored = new std::deque<MAVLINK_MSG>;

    /**
     * @brief List of all monitored messages IDs for each element in mMsgsMonitored (used to
     * retrieve the message from the mavlink message ID)
     */
    std::vector<uint32_t> mMsgsIDs = {};

    /**
     * @brief Map to retrieve the index of the message reference in the mMsgRefsList.
     * @details This map is used to retrieve the index of the message reference in the mMsgRefsList
     * based on the message ID. The key is the message ID and the value is the index of the
     * message reference in the mMsgRefsList.
     */
    std::map<uint32_t, uint32_t> mMsgsIDToMsgsMonitoredIndex = {};

    // ~~~~~~~~~~~ Messages to send ~~~~~~~~~~ //
    /**
     * @brief A deque of MAVLINK_MSG objects that are waiting to be sent.
     * @attention A container ensuring pointer validity with push_back() is required. A deque or a
     * list is adequate. A vector is not (or requires a reserve() before use).
     * @details Master list of all messages to send, acting as a MAVLINK_MSG database. AIt should
     * not be accessed directly, use a subset of the MAVLINK_MSG_LIST mMsgsToSendRefs to access
     * messages
     */
    std::deque<MAVLINK_MSG>* mMsgsToSend = new std::deque<MAVLINK_MSG>;

    /**
     * @brief List of all IDs for every message to send in mMsgsMonitored (used to retrieve
     * the message from the mavlink message ID)
     */
    std::vector<uint32_t> mMsgsToSendIDs = {};

    /**
     * @brief Map to retrieve the index of the message reference in the mMsgRefsList.
     * @details This map is used to retrieve the index of the message reference in the
     * mMsgsToSendRefs based on the message ID. The key is the message ID and the value is the index
     * of the message reference in the mMsgsToSendRefs.
     */
    std::map<uint32_t, uint32_t> mMsgsIDToMsgsToSendIndex = {};

    // ~~~~~~~~~~~~~~ Class name ~~~~~~~~~~~~~ //
    String mClassName = "MAVLINK_UTILS";
};

#endif
