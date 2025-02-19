/**
 * @file libDM_can_regs_enums.hpp
 * @brief This file contains the definition of CAN_ENUM, CAN_CONSTANTS, and CAN_DATATYPES structs.
 * CAN_ENUM struct contains enums for CAN coding type, data type, and baud rate.
 * CAN_CONSTANTS struct contains constants for CAN data size.
 * CAN_DATATYPES struct contains structs for frame and signal, and unions for converting data types
 * to binary.
 */

#ifndef can_regs_enums_hpp
#define can_regs_enums_hpp

#include <Arduino.h>
#include <vector>

constexpr uint8_t CAN_MESSAGE_MAX_SIZE = 8U; // Maximum size of a CAN message (standard frame)

/**
 * @brief This struct contains enums for CAN coding type, data type, and baud rate.
 */
struct CAN_ENUM
{
    enum class CAN_CODING_TYPE
    {
        MOTOROLA,
        INTEL
    };

    enum class DATATYPE
    {
        UINT8_T = 0,
        UINT16_T,
        UINT32_T,
        UINT64_T,
        INT8_T,
        INT16_T,
        INT32_T,
        INT64_T,
        FLOAT,
        DOUBLE,
        UNDEF
    };

    enum class BAUDRATE : uint16_t
    {
        _UNSET   = 0U,
        _125kbps = 125U,
        _250kbps = 250U,
        _500kbps = 500U,
        _1Mbps   = 1000U
    };

    enum class HEALTH : uint8_t
    {
        OK = 0U,
        WARNING, // Errors detected, can be recovered or device restarted
        ERROR    // Errors detected, cannot be recovered, device shutdown
    };
};

/**
 * @brief Defines constants for the size of data in bytes.
 */
struct CAN_CONSTANTS
{
    static const uint8_t SIZE_1_BYTE  = 1; /**< Size of data in bytes is 1. */
    static const uint8_t SIZE_2_BYTES = 2; /**< Size of data in bytes is 2. */
    static const uint8_t SIZE_4_BYTES = 4; /**< Size of data in bytes is 4. */
    static const uint8_t SIZE_8_BYTES = 8; /**< Size of data in bytes is 8. */
};

/**
 * @brief This structs contains the definition of structs used to represent frames and signals in a
 * CAN bus.
 * @details The `frameStruct` struct represents a frame in a CAN bus, containing information such as
 * the signals it contains, its name, ID, size, and whether it is enabled or not.
 *
 * The `signalStruct` struct represents a signal in a CAN bus, containing information such as its
 * name, data type, and whether it is valid or not.
 *
 * Additionally, this struct defines several unions used to convert between different data types and
 * their binary representation.
 */
struct CAN_DATATYPES
{
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    /**
     * @brief A struct representing a CAN frame.
     */
    struct frameStruct
    {
        uint16_t frameVectorIdx          = 0U; // idx of this frame in frameVector
        std::vector<uint16_t> signalsIdx = {}; // idx of signals in signalVector
        uint16_t signalsNumber           = 0U;
        String frameName                 = "";
        uint16_t frameId                 = 0x0U;
        uint8_t frameSize                = 0U;
        uint16_t maxNameStringLength = 0; // length of the longer string name inside this bus. It
                                          // helps to pad printing to have something pretty
        bool enabled = true;              // Enable or disable sending of this frame

        uint8_t receivedFrameData[CAN_MESSAGE_MAX_SIZE]; // This field is used ony when decoding a
                                                         // frame from rawRxBuffer. It should
                                                         // contain the same datas as the
                                                         // concatenated signals it contains but as
                                                         // the received signals are not always
                                                         // know, only serialized data is stored
                                                         // here.

        /**
         * @brief Set the enabled state of the frame.
         * @param state The state to set the enabled flag to.
         */
        void setEnabledState(bool state)
        {
            enabled = state;
        }

        /**
         * @brief Resets the state of the object to its default values.
         */
        void reset()
        {
            frameVectorIdx = 0U;
            signalsIdx     = {};
            signalsNumber  = 0U;
            frameName      = "";
            frameId        = 0x0U;
            frameSize      = 0U;
            enabled        = true;
            // receivedFrameData.fill(0);
        }
    };

    /**
     * @brief A struct representing a CAN signal.
     */
    struct signalStruct
    {
        void* signalAdress          = nullptr;
        String frameName            = "";
        uint16_t frameId            = 0x0;
        uint16_t frameIdx           = 0U; // idx of parent frame in frameVector
        String signalName           = "";
        CAN_ENUM::DATATYPE dataType = CAN_ENUM::DATATYPE::UNDEF;
        bool enabledSignal          = true; // If false, signal will not be sent on bus
    };
    // NOLINTEND(misc-non-private-member-variables-in-classes)

    struct CONVERTERS
    {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        using uint16_2_binary = union
        {
            uint16_t uint16Data = 0U;
            uint8_t byteTable[2];
        };

        using int16_2_binary = union
        {
            int16_t int16Data = 0;
            uint8_t byteTable[2];
        };

        using uint32_2_binary = union
        {
            uint32_t uint32Data = 0U;
            uint8_t byteTable[4];
        };

        using int32_2_binary = union
        {
            uint32_t int32Data = 0;
            uint8_t byteTable[4];
        };

        using float_2_binary = union
        {
            float floatData = 0.F;
            uint8_t byteTable[4];
        };

        using double_2_binary = union
        {
            double doubleData = 0.0;
            uint8_t byteTable[8];
        };

        using uint64_2_binary = union
        {
            uint64_t uint64Data = 0U;
            uint8_t byteTable[8];
        };

        using int64_2_binary = union
        {
            uint64_t int64Data = 0;
            uint8_t byteTable[8];
        };
        // NOLINTEND(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

        // Converters declarations
        uint16_2_binary mUint16ToBinary;
        uint32_2_binary mUint32ToBinary;
        uint64_2_binary mUint64ToBinary;
        int16_2_binary mInt16ToBinary;
        int32_2_binary mInt32ToBinary;
        int64_2_binary mInt64ToBinary;
        float_2_binary mFloatToBinary;
        double_2_binary mDoubleToBinary;
    };
};

#endif // can_regs_enums_hpp