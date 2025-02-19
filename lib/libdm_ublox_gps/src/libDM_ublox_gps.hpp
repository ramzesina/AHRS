#ifndef ubloxGPS_hpp
#define ubloxGPS_hpp

#include "Arduino.h"
#include "libDM_generic_operations.hpp"
#include "libDM_stream_logger.hpp"
#include "libDM_timer_tool.hpp"
#include <vector>
#include <array>
// #include <algorithm>
// #include "numeric"

constexpr uint16_t NAV_PVT_PAYLOAD_SIZE   = 92U;
constexpr uint16_t GPS_DETECTION_NB_TRIES = 3U;
constexpr uint16_t NAV_PVT_BUFFER_SIZE =
        180U; // Total buffer size = 244 bytes = 2*NAV_PVT + 44 margin
constexpr uint16_t NAV_PVT_ESP32_BUFFER_SIZE = 2000U;

class ubloxGPS
{
public:
    ubloxGPS(HardwareSerial* serialObj,
             uint32_t serialSpeed,
             streamLogger* streamObj,
             timerTool* timerToolObject,
             int8_t rxPin = -1,
             int8_t txPin = -1)
    {
        mSerialObj   = serialObj;
        mSerialSpeed = serialSpeed;
        mStreamObj   = streamObj;
        mTimer       = timerToolObject;
        mValidBufferNavPvt.reserve(NAV_PVT_PAYLOAD_SIZE);
        mValidBufferNavPvt.resize(NAV_PVT_PAYLOAD_SIZE, 0);
        mRxPin = rxPin;
        mTxPin = txPin;
    }

    ~ubloxGPS() = default;

    struct
    {
        uint32_t iTow            = 0U;
        uint16_t year            = 0U;
        uint8_t month            = 0U;
        uint8_t day              = 0U;
        uint8_t hour             = 0U;
        uint8_t minute           = 0U;
        uint8_t seconds          = 0U;
        int32_t nanoSeconds      = 0U;
        bool dateValid           = false;
        uint8_t fixType          = 0U;
        uint8_t numSV            = 0U;
        double longitude         = 0.0;
        double latitude          = 0.0;
        float heightAboveSea     = 0.F;
        float horizontalAccuracy = 0.F;
        float verticalAccuracy   = 0.F;
        float velNord            = 0.F;
        float velEast            = 0.F;
        float velDown            = 0.F;
        float groundSpeed        = 0.F;
        float heading            = 0.F;
        float speedAccuracy      = 0.F;
        float headingAccuracy    = 0.F;
        uint32_t timestamp       = 0U;
    } NAV_PVT;

    void begin();
    bool getNavPvtMeasures(bool forceRead = false);

private:
    bool readPacket(uint8_t header1,
                    uint8_t header2,
                    uint8_t Class,
                    uint8_t ID, // NOLINT(readability-identifier-length)
                    uint8_t packetSize,
                    uint8_t payloadSize,
                    std::vector<uint8_t>& dataVectorValid);
    std::array<uint8_t, 2> Compute_CheckSum(const uint8_t buffer[],
                                            uint8_t size); // All frame excluding headers

    bool mGpsConnected           = false;
    HardwareSerial* mSerialObj   = NULL;
    streamLogger* mStreamObj     = NULL;
    timerTool* mTimer            = NULL;
    uint32_t mSerialSpeed        = 115200U;
    int8_t mRxPin                = -1;
    int8_t mTxPin                = -1;
    uint32_t mUsNoDataTimeOut    = 5000U; // Especially for esp32's serial peripheral that can hang
    uint32_t mLastFlushTimestamp = 0U;
    // NAV_PVT
    uint16_t mNavPvtPacketSize              = 100U;
    uint16_t mNavPvtPayLoadSize             = NAV_PVT_PAYLOAD_SIZE;
    uint8_t mNavPvtHeader1                  = 0xB5U;
    uint8_t mNavPvtHeader2                  = 0x62U;
    uint8_t mNavPvtClass                    = 0x01U;
    uint8_t mNavPvtID                       = 0x07U;
    std::vector<uint8_t> mValidBufferNavPvt = {};

    float deg2rad = 3.14159265359F / 180.F;
    float rad2deg = 180.F / 3.14159265359F;
};

#endif