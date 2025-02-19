#include "libDM_ublox_gps.hpp"

#if defined(CORE_TEENSY)
DMAMEM uint8_t serial5_extra_rx_buffer[NAV_PVT_BUFFER_SIZE]; // NOLINT
#endif

void ubloxGPS::begin()
{
#if defined(CORE_TEENSY)
    mSerialObj->addMemoryForRead(serial5_extra_rx_buffer, sizeof(serial5_extra_rx_buffer));
    mSerialObj->begin(mSerialSpeed);
#elif defined(ESP_PLATFORM)
    mSerialObj->setRxBufferSize(NAV_PVT_ESP32_BUFFER_SIZE);
    pinMode(mRxPin, INPUT);
    pinMode(mTxPin, OUTPUT);
    delay(1);
    mSerialObj->begin(mSerialSpeed, SERIAL_8N1, mRxPin, mTxPin);
#endif
    delay(100);

    uint8_t curNbTry;
    for (curNbTry = 0; curNbTry < GPS_DETECTION_NB_TRIES; curNbTry++)
    {
        if (readPacket(mNavPvtHeader1,
                       mNavPvtHeader2,
                       mNavPvtClass,
                       mNavPvtID,
                       mNavPvtPacketSize,
                       mNavPvtPayLoadSize,
                       mValidBufferNavPvt))
        {
            mStreamObj->printlnUlog(false, true, "GPS : GPS detected, NAV-PVT available");
            mGpsConnected = true;
            break;
        }
        else
        {
            delay(500);
            mStreamObj->printlnUlog(
                    false, true, "GPS : no GPS detected, NAV-PVT unavailable, retrying...");
        }
    }

    if (curNbTry == (uint8_t)(GPS_DETECTION_NB_TRIES))
    {
        mStreamObj->printlnUlog(false, true, "GPS : no GPS detected");
        return;
    }
}

// 45us on esp32, 9us on teensy
bool ubloxGPS::getNavPvtMeasures(bool forceRead) // NOLINT
{
    if (!mGpsConnected && !forceRead)
        return false;

    bool newPacket   = false; // NOLINT
    bool readSuccess = false;

    newPacket = readPacket(mNavPvtHeader1,
                           mNavPvtHeader2,
                           mNavPvtClass,
                           mNavPvtID,
                           mNavPvtPacketSize,
                           mNavPvtPayLoadSize,
                           mValidBufferNavPvt);

    if (newPacket)
    {
        // SBUS frame has been found. Checksum has already been verified

        // Time validity
        uint8_t validFlag = mValidBufferNavPvt[11];

        bool curDateValid = (validFlag & 0x1) && ((validFlag >> 1) & 0x1);

        if (curDateValid && !NAV_PVT.dateValid)
        {
            String curString =
                    "GPS: NAV_PVT date is now valid \n \t Year : "
                    + String(mValidBufferNavPvt[4] | mValidBufferNavPvt[5] << 8) + "\n \t Month : "
                    + String(mValidBufferNavPvt[6]) + "\n \t Day : " + String(mValidBufferNavPvt[7])
                    + "\n \t Hour : " + String(mValidBufferNavPvt[8] + 1) + ":"
                    + String(mValidBufferNavPvt[9]) + ":" + String(mValidBufferNavPvt[10])
                    + "\n \t Itow : "
                    + String(mValidBufferNavPvt[0] | (mValidBufferNavPvt[1] << 8)
                             | (mValidBufferNavPvt[2] << 16) | (mValidBufferNavPvt[3] << 24));

            mStreamObj->printlnUlog(false, true, curString);
        }
        else if (!curDateValid && NAV_PVT.dateValid)
        {
            mStreamObj->printlnUlog(false, true, "GPS: NAV-PVT date is now invalid");
        }

        if (curDateValid) // Date and time valids
        {
            NAV_PVT.dateValid = true;
        }
        else
        {
            NAV_PVT.dateValid = false;
            return false;
        }

        uint32_t curItow = mValidBufferNavPvt[0] | (mValidBufferNavPvt[1] << 8)
                           | (mValidBufferNavPvt[2] << 16) | (mValidBufferNavPvt[3] << 24);

        if (NAV_PVT.iTow == curItow) // Data is not new
            return false;

        NAV_PVT.iTow        = curItow;
        NAV_PVT.year        = mValidBufferNavPvt[4] | mValidBufferNavPvt[5] << 8;
        NAV_PVT.month       = mValidBufferNavPvt[6];
        NAV_PVT.day         = mValidBufferNavPvt[7];
        NAV_PVT.hour        = mValidBufferNavPvt[8] + 1;
        NAV_PVT.minute      = mValidBufferNavPvt[9];
        NAV_PVT.seconds     = mValidBufferNavPvt[10];
        NAV_PVT.nanoSeconds = mValidBufferNavPvt[16] | mValidBufferNavPvt[17] << 8
                              | mValidBufferNavPvt[18] << 16 | mValidBufferNavPvt[19] << 24;

        // Fix status
        if (mValidBufferNavPvt[21] & 0x1) // GNSS fixed
        {
            uint8_t curFixType = mValidBufferNavPvt[20];
            if (curFixType != NAV_PVT.fixType)
            {
                switch (curFixType)
                {
                case 2:
                    mStreamObj->printlnUlog(false, true, "GPS: NAV_PVT fix type is now 2D");
                    break;
                case 3:
                    mStreamObj->printlnUlog(false, true, "GPS: NAV_PVT fix type is now 3D");
                    break;

                default:
                    break;
                }
            }

            NAV_PVT.fixType = curFixType;
        }
        else
        {
            if (NAV_PVT.fixType == 2 || NAV_PVT.fixType == 3)
                mStreamObj->printlnUlog(false, true, "GPS: NAV-PVT fix has been lost");

            NAV_PVT.fixType = 0;
        }

        NAV_PVT.numSV        = mValidBufferNavPvt[23];
        int32_t curLongitude = mValidBufferNavPvt[24] | mValidBufferNavPvt[25] << 8
                               | mValidBufferNavPvt[26] << 16 | mValidBufferNavPvt[27] << 24;
        NAV_PVT.longitude = (double)curLongitude * 1e-7;

        int32_t curLatitude = mValidBufferNavPvt[28] | mValidBufferNavPvt[29] << 8
                              | mValidBufferNavPvt[30] << 16 | mValidBufferNavPvt[31] << 24;
        NAV_PVT.latitude = (double)curLatitude * 1e-7;

        int32_t curHeightAboveSea = mValidBufferNavPvt[36] | mValidBufferNavPvt[37] << 8
                                    | mValidBufferNavPvt[38] << 16 | mValidBufferNavPvt[39] << 24;
        NAV_PVT.heightAboveSea = (float)curHeightAboveSea * 1e-3f;

        uint32_t curHorizontalAccuracy = mValidBufferNavPvt[40] | mValidBufferNavPvt[41] << 8
                                         | mValidBufferNavPvt[42] << 16
                                         | mValidBufferNavPvt[43] << 24;
        NAV_PVT.horizontalAccuracy = (float)curHorizontalAccuracy * 1e-3f;

        uint32_t verticalAccuracy = mValidBufferNavPvt[44] | mValidBufferNavPvt[45] << 8
                                    | mValidBufferNavPvt[46] << 16 | mValidBufferNavPvt[47] << 24;
        NAV_PVT.verticalAccuracy = (float)verticalAccuracy * 1e-3f;

        int32_t curVelNord = mValidBufferNavPvt[48] | mValidBufferNavPvt[49] << 8
                             | mValidBufferNavPvt[50] << 16 | mValidBufferNavPvt[51] << 24;
        NAV_PVT.velNord = (float)curVelNord * 1e-3f;

        int32_t curVelEast = mValidBufferNavPvt[52] | mValidBufferNavPvt[53] << 8
                             | mValidBufferNavPvt[54] << 16 | mValidBufferNavPvt[55] << 24;
        NAV_PVT.velEast = (float)curVelEast * 1e-3f;

        int32_t curVelDown = mValidBufferNavPvt[56] | mValidBufferNavPvt[57] << 8
                             | mValidBufferNavPvt[58] << 16 | mValidBufferNavPvt[59] << 24;
        NAV_PVT.velDown = (float)curVelDown * 1e-3f;

        int32_t curGroundSpeed = mValidBufferNavPvt[60] | mValidBufferNavPvt[61] << 8
                                 | mValidBufferNavPvt[62] << 16 | mValidBufferNavPvt[63] << 24;
        NAV_PVT.groundSpeed = (float)curGroundSpeed * 1e-3f;

        int32_t curHeading = mValidBufferNavPvt[64] | mValidBufferNavPvt[65] << 8
                             | mValidBufferNavPvt[66] << 16 | mValidBufferNavPvt[67] << 24;
        NAV_PVT.heading = (float)curHeading * 1e-5f * deg2rad;

        uint32_t curSpeedAccuracy = mValidBufferNavPvt[68] | mValidBufferNavPvt[69] << 8
                                    | mValidBufferNavPvt[70] << 16 | mValidBufferNavPvt[71] << 24;
        NAV_PVT.speedAccuracy = (float)curSpeedAccuracy * 1e-3f;

        uint32_t curHeadingAccuracy = mValidBufferNavPvt[72] | mValidBufferNavPvt[73] << 8
                                      | mValidBufferNavPvt[74] << 16 | mValidBufferNavPvt[75] << 24;
        NAV_PVT.headingAccuracy = (float)curHeadingAccuracy * 1e-5f * deg2rad;

        NAV_PVT.timestamp = mTimer->returnSystemTimestampUs();
        readSuccess       = true;
    }
    else if ((mTimer->returnSystemTimestampUs() - NAV_PVT.timestamp) > mUsNoDataTimeOut)
    {
        if (mTimer->returnSystemTimestampUs() > (mLastFlushTimestamp + mUsNoDataTimeOut))
        {
            mLastFlushTimestamp = mTimer->returnSystemTimestampUs();
            OP::UART::clearBuffer(mSerialObj);
        }
    }

    return readSuccess;
}

// Lire en marche arrière du plus récent au plus vieux et break dès qu'on en trouve 1. Sinon réduire
// buffer d'entrée, au pire, DMA. Un paquet = 7us
bool ubloxGPS::readPacket(uint8_t header1,
                          uint8_t header2,
                          uint8_t Class,
                          uint8_t ID, // NOLINT(readability-identifier-length)
                          uint8_t packetSize,
                          uint8_t payloadSize,
                          std::vector<uint8_t>& dataVectorValid)
{
    bool readSuccess        = false;
    uint16_t uartBufferSize = 0U;
    uartBufferSize          = mSerialObj->available();

    if (uartBufferSize < payloadSize)
        return false;

    if (uartBufferSize >= packetSize)
    {
        // cppcheck-suppress variableScope
        // cppcheck-suppress unreadVariable
        uint8_t curByte = 0U;
        // cppcheck-suppress variableScope
        // cppcheck-suppress unreadVariable
        uint8_t nextByte = 0U;
        // Extract uart buffer to temp buffer for faster parsing (ESP32 especially)
        uint8_t tempUartBuffer[uartBufferSize];
        uint16_t idxTempUartBuffer = 0U;
        OP::UART::readBytesBuffer(mSerialObj, tempUartBuffer, uartBufferSize);

        // Will extract the last packet from serial buffer
        while (idxTempUartBuffer < uartBufferSize) // NOLINT
        {
            curByte = tempUartBuffer[idxTempUartBuffer];
            idxTempUartBuffer += 1;
            nextByte = tempUartBuffer[idxTempUartBuffer];
            idxTempUartBuffer += 1;
            if (curByte == header1 && nextByte == header2)
            {
                // std::vector<uint8_t> dataFrame(packetSize - 2, 0);
                uint8_t dataFrame[packetSize - 2];
#pragma unroll
                for (size_t i = 0; i < 4; i++)
                {
                    dataFrame[i] = tempUartBuffer[idxTempUartBuffer];
                    idxTempUartBuffer += 1;
                }
                uint16_t framePayloadSize = dataFrame[2] | dataFrame[3] << 8;

                if (dataFrame[0] == Class && dataFrame[1] == ID && framePayloadSize == payloadSize)
                {
                    if ((uartBufferSize - idxTempUartBuffer) < payloadSize + 2)
                        break; // No need to parse, not enough data remaining (payload + 2 bytes
                               // checksum)

                    std::copy(&tempUartBuffer[idxTempUartBuffer],
                              &tempUartBuffer[idxTempUartBuffer + payloadSize + 2],
                              &dataFrame[4]);
                    idxTempUartBuffer += payloadSize + 2;

                    std::array<uint8_t, 2> cks = Compute_CheckSum(dataFrame, packetSize - 2);
                    if (cks[0] == dataFrame[packetSize - 4] && cks[1] == dataFrame[packetSize - 3])
                    {
                        // Update output buffer with the local one
                        std::copy(&dataFrame[0] + 4,
                                  &dataFrame[0] + mNavPvtPayLoadSize + 4,
                                  dataVectorValid.begin());
                        readSuccess = true;
                    }
                }
            }
        }
        OP::UART::clearBuffer(mSerialObj);
    }

    return readSuccess;
}

std::array<uint8_t, 2> ubloxGPS::Compute_CheckSum(const uint8_t buffer[],
                                                  uint8_t size) // All frame excluding headers
{
    std::array<uint8_t, 2> GPS_CK = {0, 0};
#pragma unroll
    for (uint8_t i = 0; i < size - 2; i++)
    {
        GPS_CK[0] += buffer[i];
        GPS_CK[1] += GPS_CK[0];
        GPS_CK[0] &= 0xFF;
        GPS_CK[1] &= 0xFF;
    }

    return GPS_CK;
}