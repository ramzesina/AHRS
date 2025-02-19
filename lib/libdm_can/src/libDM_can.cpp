#include "libDM_can.hpp"

bool CAN_DRIVER::begin()
{
    if (mPinRx == GPIO_NUM_NC || mPinTx == GPIO_NUM_NC || mBaudRate == CAN_ENUM::BAUDRATE::_UNSET)
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": invalid base configuration");
        return false;
    }

    // Driver is configured in no ackMode for diagnostics
    CAN_ENUM::BAUDRATE selectedBaudrate = CAN_ENUM::BAUDRATE::_500kbps;
    // Configure CAN Driver
    mConfig.rx_io          = mPinRx;
    mConfig.tx_io          = mPinTx;
    mConfig.mode           = TWAI_MODE_NO_ACK;
    mConfig.clkout_io      = (gpio_num_t)TWAI_IO_UNUSED;
    mConfig.clkout_divider = 0;
    mConfig.bus_off_io     = (gpio_num_t)TWAI_IO_UNUSED;
    mConfig.tx_queue_len   = mBufferSizeTX;
    mConfig.rx_queue_len   = mBufferSizeRX;
    mConfig.alerts_enabled = mConfiguredAlerts; // Are not checked for the moment

    switch (mBaudRate)
    {
    case CAN_ENUM::BAUDRATE::_125kbps:
        mTimingConfig    = TWAI_TIMING_CONFIG_125KBITS();
        selectedBaudrate = CAN_ENUM::BAUDRATE::_125kbps;
        break;
    case CAN_ENUM::BAUDRATE::_250kbps:
        mTimingConfig    = TWAI_TIMING_CONFIG_250KBITS();
        selectedBaudrate = CAN_ENUM::BAUDRATE::_250kbps;
        break;
    case CAN_ENUM::BAUDRATE::_500kbps:
        mTimingConfig     = TWAI_TIMING_CONFIG_500KBITS();
        mTimingConfig.sjw = 2;
        break;
    case CAN_ENUM::BAUDRATE::_1Mbps:
        mTimingConfig     = TWAI_TIMING_CONFIG_1MBITS();
        mTimingConfig.sjw = 2;
        selectedBaudrate  = CAN_ENUM::BAUDRATE::_1Mbps;
        break;
    default:
        mTimingConfig = TWAI_TIMING_CONFIG_500KBITS();
        break;
    }

    mFilterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    mStreamObj->printlnUlog(true,
                            true,
                            mClassName + ": driver configured, baudrate = "
                                    + String(static_cast<uint16_t>(selectedBaudrate))
                                    + ", acceptance filter = ACCEPT_ALL");

    // Install driver
    mTimerObj->startTiming(0);
#pragma unroll
    while (twai_driver_install(&mConfig, &mTimingConfig, &mFilterConfig) != ESP_OK)
    {
        if (mTimerObj->stopTiming(0) >= mNsTimeout)
        {
            mStreamObj->printlnUlog(true, true, mClassName + "failed to install CAN driver");
            return false;
        }
        delay(100); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    }

    // Start CAN driver
#pragma unroll
    while (twai_start() != ESP_OK)
    {
        delay(100); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        if (mTimerObj->stopTiming(0) >= mNsTimeout)
        {
            mStreamObj->printlnUlog(true, true, mClassName + "failed to start CAN driver");
            return false;
        }
    }

    mIsStarted = true;

    mStreamObj->printlnUlog(true, true, mClassName + ": driver succesfully started");

    delay(10);

    // Perform initial diagnostic
    uint8_t counter = 0U;
    while (!diagnoseBus())
    {
        if (mHealth == CAN_ENUM::HEALTH::ERROR || counter >= CAN_MAX_RETRY_ATTEMPTS_BEGIN)
        {
            mStreamObj->printlnUlog(
                    true,
                    true,
                    mClassName + ": failed to restart CAN driver during diagnostic, exiting");
            mHealth = CAN_ENUM::HEALTH::ERROR;
            return false;
        }
        delay(10);
        counter++;
    }

    mStreamObj->printlnUlog(
            true,
            true,
            mClassName + ": bus initial diagnostic passed, changing mode to user configured mode");

    if (!setLoopbackMode(mUserIsLoopbackMode))
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": failed to change mode to user mode");
        return false;
    }

    mStreamObj->printlnUlog(true, true, mClassName + ": driver ready");
    mIsReady = true;
    return true;
}

bool CAN_DRIVER::setLoopbackMode(bool loopbackMode)
{
    if (!mIsStarted || mHealth == CAN_ENUM::HEALTH::ERROR)
        return false;

    if (mIsLoopbackMode == loopbackMode)
        return true;

    mIsLoopbackMode = loopbackMode;

    bool result         = true;
    twai_mode_t newMode = TWAI_MODE_NO_ACK;

    if (loopbackMode)
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": changing mode to loopback");
    }
    else
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": changing mode to normal");
        newMode = TWAI_MODE_NORMAL;
    }

    mConfig.mode = newMode;
    result &= twai_stop() == ESP_OK;
    result &= twai_driver_uninstall() == ESP_OK;
    result &= twai_driver_install(&mConfig, &mTimingConfig, &mFilterConfig) == ESP_OK;
    twai_start();

    mIsLoopbackMode = loopbackMode;

    return result;
}

bool CAN_DRIVER::diagnoseBus(size_t nbFrameToSend)
{
    if (!mIsStarted || mHealth == CAN_ENUM::HEALTH::ERROR)
        return false;

    bool result                          = true;
    uint8_t testData                     = 0U;
    CAN_DATATYPES::frameStruct* curFrame = nullptr;

    // Check if frame already exists in frame vector and add it if necessary
    curFrame = getFrameFromID(CAN_DIAGNOSTIC_RESERVED_ID);
    if (curFrame == nullptr)
    {
        addVariable(&testData,
                    "testData",
                    CAN_DIAGNOSTIC_RESERVED_ID,
                    CAN_DIAGNOSTIC_RESERVED_ID_STR,
                    true);
        curFrame          = getFrameFromID(CAN_DIAGNOSTIC_RESERVED_ID);
        curFrame->enabled = false; // Disabled frame
    }

    twai_clear_transmit_queue();

    // Prepare frame
    twai_message_t message;
    message.identifier       = curFrame->frameId;
    message.extd             = 1U;
    message.data_length_code = curFrame->frameSize;

    serializeFrame(curFrame, mFrameBuffer);

    // Send frame
    std::copy(&mFrameBuffer[0], &mFrameBuffer[0] + curFrame->frameSize, &message.data[0]);

    for (size_t i = 0; i < nbFrameToSend; i++)
        result &= sendFrame(message, true);

    delay(1); // Should let the driver enough time to send one frame

    updateStatus();

    if (result && mStatus.msgs_to_tx < nbFrameToSend) // Check if at least one frame was sent
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": bus diagnostic passed");
    }
    else
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": bus diagnostic failed");
        result = false;
    }

    return result;
}

bool CAN_DRIVER::recoverBus()
{
    if (!mIsStarted || mHealth == CAN_ENUM::HEALTH::ERROR)
        return false;

    bool result = true;

    mRestartAttemptsCounter++;
    mMsLastRestartTime = mTimerObj->returnSystemTimestampMs();

    mStreamObj->printlnUlog(true, true, mClassName + ": trying to recover CAN bus");

    twai_initiate_recovery();
    twai_start();

    result = diagnoseBus(CAN_RECOVER_FRAME_SEND_NB);
    updateStatus();
    result &= mStatus.state == TWAI_STATE_RUNNING;
    if (!result)
    {
        mStreamObj->printlnUlog(true,
                                true,
                                mClassName + ": failed to recover CAN bus, attempt = "
                                        + String(mRestartAttemptsCounter));
        return false;
    }

    mErrorsCounter          = 0U;
    mRestartAttemptsCounter = 0U;
    mRestartCounter++;

    return true;
}

void CAN_DRIVER::updateStatus()
{
    if (!mIsStarted || mHealth == CAN_ENUM::HEALTH::ERROR)
        return;

    twai_get_status_info(&mStatus);
}

void CAN_DRIVER::updateErrors()
{
    if (!checkReady(CAN_ENUM::HEALTH::ERROR))
        return;

    // Restart errors
    if (mRestartCounter >= CAN_MAX_RECOVER_NB)
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": max effective restarts reached, there "
                                                      "is probably a hardware issue");
        mHealth = CAN_ENUM::HEALTH::ERROR;
    }
    if (mRestartAttemptsCounter >= CAN_MAX_RESTARTS_ATTEMPTS)
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": max restart attempts reached, there is "
                                                      "probably a hardware issue");
        mHealth = CAN_ENUM::HEALTH::ERROR;
    }

    bool checkActive = checkReady();

    // Status errors
    if (checkActive && mStatus.state != TWAI_STATE_RUNNING)
    {
        mStreamObj->printlnUlog(
                true, true, mClassName + ": bus status state error detected, try restart");
        if (!recoverBus()) // TODO: passer en loopback et effectuer les tests
        {
            setLoopbackMode(true);
            mHealth = CAN_ENUM::HEALTH::WARNING;
        }
    }

    if (mStatus.msgs_to_rx >= mBufferSizeRX)
    {
        mStreamObj->printlnUlog(
                true, true, mClassName + ": bus status RX buffer full detected, flushing");
        twai_clear_receive_queue();
    }

    if (mStatus.msgs_to_tx >= mBufferSizeTX)
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": tx buffer full detected, flushing");
        twai_clear_transmit_queue(); // TODO: mesurer le temps, réduire les buffers et
                                     // éventuellement retirer les messages
        if (mIsLoopbackMode)         // We want to perform this check only in loopback mode
        {
            mTxBufferClearingCounter++;
            if (mTxBufferClearingCounter >= CAN_MAX_TX_BUFFER_QUEUE)
            {
                mStreamObj->printlnUlog(true,
                                    true,
                                    mClassName + ": tx buffer clearing attemps reached."
                                                 "This could indicate an absense of transceiver");
                mHealth = CAN_ENUM::HEALTH::ERROR;
            }
        }
    }

    // Other errors
    if (checkActive && mErrorsCounter >= CAN_MAX_ERRORS_THRESHOLD)
    {
        mStreamObj->printlnUlog(true, true, mClassName + ": max errors threshold reached");
        if (!recoverBus())
        {
            setLoopbackMode(true);
            mHealth = CAN_ENUM::HEALTH::WARNING;
        }
    }

    // Recovery
    if (mHealth == CAN_ENUM::HEALTH::WARNING)
    {
        if ((mTimerObj->returnSystemTimestampMs() - mMsLastRestartTime)
                    > mMsBusRecoverTimeoutThreshold
            && recoverBus())
        {
            setLoopbackMode(mUserIsLoopbackMode);
            mHealth = CAN_ENUM::HEALTH::OK;
        }
    }

    // Update previous status
    if (mHealth == mHealthPrev)
        return;

    if (mHealthPrev == CAN_ENUM::HEALTH::WARNING)
    {
        if (mHealth == CAN_ENUM::HEALTH::OK)
            mStreamObj->printlnUlog(true, true, mClassName + ": bus recovered!");
        if (mHealth == CAN_ENUM::HEALTH::ERROR)
            mStreamObj->printlnUlog(
                    true, true, mClassName + ": bus critical error detected, bus goes error!");
    }

    if (mHealthPrev == CAN_ENUM::HEALTH::OK)
    {
        if (mHealth == CAN_ENUM::HEALTH::WARNING)
            mStreamObj->printlnUlog(
                    true, true, mClassName + ": bus error detected, bus goes warning!");
        if (mHealth == CAN_ENUM::HEALTH::ERROR)
            mStreamObj->printlnUlog(
                    true, true, mClassName + ": bus critical error detected, bus goes error!");
    }

    mHealthPrev = mHealth;
}

void CAN_DRIVER::setPins(gpio_num_t pinRX, gpio_num_t pinTX)
{
    mPinRx = pinRX;
    mPinTx = pinTX;
}

void CAN_DRIVER::setBaudrate(CAN_ENUM::BAUDRATE baudrate)
{
    mBaudRate = baudrate;
}

bool CAN_DRIVER::serializeFrame(const CAN_DATATYPES::frameStruct* frame, uint8_t* frameBuffer) const
{
    CAN_DATATYPES::signalStruct signal = // cppcheck-suppress unreadVariable
            CAN_DATATYPES::signalStruct();
    uint8_t frameDataIdx = 0;

    // Loop on all signals in frame and complete frame databyte
    for (size_t j = 0; j < frame->signalsIdx.size(); j++)
    {
        signal = mSignalVector[frame->signalsIdx[j]];
        switch (signal.dataType)
        {
        case CAN_ENUM::DATATYPE::UINT8_T:
            frameBuffer[frameDataIdx] = (*(uint8_t*)signal.signalAdress);
            frameDataIdx += 1;
            break;
        case CAN_ENUM::DATATYPE::UINT16_T:
            mConverters->mUint16ToBinary.uint16Data = (*(uint16_t*)signal.signalAdress);
            for (size_t k = 0; k < 2; k++)
                frameBuffer[frameDataIdx + k] = mConverters->mUint16ToBinary.byteTable[k];
            frameDataIdx += 2;
            break;
        case CAN_ENUM::DATATYPE::UINT32_T:
            mConverters->mUint32ToBinary.uint32Data = (*(uint32_t*)signal.signalAdress);
            for (size_t k = 0; k < 4; k++)
                frameBuffer[frameDataIdx + k] = mConverters->mUint32ToBinary.byteTable[k];
            frameDataIdx += 4;
            break;
        case CAN_ENUM::DATATYPE::UINT64_T:
            mConverters->mUint64ToBinary.uint64Data = (*(uint64_t*)signal.signalAdress);
            for (size_t k = 0; k < 8; k++)
                frameBuffer[frameDataIdx + k] = mConverters->mUint64ToBinary.byteTable[k];
            frameDataIdx += 8;
            break;
        case CAN_ENUM::DATATYPE::INT8_T:
            frameBuffer[frameDataIdx] = (*(int8_t*)signal.signalAdress);
            frameDataIdx += 1;
            break;
        case CAN_ENUM::DATATYPE::INT16_T:
            mConverters->mInt16ToBinary.int16Data = (*(int16_t*)signal.signalAdress);
            for (size_t k = 0; k < 2; k++)
                frameBuffer[frameDataIdx + k] = mConverters->mInt16ToBinary.byteTable[k];
            frameDataIdx += 2;
            break;
        case CAN_ENUM::DATATYPE::INT32_T:
            mConverters->mInt32ToBinary.int32Data = (*(int32_t*)signal.signalAdress);
            for (size_t k = 0; k < 4; k++)
                frameBuffer[frameDataIdx + k] = mConverters->mInt32ToBinary.byteTable[k];
            frameDataIdx += 4;
            break;
        case CAN_ENUM::DATATYPE::INT64_T:
            mConverters->mInt64ToBinary.int64Data = (*(int64_t*)signal.signalAdress);
            for (size_t k = 0; k < 8; k++)
                frameBuffer[frameDataIdx + k] = mConverters->mInt64ToBinary.byteTable[k];
            frameDataIdx += 8;
            break;
        case CAN_ENUM::DATATYPE::FLOAT:
            mConverters->mFloatToBinary.floatData = (*(float*)signal.signalAdress);
            for (size_t k = 0; k < 4; k++)
                frameBuffer[frameDataIdx + k] = mConverters->mFloatToBinary.byteTable[k];
            frameDataIdx += 4;
            break;
        case CAN_ENUM::DATATYPE::DOUBLE:
            mConverters->mDoubleToBinary.doubleData = (*(double*)signal.signalAdress);
            for (size_t k = 0; k < 8; k++)
                frameBuffer[frameDataIdx + k] = mConverters->mDoubleToBinary.byteTable[k];
            frameDataIdx += 8;
            break;
        default:
            return false;
            break;
        }
    }

    return true;
}

bool CAN_DRIVER::serializeAndSendFrame(const CAN_DATATYPES::frameStruct* frame)
{
    monitorBus();

    if (!checkReady())
        return false;

    if (!frame->enabled)
        return false;

    twai_message_t message;
    message.identifier       = frame->frameId;
    message.extd             = 1U;
    message.data_length_code = frame->frameSize;

    serializeFrame(frame, mFrameBuffer);

    // Send frame
    std::copy(&mFrameBuffer[0], &mFrameBuffer[0] + frame->frameSize, &message.data[0]);
    return sendFrame(message);
}

bool CAN_DRIVER::serializeAndSendFrame(uint32_t frameId)
{
    monitorBus();

    if (!checkReady())
        return false;

    const CAN_DATATYPES::frameStruct* frame = getFrameFromID(frameId);

    if (frame == nullptr || !frame->enabled)
        return false;

    return serializeAndSendFrame(frame);
}

// Less than 10us per frame at 500kbps
bool CAN_DRIVER::sendAllFrames() // NOLINT
{
    monitorBus();

    if (!checkReady())
        return false;

    bool result = true;

    for (size_t i = 0; i < mFrameVector.size(); i++)
    {
        if (!mFrameVector[i].enabled)
            continue;

        twai_message_t message;
        message.identifier       = mFrameVector[i].frameId;
        message.extd             = 1U;
        message.data_length_code = mFrameVector[i].frameSize;

        serializeFrame(&mFrameVector[i], mFrameBuffer);

        // Send frame
        std::copy(&mFrameBuffer[0], &mFrameBuffer[0] + mFrameVector[i].frameSize, &message.data[0]);
        result &= sendFrame(message);
    }

    return result;
}

bool CAN_DRIVER::sendFrame(twai_message_t Message, bool forceSend)
{
    if (!checkReady() && !forceSend)
        return false;

    bool res = true;
    res &= twai_transmit(&Message, pdMS_TO_TICKS(mMsTimeoutFrame)) == ESP_OK;

    if (!res)
        mErrorsCounter++;

    return res;
}

bool CAN_DRIVER::receiveFrames() // NOLINT(readability-make-member-function-const)
{
    monitorBus();

    if (!checkReady())
        return false;

    if (mStatus.msgs_to_rx == 0)
        return false;

    mRawMessageNumberRxBuffer = 0U;
    uint32_t maxIterNumber    = std::min(mStatus.msgs_to_rx, CAN_BUFFER_MAX_FRAME_NUMBER);
    bool result               = true;
    twai_message_t message    = twai_message_t();
#pragma unroll
    for (size_t i = 0; i < maxIterNumber; i++)
    {
        if (twai_receive(&message, pdMS_TO_TICKS(mMsTimeoutFrame)) == ESP_OK)
        {
            if (!(message.flags & TWAI_MSG_FLAG_RTR))
            {
                mRawMessageRxBuffer[i] = message;
                mRawMessageNumberRxBuffer++;
            }
        }
        else
        {
            result = false;
            break;
        }
    }

    return result;
}

// NOLINTBEGIN(misc-unused-parameters) //TODO: to be coded
float CAN_DRIVER::decodeFrame(twai_message_t message,
                              uint8_t startBit,
                              uint8_t signalSize,
                              CAN_ENUM::CAN_CODING_TYPE coding,
                              CAN_ENUM::DATATYPE datatype,
                              float factor,
                              float offset) const
{
    uint32_t Signal = 0;
    return static_cast<float>(Signal);
}
// NOLINTEND(misc-unused-parameters)

bool CAN_DRIVER::decodeRawRxBuffer(uint32_t frameId, CAN_DATATYPES::frameStruct& frame)
{
    if (!checkReady())
        return false;

    if (mRawMessageNumberRxBuffer == 0)
        return false;

#pragma unroll
    for (size_t i = 0; i < mRawMessageNumberRxBuffer; i++)
    {
        if (mRawMessageRxBuffer[i].identifier == frameId)
        {
            frame.reset();
            frame.frameId   = mRawMessageRxBuffer[i].identifier;
            frame.frameSize = mRawMessageRxBuffer[i].data_length_code;
            std::copy(mRawMessageRxBuffer[i].data,
                      mRawMessageRxBuffer[i].data + frame.frameSize,
                      frame.receivedFrameData);

            return true;
        }
    }

    return false;
}

void CAN_DRIVER::printMappingInfo(bool logSerialMonitor, bool logStreamLogger)
{
    if (logSerialMonitor)
    {
        mStreamObj->printlnUlog(true, false, "Mapped frames and signals:", false);
        mStreamObj->printUlog(true, false, "Frame number = ", false);
        mStreamObj->printUlog(true, false, mFrameVector.size(), false);
        mStreamObj->printUlog(true, false, "\t Signals number = ", false);
        mStreamObj->printlnUlog(true, false, mSignalVector.size(), false);

        for (size_t i = 0; i < mFrameVector.size(); i++)
        {
            mStreamObj->printUlog(true, false, mFrameVector[i].frameName, false);
            mStreamObj->printUlog(true, false, "\t Enabled frame = ", false);
            mStreamObj->printUlog(true, false, mFrameVector[i].enabled, false);
            mStreamObj->printUlog(true, false, "\t Signals number = ", false);
            mStreamObj->printUlog(true, false, mFrameVector[i].signalsNumber, false);
            mStreamObj->printUlog(true, false, "\t Frame size = ", false);
            mStreamObj->printlnUlog(true, false, mFrameVector[i].frameSize, false);
            if (mFrameVector[i].signalsIdx.size() != mFrameVector[i].signalsNumber)
            {
                mStreamObj->printUlog(true, false, "Error reading frame content \t", false);
                mStreamObj->printUlog(true, false, "signal idx size = ", false);
                mStreamObj->printUlog(true, false, mFrameVector[i].signalsIdx.size(), false);
                mStreamObj->printUlog(true, false, " =/= signals number = ", false);
                mStreamObj->printlnUlog(true, false, mFrameVector[i].signalsNumber, false);
            }
            else
            {
                for (size_t j = 0; j < mFrameVector[i].signalsIdx.size(); j++)
                {
                    if (mSignalVector[mFrameVector[i].signalsIdx[j]].enabledSignal)
                    {
                        mStreamObj->printUlog(true, false, "\t", false);
                        mStreamObj->printUlog(true, false, String(j), false);
                        if (j < 10)
                            mStreamObj->printUlog(true, false, " ", false);

                        mStreamObj->printUlog(true, false, " | ", false);
                        printSignalInfo(mSignalVector[mFrameVector[i].signalsIdx[j]], true, false);
                    }
                    else
                    {
                        mStreamObj->printUlog(true, false, "\t");
                        mStreamObj->printlnUlog(true, false, "Invalid signal");
                    }
                }
            }
            mStreamObj->printlnUlog(true, false, "", false);
        }
    }

    if (logStreamLogger)
    {
        mStreamObj->printlnUlog(false, true, "Mapped frame and signals:", false);
        mStreamObj->printUlog(false, true, "Frame number = ", false);
        mStreamObj->printUlog(false, true, mFrameVector.size(), false);
        mStreamObj->printUlog(false, true, "\t Signals number = ", false);
        mStreamObj->printlnUlog(false, true, mSignalVector.size(), false);

        for (size_t i = 0; i < mFrameVector.size(); i++)
        {
            mStreamObj->printUlog(false, true, mFrameVector[i].frameName, false);
            mStreamObj->printUlog(false, true, "\t Signals number = ", false);
            mStreamObj->printUlog(false, true, mFrameVector[i].signalsNumber, false);
            if (mFrameVector[i].signalsIdx.size() != mFrameVector[i].signalsNumber)
            {
                mStreamObj->printUlog(false, true, "Error reading frame content \t", false);
                mStreamObj->printUlog(false, true, "signal idx size = ", false);
                mStreamObj->printUlog(false, true, mFrameVector[i].signalsIdx.size(), false);
                mStreamObj->printUlog(false, true, " =/= signals number = ", false);
                mStreamObj->printlnUlog(false, true, mFrameVector[i].signalsNumber, false);
            }
            else
            {
                for (size_t j = 0; j < mFrameVector[i].signalsIdx.size(); j++)
                {
                    if (mSignalVector[mFrameVector[i].signalsIdx[j]].enabledSignal)
                    {
                        mStreamObj->printUlog(false, true, "\t", false);
                        mStreamObj->printUlog(false, true, String(j), false);
                        if (j < 10)
                            mStreamObj->printUlog(false, true, " ", false);

                        mStreamObj->printUlog(false, true, " | ", false);

                        printSignalInfo(mSignalVector[mFrameVector[i].signalsIdx[j]], false, true);
                    }
                    else
                    {
                        mStreamObj->printUlog(false, true, "\t", false);
                        mStreamObj->printlnUlog(false, true, "Invalid signal", false);
                    }
                }
            }
            mStreamObj->printlnUlog(false, true, "", false);
        }
    }
}

void CAN_DRIVER::printSignalInfo(CAN_DATATYPES::signalStruct signal,
                                 bool logSerialMonitor,
                                 bool logStreamLogger)
{
    // Padding string values
    String fullString                = "";
    uint16_t defaultNameStringLength = 15; // for standalone signals

    if (logSerialMonitor)
    {
        if (signal.enabledSignal)
        {
            fullString              = signal.signalName;
            defaultNameStringLength = mFrameVector[signal.frameIdx].maxNameStringLength;
#pragma unroll 5
            while (fullString.length() < defaultNameStringLength)
                fullString += " ";
            mStreamObj->printUlog(true, false, fullString, false);

            mStreamObj->printUlog(true, false, "\t Datatype = ", false);
            fullString                         = getStringForDatatype(signal.dataType);
            String curMaxLengthString4Datatype = "unsigned char";
            defaultNameStringLength            = curMaxLengthString4Datatype.length();
#pragma unroll 5
            while (fullString.length() < defaultNameStringLength)
                fullString += " ";
            mStreamObj->printUlog(true, false, fullString, false);

            fullString              = getStringForValue(signal);
            defaultNameStringLength = mFrameVector[signal.frameIdx].maxNameStringLength;
#pragma unroll 5
            while (fullString.length() < defaultNameStringLength)
                fullString += " ";
            mStreamObj->printUlog(true, false, fullString, false);

            mStreamObj->printUlog(true, false, "\t Orientation = ", false);
            mStreamObj->printUlog(true, false, "Scalar", false);

            mStreamObj->printUlog(true, false, "\t Dimension = ", false);
            mStreamObj->printUlog(true, false, "1", false);
        }
        else
        {
            mStreamObj->printUlog(true, false, "\t", false);
            mStreamObj->printUlog(true, false, "Disabled signal", false);
        }
        mStreamObj->printlnUlog(true, false, "", false);
    }

    if (logStreamLogger)
    {
        if (signal.enabledSignal)
        {
            fullString = signal.signalName;
#pragma unroll 5
            while (fullString.length() < defaultNameStringLength)
                fullString += " ";
            mStreamObj->printUlog(false, true, fullString, false);

            mStreamObj->printUlog(false, true, "\t Datatype = ", false);
            fullString                         = getStringForDatatype(signal.dataType);
            String curMaxLengthString4Datatype = "unsigned char";
            defaultNameStringLength            = curMaxLengthString4Datatype.length();
#pragma unroll 5
            while (fullString.length() < defaultNameStringLength)
                fullString += " ";
            mStreamObj->printUlog(false, true, fullString, false);

            mStreamObj->printUlog(false, true, "\t Orientation = ", false);
            mStreamObj->printUlog(false, true, "Scalar", false);

            mStreamObj->printUlog(false, true, "\t Dimension = ", false);
            mStreamObj->printUlog(false, true, "1", false);
        }
        else
        {
            mStreamObj->printUlog(false, true, "\t", false);
            mStreamObj->printUlog(false, true, "Invalid signal", false);
        }
        mStreamObj->printlnUlog(false, true, "", false);
    }
}

String CAN_DRIVER::getStringForDatatype(CAN_ENUM::DATATYPE datatype) const
{
    String fullString = "";
    switch (datatype)
    {
    case CAN_ENUM::DATATYPE::UINT8_T:
        fullString = "uint8_t";
        break;
    case CAN_ENUM::DATATYPE::UINT16_T:
        fullString = "uint16_t";
        break;
    case CAN_ENUM::DATATYPE::UINT32_T:
        fullString = "uint32_t";
        break;
    case CAN_ENUM::DATATYPE::UINT64_T:
        fullString = "uint64_t";
        break;
    case CAN_ENUM::DATATYPE::INT8_T:
        fullString = "int8_t";
        break;
    case CAN_ENUM::DATATYPE::INT16_T:
        fullString = "int16_t";
        break;
    case CAN_ENUM::DATATYPE::INT32_T:
        fullString = "int32_t";
        break;
    case CAN_ENUM::DATATYPE::INT64_T:
        fullString = "int64_t";
        break;
    case CAN_ENUM::DATATYPE::FLOAT:
        fullString = "float";
        break;
    case CAN_ENUM::DATATYPE::DOUBLE:
        fullString = "double";
        break;
    default:
        fullString = "unknown";
        break;
    }

    return fullString;
}

String CAN_DRIVER::getStringForValue(const CAN_DATATYPES::signalStruct& signal) const
{
    String fullString = "";

    switch (signal.dataType)
    {
    case CAN_ENUM::DATATYPE::UINT8_T:
        fullString = String(*(uint8_t*)signal.signalAdress);
        break;
    case CAN_ENUM::DATATYPE::UINT16_T:
        fullString = String(*(uint16_t*)signal.signalAdress);
        break;
    case CAN_ENUM::DATATYPE::UINT32_T:
        fullString = String(*(uint32_t*)signal.signalAdress);
        break;
    case CAN_ENUM::DATATYPE::UINT64_T:
        fullString = String(*(uint64_t*)signal.signalAdress);
        break;
    case CAN_ENUM::DATATYPE::INT8_T:
        fullString = String(*(int8_t*)signal.signalAdress);
        break;
    case CAN_ENUM::DATATYPE::INT16_T:
        fullString = String(*(int16_t*)signal.signalAdress);
        break;
    case CAN_ENUM::DATATYPE::INT32_T:
        fullString = String(*(int32_t*)signal.signalAdress);
        break;
    case CAN_ENUM::DATATYPE::INT64_T:
        fullString = String(*(int64_t*)signal.signalAdress);
        break;
    case CAN_ENUM::DATATYPE::FLOAT:
        fullString = static_cast<String>(*(float*)signal.signalAdress); // NOLINT
        break;
    case CAN_ENUM::DATATYPE::DOUBLE:
        fullString = static_cast<String>(*(double*)signal.signalAdress); // NOLINT
        break;
    default:
        fullString = "unknown";
        break;
    }

    return fullString;
}
