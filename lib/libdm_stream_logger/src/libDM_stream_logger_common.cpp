#include "libDM_stream_logger.hpp"
#include "pb_encode.h"
#include "pb_decode.h"
#include <numeric>

bool streamLogger::begin()
{

    if (!tryStart())
    {
        errorsHandler(true, "sd card mounting");
        return false;
    }

    printlnUlog(true, false, mClassName + ": sd card mounted successfully");

    if (!createNewRootDir())
    {
        mCardStatus = CARD_STATUS::CARD_KO;
        errorsHandler(true, "begin mDir creation");
        return false;
    }

    printlnUlog(true, false, mClassName + ": folder created successfully");

    if (!createLogFiles())
    {
        mCardStatus = CARD_STATUS::CARD_KO;
        errorsHandler(true, "begin file creation");
        return false;
    }
    mErrorsCounter = 0;
    printlnUlog(true, false, mClassName + ": files created successfully, ringbuffers initialized");

    String message2Log;
    switch (benchCard())
    {
    case 0:
        mCardStatus = CARD_STATUS::CARD_OK;
        message2Log = "Reading speed = " + String(mCardReadSpeed)
                      + " kbps, Writing speed = " + String(mCardWriteSpeed) + " kbps";
        printlnUlog(false, true, message2Log);
        break;

    case 1:
        mCardStatus = CARD_STATUS::CARD_OK_LOW_PERFORMANCE;
        errorsHandler(true, "sd card low performance");
        message2Log = "Reading speed = " + String(mCardReadSpeed)
                      + " kbps, Writing speed = " + String(mCardWriteSpeed) + " kbps";
        printlnUlog(false, true, message2Log);
        break;

    case 2:
        mCardStatus = CARD_STATUS::CARD_KO;
        errorsHandler(true, "bench");
        return false;
        break;

    default:
        mCardStatus = CARD_STATUS::CARD_KO;
        errorsHandler(true, "bench");
        return false;
    }

    // Handle timestamp
    printlnUlog(true, true, "Starting logger");

    computeLoopSizeBatch();
    computeRingBufferBatchSize();
    writeHeaderAndTypes();
    writeLog();
    flushLog();

    updateTimestampTimer();
    mLastFlushTime = mInternalTimeStampFloat;

    return true;
}

bool streamLogger::tryStart()
{
    bool startedSuccess = false;
#pragma unroll
    for (uint8_t i = 0; i < mNumberTryBegin; i++)
    {
        if (sd.cardBegin(mSdioConf, mSerialMonitor))
        {
            startedSuccess = true;
            break;
        }

        errorsHandler(true, "begin " + String(i));
        delay(10);
    }

    return startedSuccess;
}

bool streamLogger::flushLog()
{

    mLogBufferPacketCount = 0;

    if (!isLoggerReady())
        return false; // NOLINT(readability-simplify-boolean-expr)

    mLogFile.flush();

    return true;
}

void streamLogger::computeLoopSizeBatch()
{
    for (auto iter = mUint8Variables.begin(); iter != mUint8Variables.end(); iter++)
    {
        String message = "uint8";
        mHeaderTypes.push_back(message);
        mLoopSizeBatch += 1;
    }

    for (auto iter = mInt8Variables.begin(); iter != mInt8Variables.end(); iter++)
    {
        String message = "int8";
        mHeaderTypes.push_back(message);
        mLoopSizeBatch += 1;
    }

    for (auto iter = mUint16Variables.begin(); iter != mUint16Variables.end(); iter++)
    {
        String message = "uint16";
        mHeaderTypes.push_back(message);
        mLoopSizeBatch += 2;
    }

    for (auto iter = mInt16Variables.begin(); iter != mInt16Variables.end(); iter++)
    {
        String message = "int16";
        mHeaderTypes.push_back(message);
        mLoopSizeBatch += 2;
    }

    for (auto iter = mUint32Variables.begin(); iter != mUint32Variables.end(); iter++)
    {
        String message = "uint32";
        mHeaderTypes.push_back(message);
        mLoopSizeBatch += 4;
    }

    for (auto iter = mInt32Variables.begin(); iter != mInt32Variables.end(); iter++)
    {
        String message = "int32";
        mHeaderTypes.push_back(message);
        mLoopSizeBatch += 4;
    }

    for (auto iter = mUint64Variables.begin(); iter != mUint64Variables.end(); iter++)
    {
        String message = "uint64_t";
        mHeaderTypes.push_back(message);
        mLoopSizeBatch += 8;
    }

    for (auto iter = mFloatVariables.begin(); iter != mFloatVariables.end(); iter++)
    {
        String message = "float";
        mHeaderTypes.push_back(message);
        mLoopSizeBatch += 4;
    }

    for (auto iter = mDoubleVariables.begin(); iter != mDoubleVariables.end(); iter++)
    {
        String message = "double";
        mHeaderTypes.push_back(message);
        mLoopSizeBatch += 8;
    }
}

void streamLogger::convertLoggedVariables()
{

    uint16_2_binary sharedmemUint16;
    uint32_2_binary sharedMemUint32;
    int16_2_binary sharedmemInt16;
    int32_2_binary sharedMemInt32;
    float_2_binary sharedMemFloat;
    double_2_binary sharedMemDouble;
    uint64_2_binary sharedMemUint64;

    resetLoggerFrame();

    size_t idxReceptionBuffer                         = 0U;
    mLoggerFrame.binaryData.bytes[idxReceptionBuffer] = mStartLoopBatchHeader;
    idxReceptionBuffer++;

    for (auto iter = mUint8Variables.begin(); iter != mUint8Variables.end(); iter++)
    {
        mLoggerFrame.binaryData.bytes[idxReceptionBuffer] =
                *(*iter); // It is an vector of pointer so *iter is the pointer to the value
        idxReceptionBuffer++;
    }

    for (auto iter = mInt8Variables.begin(); iter != mInt8Variables.end(); iter++)
    {
        mLoggerFrame.binaryData.bytes[idxReceptionBuffer] =
                *(*iter); // It is an vector of pointer so *iter is the pointer to the value
        idxReceptionBuffer++;
    }

    for (auto iter = mUint16Variables.begin(); iter != mUint16Variables.end(); iter++)
    {
        sharedmemUint16.uint16Data =
                *(*iter); // It is an vector of pointer so *iter is the pointer to the value
        std::copy(sharedmemUint16.byteTable,
                  sharedmemUint16.byteTable + 2,
                  mLoggerFrame.binaryData.bytes + idxReceptionBuffer);
        idxReceptionBuffer += 2;
    }

    for (auto iter = mInt16Variables.begin(); iter != mInt16Variables.end(); iter++)
    {
        sharedmemInt16.int16Data =
                *(*iter); // It is an vector of pointer so *iter is the pointer to the value
        std::copy(sharedmemInt16.byteTable,
                  sharedmemInt16.byteTable + 2,
                  mLoggerFrame.binaryData.bytes + idxReceptionBuffer);
        idxReceptionBuffer += 2;
    }

    for (auto iter = mUint32Variables.begin(); iter != mUint32Variables.end(); iter++)
    {
        sharedMemUint32.uint32Data =
                *(*iter); // It is an vector of pointer so *iter is the pointer to the value
        std::copy(sharedMemUint32.byteTable,
                  sharedMemUint32.byteTable + 4,
                  mLoggerFrame.binaryData.bytes + idxReceptionBuffer);
        idxReceptionBuffer += 4;
    }

    for (auto iter = mInt32Variables.begin(); iter != mInt32Variables.end(); iter++)
    {
        sharedMemInt32.int32Data =
                *(*iter); // It is an vector of pointer so *iter is the pointer to the value
        std::copy(sharedMemInt32.byteTable,
                  sharedMemInt32.byteTable + 4,
                  mLoggerFrame.binaryData.bytes + idxReceptionBuffer);
        idxReceptionBuffer += 4;
    }

    for (auto iter = mUint64Variables.begin(); iter != mUint64Variables.end(); iter++)
    {
        sharedMemUint64.uint64Data =
                *(*iter); // It is an vector of pointer so *iter is the pointer to the value
        std::copy(sharedMemUint64.byteTable,
                  sharedMemUint64.byteTable + 8,
                  mLoggerFrame.binaryData.bytes + idxReceptionBuffer);
        idxReceptionBuffer += 8;
    }

    for (auto iter = mFloatVariables.begin(); iter != mFloatVariables.end(); iter++)
    {
        sharedMemFloat.floatData =
                *(*iter); // It is an vector of pointer so *iter is the pointer to the value
        std::copy(sharedMemFloat.byteTable,
                  sharedMemFloat.byteTable + 4,
                  mLoggerFrame.binaryData.bytes + idxReceptionBuffer);
        idxReceptionBuffer += 4;
    }

    for (auto iter = mDoubleVariables.begin(); iter != mDoubleVariables.end(); iter++)
    {
        sharedMemDouble.doubleData =
                *(*iter); // It is an vector of pointer so *iter is the pointer to the value
        std::copy(sharedMemDouble.byteTable,
                  sharedMemDouble.byteTable + 8,
                  mLoggerFrame.binaryData.bytes + idxReceptionBuffer);
        idxReceptionBuffer += 8;
    }

    mLoggerFrame.binaryData.size = mLoopSizeBatch + 1; // 1 headers
    mLoggerFrame.has_binaryData  = true;
}

bool streamLogger::addVariable(const uint8_t* VariableAdress, String Name)
{
    mUint8Variables.push_back(VariableAdress);
    mHeaderNames8.push_back(Name);
    mSignalsNumber++;

    return true;
}

bool streamLogger::addVariable(const uint16_t* VariableAdress, String Name)
{
    mUint16Variables.push_back(VariableAdress);
    mHeaderNames16.push_back(Name);
    mSignalsNumber++;

    return true;
}

bool streamLogger::addVariable(const uint32_t* VariableAdress, String Name)
{

    mUint32Variables.push_back(VariableAdress);
    mHeaderNames32.push_back(Name);
    mSignalsNumber++;

    return true;
}

bool streamLogger::addVariable(const int8_t* VariableAdress, String Name)
{

    mInt8Variables.push_back(VariableAdress);
    mHeaderNamesi8.push_back(Name);
    mSignalsNumber++;

    return true;
}

bool streamLogger::addVariable(const int16_t* VariableAdress, String Name)
{

    mInt16Variables.push_back(VariableAdress);
    mHeaderNamesi16.push_back(Name);
    mSignalsNumber++;

    return true;
}

bool streamLogger::addVariable(const int32_t* VariableAdress, String Name)
{

    mInt32Variables.push_back(VariableAdress);
    mHeaderNamesi32.push_back(Name);
    mSignalsNumber++;

    return true;
}

bool streamLogger::addVariable(const float* VariableAdress, String Name)
{
    mFloatVariables.push_back(VariableAdress);
    mHeaderNamesf.push_back(Name);
    mSignalsNumber++;

    return true;
}

bool streamLogger::addVariable(const double* VariableAdress, String Name)
{
    mDoubleVariables.push_back(VariableAdress);
    mHeaderNamesd.push_back(Name);
    mSignalsNumber++;

    return true;
}

bool streamLogger::addVariable(const uint64_t* VariableAdress, String Name)
{
    mUint64Variables.push_back(VariableAdress);
    mHeaderNamesd.push_back(Name);
    mSignalsNumber++;

    return true;
}

bool streamLogger::writeHeaderAndTypes()
{
    String messageHeader = "";

    // Process header
    std::vector<std::vector<String>> headers = {mHeaderNames8,
                                                mHeaderNamesi8,
                                                mHeaderNames16,
                                                mHeaderNamesi16,
                                                mHeaderNames32,
                                                mHeaderNamesi32,
                                                mHeaderNamesf,
                                                mHeaderNamesd};

    for (auto& header : headers)
    {
        messageHeader = std::accumulate(header.begin(),
                                        header.end(),
                                        messageHeader,
                                        [](const String& a, const String& b)
                                        { return a == "" ? b : a + "," + b; });
    }

    String messageTypes = "";

    // Process types
    messageTypes = std::accumulate(mHeaderTypes.begin(),
                                   mHeaderTypes.end(),
                                   messageTypes,
                                   [](const String& a, const String& b)
                                   { return a == "" ? b : a + "," + b; });

    resetLoggerFrame();

    // Set data to the first frame
    // Header
    mLoggerFrame.has_binaryHeader  = true;
    mLoggerFrame.binaryHeader.size = messageHeader.length();
    memcpy(mLoggerFrame.binaryHeader.bytes, messageHeader.c_str(), messageHeader.length());

    // Types
    mLoggerFrame.has_binaryTypes  = true;
    mLoggerFrame.binaryTypes.size = messageTypes.length();
    memcpy(mLoggerFrame.binaryTypes.bytes, messageTypes.c_str(), messageTypes.length());

    updateRingBuffer();

    return true;
}

bool streamLogger::writeProtobufHeader(const std::vector<String>& names)
{
    if (!isLoggerReady())
        return false;

    resetLoggerFrame();

    // Process header
    String messageHeader = std::accumulate(names.begin(),
                                           names.end(),
                                           String(""),
                                           [](const String& a, const String& b)
                                           { return a == "" ? b : a + "," + b; });

    // Set data to the first frame
    // Header
    mLoggerFrame.has_protobufHeader  = true;
    mLoggerFrame.protobufHeader.size = messageHeader.length();
    memcpy(mLoggerFrame.protobufHeader.bytes, messageHeader.c_str(), messageHeader.length());

    updateRingBuffer();

    return true;
}

bool streamLogger::writeMavlinkHeaderAndId(const std::vector<String>& names,
                                           const std::vector<uint32_t>& ids)
{
    if (!isLoggerReady())
        return false;

    resetLoggerFrame();

    // Process header
    String messageHeader = std::accumulate(names.begin(),
                                           names.end(),
                                           String(""),
                                           [](const String& a, const String& b)
                                           { return a == "" ? b : a + "," + b; });

    // Process ids
    String messageIds = std::accumulate(ids.begin(),
                                        ids.end(),
                                        String(""),
                                        [](const String& a, const uint32_t& b)
                                        { return a == "" ? String(b) : a + "," + String(b); });

    // Header
    mLoggerFrame.has_mavlinkHeader  = true;
    mLoggerFrame.mavlinkHeader.size = messageHeader.length();
    memcpy(mLoggerFrame.mavlinkHeader.bytes, messageHeader.c_str(), messageHeader.length());

    // Ids
    mLoggerFrame.has_mavlinkID  = true;
    mLoggerFrame.mavlinkID.size = messageIds.length();
    memcpy(mLoggerFrame.mavlinkID.bytes, messageIds.c_str(), messageIds.length());

    updateRingBuffer();

    return true;
}

bool streamLogger::printHeaderMonitor(bool addInUlog)
{
    uint16_t curmSignalsNumber = 0;

    for (auto iter = mHeaderNames8.begin(); iter != mHeaderNames8.end(); iter++)
    {
        ++curmSignalsNumber;
        if (curmSignalsNumber < mSignalsNumber)
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
            return true;
        }
    }

    for (auto iter = mHeaderNamesi8.begin(); iter != mHeaderNamesi8.end(); iter++)
    {
        ++curmSignalsNumber;
        if (curmSignalsNumber < mSignalsNumber)
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
            return true;
        }
    }

    for (auto iter = mHeaderNames16.begin(); iter != mHeaderNames16.end(); iter++)
    {
        ++curmSignalsNumber;
        if (curmSignalsNumber < mSignalsNumber)
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
            return true;
        }
    }

    for (auto iter = mHeaderNamesi16.begin(); iter != mHeaderNamesi16.end(); iter++)
    {
        ++curmSignalsNumber;
        if (curmSignalsNumber < mSignalsNumber)
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
            return true;
        }
    }

    for (auto iter = mHeaderNames32.begin(); iter != mHeaderNames32.end(); iter++)
    {
        ++curmSignalsNumber;
        if (curmSignalsNumber < mSignalsNumber)
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
            return true;
        }
    }

    for (auto iter = mHeaderNamesi32.begin(); iter != mHeaderNamesi32.end(); iter++)
    {
        ++curmSignalsNumber;
        if (curmSignalsNumber < mSignalsNumber)
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
            return true;
        }
    }

    for (auto iter = mHeaderNamesf.begin(); iter != mHeaderNamesf.end(); iter++)
    {
        ++curmSignalsNumber;
        if (curmSignalsNumber < mSignalsNumber)
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
            return true;
        }
    }

    for (auto iter = mHeaderNamesd.begin(); iter != mHeaderNamesd.end(); iter++)
    {
        ++curmSignalsNumber;
        if (curmSignalsNumber < mSignalsNumber)
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
            return true;
        }
    }

    for (auto iter = mHeaderNames64.begin(); iter != mHeaderNames64.end(); iter++)
    {
        ++curmSignalsNumber;
        if (curmSignalsNumber < mSignalsNumber)
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
            return true;
        }
    }

    return true;
}

bool streamLogger::printTypesMonitor(bool addInUlog)
{

    for (auto iter = mHeaderTypes.begin(); iter != mHeaderTypes.end(); iter++)
    {
        if (iter != std::prev(mHeaderTypes.end()))
        {
            String message = *iter + ",";
            printUlog(true, addInUlog, message);
        }
        else
        {
            printlnUlog(true, addInUlog, *iter);
        }
    }

    return true;
}

bool streamLogger::printlnLog(String message)
{

    if (mCardStatus == CARD_STATUS::NO_CARD_INSERTED || mCardStatus == CARD_STATUS::CARD_KO)
        return false; // NOLINT(readability-simplify-boolean-expr)

    mLogFile.println(message.c_str());
    mLogBufferPacketCount++;

    return true;
}

bool streamLogger::printLog(String message)
{

    if (mCardStatus == CARD_STATUS::NO_CARD_INSERTED || mCardStatus == CARD_STATUS::CARD_KO)
        return false; // NOLINT(readability-simplify-boolean-expr)

    mLogFile.print(message.c_str());
    mLogBufferPacketCount++;

    return true;
}

void streamLogger::errorsHandler(bool sdLogError, String typeError)
{

    mErrorsCounter = 0;

    if (mNbErrorsThreshold == 254)
        return;

    String message = mClassName + ": " + typeError + " default";

    if (sdLogError)
        printlnUlog(true, true, message);
    else
        printlnUlog(true, false, message);
}

void streamLogger::updateTimestampTimer()
{
    mInternalTimeStampFloat = mTimerObj->returnSystemCounterSecondsFloat();
}

void streamLogger::printCrashReport(CrashReportClass CrashReport)
{
    if (!isLoggerReady())
        return;

    PrintString printableAsString = PrintString();
    CrashReport.printTo(printableAsString);

    printlnUlog(true, true, printableAsString.getString());
    ringBuffer->print(CrashReport);
    updateRingBuffer();
    flushLog();
}

void streamLogger::computeRingBufferBatchSize()
{
    size_t curSize = mLoopSizeBatch + 1; // 1 headers
    if (curSize > LOGGER_FRAME_MAX_SIZE)
    {
        printlnUlog(true, false, mClassName + ": Loop size too big for binary log");
        mCardStatus = CARD_STATUS::CARD_KO;
        return;
    }

    uint8_t factor = 1U;
#pragma unroll
    while (mRingBufferBatchSize * factor < curSize)
        factor++;

    mRingBufferBatchSize *= factor;
}
