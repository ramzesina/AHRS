#include "libDM_stream_logger.hpp"
#include "libDM_generic_operations.hpp"
#include "pb_encode.h"
#include "pb_decode.h"
#include "libDM_protobuf_structs.hpp"

#if defined(CORE_TEENSY)
std::vector<std::string> streamLogger::getRootDirs()
{
    updateTimestampTimer();
    ExFile tempDir;
    if (!tempDir.open("/"))
        errorsHandler(true, "getRootDirs");

    std::vector<std::string> dirVector;
    uint16_t curFolderIdx = 0;

    while (mFileDir.openNext(&tempDir, O_RDONLY))
    {

        if (mFileDir.isDir())
        {
            char curName[10]; // NOLINT
            uint16_t number = 0;

            mFileDir.getName(curName, 10);
            OP::TEXT::String curRootName;
            for (size_t i = 0; i < mDirFormat.length(); i++)
                curRootName.append(curName[i]);

            if (curRootName == mDirFormat)
            {
                std::string appendName(curName);
                dirVector.push_back(appendName);

                uint8_t idxStart = 0;
                while (curName[idxStart + 3] != (char)0) // NOLINT
                    idxStart++;

                number = 0;
#    pragma unroll
                for (uint8_t i = 0; i < idxStart; i++)
                    number += (curName[i + 3] - '0') * pow10f(idxStart - i - 1);
            }
            if (number > curFolderIdx)
                curFolderIdx = number;
        }
        mFileDir.rewind();
        mFileDir.close();
    }
    return dirVector;
}

bool streamLogger::createNewRootDir()
{
    updateTimestampTimer();
    if (!mDir.open("/"))
    {
        errorsHandler(true, "createNewRootDir");
        return false;
    }

    uint16_t curFolderIdx = 0;

    while (mFileDir.openNext(&mDir, O_RDONLY))
    {
        if (mFileDir.isDir())
        {
            char curName[10];
            uint16_t number = 0;

            mFileDir.getName(curName, 10);
            OP::TEXT::String curRootName;
            for (size_t i = 0; i < mDirFormat.length(); i++)
                curRootName.append(curName[i]);

            if (curRootName == mDirFormat)
            {
                uint8_t idxStart = 0;
                while (curName[idxStart + 3] != (char)0) // NOLINT
                    idxStart++;

                number = 0;
                for (uint8_t i = 0; i < idxStart; i++)
                    number += (curName[i + 3] - '0') * pow10f(idxStart - i - 1);
            }
            if (number > curFolderIdx)
                curFolderIdx = number;
        }
        mFileDir.rewind();
        mFileDir.close();
    }

    mDirName = mDirFormat + String(curFolderIdx + 1);

    while (!sd.mkdir(mDirName))
    {
        delay(100);
        mErrorsCounter++;
        if (mErrorsCounter >= mNbErrorsThreshold)
        {
            errorsHandler(true, "createNewRootDir");
            return false;
        }
    }
    mErrorsCounter = 0;

    return true;
}

bool streamLogger::createLogFiles()
{
    updateTimestampTimer();

    String logPath = mDirName + "/" + "log.binDM";

    mErrorsCounter = 0;

    // Create log file
    while (!mLogFile.open(logPath.c_str(), O_WRONLY | O_CREAT))
    {
        delay(100);
        mErrorsCounter++;
        if (mErrorsCounter >= mNbErrorsThreshold)
        {
            errorsHandler(true, "createLogFiles");
            return false;
        }
    }
    mErrorsCounter = 0;

    // Preallocate file for speed
    if (!mLogFile.preAllocate(LOG_FILE_SIZE))
    {
        errorsHandler(true, "preAllocate file");
        mLogFile.close();
        return false;
    }

    // Initialize ring buffer
    ringBuffer->begin(&mLogFile);
    return true;
}

bool streamLogger::createNewFile(String filePath, ExFile& newFile)
{
    if (mCardStatus == CARD_STATUS::NO_CARD_INSERTED || mCardStatus == CARD_STATUS::CARD_KO)
        return false;

    if (filePath[0] != '/')
        filePath = "/" + filePath;

    uint32_t errorsCounter = 0U;

    // create ulog file
    while (!newFile.open(filePath.c_str(), O_WRONLY | O_CREAT))
    {
        delay(100);
        errorsCounter++;
        if (errorsCounter >= mNbErrorsThreshold)
        {
            errorsHandler(true, "createULogFiles");
            return false;
        }
    }

    // TODO: potentially add preallocation

    printlnUlog(true, true, mClassName + ": successfully created file " + filePath);

    return true;
}

bool streamLogger::close()
{
    updateTimestampTimer();

    if (!isLoggerReady())
        return false;

    printlnUlog(false, true, "exiting logger");

    // Last writing to log file
    writeLog();
    ringBuffer->sync();
    mLogFile.truncate();
    mLogFile.rewind();

    sd.card()->writeStop();

    if (!mLogFile.close())
    {
        errorsHandler(true, "mLogFile close");
        return false;
    }

    return true;
}

bool streamLogger::closeFile(ExFile& file)
{
    if (mCardStatus == CARD_STATUS::NO_CARD_INSERTED || mCardStatus == CARD_STATUS::CARD_KO)
        return false;

    file.close();

    return true;
}

uint8_t streamLogger::benchCard()
{
    updateTimestampTimer();

    uint8_t nbCopies = 100;
    uint8_t buffer[mBufRamDimLog];
    uint32_t FILE_SIZE = nbCopies * mBufRamDimLog;
    float speed;

    if (!mBenchFile.open("bench.binDM", O_RDWR | O_CREAT))
    {
        errorsHandler(true, "bench create file");
        return 2;
    }

    if (!mBenchFile.truncate(0))
    {
        errorsHandler(true, "bench truncate file");
        return 2;
    }

    // Write speed
    long t_now = micros();
    for (uint8_t i = 0; i < nbCopies; i++)
    {
        if (mBenchFile.write(buffer, mBufRamDimLog) != mBufRamDimLog)
        {
            errorsHandler(true, "bench writing file");
            return 2;
        }
    }
    uint8_t buffer2[4];
    mBenchFile.write(buffer2, 4);
    speed = static_cast<float>(FILE_SIZE) / static_cast<float>((micros() - t_now)) * 1000.F;
    mCardWriteSpeed = (uint32_t)speed;

    mBenchFile.rewind();

    t_now = micros();
    for (uint8_t i = 0; i < nbCopies; i++)
    {
        if (mBenchFile.read(buffer, mBufRamDimLog) != (int)mBufRamDimLog)
        {
            errorsHandler(true, "bench reading file");
            return 2;
        }
    }
    speed = static_cast<float>(FILE_SIZE) / static_cast<float>((micros() - t_now)) * 1000.F;
    mCardReadSpeed = (uint32_t)speed;

    mBenchFile.close();
    if (mCardWriteSpeed <= mKbpsLimitWriteSpeed || mCardReadSpeed <= mKbpsLimitReadSpeed)
        return 1;
    else
        return 0;
}

void streamLogger::updateLogBuffer()
{
    if (!isLoggerReady())
        return;

    updateTimestampTimer();
    convertLoggedVariables();
    updateRingBuffer();

    // Write log if necessary, using special task or not
    triggerWrite();
}

void streamLogger::updateProtobufUlogBuffer(const char* charBuffer, size_t size)
{
    if (!isLoggerReady())
        return;

    resetLoggerFrame();

    mLoggerFrame.has_ulogData  = true;
    mLoggerFrame.ulogData.size = size;
    memcpy(mLoggerFrame.ulogData.bytes, charBuffer, size);
    updateRingBuffer();

    // Write log if necessary, using special task or not
    triggerWrite();
}

void streamLogger::updateProtobufProtocolLogBuffer(const uint8_t* buffer,
                                                   size_t size,
                                                   EVENT_TYPE type,
                                                   RX_TX direction)
{
    if (!isLoggerReady())
        return;

    resetLoggerFrame();

    // Update protocol content based on the type
    switch (type)
    {
    case EVENT_TYPE::MAVLINK:
    {
        if (size > LOGGER_FRAME_MAX_MAVLINK_SIZE)
        {
            printlnUlog(true, true, mClassName + ": mavlink packet too big");
            return;
        }
        mLoggerFrame.has_mavlinkData      = true;
        mLoggerFrame.mavlinkData.size     = size + 1;
        mLoggerFrame.mavlinkData.bytes[0] = static_cast<pb_byte_t>(direction);
        memcpy(mLoggerFrame.mavlinkData.bytes + 1, buffer, size);
        break;
    }
    case EVENT_TYPE::PROTOBUF:
    {
        if (size > LOGGER_FRAME_MAX_PROTOBUF_SIZE)
        {
            printlnUlog(true, true, mClassName + ": protobuf packet too big");
            return;
        }
        mLoggerFrame.has_protobufData      = true;
        mLoggerFrame.protobufData.size     = size + 1;
        mLoggerFrame.protobufData.bytes[0] = static_cast<pb_byte_t>(direction);
        memcpy(mLoggerFrame.protobufData.bytes + 1, buffer, size);
        break;
    }
    }

    updateRingBuffer();

    // Write log if necessary, using special task or not
    triggerWrite();
}

void streamLogger::updateRingBuffer()
{
    if (!isLoggerReady())
        return;

    if (mErrorsCounter >= mNbErrorsThreshold)
    {
        mCardStatus = CARD_STATUS::CARD_KO;
        return;
    }

    bool status = false;

    // Serializing the protobuf frame
    // Add the 2 starting bytes at the beginning of the buffer
    mLoggerFrameBufferTxSize = 0U;
    mLoggerFrameBufferTx[0]  = PROTOBUF_LOGGER_FRAME_START_BYTE_1;
    mLoggerFrameBufferTx[1]  = PROTOBUF_LOGGER_FRAME_START_BYTE_2;
    // ringBuffer->write(mLoggerFrameBufferTx, PROTOBUF_FRAME_ADDED_BYTES);

    // Create a stream that will write to our buffer offsetted by 2 bytes (the two start bytes)
    pb_ostream_t writingStream =
            pb_ostream_from_buffer(mLoggerFrameBufferTx + 2, LOGGER_FRAME_MAX_SIZE);

    // Use pb_encode_delimited (message size as first varint) to write a frame from the stream to
    // allow for more robustness.
    mLoggerFrame.start_byte = PROTOBUF_START_BYTE;
    mLoggerFrame.end_byte   = PROTOBUF_END_BYTE;
    mLoggerFrame.timestamp  = mTimerObj->returnSystemTimestampUs64();
    status = pb_encode_ex(&writingStream, LoggerFrame_fields, &mLoggerFrame, PB_ENCODE_DELIMITED);

    // Check for errors
    if (!status)
    {
        printlnUlog(true,
                    false,
                    mClassName + ": Unable to encode current LoggerFrame -> "
                            + PB_GET_ERROR(&writingStream));
        return;
    }

    // Update temp buffer if it is unused
    size_t newSize = writingStream.bytes_written + 2 + 2; // 2 start bytes + 2 end bytes

    mLoggerFrameBufferTx[writingStream.bytes_written + 2] = mEndLoopBatchTermination1;
    mLoggerFrameBufferTx[writingStream.bytes_written + 3] = mEndLoopBatchTermination2;
    mLoggerFrameBufferTxSize                              = newSize;
}

bool streamLogger::writeLog()
{
    if (!isLoggerReady())
        return false;

    updateTimestampTimer();

    // Just write the Tx buffer to the ring buffer
    if (mLoggerFrameBufferTxSize == 0)
        return false;

    size_t bytesWritten = 0U;

    bytesWritten           = ringBuffer->write(mLoggerFrameBufferTx, mLoggerFrameBufferTxSize);
    bool resWritingRingBuf = bytesWritten == mLoggerFrameBufferTxSize;
    if (!resWritingRingBuf)
    {
        printlnUlog(true,
                    false,
                    mClassName + ": Unable to write fast temp buffer to ring buf, size = "
                            + String(bytesWritten) + " bytes, with expected size = "
                            + String(mLoggerFrameBufferTxSize) + " bytes");
        return false;
    }

    size_t nbByteUsed = ringBuffer->bytesUsed();
    if ((nbByteUsed + mLogFile.curPosition()) > (LOG_FILE_SIZE - mRingBufferBatchSize))
    {
        printlnUlog(false, true, "Log file is full, closing it"); // Maybe create a new one
        mLogFile.close();
        mIsLogFileFull = true;
        return false;
    }

    if (nbByteUsed >= mRingBufferBatchSize && !mLogFile.isBusy())
    {
        mIsLogWritingSector = true;
        // Not busy only allows one sector before possible busy wait.
        // Write one sector from RingBuf to file.
        if (mRingBufferBatchSize != ringBuffer->writeOut(mRingBufferBatchSize))
        {
            printlnUlog(false, true, "Failed to write sector (512 bytes)");
            mErrorsCounter++;
            mIsLogWritingSector = false;
            return false;
        }
        mIsLogWritingSector = false;
    }

    mErrorsCounter = 0;
    if ((mInternalTimeStampFloat - mLastFlushTime) >= mFlushTimerThreshold)
    {
        mLastFlushTime = mInternalTimeStampFloat;
        flushLog();
        printlnUlog(true, true, "Flushing log file");
    }

    return true;
}

bool streamLogger::formatCard()
{
    updateTimestampTimer();

    if (!sd.format(&Serial))
    {
        if (!formatCardBackup())
        {
            errorsHandler(true, "formatCard");
            return false;
        }
    }

    if (!sd.volumeBegin())
    {
        errorsHandler(true, "formatCard volumeBegin");
        return false;
    }

    mSerialMonitor->println("Card formatted");

    return true;
}

bool streamLogger::formatCardBackup()
{
    updateTimestampTimer();

    SdCard* m_card = nullptr;
    SdCardFactory cardFactory;
    uint8_t sectorBuffer[512];

    ExFatFormatter exFatFormatter;

    m_card = cardFactory.newCard(SdioConfig(FIFO_SDIO));
    if (!m_card || m_card->errorCode())
    {
        errorsHandler(true, "formatCardBackup");
        return false;
    }

    bool rtn = exFatFormatter.format(m_card, sectorBuffer, &Serial);

    if (!rtn)
    {
        errorsHandler(true, "Backup Format failed");
        return false;
    }

    return true;
}

bool streamLogger::jsonTools::readJsonFile(String filePath)
{
    if (mStreamLoggerObj->mCardStatus == CARD_STATUS::NO_CARD_INSERTED
        || mStreamLoggerObj->mCardStatus == CARD_STATUS::CARD_KO)
        return false;

    if (filePath[0] != '/')
        filePath = "/" + filePath;

    if (!mStreamLoggerObj->sd.exists(filePath))
    {
        mStreamLoggerObj->errorsHandler(true, "can not find json file " + filePath + " to read");
        return false;
    }

    ExFile file = mStreamLoggerObj->sd.open(filePath, FILE_READ);
    mStreamLoggerObj->printlnUlog(true,
                                  true,
                                  mClassName + ": Successfully opened json file " + filePath
                                          + " size = " + OP::TEXT::String(file.size()) + " bytes");
    if (!file)
    {
        mStreamLoggerObj->errorsHandler(true, "open file for read");
        return false;
    }

    size_t pos  = 0U;
    String line = "";
    while (file.available())
    {
        line = file.readStringUntil('\n');
        if (pos + line.length() < JSON_FILE_SIZE / 2)
        {
            strncpy(mJsonCharBuffer + pos, line.c_str(), line.length());
            pos += line.length();
        }
        else
        {
            mStreamLoggerObj->errorsHandler(
                    true, "readJsonFile() failed: not enough space allowed for json parsing");
            return false;
        }
    }

    // Add null terminator
    mJsonCharBuffer[pos] = (char)0;

    return deserializeJsonCharBuffer();
}

bool streamLogger::jsonTools::writeJsonDoc(streamLogger::INTERFACE interface,
                                           String filePath,
                                           DynamicJsonDocument& jsonDoc)
{
    if (interface == streamLogger::INTERFACE::INTERFACE_SD)
    {
        if (mStreamLoggerObj->mCardStatus == CARD_STATUS::NO_CARD_INSERTED
            || mStreamLoggerObj->mCardStatus == CARD_STATUS::CARD_KO)
            return false;

        if (filePath[0] != '/')
            filePath = "/" + filePath;

        ExFile file = mStreamLoggerObj->sd.open(filePath, FILE_WRITE);
        if (!file)
        {
            mStreamLoggerObj->errorsHandler(true, "open file for write");
            return false;
        }

        String jsonString = "";
        serializeJsonPretty(jsonDoc, jsonString);
        file.print(jsonString);
        mStreamLoggerObj->printlnUlog(
                true, true, mClassName + ": Successfully wrote json file to" + filePath);
        file.close();
        return true;
    }
    else
    {
        mStreamLoggerObj->errorsHandler(
                true, "writeJsonDoc() failed: unknown or not implemented interface");
        return false;
    }
}

#endif
