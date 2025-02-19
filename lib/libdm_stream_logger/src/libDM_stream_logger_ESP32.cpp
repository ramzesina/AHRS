#include "libDM_stream_logger.hpp"
#include "libDM_generic_operations.hpp"
#include "pb_encode.h"
#include "pb_decode.h"
#include "libDM_protobuf_structs.hpp"

#if defined(ESP_PLATFORM)
std::vector<std::string> streamLogger::getRootDirs()
{
    updateTimestampTimer();
    File root = sd.open("/");
    if (!root)
        errorsHandler(true, "getRootDirs");

    std::vector<std::string> dirVector;
    uint16_t curFolderIdx = 0;

    mFileDir = root.openNextFile();
    while (mFileDir)
    {
        std::array<char, 10> curName = {0};

        if (mFileDir.isDirectory())
        {
            uint16_t number = 0U;
            strncpy(curName.data(), mFileDir.name(), 10);
            OP::TEXT::String curRootName;
            for (size_t i = 0; i < mDirFormat.length(); i++)
                curRootName.append(curName[i]);

            if (curRootName == mDirFormat)
            {
                std::string appendName(curName.data());
                dirVector.push_back(appendName);

                uint8_t idxStart = 0;
                while (curName[idxStart + 3] != (char)0)
                    idxStart++;

                number = 0;
                for (uint8_t i = 0; i < idxStart; i++)
                    number += (curName[i + 3] - '0') * pow10f(idxStart - i - 1);
            }
            if (number > curFolderIdx)
                curFolderIdx = number;
        }
        mFileDir.rewindDirectory();
        mFileDir.close();
        mFileDir = root.openNextFile();
    }
    return dirVector;
}

bool streamLogger::createNewRootDir()
{
    updateTimestampTimer();
    File root = sd.open("/");
    if (!root)
    {
        errorsHandler(true, "createNewRootDir at opening");
        return false;
    }

    uint16_t curFolderIdx = 0;

    mFileDir = root.openNextFile();
    while (mFileDir)
    {
        std::array<char, 10> curName = {0};

        if (mFileDir.isDirectory())
        {
            uint16_t number = 0U;

            strncpy(curName.data(), mFileDir.name(), 10);
            OP::TEXT::String curRootName;
            for (size_t i = 0; i < mDirFormat.length(); i++)
                curRootName.append(curName[i]);

            if (curRootName == mDirFormat)
            {
                uint8_t idxStart = 0;
                while (curName[idxStart + 3] != static_cast<char>(0))
                    idxStart++;

                number = 0;
                for (uint8_t i = 0; i < idxStart; i++)
                    number += (curName[i + 3] - '0') * pow10f(idxStart - i - 1);
            }
            if (number > curFolderIdx)
                curFolderIdx = number;
        }
        mFileDir.rewindDirectory();
        mFileDir.close();
        mFileDir = root.openNextFile();
    }

    mDirName = "/" + mDirFormat + String(curFolderIdx + 1);

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

    printlnUlog(true, false, mClassName + ": created folder " + mDirName);

    return true;
}

bool streamLogger::createLogFiles()
{
    updateTimestampTimer();

    String logPath = mDirName + "/" + "log.binDM";

    // Create log file
    mLogFile = sd.open(logPath, FILE_WRITE);
    while (!mLogFile)
    {
        delay(100);
        mErrorsCounter++;
        if (mErrorsCounter >= mNbErrorsThreshold)
        {
            errorsHandler(true, "createLogFiles");
            return false;
        }
        mLogFile = sd.open(logPath, FILE_WRITE);
    }
    mErrorsCounter = 0;

    // Preallocate file for speed
    // TODO: preallocate with sdmmc_write_sectors
    // if (!mLogFile.preAllocate(LOG_FILE_SIZE))
    // {
    //     errorsHandler(true, "preAllocate file");
    //     mLogFile.close();
    //     return false;
    // }

    // Initialize ring buffer
    ringBuffer->begin(&mLogFile);
    return true;
}

bool streamLogger::createNewFile(String filePath, File& newFile)
{
    if (mCardStatus == CARD_STATUS::NO_CARD_INSERTED || mCardStatus == CARD_STATUS::CARD_KO)
        return false;

    if (filePath[0] != '/')
        filePath = "/" + filePath;

    uint32_t errorsCounter = 0U;

    // create file
    newFile = sd.open(filePath, FILE_WRITE);
    while (!newFile)
    {
        delay(100);
        errorsCounter++;
        if (errorsCounter >= mNbErrorsThreshold)
        {
            errorsHandler(true, "create new file");
            return false;
        }
        newFile = sd.open(filePath, FILE_WRITE);
    }

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

    mLogFile.close();

    return true;
}

bool streamLogger::closeFile(File& file)
{
    if (mCardStatus == CARD_STATUS::NO_CARD_INSERTED || mCardStatus == CARD_STATUS::CARD_KO)
        return false;

    file.close();

    return true;
}

uint8_t streamLogger::benchCard()
{
    updateTimestampTimer();

    uint8_t nbCopies = 10;
    static uint8_t buffer[mBufRamDimLog];
    static uint8_t bufferForRead[mBufRamDimLogForRead];

    uint32_t FILE_SIZE = nbCopies * mBufRamDimLog;
    float speed;

    if (sd.exists("/bench.binDM"))
        sd.remove("/bench.binDM");

    mBenchFile = sd.open("/bench.binDM", FILE_WRITE);
    if (!mBenchFile)
    {
        errorsHandler(true, "bench create file");
        return 2;
    }

    // Write speed
    uint64_t t_now = micros();
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

    mBenchFile.close();

    // Read speed
    mBenchFile = sd.open("/bench.binDM", FILE_READ);
    if (!mBenchFile)
    {
        errorsHandler(true, "bench open file for read bench");
        return 2;
    }
    t_now             = micros();
    size_t fileLength = 0;
    fileLength        = mBenchFile.size();
    while (fileLength > 0)
    {
        size_t bytesToRead = fileLength;
        if (bytesToRead > mBufRamDimLogForRead)
            bytesToRead = (size_t)mBufRamDimLogForRead;
        if (mBenchFile.read(bufferForRead, bytesToRead) != bytesToRead)
        {
            errorsHandler(true, "bench reading file");
            return 2;
        }
        fileLength -= bytesToRead;
    }
    speed = static_cast<float>(FILE_SIZE) / static_cast<float>((micros() - t_now)) * 1000.F;
    mCardReadSpeed = (uint32_t)speed;

    mBenchFile.close();

    return static_cast<uint8_t>(mCardWriteSpeed <= mKbpsLimitWriteSpeed
                                || mCardReadSpeed <= mKbpsLimitReadSpeed);
}

void streamLogger::updateLogBuffer()
{
    if (!isLoggerReady())
        return;

    if (xSemaphoreTake(mSharedBuffersSemaphore, LOGGER_FRAME_STREAM_BUF_SEMAPHORE_BIN_DURATION)
        == pdTRUE)
    {
        updateTimestampTimer();
        convertLoggedVariables();
        updateRingBuffer();
        xSemaphoreGive(mSharedBuffersSemaphore);
    }
    // Write log if necessary, using special task or not
    triggerWrite();
}

void streamLogger::updateProtobufUlogBuffer(const char* charBuffer, size_t size)
{
    if (!isLoggerReady())
        return;

    if (xSemaphoreTake(mSharedBuffersSemaphore, LOGGER_FRAME_STREAM_BUF_SEMAPHORE_ULOG_DURATION)
        == pdTRUE)
    {
        resetLoggerFrame();

        mLoggerFrame.has_ulogData  = true;
        mLoggerFrame.ulogData.size = size;
        memcpy(mLoggerFrame.ulogData.bytes, charBuffer, size);
        updateRingBuffer();
        xSemaphoreGive(mSharedBuffersSemaphore);
    }

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

    if (xSemaphoreTake(mSharedBuffersSemaphore, LOGGER_FRAME_STREAM_BUF_SEMAPHORE_EVENT_DURATION)
        == pdTRUE)
    {
        resetLoggerFrame();

        // Update protocol content based on the type
        switch (type)
        {
        case EVENT_TYPE::MAVLINK:
        {
            if (size > LOGGER_FRAME_MAX_MAVLINK_SIZE)
            {
                printlnUlog(true, true, mClassName + ": mavlink packet too big");
                xSemaphoreGive(mSharedBuffersSemaphore);
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
                xSemaphoreGive(mSharedBuffersSemaphore);
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
        xSemaphoreGive(mSharedBuffersSemaphore);
    }

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
    mLoggerFrameBufferTx[0] = PROTOBUF_LOGGER_FRAME_START_BYTE_1;
    mLoggerFrameBufferTx[1] = PROTOBUF_LOGGER_FRAME_START_BYTE_2;
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
    if (newSize > xStreamBufferSpacesAvailable(xStreamBuffer))
    {
        // TODO: Add a timer to check how many bytes have been lost instead of spamming the log
        if (mBufferFullCounter == 0)
            printlnUlog(true,
                        false,
                        mClassName + ": Stream buffer full started, impossible to write log");
        mBufferFullCounter++;
        return;
    }
    if (mBufferFullCounter > 0)
    {
        printlnUlog(true,
                    false,
                    mClassName + ": Stream buffer full ended, " + String(mBufferFullCounter)
                            + " frames lost");
        mBufferFullCounter = 0;
    }

    mLoggerFrameBufferTx[writingStream.bytes_written + 2] = mEndLoopBatchTermination1;
    mLoggerFrameBufferTx[writingStream.bytes_written + 3] = mEndLoopBatchTermination2;
    size_t xBytesSent = xStreamBufferSend(xStreamBuffer, mLoggerFrameBufferTx, newSize, 0);

    if (xBytesSent != newSize)
    {
        printlnUlog(true,
                    false,
                    "Error sending to stream buffer " + String(xBytesSent)
                            + "=/= " + String(newSize));
    }
}

bool streamLogger::writeLog()
{
    if (!isLoggerReady())
        return false;

    updateTimestampTimer();

    // Receive byte from streamBuffer
    size_t xReceivedBytes =
            xStreamBufferReceive(xStreamBuffer, mLoggerFrameBufferRx, LOGGER_FRAME_MAX_SIZE, 10);
    if (xReceivedBytes == 0)
    {
        printlnUlog(true, false, "No data received from stream buffer");
        return false;
    }

    // Update ringBuffer
    size_t bytesWritten = 0U;

    bytesWritten           = ringBuffer->write(mLoggerFrameBufferRx, xReceivedBytes);
    bool resWritingRingBuf = bytesWritten == xReceivedBytes;
    if (!resWritingRingBuf)
    {
        printlnUlog(true,
                    false,
                    mClassName + ": Unable to write fast temp buffer to ring buf, size = "
                            + String(bytesWritten)
                            + " bytes, with expected size = " + String(xReceivedBytes) + " bytes");
        return false;
    }

    size_t nbByteUsed = ringBuffer->bytesUsed();
    if ((nbByteUsed + mLogFile.position()) > (LOG_FILE_SIZE - mRingBufferBatchSize))
    {
        printlnUlog(false, true, "Log file is full, closing it"); // Maybe create a new one
        mLogFile.close();
        mIsLogFileFull = true;
        return false;
    }

    if (nbByteUsed >= mRingBufferBatchSize)
    {
        size_t nbSectorsToWrite = std::max(1U, nbByteUsed / mRingBufferBatchSize);
        mIsLogWritingSector     = true;
        for (size_t i = 0; i < nbSectorsToWrite; i++)
        {
            // Write one sector from RingBuf to file
            if (mRingBufferBatchSize != ringBuffer->writeOut(mRingBufferBatchSize))
            {
                printlnUlog(false, true, "Failed to write sector (512 bytes)");
                mErrorsCounter++;
                mIsLogWritingSector = false;
                return false;
            }
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

    FRESULT res               = FR_OK;
    esp_err_t err             = ESP_OK;
    BYTE pdrv                 = FF_DRV_NOT_USED;
    const size_t workbuf_size = 4096;
    void* workbuf             = NULL;
    mSerialMonitor->println("Formatting sd card");

    workbuf = ff_memalloc(workbuf_size);
    if (workbuf == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    DWORD plist[] = {100, 0, 0, 0};
    res           = f_fdisk(pdrv, plist, workbuf);
    if (res != FR_OK)
    {
        mSerialMonitor->println("f_disk error");
        return false;
    }

    size_t alloc_unit_size = 512;
    mSerialMonitor->println("Allocation size = " + String(alloc_unit_size));
    res = f_mkfs("", FM_FAT, alloc_unit_size, workbuf, workbuf_size);
    if (res != FR_OK)
    {
        mSerialMonitor->println("f_mkfs error");
        return false;
    }

    mSerialMonitor->println("Finishing partitioning card");
    free(workbuf);
    return err;

    errorsHandler(true, "formatCard");

    mSerialMonitor->println("Card formatted");

    return true;
}

bool streamLogger::readTextFile(String filePath)
{
    if (mCardStatus == CARD_STATUS::NO_CARD_INSERTED || mCardStatus == CARD_STATUS::CARD_KO)
        return false;

    if (filePath[0] != '/')
        filePath = "/" + filePath;

    if (!sd.exists(filePath))
    {
        errorsHandler(true, "can not find text file " + filePath + " to read");
        return false;
    }

    File file = sd.open(filePath, FILE_READ);
    printlnUlog(true,
                true,
                mClassName + ": Successfully opened text file " + filePath
                        + " size = " + String(file.size()) + " bytes");
    if (!file)
    {
        errorsHandler(true, "open file for read");
        return false;
    }

    String line     = "";
    uint32_t lineNb = 0;
    while (file.available() && lineNb < TEXT_STRING_BUFFER_MAX_NB_LINES)
    {
        line = file.readStringUntil('\n');
        mTextStringBuffer->push_back(line);
        lineNb++;
    }

    file.close();
    printlnUlog(true,
                true,
                mClassName + ": Successfully read text file " + filePath + " to ring buffer");
    return true;
}

String streamLogger::readTextLine(String filePath)
{
    String curLine = "";

    if (mCardStatus == CARD_STATUS::NO_CARD_INSERTED || mCardStatus == CARD_STATUS::CARD_KO)
        return curLine;

    if (filePath[0] != '/')
        filePath = "/" + filePath;

    if (!sd.exists(filePath))
    {
        errorsHandler(true, "can not find text file " + filePath + " to read");
        return curLine;
    }

    File file = sd.open(filePath, FILE_READ);
    if (!file)
    {
        errorsHandler(true, "open file for read");
        return curLine;
    }

    if (file.available())
        curLine = file.readStringUntil('\n');

    file.close();

    return curLine;
}

String streamLogger::readTextLine(File& file)
{
    String curLine = "";

    if (mCardStatus == CARD_STATUS::NO_CARD_INSERTED || mCardStatus == CARD_STATUS::CARD_KO)
        return curLine;

    if (file.available())
        curLine = file.readStringUntil('\n');

    return curLine;
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

    File file = mStreamLoggerObj->sd.open(filePath, FILE_READ);
    mStreamLoggerObj->printlnUlog(true,
                                  true,
                                  mClassName + ": Successfully opened json file " + filePath
                                          + " size = " + String(file.size()) + " bytes");
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
    mJsonCharBuffer[pos] = static_cast<char>(0);

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

        File file = mStreamLoggerObj->sd.open(filePath, FILE_WRITE);
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

    mStreamLoggerObj->errorsHandler(true,
                                    "writeJsonDoc() failed: unknown or not implemented interface");
    return false;
}

#endif
