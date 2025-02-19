#ifndef streamLogger_hpp
#define streamLogger_hpp

#include "Arduino.h"
#include "RingBuf.h"
#include "libDM_timer_tool.hpp"
#include "vector"
#include "libDM_stream_logger_boards_layer.hpp"
#include "ArduinoJson.h"
#include "frame.pb.h"

#ifdef ESP_PLATFORM
#    include "message_buffer.h"
#endif

// Size to log 400s with 807 variables
constexpr uint32_t LOG_FILE_SIZE = 1000U * 1000U * 60U * 20U; // 1.2go

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY (200 * 512)

constexpr uint16_t LOGGER_FRAME_MAX_SIZE          = 4096U + 1024U;
constexpr uint16_t LOGGER_FRAME_MAX_MAVLINK_SIZE  = 280U; // Based on array size in frame.proto
constexpr uint16_t LOGGER_FRAME_MAX_PROTOBUF_SIZE = 512U; // Based on array size in frame.proto
constexpr uint16_t LOGGER_FRAME_MAX_ULOG_SIZE     = 512U; // Based on array size in frame.proto

// A stream buffer memory might be used to echange between writing/flushing task on multithread
// systems. This exchange buffer is sized to hold 20 frames of the maximum size. A semaphore is used
// to protect the shared buffers against concurrent access. We define duration waited to write
// binary data, ulog data and events data.
constexpr uint32_t LOGGER_FRAME_STREAM_BUF_MEMORY = LOGGER_FRAME_MAX_SIZE * 20U;
// Duration waited to write binary data, ulog data and events data.
constexpr uint8_t LOGGER_FRAME_STREAM_BUF_SEMAPHORE_BIN_DURATION   = 1;  // Wait 1 tick
constexpr uint8_t LOGGER_FRAME_STREAM_BUF_SEMAPHORE_ULOG_DURATION  = 10; // Wait 10 ticks
constexpr uint8_t LOGGER_FRAME_STREAM_BUF_SEMAPHORE_EVENT_DURATION = 10; // Wait 10 ticks
// In multithreaded systems, the writing task will be triggered when the buffer size reaches this
// size. The temporary buffer will be copied to ringBuffer and flushed later to the SD card.
constexpr uint32_t LOGGER_FRAME_WRITE_LOG_THRESHOLD = 1024U * 4U; // 4ko

constexpr float FLUSH_TIMER_THRESHOLD_S = 10.F; // 10s

// Maximum size of the json exchanges files (buffer + document), which means max json file size is
// 16ko
constexpr uint32_t JSON_FILE_SIZE = 32768U;

constexpr uint32_t TEXT_STRING_BUFFER_MAX_NB_LINES   = 2000U;
constexpr uint8_t PROTOBUF_LOGGER_FRAME_START_BYTE_1 = 55U;
constexpr uint8_t PROTOBUF_LOGGER_FRAME_START_BYTE_2 = 66U;

/**
 * @brief Transmission direction.
 */
enum class RX_TX : uint8_t
{
    RX = 0,
    TX = 1
};

/**
 * @brief Transmission direction.
 */
enum class EVENT_TYPE : uint8_t
{
    PROTOBUF = 0,
    MAVLINK  = 1
};

/**
 * @brief Class representing a stream logger.
 * @details The streamLogger class provides functionality for logging and printing timestamped data
 * and events to different hardware interfaces such as serial, SD card... It supports logging for
 * multiple protocols like mavlink, protobuf. Variables added are logged to binary *.binDM files.
 * Events and debug informations are logged as text files. At startup, a new folder  is added to the
 * SD card and all files created during the session are stored in this folder. The class also
 * provides methods to read and write JSON files.
 */
class streamLogger
{
public:
    /**
     * @brief Enumeration representing different hardware interfaces.
     */
    enum class INTERFACE : uint8_t
    {
        INTERFACE_SERIAL = 0, /**< Serial interface */
        INTERFACE_SD     = 1, /**< SD card interface */
        INTERFACE_CAN    = 2, /**< CAN interface */
        INTERFACE_WIFI   = 3, /**< WiFi interface */
        INTERFACE_BLE    = 4  /**< Bluetooth Low Energy interface */
    };

    /**
     * @brief Constructs a streamLogger object.
     * @param timerToolObject Pointer to a timerTool object.
     * @param sdioConf SDIO configuration object (default: SDIO_CONF()).
     * @param serialMonitor Pointer to a Print object for serial monitoring (default: &Serial).
     * @param nbErrorsThreshold Number of errors threshold (default: 5).
     * @param flushTimerThresholdUs Flush timer value in useconds (default: 10000000).
     */
    explicit streamLogger(timerTool* timerToolObject,
                          const SDIO_CONF& sdioConf = SDIO_CONF(),
                          Print* serialMonitor      = &Serial,
                          uint8_t nbErrorsThreshold = 5,
                          float flushTimerThreshold = FLUSH_TIMER_THRESHOLD_S) :
        mNbErrorsThreshold(nbErrorsThreshold),
        mSerialMonitor(serialMonitor),
        mTimerObj(timerToolObject),
        mFlushTimerThreshold(flushTimerThreshold),
        mSdioConf(sdioConf)
    {
        mTextStringBuffer->reserve(TEXT_STRING_BUFFER_MAX_NB_LINES);
    }

    ~streamLogger()
    {
        errorsHandler(true, "exiting logger");
        close();
    }

private:
    // ====================================================== //
    // ===================== Subclasses ===================== //
    // ====================================================== //
    /**
     * @brief The jsonTools class provides functionality for reading, updating, printing, and
     * writing JSON documents.
     */
    class jsonTools
    {
    public:
        /**
         * @brief Constructs a jsonTools object with the specified streamLogger object.
         * @param logger A pointer to the streamLogger object.
         */
        explicit jsonTools(streamLogger* logger) : mStreamLoggerObj(logger) // NOLINT
        {
        }

        /**
         * @brief Destructor for the jsonTools object.
         */
        ~jsonTools() = default;

        /**
         * @brief Reads a JSON file from the specified file path.
         * @param filePath The path of the JSON file to read.
         * @return True if the JSON file was successfully read, false otherwise.
         */
        bool readJsonFile(String filePath); // TODO: test for teensy 4.X

        /**
         * @brief Updates the JSON document using a vector of strings.
         * @param buffer The vector of strings containing the JSON data.
         * @return True if the JSON document was successfully updated, false otherwise.
         */
        bool updateJsonFromStringVector(const std::vector<String>& buffer);

        /**
         * @brief Prints the JSON document.
         * @details This function uses the member variable mJsonDoc.
         */
        void printJsonDoc();

        /**
         * @brief Prints the specified JSON document.
         * @param jsonDoc The JSON document to print.
         */
        void printJsonDoc(DynamicJsonDocument& jsonDoc);

        /**
         * @brief Writes the JSON document to a file.
         * @details This function uses the member variable mJsonDoc.
         * @param interface The interface to use for writing the JSON document.
         * @param filePath The path of the file to write the JSON document to.
         * @return True if the JSON document was successfully written to the file, false otherwise.
         */
        bool writeJsonDoc(streamLogger::INTERFACE interface, String filePath);

        /**
         * @brief Writes the specified JSON document to a file.
         * @param interface The interface to use for writing the JSON document.
         * @param filePath The path of the file to write the JSON document to.
         * @param jsonDoc The JSON document to write.
         * @return True if the JSON document was successfully written to the file, false otherwise.
         */
        bool writeJsonDoc(
                streamLogger::INTERFACE interface,
                String filePath,
                DynamicJsonDocument& jsonDoc); // TODO: add other interfaces, test on teensy 4.X

        /**
         * @brief Returns a reference to the JSON document.
         * @details The JSON document can be modified by the user.
         * @return A reference to the JSON document.
         */
        DynamicJsonDocument& getJsonDocument()
        {
            return mJsonDoc;
        }

    private:
        /**
         * @brief Deserializes the JSON character buffer into the JSON document.
         * @return True if the JSON character buffer was successfully deserialized, false otherwise.
         */
        bool deserializeJsonCharBuffer();

        char* mJsonCharBuffer = new char[JSON_FILE_SIZE / 2]; // On heap to avoid stack overflow
        DynamicJsonDocument mJsonDoc = DynamicJsonDocument(
                JSON_FILE_SIZE / 2); // On heap to avoid stack overflow (DynamicJsonDocument is
                                     // allocated on heap by default)
        streamLogger* mStreamLoggerObj = NULL;

        String mClassName = "jsonTools";
    };

public:
    // ====================================================== //
    // =================== Public methods =================== //
    // ====================================================== //

    /**
     * @brief Formats the card.
     * @return true if the card is successfully formatted, false otherwise.
     */
    bool formatCard();

    /**
     * @brief Initializes the stream logger.
     * @details This method initializes SD card SDIO communication, create a new folder, au do a
     * speed measurement for read/write.
     * @return true if the stream logger is successfully initialized, false otherwise.
     */
    bool begin();

    /**
     * @brief Indicate if the logger is ready to log.
     * @return true if the logger is ready to log, false otherwise.
     */
    inline bool isLoggerReady() const
    {
        return (mCardStatus != CARD_STATUS::NO_CARD_INSERTED && mCardStatus != CARD_STATUS::CARD_KO
                && !mIsLogFileFull);
    }

    /**
     * @brief Handles errors and logs them if necessary.
     * @param sdLogError Flag indicating whether to log the error to an SD card.
     * @param typeError The type of error that occurred.
     */
    void errorsHandler(bool sdLogError, String typeError);

    /**
     * @brief Performs the benchmarking of the card.
     * @details This function measures the performance, print it and write it in the log file.
     * @return The benchmark result as a uint8_t value (enum)
     */
    uint8_t benchCard();

    /**
     * @brief Get the root directories.
     * @details This function returns a vector of strings representing the root directories.
     * @return std::vector<std::string> The root directories.
     */
    std::vector<std::string> getRootDirs();

    /**
     * @brief Creates a new root directory.
     * @return true if the root directory was successfully created, false otherwise.
     */
    bool createNewRootDir();

    /**
     * @brief Adds a variable to the logger.
     * @details This function adds a variable to the logger by specifying its address and name.
     * Variable type is handled by function overloading
     * @param VariableAddress The address of the variable to be added.
     * @param Name The name of the variable.
     * @return True if the variable was successfully added, false otherwise.
     */
    bool addVariable(const uint8_t* VariableAdress, String Name);
    bool addVariable(const uint16_t* VariableAdress, String Name);
    bool addVariable(const uint32_t* VariableAdress, String Name);
    bool addVariable(const int8_t* VariableAdress, String Name);
    bool addVariable(const int16_t* VariableAdress, String Name);
    bool addVariable(const int32_t* VariableAdress, String Name);
    bool addVariable(const float* VariableAdress, String Name);
    bool addVariable(const double* VariableAdress, String Name);
    bool addVariable(const uint64_t* VariableAdress, String Name);

    /**
     * @brief Adds protobuf Frames names to the protobuf header field
     * @param names The names of the protobuf frames names to add.
     * @return true if the message names were successfully added, false otherwise.
     */
    bool writeProtobufHeader(const std::vector<String>& names);

    /**
     * @brief Adds Mavlink Frames names and ID to the mavlink header field
     * @param names The names of the mavlink frames names to add.
     * @param ids The ids of the mavlink frames names to add.
     * @return true if the message names were successfully added, false otherwise.
     */
    bool writeMavlinkHeaderAndId(const std::vector<String>& names,
                                 const std::vector<uint32_t>& ids);

    /**
     * @brief Debug function that prints the header in the Serial monitor and in the ulog file.
     * @param addInUlog Flag indicating whether to add the header in the ulog.
     * @return True if the header was printed successfully, false otherwise.
     */
    bool printHeaderMonitor(bool addInUlog = false);

    /**
     * @brief Debug function that prints the types in the Serial monitor and in the ulog file.
     * @param addInUlog Flag indicating whether to add the types in the ulog.
     * @return True if the types were printed successfully, false otherwise.
     */
    bool printTypesMonitor(bool addInUlog = false);

    /**
     * @brief Updates the log buffer
     * @details This function loops on all the added variables and update the log ring buffer with
     * the updated values.
     */
    void updateLogBuffer();

    /**
     * @brief Set the Write Log Task object
     * @details If using multicore/multitask processor, it can be interesting to write/flush the
     * logs on another thread/core task to avoid slowing down the main task.
     * @warning If a task handle is provided (not NULL), the writing task will not be triggered at
     * each update of (ulog, log, protobuf, mavlink) buffer. The library will automatically trigger
     * the task following the sizeThreshold provided in the configuration.
     * @note As the task notification is often RTOS dependant, the triggerWrite() function has to be
     * implemented for each RTOS.
     * @param taskHandle The writing task handle
     * @param sizeThreshold The size threshold to trigger the writing task
     */
    inline void setWriteLogTask(void* taskHandle,
                                uint32_t sizeThreshold = LOGGER_FRAME_WRITE_LOG_THRESHOLD)
    {
        mWriteLogTaskHandle        = taskHandle;
        mWriteLogTaskSizeThreshold = sizeThreshold;
        printlnUlog(true,
                    true,
                    mClassName + "Write log task set with threshold = " + String(sizeThreshold));
    }

    /**
     * @brief Write the crash report to ulog file
     * @details This function write the last crash to the ulog text file if crash report is
     * available
     * @param CrashReport The crash report to be printed.
     */
    void printCrashReport(CrashReportClass CrashReport);

    /**
     * @brief Transfer ring buffer to sd card buffer and ensure data is flushed regularly
     * @param updateTimestamp Flag indicating whether to update the timestamp of the log entry.
     * Default is true.
     */
    bool writeLog();

    /**
     * @brief Flushes the sd card buffer to the sd card (data will be written)
     * @return true if the flush was successful, false otherwise.
     */
    bool flushLog();

    /**
     * @brief Closes the file
     * @return true if the stream logger is successfully closed, false otherwise.
     */
    bool close();

    /**
     * @brief Prints the given message to the serial monitor and/or a file.
     * @details This function prints the given message to the serial monitor and/or a file,
     * depending on the specified options. If `print_In_Monitor` is set to `true`, the message will
     * be printed to the serial monitor. If `print_In_File` is set to `true`, the message will be
     * printed to the ulog text file. The `message` parameter represents the message to be printed.
     * The `useTimeStamp` parameter determines whether a timestamp should be added to the message.
     * By default, a timestamp is added to the message.
     * @tparam Type The type of the message.
     * @param print_In_Monitor Whether to print the message to the serial monitor.
     * @param print_In_File Whether to print the message to a file.
     * @param message The message to be printed.
     * @param useTimeStamp Whether to add a timestamp to the message (default is true).
     */
    template <typename Type>
    void printUlog(bool print_In_Monitor,
                   bool print_In_File,
                   const Type& message,
                   bool useTimeStamp = true)
    {
        String messageOrig    = String(message);
        String messageMonitor = messageOrig;

        if (useTimeStamp && print_In_Monitor)
        {
            updateTimestampTimer();
            messageMonitor = String(mInternalTimeStampFloat, 3) + ": " + messageMonitor;
        }

        if (print_In_Monitor)
            mSerialMonitor->print(messageMonitor);

        if (!isLoggerReady())
            return;

        if (print_In_File)
            updateProtobufUlogBuffer(messageOrig.c_str(), messageOrig.length());
    }

    /**
     * @brief Prints the specified message to the Ulog.
     * @note This function is the implementation of the `printUlog` function for the `String` type.
     * @param print_In_Monitor Whether to print the message in the monitor.
     * @param print_In_File Whether to print the message in a file.
     * @param message The message to be printed.
     * @param useTimeStamp Whether to add a timestamp to the message (default is true).
     */
    void printUlog(bool print_In_Monitor,
                   bool print_In_File,
                   String message,
                   bool useTimeStamp = true)
    {
        String messageOrig    = message;
        String messageMonitor = messageOrig;

        if (useTimeStamp && print_In_Monitor)
        {
            updateTimestampTimer();
            messageMonitor = String(mInternalTimeStampFloat, 3) + ": " + messageMonitor;
        }

        if (print_In_Monitor)
            mSerialMonitor->print(messageMonitor);

        if (!isLoggerReady())
            return;

        if (print_In_File)
            updateProtobufUlogBuffer(messageOrig.c_str(), messageOrig.length());
    }

    /**
     * @brief Prints a message to the serial monitor and/or a file and adds a newline character at
     * the end.
     * @details This function prints the specified message to the serial monitor and/or a file,
     * with an optional timestamp. The message can be printed to the serial monitor,
     * a file, or both, depending on the provided parameters. If the message is printed
     * to a file, it is also added to a ring buffer for later retrieval.
     *
     * @tparam Type The type of the message to be printed.
     * @param print_In_Monitor Whether to print the message to the serial monitor.
     * @param print_In_File Whether to print the message to a file.
     * @param message The message to be printed.
     * @param useTimeStamp Whether to add a timestamp to the message (default is true).
     */
    template <typename Type>
    void printlnUlog(bool print_In_Monitor,
                     bool print_In_File,
                     Type message,
                     bool useTimeStamp = true)
    {
        String messageOrig    = String(message) + "\n";
        String messageMonitor = messageOrig;

        if (useTimeStamp && print_In_Monitor)
        {
            updateTimestampTimer();
            messageMonitor = String(mInternalTimeStampFloat, 3) + ": " + messageMonitor;
        }

        if (print_In_Monitor)
            mSerialMonitor->print(messageMonitor);

        if (!isLoggerReady())
            return;

        if (print_In_File)
            updateProtobufUlogBuffer(messageOrig.c_str(), messageOrig.length());
    }

    /**
     * @brief Prints a message to the ulog text file and adds a newline character at the end.
     * @note This function is the implementation of the `printlnUlog` function for the `String`
     * type.
     * @param print_In_Monitor Whether to print the message in the monitor.
     * @param print_In_File Whether to print the message in a file.
     * @param message The message to be printed.
     * @param useTimeStamp Whether to add a timestamp to the message (default is true).
     */
    void printlnUlog(bool print_In_Monitor,
                     bool print_In_File,
                     String message,
                     bool useTimeStamp = true)
    {
        String messageOrig    = message + "\n";
        String messageMonitor = messageOrig;

        if (useTimeStamp && print_In_Monitor)
        {
            updateTimestampTimer();
            messageMonitor = String(mInternalTimeStampFloat, 3) + ": " + messageMonitor;
        }

        if (print_In_Monitor)
            mSerialMonitor->print(messageMonitor);

        if (!isLoggerReady())
            return;

        if (print_In_File)
            updateProtobufUlogBuffer(messageOrig.c_str(), messageOrig.length());
    }

    /**
     * @brief Returns the local directory name (the one created at the beginning of the session)
     * @return The local directory name as a string.
     */
    String getLocalDirName()
    {
        return mDirName;
    }

    /**
     * @brief Returns the reference to the jsonTools object used by the streamLogger.
     * @return A reference to the jsonTools object.
     */
    jsonTools& getJsonTools()
    {
        return mJsonTool;
    }

    /**
     * @brief Update protobuf LoggerFrame and append it to ring buffer depending on its type
     * @param buffer is the buffer to write to the protobuf LoggerFrame
     * @param size is the size of the buffer
     * @param type is the type of the buffer (PROTOBUF, MAVLINK)
     * @param direction is the direction of the buffer (RX, TX)
     */
    void updateProtobufProtocolLogBuffer(const uint8_t* buffer,
                                         size_t size,
                                         EVENT_TYPE type,
                                         RX_TX direction);

    std::vector<String>* getTextStringBuffer()
    {
        return mTextStringBuffer;
    }

// ~~~~~ File handling from apps API ~~~~~ //
#if defined(CORE_TEENSY)
    /**
     * @brief Creates a new file at the specified file path.
     * @param filePath The path of the file to be created.
     * @param newFile The reference to the ExFile object that will hold the newly created file.
     * @return true if the file was successfully created, false otherwise.
     */
    bool createNewFile(String filePath, ExFile& newFile);

    /**
     * @brief Closes the specified file.
     * @param file The file to be closed.
     * @return True if the file was successfully closed, false otherwise.
     */
    bool closeFile(ExFile& file);
    // Following function are generic, the output can be converted to json, but it is more memory
    // efficient to use json toolbox if the file only contains json
    /**
     * @brief Reads a text file from the specified file path.
     * @details The read content will be stored in the String vector mTextStringBuffer
     * @param filePath The path of the text file to be read.
     * @return True if the file was successfully read, false otherwise.
     */
    bool readTextFile(String filePath); // TODO: to write
    /**
     * @brief Reads a single line of text from a file.
     * @param filePath The path to the file.
     * @return The text line read from the file.
     */
    String readTextLine(String filePath); // TODO: to write
    /**
     * @brief Reads a line of text from the specified file.
     * @param file The file to read from.
     * @return The line of text read from the file.
     */
    String readTextLine(ExFile& file); // TODO: to write
#elif defined(ESP_PLATFORM)
    /**
     * @brief Creates a new file at the specified file path.
     * @param filePath The path of the file to be created.
     * @param newFile The reference to the ExFile object that will hold the newly created file.
     * @return true if the file was successfully created, false otherwise.
     */
    bool createNewFile(String filePath, File& newFile);
    /**
     * @brief Closes the specified file.
     * @param file The file to be closed.
     * @return True if the file was successfully closed, false otherwise.
     */
    bool closeFile(File& file);
    // Following function are generic, the output can be converted to json, but it is more memory
    // efficient to use json toolbox if the file only contains json
    /**
     * @brief Reads a text file from the specified file path.
     * @details The read content will be stored in the String vector mTextStringBuffer
     * @param filePath The path of the text file to be read.
     * @return True if the file was successfully read, false otherwise.
     */
    bool readTextFile(String filePath);
    /**
     * @brief Reads a single line of text from a file.
     * @param filePath The path to the file.
     * @return The text line read from the file.
     */
    String readTextLine(String filePath);
    /**
     * @brief Reads a line of text from the specified file.
     * @param file The file to read from.
     * @return The line of text read from the file.
     */
    String readTextLine(File& file);
#endif

private:
    // ====================================================== //
    // =================== Private classes ================== //
    // ====================================================== //
    enum class CARD_STATUS : uint8_t
    {
        NO_CARD_INSERTED = 0,
        CARD_OK,
        CARD_OK_LOW_PERFORMANCE,
        CARD_KO
    };

    // ====================================================== //
    // =================== Private methods ================== //
    // ====================================================== //
    bool printLog(String message);
    bool printlnLog(String message);
    bool createLogFiles();
    bool writeHeaderAndTypes();
    bool tryStart();
    bool formatCardBackup();
    void computeLoopSizeBatch();
    void convertLoggedVariables();

    void updateRingBuffer(); // Serialize protoframe and append it to ring buffer
    void updateTimestampTimer();
    void computeRingBufferBatchSize();

    /* void feedRamLogBuffer(const uint8_t *buffer, size_t size); // idée: à la config, on
     * sauvegarde un vecteur de pointeurs vers les variables à logger. On stocke tout ça avec la
     * taille et le nom des variables (écrits en en-tête) dans un std::array. A chaque tour, on
     * récupère les pointeurs, on convertit en un buffer d'uint avec 2 headers, 2 terminaisons et un
     * checksum et on feed le buffer (feedLogBuffer qui bufferise en ram). Quand le buffer atteint
     * la taille mBufRamDimLog, on write le log. Toutes les N write (config), on flush dans la
     * carte */

    /**
     * @brief Update protobuf LoggerFrame with ulog ascii buffer
     * @param buffer is the buffer to write to the protobuf LoggerFrame
     * @param size is the size of the buffer
     */
    void updateProtobufUlogBuffer(const char* charBuffer, size_t size);

    /**
     * @brief Reset Logger Frame content
     */
    inline void resetLoggerFrame()
    {
        mLoggerFrame = LoggerFrame_init_zero;
    }

    CARD_STATUS mCardStatus = CARD_STATUS::NO_CARD_INSERTED;

    uint8_t mNbErrorsThreshold = 0U;

    SL::SdCardType sd = SL::SdCardType();

#if defined(CORE_TEENSY)
    ExFile mLogFile   = ExFile();
    ExFile mFileDir   = ExFile();
    ExFile mBenchFile = ExFile();
    ExFile mDir       = ExFile();
#elif defined(ESP_PLATFORM)
    File mLogFile   = File();
    File mFileDir   = File();
    File mBenchFile = File();
    File mDir       = File();
#endif

    const String mDirFormat                 = "Dir";
    String mDirName                         = "";
    const uint32_t mKbpsLimitWriteSpeed     = 10000U;
    const uint32_t mKbpsLimitReadSpeed      = 15000U;
    uint32_t mCardWriteSpeed                = 0U;
    uint32_t mCardReadSpeed                 = 0U;
    const uint8_t mStartLoopBatchHeader     = 126;  // ~
    const uint8_t mEndLoopBatchTermination1 = '\r'; // CR
    const uint8_t mEndLoopBatchTermination2 = '\n'; // LF
    const uint8_t mNumberTryBegin           = 5U;
    uint16_t mSignalsNumber                 = 0U;

    Print* mSerialMonitor                         = NULL;
    timerTool* mTimerObj                          = NULL;
    uint16_t mUlogBufferLineCount                 = 0U;
    uint16_t mLogBufferPacketCount                = 0U;
    uint8_t mErrorsCounter                        = 0U;
    std::vector<String> mHeaderNames8             = {};
    std::vector<String> mHeaderNames16            = {};
    std::vector<String> mHeaderNames32            = {};
    std::vector<String> mHeaderNamesi8            = {};
    std::vector<String> mHeaderNamesi16           = {};
    std::vector<String> mHeaderNamesi32           = {};
    std::vector<String> mHeaderNamesf             = {};
    std::vector<String> mHeaderNamesd             = {};
    std::vector<String> mHeaderNames64            = {};
    std::vector<String> mHeaderTypes              = {};
    std::vector<const uint8_t*> mUint8Variables   = {};
    std::vector<const uint16_t*> mUint16Variables = {};
    std::vector<const uint32_t*> mUint32Variables = {};
    std::vector<const int8_t*> mInt8Variables     = {};
    std::vector<const int16_t*> mInt16Variables   = {};
    std::vector<const int32_t*> mInt32Variables   = {};
    std::vector<const float*> mFloatVariables     = {};
    std::vector<const double*> mDoubleVariables   = {};
    std::vector<const uint64_t*> mUint64Variables = {};
    uint16_t mLoopSizeBatch                       = 0U;
    float mFlushTimerThreshold                    = 0.F;
    float mInternalTimeStampFloat                 = 0.0F;

#if defined(CORE_TEENSY)
    RingBuf<ExFile, RING_BUF_CAPACITY>* ringBuffer = new RingBuf<ExFile, RING_BUF_CAPACITY>; // RAM2
    bool triggerWrite()
    {
        return writeLog();
    }
#elif defined(ESP_PLATFORM)
    RingBuf<File, RING_BUF_CAPACITY>* ringBuffer = new RingBuf<File, RING_BUF_CAPACITY>;
    uint8_t* streamBufferMemory =
            new uint8_t[LOGGER_FRAME_STREAM_BUF_MEMORY]; // To hold multiple frames copied to ring
                                                         // buffer
    StaticStreamBuffer_t xStreamBufferStruct = StaticStreamBuffer_t();
    StreamBufferHandle_t xStreamBuffer = xStreamBufferCreateStatic(LOGGER_FRAME_STREAM_BUF_MEMORY,
                                                                   1,
                                                                   streamBufferMemory,
                                                                   &xStreamBufferStruct);
    SemaphoreHandle_t mSharedBuffersSemaphore = xSemaphoreCreateMutex();
    uint32_t mBufferFullCounter               = 0U;
    /**
     * @brief Trigger the log writing task
     * @details This function only check the size threshold and trigger the writing task if an
     * handle exists. Otherwise, it will trigger the writing task directly.
     * @return true if the writing task is triggered, false otherwise.
     */
    bool triggerWrite()
    {
        if (mWriteLogTaskHandle == nullptr)
            return writeLog();

        if (xStreamBufferBytesAvailable(xStreamBuffer) >= mWriteLogTaskSizeThreshold)
        {
            // Notify the task that the buffer is ready to be written
            xTaskNotifyGive(mWriteLogTaskHandle);
            return true;
        }

        return false;
    }
#endif

    /**
     * @brief Struct form of the LoggerFrame protobuf message.
     */
    LoggerFrame mLoggerFrame = LoggerFrame();
    /**
     * @brief A buffer to hold a single serialized protobuf message.
     */
    uint8_t* mLoggerFrameBufferTx     = new uint8_t[LOGGER_FRAME_MAX_SIZE];
    uint32_t mLoggerFrameBufferTxSize = 0U;
    /**
     * @brief A buffer to receive a single serialized from writing task.
     */
    uint8_t* mLoggerFrameBufferRx = new uint8_t[LOGGER_FRAME_MAX_SIZE];

    void* mWriteLogTaskHandle           = nullptr;
    uint32_t mWriteLogTaskSizeThreshold = 1U;

    SDIO_CONF mSdioConf;
    jsonTools mJsonTool                 = jsonTools(this);
    static const uint32_t mBufRamDimLog = 32768U;
    static const uint32_t mBufRamDimLogForRead =
            512U; // Used for esp32 that fails after certain time if size is above sector size
    bool mIsLogFileFull      = false;
    float mLastFlushTime     = 0.F;
    bool mIsLogWritingSector = false;
    uint32_t mRingBufferBatchSize =
            512U; // teensy: 7 us for each writeout, 1024 = 30us, 2048 = 80us. ESP32 is much
                  // slower(7ms at each writeout but can be improved with multitask writing ringbuf
                  // with value in one task and writing to sd card in another task)

    std::vector<String>* mTextStringBuffer =
            new std::vector<String>; // On heap to avoid stack overflow
    String mClassName = "streamLogger";

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
};

#endif