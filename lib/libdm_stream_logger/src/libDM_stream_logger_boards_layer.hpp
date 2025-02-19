/**
 * @file libDM_stream_logger_boards_layer.hpp
 * @brief This file contains the definition of the SDIO_CONF struct and the SL::SdCardType class.
 * @details The SDIO_CONF struct represents the configuration parameters for SDIO communication.
 * It includes the clock frequency, pin assignments, mount point, and other options. This adaptation
 * layer is currently used for the Teensy 4.X and ESP32 boards.
 */
// Stream logger has been created for teensy 4.X. This layer is used to make it compatible with
// other boards.

#if defined(CORE_TEENSY)
#    include <CrashReport.h>
#    include "SdFat.h"
#elif defined(ESP_PLATFORM)
#    include "SD_MMC.h"
#    include "vfs_api.h"
#    include "sdmmc_cmd.h"
#    include "diskio_impl.h"
#    include "esp_vfs_fat.h"
#endif

struct SDIO_CONF
/**
 * @brief Configuration class for SDIO.
 * @details This class represents the configuration for SDIO (Secure Digital Input/Output).
 * It provides parameters such as clock frequency, pin assignments, mountpoint, and format behavior.
 */
{
    SDIO_CONF() = default;
    SDIO_CONF(uint32_t clock,
              int8_t pinSdioClk,
              int8_t pinSdioCmd,
              int8_t pinSdioDat0,
              int8_t pinSdioDat1,
              int8_t pinSdioDat2,
              int8_t pinSdioDat3,
              const char* mountpoint,
              bool formatIfMountFailed) :
        mClock(clock),
        mPinSdioClk(pinSdioClk),
        mPinSdioCmd(pinSdioCmd),
        mPinSdioDat0(pinSdioDat0),
        mPinSdioDat1(pinSdioDat1),
        mPinSdioDat2(pinSdioDat2),
        mPinSdioDat3(pinSdioDat3),
        mMountpoint(mountpoint),
        mFormatIfMountFailed(formatIfMountFailed),
        mIsValid(true) {};
    uint32_t mClock           = 40000U; // [kHz]
    int8_t mPinSdioClk        = -1;
    int8_t mPinSdioCmd        = -1;
    int8_t mPinSdioDat0       = -1;
    int8_t mPinSdioDat1       = -1;
    int8_t mPinSdioDat2       = -1;
    int8_t mPinSdioDat3       = -1;
    const char* mMountpoint   = "/sdcard";
    bool mFormatIfMountFailed = false;
    bool mIsValid             = false;
};

// ====================================================== //
// ================ Non existing classes ================ //
// ====================================================== //
// This section if for classes that are not available on the board.
#if defined(CORE_TEENSY)

#elif defined(ESP_PLATFORM)
class CrashReportClass : public Printable
{
public:
    /**
     * @brief Prints a message indicating that CrashReport is not available on esp32.
     * @param p The Print object to print the message to.
     * @return The number of characters printed.
     */
    size_t printTo(Print& p) const
    {
        p.println("CrashReport is not available on esp32");
        return 1;
    }
};
#endif

/**
 * @brief Class to convert printable objects to String
 */

class PrintString : public Print
{
public:
    PrintString() = default;

    size_t write(uint8_t ch) override
    {
        buffer += (char)ch;
        return 1;
    }

    size_t write(const uint8_t* buffer, size_t size) override
    {
        for (size_t i = 0; i < size; i++)
        {
            write(buffer[i]);
        }
        return size;
    }

    String getString() const
    {
        return buffer;
    }

private:
    String buffer = "";
};

// ====================================================== //
// ================== Modified classes ================== //
// ====================================================== //
// Define redefinition namespace for modified classes. Was impossible to do with File/ExFile as
// there is too much difference between the two for esp32 and teensy.
/**
 * @namespace SL
 * @brief Namespace containing classes related to SL (Stream Logger), used for card initialization,
 * card types...
 */
namespace SL
{
#if defined(CORE_TEENSY)
class SdCardType : public SdExFat
{
public:
    SdCardType() : SdExFat() {}; // NOLINT
    /**
     * @brief Initializes the SD card and sets up the SDIO interface.
     * @param sdioConf The configuration parameters for the SDIO interface.
     * @param serialMonitor A pointer to the serial monitor object for printing infos (not used for
     * teensy).
     * @return True if the SD card initialization is successful, false otherwise.
     */
    bool cardBegin(const SDIO_CONF& sdioConf, Print* serialMonitor)
    {
        return (this)->begin(SdioConfig(FIFO_SDIO));
    }
};
#elif defined(ESP_PLATFORM)
class SdCardType : public fs::SDMMCFS
{
public:
    SdCardType() : fs::SDMMCFS(FSImplPtr(new VFSImpl())) {}; // NOLINT

    /**
     * @brief Initializes the SD card and sets up the SDIO interface.
     * @param sdioConf The configuration parameters for the SDIO interface.
     * @param serialMonitor A pointer to the Print object used for serial printing.
     * @return True if the initialization is successful, false otherwise.
     */
    bool cardBegin(const SDIO_CONF& sdioConf, Print* serialMonitor)
    {
        if (!sdioConf.mIsValid)
            return false;

        pinMode(sdioConf.mPinSdioCmd, OUTPUT);
        this->setPins(sdioConf.mPinSdioClk,
                      sdioConf.mPinSdioCmd,
                      sdioConf.mPinSdioDat0,
                      sdioConf.mPinSdioDat1,
                      sdioConf.mPinSdioDat2,
                      sdioConf.mPinSdioDat3);
        if (!this->begin(
                    sdioConf.mMountpoint, false, sdioConf.mFormatIfMountFailed, sdioConf.mClock))
            return false;

        uint8_t cardType  = 0U;
        uint64_t cardSize = 0U;

        cardType = this->cardType();

        switch (cardType)
        {
        case CARD_NONE:
            serialMonitor->println("streamLogger: card is NONE");
            return false;
            break;
        case CARD_MMC:
            cardSize = this->cardSize() / (1024 * 1024);
            serialMonitor->println("streamLogger: card is MMC, size (MB) = " + ::String(cardSize));
            return true;
            break;
        case CARD_SD:
            cardSize = this->cardSize() / (1024 * 1024);
            serialMonitor->println("streamLogger: card is SDSC, size (MB) = " + ::String(cardSize));
            return true;
            break;
        case CARD_SDHC:
            cardSize = this->cardSize() / (1024 * 1024);
            serialMonitor->println("streamLogger: card is SDHC, size (MB) = " + ::String(cardSize));
            return true;
            break;
        default:
            serialMonitor->println("streamLogger: card is UNKNOWN type");
            return false;
            break;
        }
    }
};
#endif
} // namespace SL