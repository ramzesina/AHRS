#include "common.hpp"

// ====================================================== //
// ================== Config structures ================= //
// ====================================================== //

const PIN_CONFIG pinConf         = PIN_CONFIG();     // NOLINT
const GENERAL_CONFIG generalConf = GENERAL_CONFIG(); // NOLINT

// ====================================================== //
// ============== Global object definitions ============= //
// ====================================================== //

timerTool myTimer;
SDIO_CONF sdioConf(generalConf.sdioClock,
                   pinConf.pinSdioClk,
                   pinConf.pinSdioCmd,
                   pinConf.pinSdioDat0,
                   pinConf.pinSdioDat1,
                   pinConf.pinSdioDat2,
                   pinConf.pinSdioDat3,
                   generalConf.sdioMountpoint,
                   false);
streamLogger streamObj(&myTimer, sdioConf);