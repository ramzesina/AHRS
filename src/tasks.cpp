#include "tasks.hpp"
#include <array>
#include <iterator>

#include "libDM_icm42688.hpp"
#include "libDM_mmc5983ma.hpp"
#include "libDM_ublox_gps.hpp"
#include "libMM_lps22hb.hpp"
#include "libDM_msg_center.hpp"
#include "libDM_SRX_10_DOF_interface.hpp"
#include "libDM_SRX_FFT64_interface.hpp"

// ====================================================== //
// ======================= Objects ====================== //
// ====================================================== //
ICM42688_SPI icm(generalConf.icmSpiClock,
                 pinConf.pinSS,
                 &streamObj,
                 &myTimer,
                 &SPI,
                 ABSTRACT_SENSOR_SPI::SPI_CONF(),
                 true,
                 false);
MMC5983MA_SPI mmc(generalConf.mmcSpiClock,
                  pinConf.pinMmcSS,
                  &streamObj,
                  &myTimer,
                  &SPI,
                  ABSTRACT_SENSOR_SPI::SPI_CONF());
MSG_CENTER msgCenter(&streamObj, &myTimer);
ubloxGPS GPS(&Serial1,
             generalConf.gpsBaudrate,
             &streamObj,
             &myTimer,
             pinConf.pinGpsRx,
             pinConf.pinGpsTx);
LPS22HB_SPI lps(10000000,
                pinConf.pinLpsSS,
                false,
                &streamObj,
                &myTimer,
                &SPI,
                ABSTRACT_SENSOR_SPI::SPI_CONF());
SRX_MODEL_INS_10_DOF ins;
SRX_MODEL_FFT fft;

// ====================================================== //
// ===================== Global vars ==================== //
// ====================================================== //

ABSTRACT_IMU::IMU_OUTPUT resIMU;
ABSTRACT_MAG::MAG_OUTPUT resMAG;
ABSTRACT_BARO::BARO_OUTPUT resBARO;

// Outputs from INS:
std::array<float, 4> OBS_localQuat{{1.F, 0.F, 0.F, 0.F}};
float OBS_dstAltitude = 0.F;
float OBS_spdUp       = 0.F;
std::array<float, 3> OBS_linearAccelerationLocal{{0.F, 0.F, 0.F}};
float OBS_yawLocalOffsetted = 0.F;
std::array<float, 3> OBS_yawPitchRollLocal{{0.F, 0.F, 0.F}};
std::array<float, 3> OBS_yawPitchRollGlobal{{0.F, 0.F, 0.F}};
float SUP_modelTimer   = 0.F;
uint8_t SUP_immobility = 0U;

// Time
float timeCore1 = 0.F;

// Global shared task variables:
bool mainTaskStarted = false;

// ##################################################################### //
// ########################## Threads content ########################## //
// ##################################################################### //
void taskMain(void* pvParameters) // NOLINT(misc-unused-parameters)
{
    TickType_t xLastWakeTime_Main = 0U;

    // ====================================================== //
    // ====================== Init ICM ====================== //
    // ====================================================== //
    if (SENSORS_CONFIG.sensorIMU.enable)
    {
        icm.softResetSensor();
        icm.begin();
        icm.setGyroOdrRange(ICM42688_ENUM::ODR::_1kHz, ICM42688_ENUM::GYRO_RANGE::_2000dps);
        SENSORS_CONFIG.sensorIMU.setPeriod(SENSORS_CONFIG.sensorIMU.period); // IMU period = 2ms
        SENSORS_CONFIG.sensorIMU.setRatio(TASKS_CONFIG.taskMain.period);     // Task period = 2ms
        streamObj.printlnUlog(true,
                              true,
                              "Main task: IMU task ratio = "
                                      + String(SENSORS_CONFIG.sensorIMU.ratioToTask));
        delay(1);
        icm.setAccOdrRange(ICM42688_ENUM::ODR::_1kHz, ICM42688_ENUM::ACC_RANGE::_16g);
        icm.setGyroAccelFilter(ICM42688_ENUM::FILTER_BW::ODR_DIV_4,
                               ICM42688_ENUM::FILTER_BW::ODR_DIV_4,
                               ICM42688_ENUM::FILTER_ORDER::_2nd_ORDER,
                               ICM42688_ENUM::FILTER_ORDER::_2nd_ORDER);
        delay(100);                                     // NOLINT
        if (icm.checkImmobility(0.3F, 200U, 1U, 1000U)) // NOLINT
            icm.calibGyro(2000);                        // NOLINT
        icm.flushFifo(true);
        delay(100); // NOLINT
    }

    // ====================================================== //
    // ====================== Init MMC ====================== //
    // ====================================================== //
    if (SENSORS_CONFIG.sensorMag.enable)
    {
        mmc.softResetSensor();
        delay(10); // NOLINT
        mmc.begin();
        mmc.setMagOdrBw(MMC5983MA_ENUM::ODR::_100HZ, MMC5983MA_ENUM::BW::_200HZ);
        SENSORS_CONFIG.sensorMag.setPeriod(SENSORS_CONFIG.sensorMag.period); // MAG period = 10ms
        SENSORS_CONFIG.sensorMag.setRatio(TASKS_CONFIG.taskMain.period);     // Task period = 2ms
        streamObj.printlnUlog(true,
                              true,
                              "Main task: MAG task ratio = "
                                      + String(SENSORS_CONFIG.sensorMag.ratioToTask));
        mmc.setAutoSetResetState(true);
    }

    // ====================================================== //
    // ====================== Init LPS ====================== //
    // ====================================================== //
    if (SENSORS_CONFIG.sensorBaro.enable)
    {
        lps.begin();
        lps.setOdr(LPS22HB_ENUM::ODR::_75HZ);
        delay(100);                                                            // NOLINT
        SENSORS_CONFIG.sensorBaro.setPeriod(SENSORS_CONFIG.sensorBaro.period); // BARO period = 14ms
        SENSORS_CONFIG.sensorBaro.setRatio(TASKS_CONFIG.taskMain.period);      // Task period = 2ms
        streamObj.printlnUlog(true,
                              true,
                              "Main task: BARO task ratio = "
                                      + String(SENSORS_CONFIG.sensorBaro.ratioToTask));
        // lps.setLowPassFilterState(LPS22HB_ENUM::FILTER_BW::ODR_DIV_20);
        lps.configureReferenceParameters(
                ABSTRACT_BARO_ENUM::BARO_VALUES::INVALID_PRESSURE,
                ABSTRACT_BARO_ENUM::BARO_VALUES::DEFAULT_SEA_LEVEL_TEMPERATURE,
                ABSTRACT_BARO_ENUM::BARO_VALUES::INVALID_ALTITUDE);
    }

    // ====================================================== //
    // ================ Init INS fusion code ================ //
    // ====================================================== //

    SRX_MODEL_INS_10_DOF::modelInputs insInputs = SRX_MODEL_INS_10_DOF::modelInputs();
    insInputs.reset                             = false;
    ins.setInputs(insInputs);

    // ====================================================== //
    streamObj.printlnUlog(true, true, "Main task: Started");
    mainTaskStarted    = true;
    xLastWakeTime_Main = xTaskGetTickCount(); // Init schedule timer for this task
    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime_Main, TASKS_CONFIG.taskMain.period);
        xLastWakeTime_Main = xTaskGetTickCount(); // To avoid passed blocks, extremely important!!

        timeCore1 = myTimer.returnSystemCounterSecondsFloat();

        // Read primary sensors
        // ~~~~~~~~~~~~~~~~~ ICM ~~~~~~~~~~~~~~~~~ //

        icm.readFifo();
        resIMU = icm.getOutput();

		///////////////////////////////////////////////////////////////////////////////////
		//*******************************************************************************
		// Вивід даних IMU на Serial
		/*
		Serial.print("IMU: Gyro (X,Y,Z): ");
		Serial.print(resIMU.dpsGyroData[0], 2); Serial.print(", ");
		Serial.print(resIMU.dpsGyroData[1], 2); Serial.print(", ");
		Serial.println(resIMU.dpsGyroData[2], 2);
		Serial.print("IMU: Accel (X,Y,Z): ");
		Serial.print(resIMU.gAccelData[0], 2); Serial.print(", ");
		Serial.print(resIMU.gAccelData[1], 2); Serial.print(", ");
		Serial.println(resIMU.gAccelData[2], 2);
		*/
		//*******************************************************************************
		///////////////////////////////////////////////////////////////////////////////////

        // Feed INS
        insInputs.gyroDpsRate[0] = -resIMU.dpsGyroData[1];
        insInputs.gyroDpsRate[1] = -resIMU.dpsGyroData[0];
        insInputs.gyroDpsRate[2] = -resIMU.dpsGyroData[2];
        insInputs.acceleroG[0]   = -resIMU.gAccelData[1];
        insInputs.acceleroG[1]   = -resIMU.gAccelData[0];
        insInputs.acceleroG[2]   = -resIMU.gAccelData[2];
        insInputs.timestampUs    = myTimer.returnSystemTimestampUs64();

        // ~~~~~~~~~~~~~~~~~ MMC ~~~~~~~~~~~~~~~~~ //
        if (SENSORS_CONFIG.sensorMag.checkRatio())
        {
            mmc.readMag();
            resMAG = mmc.getOutput();
            if (resMAG.usMagTimestamp != SENSORS_CONFIG.sensorMag.timestampPrev)
            {
                SENSORS_CONFIG.sensorMag.timestampPrev = resMAG.usMagTimestamp;
                SENSORS_CONFIG.sensorMag.resetRatioCounter();

				///////////////////////////////////////////////////////////////////////////////////
				//*******************************************************************************
				// Вивід даних магнітометра на Serial
				/*
				Serial.print("MAG: (X,Y,Z): ");
				Serial.print(resMAG.GMagData[0], 2); Serial.print(", ");
				Serial.print(resMAG.GMagData[1], 2); Serial.print(", ");
				Serial.println(resMAG.GMagData[2], 2);
				*/
				//*******************************************************************************
				///////////////////////////////////////////////////////////////////////////////////

                // Feed INS
                insInputs.magGauss[0]    = -resMAG.GMagData[0];
                insInputs.magGauss[1]    = resMAG.GMagData[1];
                insInputs.magGauss[2]    = resMAG.GMagData[2];
                insInputs.magTimestampUs = resMAG.usMagTimestamp;
            }
        }

        // ~~~~~~~~~~~~~~~ LPS22HB ~~~~~~~~~~~~~~~ //
        if (SENSORS_CONFIG.sensorBaro.checkRatio())
        {
            lps.readTemperaturePressureAltitude();
            resBARO = lps.getOutput();

            if (resBARO.usPressureTimestamp != SENSORS_CONFIG.sensorBaro.timestampPrev)
            {
                SENSORS_CONFIG.sensorBaro.timestampPrev = resBARO.usPressureTimestamp;
                SENSORS_CONFIG.sensorBaro.resetRatioCounter();

				///////////////////////////////////////////////////////////////////////////////////
				//*******************************************************************************
				// Вивід даних магнітометра на Serial
				/*
				Serial.print("BARO: Altitude: ");
                Serial.println(resBARO.mRelativeAltitude, 2);
				*/
				//*******************************************************************************
				///////////////////////////////////////////////////////////////////////////////////

                // Feed INS
                insInputs.barometerAltitudeAmsl = resBARO.mAbsoluteAltitude;
                insInputs.barometerAltitudeRel  = resBARO.mRelativeAltitude;
                insInputs.barometerTimestampUs  = resBARO.usPressureTimestamp;
            }
        }

        ins.setInputs(insInputs);
        ins.step();

        // ~~~~~~~~~~~~~ Serial send ~~~~~~~~~~~~~ //

		///////////////////////////////////////////////////////////////////////////////////
		//*******************************************************************************
        // Вивід результату фузіону на Serial
        Serial.print("INS: Quat (W,X,Y,Z): ");
        Serial.print(ins.getOutputs().localToBodyQuat[0], 2); Serial.print(", ");
        Serial.print(ins.getOutputs().localToBodyQuat[1], 2); Serial.print(", ");
        Serial.print(ins.getOutputs().localToBodyQuat[2], 2); Serial.print(", ");
        Serial.print(ins.getOutputs().localToBodyQuat[3], 2); Serial.println(); // Замість println() для останнього числа

        // Для dstAltitude
        float altitude = *(ins.getOutputs().dstAltitude); // Додано розпакування вказівника
        Serial.print("INS: Altitude: ");
        Serial.print(altitude, 2); Serial.println(); // Тепер можемо форматувати

        // Для spdUp
        float speedUp = *(ins.getOutputs().spdUp); // Аналогічно для spdUp
        Serial.print("INS: Velocity Up: ");
        Serial.print(speedUp, 2); Serial.println(); // Форматування
/*
        // Вивід глобального курсу
		Serial.print("INS: Yaw (Course) Global: ");
		Serial.println(OBS_yawPitchRollGlobal[0], 2);

		Serial.print("INS: Yaw, Pitch, Roll Global: ");
		Serial.print(OBS_yawPitchRollGlobal[0], 2); Serial.print(", "); // Yaw
		Serial.print(OBS_yawPitchRollGlobal[1], 2); Serial.print(", "); // Pitch
		Serial.println(OBS_yawPitchRollGlobal[2], 2); // Roll

		Serial.print("IMU: Gyro (X,Y,Z): ");
		Serial.print(resIMU.dpsGyroData[0], 2); Serial.print(", ");
		Serial.print(resIMU.dpsGyroData[1], 2); Serial.print(", ");
		Serial.println(resIMU.dpsGyroData[2], 2);
*/
		///////////////////////////////////////////////////////////////////////////////////

        // ~~~~~~~~~~~~~~~~~ SD ~~~~~~~~~~~~~~~~~ //
        streamObj.updateLogBuffer(); // The fastest and most prioritary task updates buffers
    }
    vTaskDelete(NULL);
}

void taskCom(void* pvParameters) // NOLINT(misc-unused-parameters)
{
    TickType_t xLastWakeTime_Send = 0U;

    // ====================================================== //
    // ================ Configure msg_center ================ //
    // ====================================================== //
    delay(1000); // NOLINT
    msgCenter.begin();
    // Grab configuration struct and messages
    MSG_CENTER_STRUCTS::MSG_CENTER_CONFIG* msgCenterEmissionConfig = msgCenter.getEmissionConfig();
    MSG_CENTER_STRUCTS::MSG_CENTER_CONFIG* msgCenterReceptionConfig =
            msgCenter.getReceptionConfig();
    MSG_CENTER_STRUCTS::MSG_CENTER_MSG* msgCenterEmissionMessages = msgCenter.getMessagesToSend();
    MSG_CENTER_STRUCTS::MSG_CENTER_MSG* msgCenterReceptionMessages =
            msgCenter.getReceivedMessages();

    // Attach hardware interfaces
    msgCenter.attachHardwareInterface(
            &Serial2, 921600, pinConf.pinProtocolRx, pinConf.pinProtocolTx);

    // Configure hardware interface
    msgCenter.setProtocolHardwareInterface(MSG_CENTER_STRUCTS::PROTOCOL::PROTOCOL_PROTOBUF,
                                           MSG_CENTER_STRUCTS::HW_INTERFACE::INT_SERIAL);

    // By default only protobuf is used for data broadcast

    // Activate messages reception
    msgCenterReceptionConfig->PROTOBUF_MSG_STATE.MSG = true;
    msgCenter.updateReceivedMessagesState(MSG_CENTER_STRUCTS::PROTOCOL::PROTOCOL_PROTOBUF);

    // Schedule messages sending (5ms ticks)
    timerTool::counterAndThreshold counterMsgProtobuf(5U, 100U); // 100ms, 10Hz

    while (!mainTaskStarted)
        delay(1);
    streamObj.printlnUlog(true, true, "Send task: Started");
    xLastWakeTime_Send = xTaskGetTickCount(); // Init schedule timer for this task

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime_Send, TASKS_CONFIG.taskCom.period);
        xLastWakeTime_Send = xTaskGetTickCount(); // To avoid passed blocks, extremely important!!

        // ~~~~~~~~~~~~~~ MSG_CENTER ~~~~~~~~~~~~~ //
        // Emission
        msgCenterEmissionConfig->reset();

        if (counterMsgProtobuf.step(true))
        {
            msgCenterEmissionMessages->PROTOBUF_MSG.MSG->resetFrame();
            ProtoFrame& frame = msgCenterEmissionMessages->PROTOBUF_MSG.MSG->getFrame();
            // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
            frame.has_attitude                           = true;
            frame.attitude.has_attitude_local            = true;
            frame.attitude.attitude_local.has_quaternion = true;
            frame.attitude.attitude_local.quaternion.qw  = 0.9F;
            frame.attitude.attitude_local.quaternion.qx  = 0.1F;
            frame.attitude.attitude_local.quaternion.qy  = 0.2F;
            frame.attitude.attitude_local.quaternion.qz  = 0.3F;

            frame.attitude.has_body_rates     = true;
            frame.attitude.body_rates.has_pqr = true;
            frame.attitude.body_rates.pqr.x   = 0.4F;
            frame.attitude.body_rates.pqr.y   = 0.5F;
            frame.attitude.body_rates.pqr.z   = 0.6F;

            // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

            msgCenterEmissionConfig->PROTOBUF_MSG_STATE.MSG = true;
        }

        msgCenter.sendMessages();

        // Reception
        msgCenter.receiveMessages();

        if (msgCenterReceptionMessages->PROTOBUF_MSG.getIsNewMsg())
        {
            Serial.println("PROTOBUF msg received");
            ProtoFrame& frame = msgCenterReceptionMessages->PROTOBUF_MSG.MSG->getFrame();
            if (frame.has_attitude)
            {
                if (frame.attitude.has_attitude_local)
                {
                    if (frame.attitude.attitude_local.has_quaternion)
                    {
                        Serial.println("PROTOBUF quaternion received");
                        Serial.println(
                                "qw = " + String(frame.attitude.attitude_local.quaternion.qw)
                                + " qx = " + String(frame.attitude.attitude_local.quaternion.qx)
                                + " qy = " + String(frame.attitude.attitude_local.quaternion.qy)
                                + " qz = " + String(frame.attitude.attitude_local.quaternion.qz));
                    }
                }
                if (frame.attitude.has_body_rates)
                {
                    if (frame.attitude.body_rates.has_pqr)
                    {
                        Serial.println("PROTOBUF pqr received");
                        Serial.println("p = " + String(frame.attitude.body_rates.pqr.x)
                                       + " q = " + String(frame.attitude.body_rates.pqr.y)
                                       + " r = " + String(frame.attitude.body_rates.pqr.z));
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void taskCalib(void* pvParameters) // NOLINT(misc-unused-parameters)
{
    TickType_t xLastWakeTime_Calib = 0U;

    // Calibrations config
    const uint32_t magCalibDuration = 30000U; // [ms]

    // Wait for main task sensors init
    while (!mainTaskStarted)
        delay(1);

    streamObj.printlnUlog(true, true, "Calib task: Started");
    streamObj.printlnUlog(true,
                          true,
                          "Calib task: Will suspend other task0 tasks."
                          "They will be restored, at the end of the process");
    vTaskSuspend(TaskHandleMain);
    delay(1);

    xLastWakeTime_Calib = xTaskGetTickCount(); // Init schedule timer for this task

    // ====================================================== //
    // ====================== MAG calib ===================== //
    // ====================================================== //
    if (mmc.ABSTRACT_MAG::checkBeginState())
    {
        bool calibrationRes = mmc.calibMag(magCalibDuration);
        streamObj.printlnUlog(true,
                              true,
                              "Calib task: Magnetometer calibration result = "
                                      + String(calibrationRes));
    }
    else
    {
        streamObj.printlnUlog(
                true,
                true,
                "Calib task: Magnetometer calibration failed as sensor is not detected");
    }

    vTaskDelete(NULL);
}

void taskSecondary(void* pvParameters) // NOLINT(misc-unused-parameters)
{
    TickType_t xLastWakeTime_Log = 0U;

    // Ublox GPS init
    GPS.begin();
    SENSORS_CONFIG.sensorGPS.setPeriod(SENSORS_CONFIG.sensorGPS.period);  // GPS period = 200ms
    SENSORS_CONFIG.sensorGPS.setRatio(TASKS_CONFIG.taskSecondary.period); // Task period = 20ms
    streamObj.printlnUlog(true,
                          true,
                          "Secondary task: GPS task ratio = "
                                  + String(SENSORS_CONFIG.sensorGPS.ratioToTask));

    while (!mainTaskStarted)
        delay(1);
    streamObj.printlnUlog(true, true, "Secondary task: Started");

    xLastWakeTime_Log = xTaskGetTickCount(); // Init schedule timer for this task

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime_Log, TASKS_CONFIG.taskSecondary.period);
        xLastWakeTime_Log = xTaskGetTickCount(); // To avoid passed blocks, extremely important!!

        // Read slow sensors
        // ~~~~~~~~~~~~~~~~~ GPS ~~~~~~~~~~~~~~~~~ //
        if (SENSORS_CONFIG.sensorGPS.checkRatio())
        {
            GPS.getNavPvtMeasures();
            if (GPS.NAV_PVT.timestamp != SENSORS_CONFIG.sensorGPS.timestampPrev)
            {
                SENSORS_CONFIG.sensorGPS.timestampPrev = GPS.NAV_PVT.timestamp;
                SENSORS_CONFIG.sensorGPS.resetRatioCounter();
            }
        }

        // ~~~~~~~~~~~~~~~ Sd card ~~~~~~~~~~~~~~~ //

        // streamObj.writeLog(); TODO: now done by a special task
    }
    vTaskDelete(NULL);
}

void taskDebug(void* pvParameters) // NOLINT(misc-unused-parameters)
{
    TickType_t xLastWakeTime_Debug = 0U;

    while (!mainTaskStarted)
        delay(1);
    streamObj.printlnUlog(true, true, "Debug task: Started");
    xLastWakeTime_Debug = xTaskGetTickCount(); // Init schedule timer for this task

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime_Debug, TASKS_CONFIG.taskDebug.period);
        xLastWakeTime_Debug = xTaskGetTickCount(); // To avoid passed blocks, extremely important!!
        // streamObj.printlnUlog(true, true, "Debug task: Hello world");
    }
    vTaskDelete(NULL);
}

void taskWriteLog(void* pvParameters) // NOLINT(misc-unused-parameters)
{
    streamObj.setWriteLogTask(TaskHandleWriteLog); // Write logs every time size threshold is
                                                   // reached (using default lib value)
    while (!mainTaskStarted)
        delay(1);
    streamObj.printlnUlog(true, true, "Write log task: Started");

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        streamObj.writeLog();
    }
    vTaskDelete(NULL);
}

// ====================================================== //
// ======================== Setup ======================= //
// ====================================================== //
void pinInit()
{
    pinMode(pinConf.pinSS, OUTPUT);
    pinMode(pinConf.pinLpsSS, OUTPUT);
    pinMode(pinConf.pinMmcSS, OUTPUT);
    digitalWrite(pinConf.pinLpsSS, HIGH);
    digitalWrite(pinConf.pinSS, HIGH);
    digitalWrite(pinConf.pinMmcSS, HIGH);
}

void protocolInit()
{
    // SPI init
    SPI.begin(pinConf.pinClk, pinConf.pinMiso, pinConf.pinMosi, pinConf.pinSS);
    delay(1);
}

void rtosSetup()
{
    // ====================================================== //
    // ======================= Core 0 ======================= //
    // ====================================================== //
    /**
     * @brief Main task
     * @details This task is responsible for the main loop of the program: reading sensors and
     * fusing the data.
     */
    if (TASKS_CONFIG.taskMain.enable)
        xTaskCreatePinnedToCore(taskMain,   /* Function to implement the task */
                                "taskMain", /* Name of the task */
                                TASKS_CONFIG.taskMain.stackSize,  /* Stack size in words */
                                NULL,                             /* Task input parameter */
                                TASKS_CONFIG.taskMain.priority,   /* Priority of the task */
                                TASKS_CONFIG.taskMain.TaskHandle, /* Task handle. */
                                TASKS_CONFIG.taskMain.coreID); /* Core where the task should run */

    /**
     * @brief Send task
     * @details This task is responsible for primary data streaming (serial, can)
     */
    if (TASKS_CONFIG.taskCom.enable)
        xTaskCreatePinnedToCore(taskCom,                        /* Function to implement the task */
                                "taskCom",                      /* Name of the task */
                                TASKS_CONFIG.taskCom.stackSize, /* Stack size in words */
                                NULL,                           /* Task input parameter */
                                TASKS_CONFIG.taskCom.priority,  /* Priority of the task */
                                TASKS_CONFIG.taskCom.TaskHandle, /* Task handle. */
                                TASKS_CONFIG.taskCom.coreID); /* Core where the task should run */

    /**
     * @brief Calibration task
     * @details This task is responsible for sensor calibration. This will suspend other core 0
     tasks during execution.
     */
    if (TASKS_CONFIG.taskCalib.enable)
        xTaskCreatePinnedToCore(taskCalib,   /* Function to implement the task */
                                "taskCalib", /* Name of the task */
                                TASKS_CONFIG.taskCalib.stackSize,  /* Stack size in words */
                                NULL,                              /* Task input parameter */
                                TASKS_CONFIG.taskCalib.priority,   /* Priority of the task */
                                TASKS_CONFIG.taskCalib.TaskHandle, /* Task handle. */
                                TASKS_CONFIG.taskCalib.coreID); /* Core where the task should run */

    // ====================================================== //
    // ======================= Core 1 ======================= //
    // ====================================================== //
    /**
     * @brief Task for slower rate operations (logging, reading slow sensors...)
     * @details This task is responsible for logging data to the SD card, reading slow sensors
     * (GPS...)
     */
    if (TASKS_CONFIG.taskSecondary.enable)
        xTaskCreatePinnedToCore(
                taskSecondary,                         /* Function to implement the task */
                "taskSecondary",                       /* Name of the task */
                TASKS_CONFIG.taskSecondary.stackSize,  /* Stack size in words */
                NULL,                                  /* Task input parameter */
                TASKS_CONFIG.taskSecondary.priority,   /* Priority of the task */
                TASKS_CONFIG.taskSecondary.TaskHandle, /* Task handle. */
                TASKS_CONFIG.taskSecondary.coreID);    /* Core where the task should run */
    // TODO: update descriptions
    /**
     * @brief Task for slower rate operations (logging, reading slow sensors...)
     * @details This task is responsible for logging data to the SD card, reading slow sensors
     * (GPS...)
     */
    if (TASKS_CONFIG.taskWriteLog.enable)
        xTaskCreatePinnedToCore(
                taskWriteLog,                         /* Function to implement the task */
                "taskWriteLog",                       /* Name of the task */
                TASKS_CONFIG.taskWriteLog.stackSize,  /* Stack size in words */
                NULL,                                 /* Task input parameter */
                TASKS_CONFIG.taskWriteLog.priority,   /* Priority of the task */
                TASKS_CONFIG.taskWriteLog.TaskHandle, /* Task handle. */
                TASKS_CONFIG.taskWriteLog.coreID);    /* Core where the task should run */

    /**
     * @brief Debug task
     * @details Slow rate task to print debug information
     */
    if (TASKS_CONFIG.taskDebug.enable)
        xTaskCreatePinnedToCore(taskDebug,   /* Function to implement the task */
                                "taskDebug", /* Name of the task */
                                TASKS_CONFIG.taskDebug.stackSize,  /* Stack size in words */
                                NULL,                              /* Task input parameter */
                                TASKS_CONFIG.taskDebug.priority,   /* Priority of the task */
                                TASKS_CONFIG.taskDebug.TaskHandle, /* Task handle. */
                                TASKS_CONFIG.taskDebug.coreID); /* Core where the task should run */

    String activationString =
            "Activated tasks state: \ntaskMain: " + String(TASKS_CONFIG.taskMain.enable)
            + "\ntaskCom: " + String(TASKS_CONFIG.taskCom.enable)
            + "\ntaskCalib: " + String(TASKS_CONFIG.taskCalib.enable)
            + "\ntaskSecondary: " + String(TASKS_CONFIG.taskSecondary.enable)
            + "\ntaskDebug: " + String(TASKS_CONFIG.taskDebug.enable);

    streamObj.printlnUlog(true, true, activationString);

    activationString = "Activated sensors: \nIMU: " + String(SENSORS_CONFIG.sensorIMU.enable)
                       + "\nBARO: " + String(SENSORS_CONFIG.sensorBaro.enable)
                       + "\nMAG: " + String(SENSORS_CONFIG.sensorMag.enable)
                       + "\nGPS: " + String(SENSORS_CONFIG.sensorGPS.enable);

    streamObj.printlnUlog(true, true, activationString);
}

void initializeModel()
{
    ins.initialize();
}

void initializeModelLogger()
{
    streamObj.addVariable(ins.getOutputs().localToBodyQuat, "OBS_localQuatW");
    streamObj.addVariable(ins.getOutputs().localToBodyQuat + 1, "OBS_localQuatX");
    streamObj.addVariable(ins.getOutputs().localToBodyQuat + 2, "OBS_localQuatY");
    streamObj.addVariable(ins.getOutputs().localToBodyQuat + 3, "OBS_localQuatZ");

    streamObj.addVariable(ins.getOutputs().dstAltitude, "OBS_dstAltitude");
    streamObj.addVariable(ins.getOutputs().spdUp, "OBS_spdUp");

    streamObj.addVariable(ins.getOutputs().linearAccelerationLocal, "OBS_linearAccelerationLocalX");
    streamObj.addVariable(ins.getOutputs().linearAccelerationLocal + 1,
                          "OBS_linearAccelerationLocalY");
    streamObj.addVariable(ins.getOutputs().linearAccelerationLocal + 2,
                          "OBS_linearAccelerationLocalZ");

    streamObj.addVariable(ins.getOutputs().yawLocalOffsetted, "OBS_yawLocalOffsetted");

    streamObj.addVariable(ins.getOutputs().yawPitchRollLocal, "OBS_yawLocal");
    streamObj.addVariable(ins.getOutputs().yawPitchRollLocal + 1, "OBS_pitchLocal");
    streamObj.addVariable(ins.getOutputs().yawPitchRollLocal + 2, "OBS_rollLocal");

    streamObj.addVariable(ins.getOutputs().yawPitchRollGlobal, "OBS_yawGlobal");
    streamObj.addVariable(ins.getOutputs().yawPitchRollGlobal + 1, "OBS_pitchGlobal");
    streamObj.addVariable(ins.getOutputs().yawPitchRollGlobal + 2, "OBS_rollGlobal");

    streamObj.addVariable(ins.getOutputs().modelTimer, "SUP_modelTimer");
    streamObj.addVariable(ins.getOutputs().immobility, "SUP_bImmobility");

    streamObj.addVariable(&timeCore1, "t");
}

void initializeSensorsLogger()
{
    // Onboard sensors:
    streamObj.addVariable(&resIMU.dpsGyroData[0], "CAP_gyroX");
    streamObj.addVariable(&resIMU.dpsGyroData[1], "CAP_gyroY");
    streamObj.addVariable(&resIMU.dpsGyroData[2], "CAP_gyroZ");

    streamObj.addVariable(&resIMU.gAccelData[0], "CAP_accX");
    streamObj.addVariable(&resIMU.gAccelData[1], "CAP_accY");
    streamObj.addVariable(&resIMU.gAccelData[2], "CAP_accZ");

    streamObj.addVariable(&resMAG.GMagData[0], "CAP_magX");
    streamObj.addVariable(&resMAG.GMagData[1], "CAP_magY");
    streamObj.addVariable(&resMAG.GMagData[2], "CAP_magZ");
    streamObj.addVariable(&resMAG.usMagTimestamp, "CAP_magTimestamp");

    streamObj.addVariable(&resBARO.mRelativeAltitude, "CAP_baroAltitude");
    streamObj.addVariable(&resBARO.usPressureTimestamp, "CAP_baroTimestamp");
}

void initializeUtilityLogger() {}

void startLogger()
{
    streamObj.begin(); // Has to be begun after SPI init
}
