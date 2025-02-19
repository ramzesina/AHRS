#ifndef tasks_hpp
#define tasks_hpp

#include "common.hpp"

// ====================================================== //
// ================= Tasks declarations ================= //
// ====================================================== //
// Asynchronous logging
void taskMain(void* pvParameters);
void taskCom(void* pvParameters);
void taskCalib(void* pvParameters);
void taskSecondary(void* pvParameters);
void taskDebug(void* pvParameters);
void taskSensorRef(void* pvParameters);

static TaskHandle_t TaskHandleMain;
static TaskHandle_t TaskHandleSend;
static TaskHandle_t TaskHandleCalib;
static TaskHandle_t TaskHandleSecondary;
static TaskHandle_t TaskHandleDebug;
static TaskHandle_t TaskHandleWriteLog;

// ====================================================== //
// ================= Tasks configuration ================ //
// ====================================================== //
// Use this configurations to configure tasks at compile time.

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
/**
 * @brief Structure representing the configuration of a task.
 */
struct TASKS_UNIT_CONFIG
{
    bool enable              = true; /**< Flag indicating whether the task is enabled or not. */
    UBaseType_t priority     = 0;    /**< Priority of the task. */
    uint32_t stackSize       = 2048; /**< Size of the task's stack. */
    uint32_t period          = 2U;   /**< Period of the task in milliseconds. */
    BaseType_t coreID        = 0;    /**< ID of the core on which the task should run. */
    TaskHandle_t* TaskHandle = NULL; /**< Pointer to the task handle. */

    /**
     * @brief Constructor for the TASKS_UNIT_CONFIG structure.
     * @param enable Flag indicating whether the task is enabled or not.
     * @param priority Priority of the task.
     * @param stackSize Size of the task's stack.
     * @param period Period of the task in milliseconds.
     * @param coreID ID of the core on which the task should run.
     * @param TaskHandle Pointer to the task handle.
     */
    TASKS_UNIT_CONFIG(bool enable,
                      UBaseType_t priority,
                      uint32_t stackSize,
                      uint32_t period,
                      BaseType_t coreID,
                      TaskHandle_t* TaskHandle) :
        enable(enable),
        priority(priority),
        stackSize(stackSize),
        period(period),
        coreID(coreID),
        TaskHandle(TaskHandle)
    {
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

/**
 * @brief Configuration structure for tasks.
 */
struct
{
    TASKS_UNIT_CONFIG taskMain  = TASKS_UNIT_CONFIG(true, 21, 2048 * 8, 2, 0, &TaskHandleMain);
    TASKS_UNIT_CONFIG taskCom   = TASKS_UNIT_CONFIG(false, 19, 2048 * 6, 5, 0, &TaskHandleSend);
    TASKS_UNIT_CONFIG taskCalib = TASKS_UNIT_CONFIG(false, 18, 2048 * 2, 2, 0, &TaskHandleCalib);
    TASKS_UNIT_CONFIG taskSecondary =
            TASKS_UNIT_CONFIG(true, 21, 2048 * 8, 20, 1, &TaskHandleSecondary);
    TASKS_UNIT_CONFIG taskWriteLog =
            TASKS_UNIT_CONFIG(true, 21, 2048 * 8, 2, 1, &TaskHandleWriteLog);
    TASKS_UNIT_CONFIG taskDebug = TASKS_UNIT_CONFIG(true, 1, 2048, 100, 1, &TaskHandleDebug);
} TASKS_CONFIG;

// ====================================================== //
// ================ Sensors configuration =============== //
// ====================================================== //
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)

/**
 * @class SENSOR_UNIT_CONFIG
 * @brief This class represents the configuration for a sensor.
 */
class SENSOR_UNIT_CONFIG
{
public:
    bool enable     = true;
    uint32_t period = 2U; // [ms]
    uint32_t ratioToTask =
            1U; // [] how many ratio of the current loop are necessary to require sensor parsing
    uint32_t timestampPrev    = 0U; // [us]
    uint32_t taskTicksCounter = 0U; // [] counts the number of task ticks since last sensor parsing

    /**
     * @brief Constructs a new SENSOR_UNIT_CONFIG object with the specified enable and period
     * values.
     * @param enable Whether the sensor unit task is enabled or not.
     * @param period The period of the sensor unit task in milliseconds.
     */
    SENSOR_UNIT_CONFIG(bool enable, uint32_t period) : enable(enable), period(period) {}

    /**
     * @brief Sets the ratio of the current loop necessary to require sensor parsing.
     * @param taskPeriod The period of the current loop in milliseconds.
     * @return True if the ratio was set successfully, false otherwise.
     */
    bool setRatio(uint32_t taskPeriod)
    {
        ratioToTask = 1U;

        if (taskPeriod <= period)
        {
            ratioToTask = period / taskPeriod;
            return true; // NOLINT(readability-simplify-boolean-expr)
        }

        return false;
    }

    /**
     * @brief Sets the period of the sensor unit task.
     * @param newPeriod The new period of the sensor unit task in milliseconds.
     */
    void setPeriod(uint32_t newPeriod)
    {
        period = newPeriod;
    }

    /**
     * @brief Checks if the sensor has to be parsed in the current loop.
     * @return True if the sensor has to be parsed, false otherwise.
     */
    bool checkRatio()
    {
        ++taskTicksCounter;
        if (taskTicksCounter >= ratioToTask)
            return true; // NOLINT(readability-simplify-boolean-expr)

        return false;
    }

    /**
     * @brief Resets the task ticks counter.
     */
    void resetRatioCounter()
    {
        taskTicksCounter = 0U;
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

/**
 * @brief Configuration structure for sensor units.
 */
struct
{
    SENSOR_UNIT_CONFIG sensorIMU  = SENSOR_UNIT_CONFIG(true, 2);   // 500Hz
    SENSOR_UNIT_CONFIG sensorMag  = SENSOR_UNIT_CONFIG(true, 10);  // 100Hz
    SENSOR_UNIT_CONFIG sensorBaro = SENSOR_UNIT_CONFIG(true, 14);  // 71.4Hz
    SENSOR_UNIT_CONFIG sensorGPS  = SENSOR_UNIT_CONFIG(true, 200); // 5Hz
} SENSORS_CONFIG;

// ====================================================== //
// ================= Methods definitions ================ //
// ====================================================== //
/**
 * @brief Initializes pins configuration
 */
void pinInit();

/**
 * @brief Initializes used protocols (SPI...)
 * @note The initialized protocols can be reconfigured by tasks
 */
void protocolInit();

/**
 * @brief Initializes rtos configuration (tasks...)
 */
void rtosSetup();

/**
 * @brief Initializes INS fusion
 */
void initializeModel();

/**
 * @brief Adds INS outputs to the logger
 */
void initializeModelLogger();

/**
 * @brief Adds sensors outputs to the logger
 */
void initializeSensorsLogger();

/**
 * @brief Adds various outputs to the logger
 */
void initializeUtilityLogger();

/**
 * @brief Starts the logger
 */
void startLogger();

#endif