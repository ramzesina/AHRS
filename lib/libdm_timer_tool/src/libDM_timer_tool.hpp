// Simplified version of teensy timer tool.
// Great infos:
// https://github.com/TeensyUser/doc/wiki/implementing-a-high-resolution-teensy-clock#the-periodic-timer-of-the-real-time-clock
// https://github.com/TeensyUser/doc/wiki/Using-The-Cycle-Counter

#ifndef timerTool_hpp
#define timerTool_hpp

#include <Arduino.h>
#include <array>

#if defined(CORE_TEENSY)
#    define TIMERTOOL_GET_CYCLES_FCT ARM_DWT_CYCCNT
#    define CLK_FREQ                 F_CPU
#elif defined(ESP_PLATFORM)
#    define TIMERTOOL_GET_CYCLES_FCT xthal_get_ccount()
#    define CLK_FREQ                 F_CPU
#endif

// Multithread access to the timerTool class can lead to wrong wrapping detection.
constexpr uint32_t WRAPPING_VALUE_THRESHOLD    = 10000000; // 10s
constexpr uint32_t WRAPPING_VALUE_MIN_OLD_VAL  = 0xFFFFFFFF - WRAPPING_VALUE_THRESHOLD;
constexpr uint32_t WRAPPING_VALUE_MIN_LOW_PART = WRAPPING_VALUE_THRESHOLD;

class timerTool
{
public:
    timerTool() = default;

    void begin(Print* serialMonitor = &Serial)
    {
#if defined(CORE_TEENSY)
        ARM_DEMCR |= ARM_DEMCR_TRCENA;
        ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
#endif
        curSerialMonitor = serialMonitor;
    }

    double returnSystemCounterSecondsDouble();
    float returnSystemCounterSecondsFloat();
    double returnSystemCounterNs();
    uint64_t returnSystemCounterUsU64();
    double returnsystemCounterUs();
    uint64_t returnInternalCounter();

    // Efficient timestamping function:
    uint32_t returnSystemTimestampUs();   // Wrap in 1hour11mns
    uint64_t returnSystemTimestampUs64(); // Wrap in 584942 years
    uint32_t returnSystemTimestampMs();

    // Measure intervals:
    void startTiming(uint8_t idx = 0);                      // measure exec time start. Till 20 meas
    double stopTiming(uint8_t idx = 0, bool print = false); // measure exec time stop. Till 20 meas

    // delay functions
    void delayNanosecondsAlt(uint64_t delay);

    // ====================================================== //
    // ================== Secondary classes ================= //
    // ====================================================== //
    /**
     * @brief Simple counter and threshold class.
     * @details Can be used for simple scheduling tasks.
     */
    class counterAndThreshold
    {
    public:
        /**
         * @brief Constructs a counterAndThreshold object with the given threshold.
         * @param threshold The threshold value to set for the object.
         */
        explicit counterAndThreshold(uint32_t threshold) : mThreshold(threshold) {}
        /**
         * @brief Constructs a counterAndThreshold object with autocoputed threshold.
         * @param loopDuration The duration of the loop in milliseconds.
         * @param thresholdMs The threshold duration in milliseconds.
         * @return True if the loop duration exceeds the threshold, false otherwise.
         */
        counterAndThreshold(uint32_t loopDuration, uint32_t thresholdMs)
        {
            mThreshold = thresholdMs / loopDuration;
        }

        /**
         * @brief Executes a single step of the task.
         * @warning If using the boolean return to determine if the threshold has been reached, the
         * user is responsible to call reset() after the threshold has been reached. Another way is
         * to use autoreset option.
         * @param autoreset If true, the counter will be reset when the threshold is reached.
         * @return true if the task should continue running, false if it should stop.
         */
        inline bool step(bool autoreset = false)
        {
            mCounter++;
            if (!mHasReachedThreshold && mCounter >= mThreshold)
                mHasReachedThreshold = true;

            const bool res = mHasReachedThreshold;
            if (autoreset && mHasReachedThreshold)
                reset();

            return res;
        }

        /**
         * @brief Sets the threshold value for the task.
         * @param newThreshold The new threshold value to set.
         */
        inline void setThreshold(uint32_t newThreshold)
        {
            mThreshold = newThreshold;
        }

        /**
         * @brief Checks if the trigger condition for this task has been met.
         * @return true if the trigger condition has been met, false otherwise.
         */
        inline bool hasTriggered()
        {
            const bool res = mHasReachedThreshold;
            if (res)
                reset();

            return res;
        }

        /**
         * @brief Resets the object to its initial state.
         */
        inline void reset()
        {
            mCounter             = 0U;
            mHasReachedThreshold = false;
        }

    private:
        uint32_t mCounter   = 0U;
        uint32_t mThreshold = 0U;

        bool mHasReachedThreshold = false;
    };

private:
    void updateAllCounters();
    uint64_t getTimer64();

    // Internal constants. Warning!! The value is valid only if a function above has been called as
    // updateAllCounters() is called inside of each one.
    uint64_t internalCounter          = 0;
    double systemCounterSecondsDouble = 0.0;
    float systemCounterSecondsFloat   = 0.F;
    double systemCounterNs            = 0.0;
    uint64_t systemCounterUsU64       = 0;
    double systemCounterUs            = 0.0;

    Print* curSerialMonitor = NULL;

    // Const
    const double INVERT_F_CPU_DOUBLE = 1.0 / static_cast<double>(CLK_FREQ);
    const float INVERT_F_CPU_FLOAT   = 1.F / static_cast<float>(CLK_FREQ);
    const double COUNTER_2_NS_DOUBLE = 1e9 / static_cast<double>(CLK_FREQ);
    const double COUNTER_2_US_DOUBLE = static_cast<double>(1e6 / static_cast<double>(CLK_FREQ));
    const float COUNTER_2_US_FLOAT   = 1e6F / static_cast<float>(CLK_FREQ);
    const uint32_t UINT_US_2_COUNTER = CLK_FREQ / 1000000;    // To have big integer number
    const uint64_t COUNTER_2_NS_UINT = 1000000000 / CLK_FREQ; // Valid as long as CLK_FREQ < 1GHz
    const float FLOAT_US_TO_SECONDS  = 1e-6F;

    // NOLINTBEGIN(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    using val64_reg_t = struct
    {
        union
        {
            struct
            {
                uint32_t lowPart;
                uint32_t highPart;
            };
            uint64_t val64 = 0ULL;
        };
        uint32_t oldVal = 0U;
    };
    // NOLINTEND(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    // The following tip is used to avoid division for delay nanoseconds. Instead of doing
    // delay/COUNTER_2_NS_UINT, we compute what power of 2 is COUNTER_2_NS_UINT and right shift
    // delay by this number. This is not exact but way way faster and works great with esp32 because
    // F_CPU = 240000000 ~= 1000000000 / (2*2)
    const uint32_t RIGHT_SHIFT_VAL_COUNTER_2_NS_UINT = log2(COUNTER_2_NS_UINT);

    static const uint8_t maxIntervalMeasures = 20;

    std::array<uint64_t, maxIntervalMeasures> internalCounterPrev{{0}};
    // timer calibration values

    // To calibrate with interval timer. Consider it as a perfect 1000us timer and call:
    // myTimer.stopTiming();
    // myTimer.startTiming();
    // Adjust timing duration until it gives 1000000.00

#if defined(CORE_TEENSY)
    const double timingFunctionDuration                = 1358; // ns
    const double timingFunctionDurationOffsetWithPrint = 3095; // ns
#elif defined(ESP_PLATFORM)
    // TODO: measure those value
    const double timingFunctionDuration                = 0; // ns
    const double timingFunctionDurationOffsetWithPrint = 0; // ns
#else
    const double timingFunctionDuration                = 0; // ns
    const double timingFunctionDurationOffsetWithPrint = 0; // ns
#endif
};

#endif