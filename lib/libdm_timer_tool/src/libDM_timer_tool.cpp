#include "libDM_timer_tool.hpp"

uint64_t timerTool::getTimer64()
{
    static uint32_t oldCycles = TIMERTOOL_GET_CYCLES_FCT;
    static uint32_t highDWORD = 0;

    uint32_t newCycles = TIMERTOOL_GET_CYCLES_FCT;
    if (newCycles < oldCycles)
    {
        ++highDWORD;
    }
    oldCycles       = newCycles;
    internalCounter = (((uint64_t)highDWORD << 32) | newCycles);
    return internalCounter;
}

uint64_t timerTool::returnInternalCounter()
{
    getTimer64();
    return internalCounter;
}

uint32_t timerTool::returnSystemTimestampUs()
{
    // TODO: Replace with the function below and update all libs to use uint64 instead of uint32
    return micros();
}

uint64_t timerTool::returnSystemTimestampUs64()
{
    static val64_reg_t timestamp; // NOLINT(misc-const-correctness)

    timestamp.lowPart = returnSystemTimestampUs();
    if (timestamp.oldVal > WRAPPING_VALUE_MIN_OLD_VAL)
    {
        if (timestamp.lowPart < WRAPPING_VALUE_MIN_LOW_PART && timestamp.lowPart < timestamp.oldVal)
            ++timestamp.highPart;
    }

    timestamp.oldVal = timestamp.lowPart;
    return timestamp.val64;
}

uint32_t timerTool::returnSystemTimestampMs()
{
    return millis();
}

double timerTool::returnSystemCounterSecondsDouble()
{
    updateAllCounters();
    return systemCounterSecondsDouble;
}

float timerTool::returnSystemCounterSecondsFloat()
{
    return static_cast<float>(returnSystemTimestampUs64()) * FLOAT_US_TO_SECONDS;
}

double timerTool::returnSystemCounterNs()
{
    updateAllCounters();
    return systemCounterNs;
}

uint64_t timerTool::returnSystemCounterUsU64()
{
    updateAllCounters();
    return systemCounterUsU64;
}

double timerTool::returnsystemCounterUs()
{
    updateAllCounters();
    return systemCounterUs;
}

// NOLINTBEGIN(readability-convert-member-functions-to-static,readability-make-member-function-const)
void timerTool::startTiming(uint8_t idx)
{
    // t_now = ARM_DWT_CYCCNT;
    if (idx >= maxIntervalMeasures)
    {
        curSerialMonitor->println("Error, only 20 measures max");
        return;
    }
    internalCounterPrev[idx] = getTimer64();
    // Serial.println(ARM_DWT_CYCCNT - t_now);
}

double timerTool::stopTiming(uint8_t idx, bool print)
{
    // t_now = ARM_DWT_CYCCNT;
    if (idx >= maxIntervalMeasures)
    {
        curSerialMonitor->println("Error, only 20 measures max");
        return -1.0;
    }

    double deltaValNs = (getTimer64() - internalCounterPrev[idx]) * COUNTER_2_NS_DOUBLE
                        + timingFunctionDuration;
    if (print)
    {
        deltaValNs += timingFunctionDurationOffsetWithPrint;
        Serial.print("Measure " + String(idx) + " Delta : ns = ");
        Serial.print("\t");
        Serial.print(deltaValNs);
        Serial.print("\t us = ");
        Serial.print(deltaValNs * 0.001);
        Serial.print("\t ms = ");
        Serial.print(deltaValNs * 0.000001, 4);
        Serial.print("\t s = ");
        Serial.println(deltaValNs * 0.000000001, 6);
    }
    return deltaValNs;
    // Serial.println(ARM_DWT_CYCCNT - t_now);
}
// NOLINTEND(readability-convert-member-functions-to-static,readability-make-member-function-const)

void timerTool::updateAllCounters()
{
    getTimer64();
    systemCounterSecondsDouble = internalCounter * INVERT_F_CPU_DOUBLE;
    systemCounterSecondsFloat  = returnSystemCounterSecondsFloat();
    systemCounterNs            = internalCounter * COUNTER_2_NS_DOUBLE;
    systemCounterUs            = internalCounter * COUNTER_2_US_DOUBLE;
    systemCounterUsU64         = (uint64_t)systemCounterUs;
}

#if defined(ESP_PLATFORM)
void IRAM_ATTR // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
timerTool::delayNanosecondsAlt(uint64_t delay)
{
    if (delay < COUNTER_2_NS_UINT)
        return;

    uint64_t endCnt = getTimer64() + (delay >> RIGHT_SHIFT_VAL_COUNTER_2_NS_UINT);
    while (getTimer64() < endCnt)
        NOP();
}
#endif

#if defined(CORE_TEENSY)
void timerTool::delayNanosecondsAlt(uint64_t delay)
{
    delayNanoseconds(delay);
}
#endif