#ifndef libDM_generic_operations
#define libDM_generic_operations

#include <Arduino.h>
#include <array>

namespace OP
{
/**
 * @brief Generic operations for digital pin io manipulations
 */
namespace PIN_IO
{
static inline __attribute__((always_inline)) void digitalWriteFast(uint8_t pin, uint8_t state)
{
#if defined(CORE_TEENSY)
    digitalWriteFast(pin, state);
#elif defined(ESP_PLATFORM)
    // NOLINTBEGIN(bugprone-branch-clone)
    if (state)
    {
        if (pin < 32)
            GPIO.out_w1ts = ((uint32_t)1 << pin);
        else if (pin < 34)
            GPIO.out1_w1ts.val = ((uint32_t)1 << (pin - 32));
    }
    else
    {
        if (pin < 32)
            GPIO.out_w1tc = ((uint32_t)1 << pin);
        else if (pin < 34)
            GPIO.out1_w1tc.val = ((uint32_t)1 << (pin - 32));
    }
    // NOLINTEND(bugprone-branch-clone)
#endif
}
} // namespace PIN_IO

/**
 * @brief Generic operations for UART communication
 */
namespace UART
{
static inline __attribute__((always_inline)) void readBytesBuffer(HardwareSerial* serialObj,
                                                                  uint8_t* buf,
                                                                  size_t size)
{
#if defined(CORE_TEENSY)
    for (uint16_t i = 0; i < size; i++)
        buf[i] = serialObj->read();

#elif defined(ESP_PLATFORM)
    serialObj->readBytes(buf, size);
#endif
}

static inline __attribute__((always_inline)) void clearBuffer(HardwareSerial* serialObj)
{
#if defined(CORE_TEENSY)
    serialObj->clear();
#elif defined(ESP_PLATFORM)
    serialObj->flush(false);
#endif
}
} // namespace UART

/**
 * @brief Generic operations for text manipulations
 */
namespace TEXT
{
// String class is not the same on esp32 and teensy
#if defined(CORE_TEENSY)
class String : public ::String
{
public:
    // constructors inherited from String.
    String(const char* cstr = (const char*)NULL) : ::String(cstr){};
    String(const __FlashStringHelper* pgmstr) : ::String(pgmstr){};
    String(const String& str) : ::String(str){};
#    if __cplusplus >= 201103L || defined(__GXX_EXPERIMENTAL_CXX0X__)
    String(String&& rval) : ::String(rval){};
    String(StringSumHelper&& rval) : ::String(rval){};
#    endif
    String(char c) : ::String(c){};
    String(int num, unsigned char base = 10) : ::String(num, base) {}
    String(unsigned int num, unsigned char base = 10) : ::String(num, base) {}
    String(long num, unsigned char base = 10) : ::String(num, base) {}
    String(unsigned long num, unsigned char base = 10) : ::String(num, base) {}
    String(float num, unsigned char digits = 2) : ::String(num, digits) {}
    String(double num, unsigned char digits = 2) : ::String(static_cast<float>(num), digits) {}

    // Ambiguous overload for uint64_t
    explicit String(uint64_t value)
    {
        String((unsigned long long)value);
    }
};
#elif defined(ESP_PLATFORM)
class String : public ::String
{
public:
    using ::String::String; // Inherit all constructors of ::String

    template <class T>
    void append(T string)
    {
        (this)->concat(string);
    }
};
#endif

template <typename T, size_t N>
static inline __attribute__((always_inline)) ::String getStringFromArray(
        const std::array<T, N>& array)
{
    ::String str  = "[";
    size_t length = array.size();
#pragma unroll
    for (size_t i = 0; i < length; i++)
    {
        str += ::String(array[i]);
        if (i < length - 1) str += ",";
    }
    str += "]";
    return str;
}
} // namespace TEXT

} // namespace OP

#endif