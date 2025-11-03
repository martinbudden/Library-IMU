#include "BarometerBase.h"
#include <cassert>


#if defined(FRAMEWORK_RPI_PICO)
#include <pico/time.h>
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#endif


BarometerBase::BarometerBase(BUS_BASE& busBase) :
    _busBase(&busBase)
{
}

void BarometerBase::delayMs(int ms)
{
#if defined(FRAMEWORK_RPI_PICO)
    sleep_ms(ms);
#elif defined(FRAMEWORK_ESPIDF)
    (void)ms;
#elif defined(FRAMEWORK_STM32_CUBE)
    (void)ms;
#elif defined(FRAMEWORK_TEST)
    (void)ms;
#else // defaults to FRAMEWORK_ARDUINO
    delay(ms);
#endif
}
