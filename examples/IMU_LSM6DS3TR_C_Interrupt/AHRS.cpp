#include "AHRS.h"
#if defined(FRAMEWORK_RPI_PICO)
#include <boards/pico.h> // for PICO_DEFAULT_LED_PIN
#include <hardware/gpio.h>
#include <hardware/irq.h>
#endif

AHRS* AHRS::ahrs {nullptr};

/*!
IMU data ready Interrupt Service Routine (ISR)
*/
void AHRS::imuDataReadyISR()
{
    gpio_put(PICO_DEFAULT_LED_PIN, 1); // to confirm the ISR has been called
    ++ahrs->_imuDataReadyCount; // keep a count of how many times the ISR has been triggered, for instrumentation

    // clear the interrupt
    irq_clear(ahrs->_userIrq);
    // unlock the data ready mutex
    ahrs->UNLOCK_IMU_DATA_READY();
}

AHRS::~AHRS()
{
    user_irq_unclaim(_userIrq);
}

AHRS::AHRS(IMU_Base& imuSensor) :
    _IMU(imuSensor)
{
    ahrs = this;

    mutex_init(&_imuDataReadyMutex);

    enum { PANIC_IF_NONE_AVAILABLE = true };
    _userIrq = user_irq_claim_unused(PANIC_IF_NONE_AVAILABLE); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    irq_set_enabled(_userIrq, true);

    enum { IRQ_PRIORITY = 128 };
    //irq_add_shared_handler(_userIrq, &imuDataReadyISR, IRQ_PRIORITY);
    irq_set_exclusive_handler(_userIrq,  &imuDataReadyISR);

    _IMU.setInterrupt(_userIrq);
}
