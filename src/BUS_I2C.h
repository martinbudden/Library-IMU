#pragma once

#include "BUS_BASE.h"

#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal_i2c.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal_i2c.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal_i2c.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal_i2c.h>
#endif
#endif

#if defined(FRAMEWORK_RPI_PICO)
typedef struct i2c_inst i2c_inst_t;
#elif defined(FRAMEWORK_ESPIDF)
#include "driver/i2c_master.h" // cppcheck-suppress missingInclude
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Wire.h>
#if defined(FRAMEWORK_ARDUINO_RPI_PICO)
#elif defined(FRAMEWORK_ARDUINO_ESP32)
#elif defined(FRAMEWORK_ARDUINO_STM32)
#endif // FRAMEWORK_ARDUINO
#endif // FRAMEWORK

#if !defined(IRAM_ATTR)
#define IRAM_ATTR
#endif


class BUS_I2C  : public BUS_BASE {
public:
    struct i2c_pins_t {
        uint8_t sda;
        uint8_t scl;
        uint8_t irq;
    };
    struct stm32_i2c_pins_t {
        port_pin_t sda;
        port_pin_t scl;
        port_pin_t irq;
    };
public:
    BUS_I2C(uint8_t I2C_address, bus_index_e I2C_index);
    explicit BUS_I2C(uint8_t I2C_address) : BUS_I2C(I2C_address, BUS_INDEX_0) {}
    BUS_I2C(uint8_t I2C_address, bus_index_e I2C_index, const i2c_pins_t& pins);
    BUS_I2C(uint8_t I2C_address, const i2c_pins_t& pins) : BUS_I2C(I2C_address, BUS_INDEX_0, pins) {}

    BUS_I2C(uint8_t I2C_address, bus_index_e I2C_index, const stm32_i2c_pins_t& pins);
    BUS_I2C(uint8_t I2C_address, const stm32_i2c_pins_t& pins) : BUS_I2C(I2C_address, BUS_INDEX_0, pins) {}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) &&!defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
    BUS_I2C(uint8_t I2C_address, TwoWire& wire, const i2c_pins_t& pins);
#endif
public:
    void init();
    void setInterruptDriven(irq_level_e irqLevel);
    FAST_CODE bool readDeviceData();
    FAST_CODE uint8_t readRegister(uint8_t reg) const;
    FAST_CODE uint8_t readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const;
    FAST_CODE bool readRegister(uint8_t reg, uint8_t* data, size_t length) const;
    FAST_CODE bool readBytes(uint8_t* data, size_t length) const;
    FAST_CODE bool readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const;
    FAST_CODE uint8_t writeRegister(uint8_t reg, uint8_t data);
    FAST_CODE uint8_t writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    FAST_CODE uint8_t writeBytes(const uint8_t* data, size_t length);
private:
    static BUS_I2C* bus; //!< alias of `this` to be used in interrupt service routine
    bus_index_e _I2C_index {};
    stm32_i2c_pins_t _pins {};
#if defined(FRAMEWORK_RPI_PICO)
    enum { RETAIN_CONTROL_OF_BUS = true };
    enum { DONT_RETAIN_CONTROL_OF_BUS = false };
    FAST_CODE static void dataReadyISR(unsigned int gpio, uint32_t events);
    i2c_inst_t* _I2C {};
#elif defined(FRAMEWORK_ESPIDF)
    FAST_CODE static void dataReadyISR();
    i2c_master_bus_handle_t _bus_handle {};
    i2c_master_dev_handle_t _dev_handle {};
#elif defined(FRAMEWORK_STM32_CUBE)
    FAST_CODE static void dataReadyISR(); // cppcheck-suppress unusedPrivateFunction
    mutable I2C_HandleTypeDef _I2C {};
#elif defined(FRAMEWORK_TEST)
    static void dataReadyISR();
#else // defaults to FRAMEWORK_ARDUINO
    FAST_CODE static void dataReadyISR(); // cppcheck-suppress unusedPrivateFunction
    TwoWire& _wire;
#endif
    uint8_t _I2C_address {};
};
