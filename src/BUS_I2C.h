#pragma once

#include "BUS_BASE.h"

#if defined(FRAMEWORK_RPI_PICO)
typedef struct i2c_inst i2c_inst_t;
#elif defined(FRAMEWORK_ESPIDF)
#include "driver/i2c_master.h" // cppcheck-suppress missingInclude
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Wire.h>
#endif // FRAMEWORK

#if !defined(IRAM_ATTR)
#define IRAM_ATTR
#endif


class BUS_I2C  : public BUS_BASE {
public:
    enum i2c_index_e { I2C_INDEX_0, I2C_INDEX_1, I2C_INDEX_2, I2C_INDEX_3 };
    struct pins_t {
        uint8_t sda;
        uint8_t scl;
        uint8_t irq;
        uint8_t irqLevel;
    };
public:
    BUS_I2C(uint8_t I2C_address, i2c_index_e I2C_index);
    explicit BUS_I2C(uint8_t I2C_address) : BUS_I2C(I2C_address, I2C_INDEX_0) {}
    BUS_I2C(uint8_t I2C_address, i2c_index_e I2C_index, const pins_t& pins);
    BUS_I2C(uint8_t I2C_address, const pins_t& pins) : BUS_I2C(I2C_address, I2C_INDEX_0, pins) {}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_TEST)
    BUS_I2C(uint8_t I2C_address, TwoWire& wire, const pins_t& pins);
#endif
public:
    void setInterruptDriven();
    void setDeviceRegister(uint8_t deviceRegister, uint8_t* readBuf, size_t readLength);
    IRAM_ATTR bool readDeviceRegister();
    IRAM_ATTR uint8_t readRegister(uint8_t reg) const;
    IRAM_ATTR uint8_t readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const;
    IRAM_ATTR bool readRegister(uint8_t reg, uint8_t* data, size_t length) const;
    IRAM_ATTR bool readBytes(uint8_t* data, size_t length) const;
    IRAM_ATTR bool readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const;
    IRAM_ATTR uint8_t writeRegister(uint8_t reg, uint8_t data);
    IRAM_ATTR uint8_t writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    IRAM_ATTR uint8_t writeBytes(const uint8_t* data, size_t length);
private:
    static BUS_I2C* bus; //!< alias of `this` to be used in interrupt service routine
    i2c_index_e _I2C_index {};
    pins_t _pins;
#if defined(FRAMEWORK_RPI_PICO)
    enum { RETAIN_CONTROL_OF_BUS = true };
    enum { DONT_RETAIN_CONTROL_OF_BUS = false };
    static void dataReadyISR(unsigned int gpio, uint32_t events);
    i2c_inst_t* _I2C;
#elif defined(FRAMEWORK_ESPIDF)
    IRAM_ATTR static void dataReadyISR();
    i2c_master_bus_handle_t _bus_handle {};
    i2c_master_dev_handle_t _dev_handle {};
#elif defined(FRAMEWORK_TEST)
    static void dataReadyISR();
#else // defaults to FRAMEWORK_ARDUINO
    IRAM_ATTR static void dataReadyISR(); // cppcheck-suppress unusedPrivateFunction
    TwoWire& _wire;
#endif
    uint8_t _I2C_address;
};
