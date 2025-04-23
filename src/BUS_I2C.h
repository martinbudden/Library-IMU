#pragma once

#include <cstddef>
#include <cstdint>

#if defined(FRAMEWORK_PICO)
typedef struct i2c_inst i2c_inst_t;
#elif defined(FRAMEWORK_ESPIDF)
#include "driver/i2c_master.h" // cppcheck-suppress missingInclude
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Wire.h>
#endif


class BUS_I2C {
public:
    BUS_I2C(uint8_t I2C_address, uint8_t SDA_pin, uint8_t SCL_pin);
public:
    uint8_t readRegister(uint8_t reg) const;
    uint8_t readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const;
    bool readRegister(uint8_t reg, uint8_t* data, size_t length) const;
    bool readBytes(uint8_t* data, size_t length) const;
    bool readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const;
    uint8_t writeRegister(uint8_t reg, uint8_t data);
    uint8_t writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    uint8_t writeBytes(const uint8_t* data, size_t length);
private:
#if defined(FRAMEWORK_PICO)
    i2c_inst_t* _I2C;
#elif defined(FRAMEWORK_ESPIDF)
    i2c_master_bus_handle_t _bus_handle {};
    i2c_master_dev_handle_t _dev_handle {};
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    TwoWire& _wire;
#endif
    uint8_t _I2C_address;
    uint8_t _SDA_pin;
    uint8_t _SCL_pin;
    uint8_t _filler {0};
};
