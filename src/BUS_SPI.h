#pragma once

#include <cstddef>
#include <cstdint>

#if defined(FRAMEWORK_PICO)
typedef struct spi_inst spi_inst_t;
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <SPI.h>
#endif


class BUS_SPI {
public:
    BUS_SPI(uint32_t frequency, uint8_t CS_pin);
    BUS_SPI(uint32_t frequency, uint8_t CS_pin, uint8_t SCK_pin, uint8_t CIPO_pin, uint8_t COPI_pin);
public:
    uint8_t readRegister(uint8_t reg) const;
    uint8_t readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const;
    bool readRegister(uint8_t reg, uint8_t* data, size_t length) const;
    bool readBytes(uint8_t* data, size_t length) const;
    bool readBytesWithTimeout(uint8_t* data, size_t length, uint8_t timeoutMs) const;
    uint8_t writeRegister(uint8_t reg, uint8_t data);
    uint8_t writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    uint8_t writeBytes(const uint8_t* data, size_t length);
private:
#if defined(FRAMEWORK_PICO)
    spi_inst_t* _spi;
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    SPIClass& _spi;
#endif
    uint32_t _clockDivider {1};
    uint32_t _frequency {1000000};
    uint8_t _CS_pin {};
    uint8_t _SCK_pin {};
    uint8_t _CIPO_pin {};
    uint8_t _COPI_pin {};
    uint8_t _spiIndex {};
};
