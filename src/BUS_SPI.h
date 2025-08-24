#pragma once

#include "BUS_BASE.h"

//#define USE_IMU_SPI_HARDWARE_CHIP_SELECT

#if defined(FRAMEWORK_RPI_PICO)
#if defined(USE_IMU_SPI_DMA)
#include "hardware/dma.h"
#endif
typedef struct spi_inst spi_inst_t;
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#elif defined(FRAMEWORK_STM32_CUBE) || defined(USE_ARDUINO_STM32)
#include "stm32f4xx_hal.h"
#else // defaults to FRAMEWORK_ARDUINO
#if defined(USE_ARDUINO_ESP32)
#endif
#include <SPI.h>
#endif // FRAMEWORK

#if !defined(IRAM_ATTR)
#define IRAM_ATTR
#endif


class BUS_SPI : public BUS_BASE {
public:
    struct pins_t {
        uint8_t cs;
        uint8_t sck;
        uint8_t cipo; // RX, CIPO, MISO, POCI
        uint8_t copi; // TX, COPI, MOSI, PICO
        uint8_t irq; // interrupt pin
    };
    static constexpr uint8_t READ_BIT = 0x80U;
public:
    virtual ~BUS_SPI();
    BUS_SPI(uint32_t frequency, bus_index_e SPI_index, const pins_t& pins);
public:
    void configureDMA();
    void setInterruptDriven(irq_level_e irqLevel);
    IRAM_ATTR bool readDeviceData();
    IRAM_ATTR bool readDeviceDataDMA();
    IRAM_ATTR uint8_t readRegister(uint8_t reg) const;
    IRAM_ATTR uint8_t readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const;
    IRAM_ATTR bool readRegister(uint8_t reg, uint8_t* data, size_t length) const;
    IRAM_ATTR bool readBytes(uint8_t* data, size_t length) const;
    IRAM_ATTR bool readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const;
    IRAM_ATTR uint8_t writeRegister(uint8_t reg, uint8_t data);
    IRAM_ATTR uint8_t writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    IRAM_ATTR uint8_t writeBytes(const uint8_t* data, size_t length);
    IRAM_ATTR void cs_select() const;
    IRAM_ATTR void cs_deselect() const;
public:
    static BUS_SPI* bus; //!< alias of `this` to be used in interrupt service routine
private:
    uint32_t _clockDivider {1};
    uint32_t _frequency {1000000};
    bus_index_e _SPI_index {};
    pins_t _pins;
#if defined(FRAMEWORK_RPI_PICO)
    spi_inst_t* _spi {};
    uint32_t _dmaInterruptNumber {};
    uint32_t _dmaRxChannel {};
    uint32_t _dmaTxChannel {};
    enum { START_NOW = true, DONT_START_YET = false };
    static void dataReadyISR(unsigned int gpio, uint32_t events);
    static void dmaRxCompleteISR();
#elif defined(FRAMEWORK_ESPIDF)
    IRAM_ATTR static void dataReadyISR();
#elif defined(FRAMEWORK_STM32_CUBE) || defined(USE_ARDUINO_STM32)
    mutable SPI_HandleTypeDef _spi {};
#elif defined(FRAMEWORK_TEST)
    static void dataReadyISR();
#else // defaults to FRAMEWORK_ARDUINO
    IRAM_ATTR static void dataReadyISR();// cppcheck-suppress unusedPrivateFunction
    mutable volatile uint32_t* _csOut {};
    uint32_t _csBit {};
#if defined(USE_ARDUINO_ESP32)
#else
    SPIClass& _spi;
#endif
#endif // FRAMEWORK
#if defined(USE_IMU_SPI_HARDWARE_CHIP_SELECT)
    mutable std::array<uint8_t, 256> _writeReadBuf {};
#endif
};
