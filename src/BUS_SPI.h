#pragma once

#include "BUS_BASE.h"

#if defined(FRAMEWORK_RPI_PICO)
#if defined(USE_IMU_SPI_DMA)
#include "hardware/dma.h"
#endif
typedef struct spi_inst spi_inst_t;
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <SPI.h>
#endif // FRAMEWORK

#if !defined(IRAM_ATTR)
#define IRAM_ATTR
#endif


class BUS_SPI : public BUS_BASE {
public:
    enum spi_index_e { SPI_INDEX_0, SPI_INDEX_1, SPI_INDEX_2, SPI_INDEX_3 };
    struct pins_t {
        uint8_t cs;
        uint8_t sck;
        uint8_t cipo; // RX, CIPO, MISO, POCI
        uint8_t copi; // TX, COPI, MOSI, PICO
        uint8_t irq; // interrupt pin
        uint8_t irqLevel; // interrupt level, ie low, high, edge rise, edge fall, edge change
    };
    static constexpr uint8_t READ_BIT = 0x80U;
public:
#if defined(USE_IMU_SPI_DMA)
    virtual ~BUS_SPI();
#endif
    BUS_SPI(uint32_t frequency, spi_index_e SPI_index, const pins_t& pins);
    BUS_SPI(uint32_t frequency, spi_index_e SPI_index, uint8_t CS_pin);
    BUS_SPI(uint32_t frequency, uint8_t CS_pin) : BUS_SPI(frequency, SPI_INDEX_0, CS_pin) {}
public:
    void configureDMA();
    void setInterruptDriven();
    void setDeviceRegister(uint8_t deviceRegister, uint8_t* readBuf, size_t readLength);
    IRAM_ATTR bool readDeviceRegister();
    IRAM_ATTR bool readDeviceRegisterDMA(); // for testing DMA
    IRAM_ATTR uint8_t readRegister(uint8_t reg) const;
    IRAM_ATTR uint8_t readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const;
    IRAM_ATTR bool readRegister(uint8_t reg, uint8_t* data, size_t length) const;
    IRAM_ATTR bool readBytes(uint8_t* data, size_t length) const;
    IRAM_ATTR bool readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const;
    IRAM_ATTR uint8_t writeRegister(uint8_t reg, uint8_t data);
    IRAM_ATTR uint8_t writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    IRAM_ATTR uint8_t writeBytes(const uint8_t* data, size_t length);
private:
    static BUS_SPI* bus; //!< alias of `this` to be used in interrupt service routine
    uint32_t _clockDivider {1};
    uint32_t _frequency {1000000};
    spi_index_e _SPI_index {};
    pins_t _pins;
#if defined(FRAMEWORK_RPI_PICO)
    static void dataReadyISR(unsigned int gpio, uint32_t events);
    spi_inst_t* _spi;
    //mutable std::array<uint8_t, 256> _writeReadBuf {};
    static void dmaRxCompleteISR();
    uint32_t _dmaRxChannel {};
    uint32_t _dmaTxChannel {};
#elif defined(FRAMEWORK_ESPIDF)
    IRAM_ATTR static void dataReadyISR();
#elif defined(FRAMEWORK_TEST)
    static void dataReadyISR();
#else // defaults to FRAMEWORK_ARDUINO
    IRAM_ATTR static void dataReadyISR();// cppcheck-suppress unusedPrivateFunction
    SPIClass& _spi;
    volatile uint32_t* _csOut {};
    uint32_t _csBit {};
#endif
};
