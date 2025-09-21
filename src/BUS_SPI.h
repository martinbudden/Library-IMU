#pragma once

#include "BUS_BASE.h"

//#define LIBRARY_IMU_USE_SPI_HARDWARE_CHIP_SELECT //!!TODO: make this based on member data, not build flag

#if defined(FRAMEWORK_RPI_PICO)
#if defined(LIBRARY_IMU_USE_SPI_DMA)
#include "hardware/dma.h"
#endif
typedef struct spi_inst spi_inst_t;
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal_spi.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal_spi.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal_spi.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal_spi.h>
#endif
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <SPI.h>
#endif // FRAMEWORK


class BUS_SPI : public BUS_BASE {
public:
    struct pins_t {
        uint8_t cs;
        uint8_t sck;
        uint8_t cipo; // RX, CIPO, MISO, POCI
        uint8_t copi; // TX, COPI, MOSI, PICO
        uint8_t irq; // interrupt pin
    };
    struct stm32_spi_pins_t {
        port_pin_t cs;
        port_pin_t sck;
        port_pin_t cipo; // RX, CIPO, MISO, POCI
        port_pin_t copi; // TX, COPI, MOSI, PICO
        port_pin_t irq; // interrupt pin
    };
    static constexpr uint8_t READ_BIT = 0x80U;
public:
    virtual ~BUS_SPI();
    BUS_SPI(uint32_t frequency, bus_index_e SPI_index, const stm32_spi_pins_t& pins);
    BUS_SPI(uint32_t frequency, bus_index_e SPI_index, const pins_t& pins);
public:
    void init();
    void configureDMA();
    void setInterruptDriven(irq_level_e irqLevel);
    FAST_CODE bool readDeviceData();
    FAST_CODE bool readDeviceDataDMA();
    FAST_CODE uint8_t readRegister(uint8_t reg) const;
    FAST_CODE uint8_t readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const;
    FAST_CODE bool readRegister(uint8_t reg, uint8_t* data, size_t length) const;
    FAST_CODE bool readBytes(uint8_t* data, size_t length) const;
    FAST_CODE bool readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const;
    FAST_CODE uint8_t writeRegister(uint8_t reg, uint8_t data);
    FAST_CODE uint8_t writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    FAST_CODE uint8_t writeBytes(const uint8_t* data, size_t length);
    FAST_CODE static void cs_select();
    FAST_CODE static void cs_deselect();
    inline uint16_t getIrqPin() const { return _pins.irq.pin; }
public:
    static BUS_SPI* bus; //!< alias of `this` to be used in interrupt service routine
private:
    uint32_t _clockDivider {1};
    uint32_t _frequency {1000000};
    bus_index_e _SPI_index {};
    stm32_spi_pins_t _pins {};
#if defined(FRAMEWORK_RPI_PICO)
    spi_inst_t* _spi {};
    uint32_t _dmaInterruptNumber {};
    uint32_t _dmaRxChannel {};
    uint32_t _dmaTxChannel {};
    enum { START_NOW = true, DONT_START_YET = false };
    static void dataReadyISR(unsigned int gpio, uint32_t events);
    static void dmaRxCompleteISR();
#elif defined(FRAMEWORK_ESPIDF)
    FAST_CODE static void dataReadyISR();
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    mutable SPI_HandleTypeDef _spi {};
#elif defined(FRAMEWORK_TEST)
    static void dataReadyISR();
#else // defaults to FRAMEWORK_ARDUINO
    FAST_CODE static void dataReadyISR();// cppcheck-suppress unusedPrivateFunction
    mutable volatile uint32_t* _csOut {};
    uint32_t _csBit {};
#if defined(FRAMEWORK_ARDUINO_ESP32)
#else
    SPIClass& _spi;
#endif
#endif // FRAMEWORK
#if defined(LIBRARY_IMU_USE_SPI_HARDWARE_CHIP_SELECT)
    mutable std::array<uint8_t, 256> _writeReadBuf {};
#endif
};
