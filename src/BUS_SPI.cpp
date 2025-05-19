#include "BUS_SPI.h"

#if defined(FRAMEWORK_RPI_PICO)
#if defined(USE_IMU_SPI_DMA)
#include "hardware/dma.h"
#endif
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include <array>
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#include <SPI.h>
#endif

#define MAP_CS_FOR_SPI

BUS_SPI* BUS_SPI::bus {nullptr};

/*!
Data ready interrupt service routine (ISR)

Currently support only one interrupt, but could index action off the interrupt pin
*/
#if defined(FRAMEWORK_RPI_PICO)
void BUS_SPI::dataReadyISR(unsigned int gpio, uint32_t events)
{
    // reading the register resets the interrupt
#if defined(USE_IMU_SPI_DMA)
    // _dmaTx configuration hasn't changed
    dma_channel_configure(bus->_dmaRx, &bus->_dmaRxConfig,
                          bus->_readBuf, // destination, write SPI data to data
                          &spi_get_hw(bus->_spi)->dr, // source, read data from SPI
                          _bus->_readlength, // element count (each element is 8 bits)
                          false); // don't start yet
    dma_start_channel_mask((1u << bus->_dmaTX) | (1u << bus->_dmaRX));
    // wait for rx to complete
    dma_channel_wait_for_finish_blocking(bus->_dmaRX);
#else
    bus->readRegister(bus->_readRegister, bus->_readBuf, bus->_readLength);
#endif
    // set the user IRQ so that the AHRS can process the data
    irq_set_pending(bus->_userIrq);
}
#else
INSTRUCTION_RAM_ATTR void BUS_SPI::dataReadyISR()
{
#if defined(USE_IMU_SPI_DMA)
    static_assert(false);
#else
    // for the moment, just read the predefined register into the predefined read buffer
    bus->readRegister(bus->_readRegister, bus->_readBuf, bus->_readLength);
#endif
}
#endif


#if defined(FRAMEWORK_RPI_PICO)
static inline void cs_select(uint8_t CS_pin) {
#if defined(MAP_CS_FOR_SPI)
    (void)CS_pin;
#else
    asm volatile("nop \n nop \n nop");
    gpio_put(uint8_t CS_pin, 0);  // Active low
    asm volatile("nop \n nop \n nop");
#endif
}

static inline void cs_deselect(uint8_t CS_pin) {
#if defined(MAP_CS_FOR_SPI)
    (void)CS_pin;
#else
    asm volatile("nop \n nop \n nop");
    gpio_put(uint8_t CS_pin, 1);
    asm volatile("nop \n nop \n nop");
#endif
}
#endif // FRAMEWORK_RPI_PICO

#if defined(USE_IMU_SPI_DMA)
BUS_SPI::~BUS_SPI()
{
#if defined(FRAMEWORK_RPI_PICO)
    dma_channel_unclaim(dma_tx);
    dma_channel_unclaim(dma_rx);
#endif
}
#endif

BUS_SPI::BUS_SPI(uint32_t frequency, spi_index_t SPI_index, uint8_t CS_pin)
#if defined(FRAMEWORK_RPI_PICO)
    : BUS_SPI(frequency, SPI_index, { .cs=CS_pin, .sck=PICO_DEFAULT_SPI_SCK_PIN, .cipo=PICO_DEFAULT_SPI_RX_PIN, .copi=PICO_DEFAULT_SPI_TX_PIN, .irq=IRQ_NOT_SET, .irqLevel=0})
    // there is also PICO_DEFAULT_SPI_CSN_PIN
#elif defined(FRAMEWORK_ESPIDF)
    : BUS_SPI(frequency, SPI_index, { .cs=CS_pin, .sck=0, .cipo=0, .copi=0, .irq=IRQ_NOT_SET, .irqLevel=0 })
#else // defaults to FRAMEWORK_ARDUINO
     : BUS_SPI(frequency, SPI_index, { .cs=CS_pin, .sck=0, .cipo=0, .copi=0, .irq=IRQ_NOT_SET, .irqLevel=0 })
#endif
{
    (void)SPI_index;
}

BUS_SPI::BUS_SPI(uint32_t frequency, spi_index_t SPI_index, const pins_t& pins, uint8_t readRegister, uint8_t* readBuf, size_t readLength) :
    _frequency(frequency)
    ,_SPI_index(SPI_index)
    ,_pins(pins)
    ,_readRegister(readRegister | READ_BIT)
    ,_readBuf(readBuf)
    ,_readLength(readLength)
#if defined(FRAMEWORK_RPI_PICO)
#if defined(USE_IMU_SPI_DMA)
    ,_dmaRx(dma_claim_unused_channel(true))
    ,_dmaTx(dma_claim_unused_channel(true))
#endif
    ,_spi(SPI_index == SPI_INDEX_1 ? spi1 : spi0)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    ,_spi(SPI)
#endif
{
    bus = this;
#if defined(FRAMEWORK_RPI_PICO)
    static_assert(static_cast<int>(IRQ_LEVEL_LOW) == GPIO_IRQ_LEVEL_LOW);
    static_assert(static_cast<int>(IRQ_LEVEL_HIGH) == GPIO_IRQ_LEVEL_HIGH);
    static_assert(static_cast<int>(IRQ_EDGE_FALL) == GPIO_IRQ_EDGE_FALL);
    static_assert(static_cast<int>(IRQ_EDGE_RISE) == GPIO_IRQ_EDGE_RISE);

    spi_init(_spi, _frequency);
    gpio_set_function(_pins.cipo, GPIO_FUNC_SPI);
    gpio_set_function(_pins.sck, GPIO_FUNC_SPI);
    gpio_set_function(_pins.copi, GPIO_FUNC_SPI);
#if defined(MAP_CS_FOR_SPI)
    gpio_set_function(_pins.cs, GPIO_FUNC_SPI); // map the CS pin, so the RP2040/2340 will automatically toggle the CS pin for SPI
    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(_pins.cipo, _pins.copi, _pins.sck, _pins.cs, GPIO_FUNC_SPI));
#else
    bi_decl(bi_3pins_with_func(_pins.cipo, _pins.copi, _pins.sck, GPIO_FUNC_SPI));
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(_pins.cs);
    gpio_set_dir(_pins.cs, GPIO_OUT);
    gpio_put(_pins.cs, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(_pins.cs, "SPI CS"));
#endif

#if defined(USE_IMU_SPI_DMA)
    _dmaTxConfig = dma_channel_get_default_config(_dmaTx);
    channel_config_set_transfer_data_size(&_dmaTxConfig, DMA_SIZE_8); // 8-bit transfers
    channel_config_set_dreq(&_dmaTxConfig, spi_get_dreq(_spi, true));
    channel_config_set_read_increment(&_dmaTxConfig, false);
    channel_config_set_write_increment(&_dmaTxConfig, false);
    dma_channel_configure(_dmaTx, &_dmaTxConfig,
                          &spi_get_hw(_spi)->dr, // destination, write data to SPI
                          &_readRegister, // source, send the value of the register we want to read
                          _readLength, // number of bytes to read (each element is DMA_SIZE_8, ie 8 bits)
                          false); // don't start yet

    _dmaRxConfig = dma_channel_get_default_config(_dmaRx);
    channel_config_set_transfer_data_size(&_dmaRxConfig, DMA_SIZE_8); // 8-bit transfers
    channel_config_set_dreq(&_dmaRxConfig, spi_get_dreq(_spi, false));
    channel_config_set_read_increment(&_dmaRxConfig, false);
    channel_config_set_write_increment(&_dmaRxConfig, true);
    dma_channel_configure(_dmaRx, &_dmaRxConfig,
                          _readBuf, // destination, write data to readBuf
                          &spi_get_hw(_spi)->dr, // source, read data from SPI
                          _readLength, // number of bytes to read (each element is DMA_SIZE_8, ie 8 bits)
                          false); // don't start yet
#endif
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    _spi.begin();
    pinMode(_pins.cs, OUTPUT);
    digitalWrite(_pins.cs, HIGH);
#endif
}

BUS_SPI::BUS_SPI(uint32_t frequency, spi_index_t SPI_index, const pins_t& pins) :
    BUS_SPI(frequency, SPI_index, pins, 0, nullptr, 0)
{
}

void BUS_SPI::setInterrupt(int userIrq)
{
    _userIrq = userIrq;
#if defined(FRAMEWORK_RPI_PICO)
    assert(_pins.irq != IRQ_NOT_SET);
    assert(_pins.irqLevel != 0);
    gpio_init(_pins.irq);
    gpio_set_irq_enabled_with_callback(_pins.irq, _pins.irqLevel, true, &dataReadyISR);
#endif
}

uint8_t BUS_SPI::readRegister(uint8_t reg) const
{
#if defined(FRAMEWORK_RPI_PICO)
    cs_select(_pins.cs);
    reg |= READ_BIT;
    spi_write_blocking(_spi, &reg, 1);
    sleep_ms(10);
    uint8_t ret;
    spi_read_blocking(_spi, 0, &ret, 1);
    cs_deselect(_pins.cs);
    return ret;
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_pins.cs, LOW);
    _spi.transfer(reg | READ_BIT);
    const uint8_t ret = _spi.transfer(0); // NOLINT(cppcoreguidelines-init-variables) false positive
    digitalWrite(_pins.cs, HIGH);
    _spi.endTransaction();
    return ret;
#endif
    return 0;
}

uint8_t BUS_SPI::readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const
{
    (void)timeoutMs;
    return readRegister(reg);
}

bool BUS_SPI::readRegister(uint8_t reg, uint8_t* data, size_t length) const
{
#if defined(FRAMEWORK_RPI_PICO)
#if defined(MAP_CS_FOR_SPI)
    reg |= READ_BIT;
    spi_write_blocking(_spi, &reg, 1);
    spi_read_blocking(_spi, 0, data, length);
#else
    cs_select(_pins.cs);
    reg |= READ_BIT;
    spi_write_blocking(_spi, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(_spi, 0, data, length);
    cs_deselect(_pins.cs);
#endif
    return true;
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    *data = 0;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    *data = 0;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_pins.cs, LOW);
    _spi.transfer(reg | READ_BIT);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_pins.cs, HIGH);
    _spi.endTransaction();
    return true;
#endif
    return false;
}

bool BUS_SPI::readRegisterDMA(uint8_t reg, uint8_t* data, size_t length)
{
    _readRegister = reg | READ_BIT;
#if defined(USE_IMU_SPI_DMA)
#if defined(FRAMEWORK_RPI_PICO)
    dma_channel_configure(_dmaRx, &_dmaRxConfig,
                          data, // destination, write SPI data to data
                          &spi_get_hw(_spi)->dr, // source, read data from SPI
                          length, // element count (each element is 8 bits)
                          false); // don't start yet
    dma_start_channel_mask((1u << _dmaTX) | (1u << _dmaRX));
    // wait for rx to complete
    dma_channel_wait_for_finish_blocking(_dmaRX);
    return true;
#else
    (void)data;
    (void)length;
    return true;
#endif
#else
    return readRegister(reg, data, length);
#endif // USE_IMU_SPI_DMA
}

bool BUS_SPI::readBytes(uint8_t* data, size_t length) const
{
#if defined(FRAMEWORK_RPI_PICO)
    cs_select(_pins.cs);
    spi_read_blocking(_spi, 0, data, length);
    cs_deselect(_pins.cs);
    return true;
#elif defined(FRAMEWORK_ESPIDF)
    *data = 0;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    *data = 0;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_pins.cs, LOW);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_pins.cs, HIGH);
    _spi.endTransaction();
    return true;
#endif
    return false;
}

bool BUS_SPI::readBytesWithTimeout(uint8_t* data, size_t length, uint8_t timeoutMs) const
{
    (void)timeoutMs;
    return readBytes(data, length);
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, uint8_t data)
{
#if defined(FRAMEWORK_RPI_PICO)
    std::array<uint8_t, 2> buf = { reg & 0x7FU, data }; // remove read bit as this is a write
    cs_select(_pins.cs);
    spi_write_blocking(_spi, &buf[0], 2);
    cs_deselect(_pins.cs);
    sleep_ms(10);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_pins.cs, LOW);
    _spi.transfer(reg);
    _spi.transfer(data);
    digitalWrite(_pins.cs, HIGH);
    _spi.endTransaction();
#endif
    return 0;
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
    cs_select(_pins.cs);
    reg &= 0x7FU;
    spi_write_blocking(_spi, &reg, 1);
    spi_write_blocking(_spi, data, length);
    cs_deselect(_pins.cs);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_pins.cs, LOW);
    _spi.transfer(reg);
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_pins.cs, HIGH);
    _spi.endTransaction();
#endif
    return 0;
}

uint8_t BUS_SPI::writeBytes(const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
    cs_select(_pins.cs);
    spi_write_blocking(_spi, data, length);
    cs_deselect(_pins.cs);
#elif defined(FRAMEWORK_ESPIDF)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_pins.cs, LOW);
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_pins.cs, HIGH);
    _spi.endTransaction();
#endif
    return 0;
}
