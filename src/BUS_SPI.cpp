#include "BUS_SPI.h"

//#define MAP_CS_FOR_SPI // not got this working, it's probably not worth the bother

#include <cassert>

#if defined(FRAMEWORK_RPI_PICO)
#include <hardware/spi.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <boards/pico.h> // for PICO_DEFAULT_LED_PIN
#if defined(MAP_CS_FOR_SPI)
#include <cstring>
#endif
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#include <SPI.h>
#endif


BUS_SPI* BUS_SPI::bus {nullptr};

/*!
Data ready interrupt service routine (ISR)

Currently support only one interrupt, but could index action off gpio pin
*/
#if defined(FRAMEWORK_RPI_PICO)
void BUS_SPI::dataReadyISR(unsigned int gpio, uint32_t events)
{
    // reading the register resets the interrupt
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
#if defined(USE_IMU_SPI_DMA)
    // _dmaTx configuration hasn't changed
    dma_channel_configure(bus->_dmaRx, &bus->_dmaRxConfig,
                          bus->_readBuf + SPI_BUFFER_SIZE - 1, // destination, write SPI data to data
                          &spi_get_hw(bus->_spi)->dr, // source, read data from SPI
                          bus->_readLength + 1 - SPI_BUFFER_SIZE, // element count (each element is 8 bits)
                          false); // don't start yet
    dma_start_channel_mask((1u << bus->_dmaTx) | (1u << bus->_dmaRx));
    // wait for rx to complete
    dma_channel_wait_for_finish_blocking(bus->_dmaRx);
#else
    bus->readRegister(bus->_readRegister, bus->_readBuf + SPI_BUFFER_SIZE, bus->_readLength - SPI_BUFFER_SIZE);
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
    bus->readRegister(bus->_readRegister, bus->_readBuf + SPI_BUFFER_SIZE, bus->_readLength - SPI_BUFFER_SIZE); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
#if defined(USE_FREERTOS)
    bus->UNLOCK_IMU_DATA_READY_FROM_ISR();
#endif
#endif
}
#endif


#if defined(FRAMEWORK_RPI_PICO)
static inline void cs_select(uint8_t CS_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_pin, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(uint8_t CS_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_pin, 1);
    asm volatile("nop \n nop \n nop");
}
#endif // FRAMEWORK_RPI_PICO

#define CS_LOW() ({*_csOut &= ~_csBit;}) // NOLINT(cppcoreguidelines-macro-usage)
#define CS_HIGH() ({*_csOut |= _csBit;}) // NOLINT(cppcoreguidelines-macro-usage)
//#define CS_LOW() ({digitalWrite(_pins.cs, LOW);})
//#define CS_HIGH() ({digitalWrite(_pins.cs, HIGH);})



#if defined(USE_IMU_SPI_DMA)
BUS_SPI::~BUS_SPI()
{
#if defined(FRAMEWORK_RPI_PICO)
    dma_channel_unclaim(_dmaRx);
    dma_channel_unclaim(_dmaTx);
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
}

BUS_SPI::BUS_SPI(uint32_t frequency, spi_index_t SPI_index, const pins_t& pins) :
    _frequency(frequency)
    ,_SPI_index(SPI_index)
    ,_pins(pins)
#if defined(FRAMEWORK_RPI_PICO)
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

#elif defined(FRAMEWORK_ESPIDF)

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
    _spi.begin();
    pinMode(_pins.cs, OUTPUT);
    CS_HIGH();

    _csBit = digitalPinToBitMask(_pins.cs); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    _csOut = portOutputRegister(digitalPinToPort(_pins.cs)); // NOLINT(cppcoreguidelines-prefer-member-initializer)

#endif
}

void BUS_SPI::setInterrupt(int userIrq, uint8_t readRegister, uint8_t* readBuf, size_t readLength)
{
    assert(_pins.irq != IRQ_NOT_SET);

    _readRegister = readRegister;
    _readBuf = readBuf;
    _readLength = readLength;
#if defined(USE_FREERTOS)
    _imuDataReadyQueue = reinterpret_cast<QueueHandle_t>(userIrq);
#else
    _userIrq = userIrq;
#endif
#if defined(USE_ARDUINO_ESP32)
    pinMode(_pins.irq, INPUT);
    attachInterrupt(digitalPinToInterrupt(_pins.irq), &dataReadyISR, _pins.irqLevel); // esp32-hal-gpio.h
#elif defined(FRAMEWORK_RPI_PICO)
    assert(_pins.irqLevel != 0);
    gpio_init(_pins.irq);
    enum { IRQ_ENABLED = true };
    gpio_set_irq_enabled_with_callback(_pins.irq, _pins.irqLevel, IRQ_ENABLED, &dataReadyISR);
#if defined(USE_IMU_SPI_DMA)
    _dmaTx = dma_claim_unused_channel(true);
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

    _dmaRx = dma_claim_unused_channel(true);
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
#endif // USE_IMU_SPI_DMA
#endif
}

uint8_t BUS_SPI::readRegister(uint8_t reg) const
{
#if defined(FRAMEWORK_RPI_PICO)
#if defined(MAP_CS_FOR_SPI)
    std::array<uint8_t, 2> outBuf = { reg | READ_BIT, 0 }; // remove read bit as this is a write
    spi_write_read_blocking(_spi, &outBuf[0], &_writeReadBuf[0], 2);
    reg = _writeReadBuf[1];
#else
    cs_select(_pins.cs);
    reg |= READ_BIT;
    spi_write_blocking(_spi, &reg, 1);
    spi_read_blocking(_spi, 0, &reg, 1);
    cs_deselect(_pins.cs);
#endif
    return reg;
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    CS_LOW();
    _spi.transfer(reg | READ_BIT);
    const uint8_t ret = _spi.transfer(0); // NOLINT(cppcoreguidelines-init-variables) false positive
    CS_HIGH();
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
    reg |= READ_BIT;
#if defined(MAP_CS_FOR_SPI)
    _writeReadBuf[0] = reg;
    std::array<uint8_t, 256> buf;
    spi_write_read_blocking(_spi, &_writeReadBuf[0], &buf[0], length+1);
    memcpy(data, &buf[1], length);
#else
    cs_select(_pins.cs);
    spi_write_blocking(_spi, &reg, 1);
    spi_read_blocking(_spi, 0, data, length);
    //const_cast<BUS_SPI*>(this)->readRegisterDMA(reg, data, length);
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
    CS_LOW();
    _spi.transfer(reg | READ_BIT);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    CS_HIGH();
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
    std::array<uint8_t, 256> buf;
    dma_channel_configure(_dmaRx, &_dmaRxConfig,
                          &buf[0], // destination, write SPI data to data
                          &spi_get_hw(_spi)->dr, // source, read data from SPI
                          length+1, // element count (each element is 8 bits)
                          false); // don't start yet
    dma_start_channel_mask((1u << _dmaTx) | (1u << _dmaRx));
    // wait for rx to complete
    dma_channel_wait_for_finish_blocking(_dmaRx);
    memcpy(data, &buf[1], length);
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
#if defined(MAP_CS_FOR_SPI)
    spi_write_read_blocking(_spi, &_writeReadBuf[0], data, length);
#else
    cs_select(_pins.cs);
    spi_read_blocking(_spi, 0, data, length);
    cs_deselect(_pins.cs);
#endif
    return true;
#elif defined(FRAMEWORK_ESPIDF)
    *data = 0;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    *data = 0;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    CS_LOW();
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    CS_HIGH();
    _spi.endTransaction();
    return true;
#endif
    return false;
}

bool BUS_SPI::readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const
{
    (void)timeoutMs;
    return readBytes(data, length);
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, uint8_t data)
{
#if defined(FRAMEWORK_RPI_PICO)
    std::array<uint8_t, 2> outBuf = { reg & 0x7FU, data }; // remove read bit as this is a write
#if defined(MAP_CS_FOR_SPI)
    spi_write_read_blocking(_spi, &outBuf[0], &_writeReadBuf[0], 2);
#else
    cs_select(_pins.cs);
    spi_write_blocking(_spi, &outBuf[0], 2);
    cs_deselect(_pins.cs);
#endif
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    CS_LOW();
    _spi.transfer(reg);
    _spi.transfer(data);
    CS_HIGH();
    _spi.endTransaction();
#endif
    return 0;
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
    reg &= 0x7FU;
#if defined(MAP_CS_FOR_SPI)
    _writeReadBuf[0] = reg;
    memcpy(&_writeReadBuf[1], data, length);
    std::array<uint8_t, 256> buf;
    spi_write_read_blocking(_spi, &_writeReadBuf[0], &buf[0], length+1);
#else
    cs_select(_pins.cs);
    spi_write_blocking(_spi, &reg, 1);
    spi_write_blocking(_spi, data, length);
    cs_deselect(_pins.cs);
#endif
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
    CS_LOW();
    _spi.transfer(reg);
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    CS_HIGH();
    _spi.endTransaction();
#endif
    return 0;
}

uint8_t BUS_SPI::writeBytes(const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
#if defined(MAP_CS_FOR_SPI)
    spi_write_read_blocking(_spi, data, &_writeReadBuf[0], length);
#else
    cs_select(_pins.cs);
    spi_write_blocking(_spi, data, length);
    cs_deselect(_pins.cs);
#endif
#elif defined(FRAMEWORK_ESPIDF)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    CS_LOW();
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    CS_HIGH();
    _spi.endTransaction();
#endif
    return 0;
}
