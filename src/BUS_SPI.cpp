#include "BUS_SPI.h"

//#define MAP_CS_FOR_SPI // not got this working, it's probably not worth the bother
//#define MAP_CS_FOR_SPI_WRITE_READ

#include <cassert>

#if defined(FRAMEWORK_RPI_PICO)
#include <boards/pico.h> // for PICO_DEFAULT_LED_PIN
#include <hardware/dma.h>
#include <hardware/spi.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#if defined(MAP_CS_FOR_SPI_WRITE_READ)
#include <cstring>
#endif
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#include <SPI.h>
#endif


BUS_SPI* BUS_SPI::bus {nullptr}; // copy of this for use in ISRs

#if defined(FRAMEWORK_RPI_PICO)
static inline void cs_select(uint8_t CS_pin) {
#if defined(MAP_CS_FOR_SPI_WRITE_READ)
    (void)CS_pin;
#else
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_pin, 0);  // Active low
    asm volatile("nop \n nop \n nop");
#endif
}

static inline void cs_deselect(uint8_t CS_pin) {
#if defined(MAP_CS_FOR_SPI_WRITE_READ)
    (void)CS_pin;
#else
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_pin, 1);
    asm volatile("nop \n nop \n nop");
#endif
}
#else
#define CS_LOW() ({*_csOut &= ~_csBit;}) // NOLINT(cppcoreguidelines-macro-usage)
#define CS_HIGH() ({*_csOut |= _csBit;}) // NOLINT(cppcoreguidelines-macro-usage)
//#define CS_LOW() ({digitalWrite(_pins.cs, LOW);})
//#define CS_HIGH() ({digitalWrite(_pins.cs, HIGH);})
#endif // FRAMEWORK

/*!
Data ready interrupt service routine (ISR)

Currently support only one interrupt, but could index action off gpio pin
*/
#if defined(FRAMEWORK_RPI_PICO)
void BUS_SPI::dataReadyISR(unsigned int gpio, uint32_t events)
{
    // reading the register resets the interrupt
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
#if defined(USE_IMU_SPI_DMA_IN_ISR)
    // data ready signalled in dmaRxCompleteISR
    const int start = SPI_BUFFER_SIZE - 1;
    enum { START_NOW = true, DONT_START_YET = false };
    dma_channel_set_trans_count(bus->_dmaTxChannel, bus->_readLength - start, DONT_START_YET);
    dma_channel_set_trans_count(bus->_dmaRxChannel, bus->_readLength - start, DONT_START_YET);
    dma_channel_set_write_addr(bus->_dmaRxChannel, bus->_readBuf + start, DONT_START_YET);
    cs_select(bus->_pins.cs);
    dma_start_channel_mask((1u << bus->_dmaTxChannel) | (1u << bus->_dmaRxChannel));
    // wait for rx to complete
    dma_channel_wait_for_finish_blocking(bus->_dmaRxChannel);
    cs_deselect(bus->_pins.cs);
#else
    bus->readDeviceRegisterDMA();
    bus->SIGNAL_DATA_READY_FROM_ISR();
#endif
}

void BUS_SPI::dmaRxCompleteISR()
{
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
    dma_channel_acknowledge_irq0(bus->_dmaRxChannel);
    //cs_deselect(bus->_pins.cs);
    bus->SIGNAL_DATA_READY_FROM_ISR();
}
#else
INSTRUCTION_RAM_ATTR void BUS_SPI::dataReadyISR()
{
#if defined(USE_IMU_SPI_DMA)
    static_assert(false); // assert false until this is implemented
#else
    // for the moment, just read the predefined register into the predefined read buffer
    bus->readDeviceRegister();
    bus->SIGNAL_DATA_READY_FROM_ISR();
#endif
}
#endif

#if defined(USE_IMU_SPI_DMA)
BUS_SPI::~BUS_SPI()
{
#if defined(FRAMEWORK_RPI_PICO)
    dma_channel_unclaim(_dmaRxChannel);
    dma_channel_cleanup(_dmaRxChannel);
    dma_channel_unclaim(_dmaTxChannel);
    dma_channel_cleanup(_dmaTxChannel);
#endif
}
#endif

BUS_SPI::BUS_SPI(uint32_t frequency, spi_index_e SPI_index, uint8_t CS_pin)
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

BUS_SPI::BUS_SPI(uint32_t frequency, spi_index_e SPI_index, const pins_t& pins) :
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

    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_LSB_FIRST); // channel, bits per transfer, polarity, phase, order
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
    mutex_init(&_dataReadyMutex);

#elif defined(FRAMEWORK_ESPIDF)

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
    _spi.begin();
    _csBit = digitalPinToBitMask(_pins.cs); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    _csOut = portOutputRegister(digitalPinToPort(_pins.cs)); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    pinMode(_pins.cs, OUTPUT);
    CS_HIGH();
#if defined(USE_FREERTOS)
    _dataReadyQueue = xQueueCreateStatic(IMU_DATA_READY_QUEUE_LENGTH, sizeof(_dataReadyQueueItem), &_dataReadyQueueStorageArea[0], &_dataReadyQueueStatic);
    configASSERT(_dataReadyQueue);
    const UBaseType_t messageCount = uxQueueMessagesWaiting(_dataReadyQueue);
    assert(messageCount == 0);
#endif

#endif // FRAMEWORK
    configureDMA();
}

void BUS_SPI::configureDMA()
{
#if defined(USE_IMU_SPI_DMA)
#if defined(FRAMEWORK_RPI_PICO)
    enum { START_NOW = true, DONT_START_YET = false };
    const int start = SPI_BUFFER_SIZE - 1;

    // DMA transmit channel, writes value of IMU register to SPI
    _dmaTxChannel = dma_claim_unused_channel(true);
    dma_channel_config dmaTxChannelConfig = dma_channel_get_default_config(_dmaTxChannel);
    channel_config_set_transfer_data_size(&dmaTxChannelConfig, DMA_SIZE_8); // 8-bit transfers
    channel_config_set_dreq(&dmaTxChannelConfig, spi_get_dreq(_spi, true));
    channel_config_set_write_increment(&dmaTxChannelConfig, false); // always write to SPI data register
    channel_config_set_read_increment(&dmaTxChannelConfig, false); // don't increment, we value of IMU register each time
    channel_config_set_irq_quiet(&dmaTxChannelConfig, true); // no IRQs on transmit channel
    dma_channel_configure(_dmaTxChannel, &dmaTxChannelConfig,
                        &spi_get_hw(_spi)->dr, // destination, write data to SPI data register
                        &_deviceRegister, // source, send the value of the IMU register we want to read
                        _readLength - start, // number of bytes to read (each element is DMA_SIZE_8, ie 8 bits)
                        DONT_START_YET); // don't start yet

    // DMA receive channel, reads from SPI to _readBuf
    _dmaRxChannel = dma_claim_unused_channel(true);
    dma_channel_config dmaRxChannelConfig = dma_channel_get_default_config(_dmaRxChannel);
    channel_config_set_dreq(&dmaRxChannelConfig, spi_get_dreq(_spi, false));
    channel_config_set_transfer_data_size(&dmaRxChannelConfig, DMA_SIZE_8); // 8-bit transfers
    channel_config_set_write_increment(&dmaRxChannelConfig, true);
    channel_config_set_read_increment(&dmaRxChannelConfig, false); // always read from SPI data register
#if defined(USE_IMU_SPI_DMA_IN_ISR)
    channel_config_set_irq_quiet(&dmaRxChannelConfig, false);
    dma_channel_set_irq0_enabled(_dmaRxChannel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dmaRxCompleteISR);
    irq_set_enabled(DMA_IRQ_0, true);
#else
    channel_config_set_irq_quiet(&dmaRxChannelConfig, true);
#endif
    dma_channel_configure(_dmaRxChannel, &dmaRxChannelConfig,
                        _readBuf + start, // destination, write data to readBuf
                        &spi_get_hw(_spi)->dr, // source, read data from SPI data register
                        _readLength - start, // number of bytes to read (each element is DMA_SIZE_8, ie 8 bits)
                        DONT_START_YET); // don't start yet
#endif // FRAMEWORK_RPI_PICO
#endif // USE_IMU_SPI_DMA
}

void BUS_SPI::setDeviceRegister(uint8_t deviceRegister, uint8_t* readBuf, size_t readLength)
{
    _deviceRegister = deviceRegister | READ_BIT;
    _readBuf = readBuf;
    _readLength = readLength;
}

/*!
Set this bus so reads interrupt driven.
When the IMU interrupt pin indicates data ready, the dataReadyISR is called and the data is read in the ISR.

This routine sets the GPIO IRQ pin to input an attaches the dataReadyISR to be triggered by that pin.
*/
void BUS_SPI::setInterruptDriven() // NOLINT(readability-make-member-function-const)
{
    assert(_pins.irq != IRQ_NOT_SET);

#if defined(USE_ARDUINO_ESP32)
    pinMode(_pins.irq, INPUT);
    attachInterrupt(digitalPinToInterrupt(_pins.irq), &dataReadyISR, _pins.irqLevel); // esp32-hal-gpio.h
#elif defined(FRAMEWORK_RPI_PICO)
    assert(_pins.irqLevel != 0);
    gpio_init(_pins.irq);
    gpio_set_dir(_pins.irq, GPIO_IN);
    enum { IRQ_ENABLED = true };
    gpio_set_irq_enabled_with_callback(_pins.irq, _pins.irqLevel, IRQ_ENABLED, &dataReadyISR);
#endif
}

uint8_t BUS_SPI::readRegister(uint8_t reg) const
{
#if defined(FRAMEWORK_RPI_PICO)
    cs_select(_pins.cs);
    std::array<uint8_t, 2> outBuf = { reg | READ_BIT, 0 };
    std::array<uint8_t, 2> inBuf;
    spi_write_read_blocking(_spi, &outBuf[0], &inBuf[0], 2);
    reg = inBuf[1];
    cs_deselect(_pins.cs);
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

bool BUS_SPI::readDeviceRegisterDMA()
{
//dma_channel_set_irq0_enabled(channel, enabled);
//dma_channel_acknowledge_irq0(channel)
//dma_channel_get_irq0_status (uint channel)
//dma_channel_set_read_addr
//dma_channel_set_write_addr
//dma_channel_set_trans_count
#if defined(USE_IMU_SPI_DMA)
#if defined(FRAMEWORK_RPI_PICO)
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
    const int start = SPI_BUFFER_SIZE - 1;
    enum { START_NOW = true, DONT_START_YET = false };
    dma_channel_set_trans_count(_dmaTxChannel, _readLength - start, DONT_START_YET);
    dma_channel_set_trans_count(_dmaRxChannel, _readLength - start, DONT_START_YET);
    dma_channel_set_write_addr(_dmaRxChannel, _readBuf + start, DONT_START_YET);
#if 0
    dma_channel_configure(_dmaTxChannel, &dmaTxChannelConfig,
                        &spi_get_hw(_spi)->dr, // destination, write data to SPI data register
                        &_deviceRegister, // source, send the value of the IMU register we want to read
                        _readLength - start, // number of bytes to read (each element is DMA_SIZE_8, ie 8 bits)
                        DONT_START_YET); // don't start yet
    dma_channel_configure(_dmaRxChannel, &dmaRxChannelConfig,
                          _readBuf + start, // destination, write SPI data to data
                          &spi_get_hw(_spi)->dr, // source, read data from SPI
                          _readLength - start, // element count (each element is 8 bits)
                          DONT_START_YET); // don't start yet
#endif
    cs_select(_pins.cs);
    dma_start_channel_mask((1u << _dmaTxChannel) | (1u << _dmaRxChannel));
    // wait for rx to complete
    dma_channel_wait_for_finish_blocking(_dmaRxChannel);
    cs_deselect(_pins.cs);
    return true;
#else
    return readDeviceRegister();
#endif
#else
    return readDeviceRegister();
#endif // USE_IMU_SPI_DMA
}

bool BUS_SPI::readDeviceRegister()
{
#if defined(FRAMEWORK_RPI_PICO)
    cs_select(_pins.cs);
    const int start = SPI_BUFFER_SIZE - 1;
    _readBuf[start] = _deviceRegister;
    spi_write_read_blocking(_spi, _readBuf + start, _readBuf + start, _readLength - start);
    cs_deselect(_pins.cs);
    return true;
#else
    return readRegister(_deviceRegister, _readBuf + SPI_BUFFER_SIZE, _readLength - SPI_BUFFER_SIZE); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
#endif
}

bool BUS_SPI::readRegister(uint8_t reg, uint8_t* data, size_t length) const
{
#if defined(FRAMEWORK_RPI_PICO)
    reg |= READ_BIT;
#if defined(MAP_CS_FOR_SPI_WRITE_READ)
    _writeReadBuf[0] = reg;
    std::array<uint8_t, 256> buf;
    spi_write_read_blocking(_spi, &_writeReadBuf[0], &buf[0], length+1);
    memcpy(data, &buf[1], length);
#else
    cs_select(_pins.cs);
    spi_write_blocking(_spi, &reg, 1);
    spi_read_blocking(_spi, 0, data, length); // 0 is the value written as SPI is being read
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

bool BUS_SPI::readBytes(uint8_t* data, size_t length) const
{
#if defined(FRAMEWORK_RPI_PICO)
#if defined(MAP_CS_FOR_SPI_WRITE_READ)
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
    std::array<uint8_t, 2> inBuf;
    cs_select(_pins.cs);
    spi_write_read_blocking(_spi, &outBuf[0], &inBuf[0], 2);
    cs_deselect(_pins.cs);
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
#if defined(MAP_CS_FOR_SPI_WRITE_READ)
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
#if defined(MAP_CS_FOR_SPI_WRITE_READ)
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
