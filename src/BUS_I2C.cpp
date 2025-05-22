#include "BUS_I2C.h"

#include <cassert>
#if defined(FRAMEWORK_RPI_PICO)
#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <boards/pico.h> // for PICO_DEFAULT_LED_PIN
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#endif


BUS_I2C* BUS_I2C::bus {nullptr};

/*!
Data ready interrupt service routine (ISR)

Currently support only one interrupt, but could index action off the gpio pin
*/
#if defined(FRAMEWORK_RPI_PICO)
void BUS_I2C::dataReadyISR(unsigned int gpio, uint32_t events)
{
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
    // reading the IMU register resets the interrupt
    bus->readDeviceRegister();
    bus->SIGNAL_DATA_READY_FROM_ISR();
}
#else
INSTRUCTION_RAM_ATTR void BUS_I2C::dataReadyISR()
{
    // reading the IMU register resets the interrupt
    bus->readDeviceRegister();
    bus->SIGNAL_DATA_READY_FROM_ISR();
}
#endif

BUS_I2C::BUS_I2C(uint8_t I2C_address, i2c_index_t I2C_index, const pins_t& pins) :
    _I2C_index(I2C_index),
    _pins(pins),
#if defined(FRAMEWORK_RPI_PICO)
    _I2C(I2C_index == I2C_INDEX_1 ? i2c1 : i2c0),
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    _wire(Wire),
#endif
    _I2C_address(I2C_address)
{
    bus = this;

#if defined(FRAMEWORK_RPI_PICO)
    static_assert(static_cast<int>(IRQ_LEVEL_LOW) == GPIO_IRQ_LEVEL_LOW);
    static_assert(static_cast<int>(IRQ_LEVEL_HIGH) == GPIO_IRQ_LEVEL_HIGH);
    static_assert(static_cast<int>(IRQ_EDGE_FALL) == GPIO_IRQ_EDGE_FALL);
    static_assert(static_cast<int>(IRQ_EDGE_RISE) == GPIO_IRQ_EDGE_RISE);

    i2c_init(_I2C, 400 * 1000);
    gpio_set_function(_pins.sda, GPIO_FUNC_I2C); // PICO_DEFAULT_I2C_SDA_PIN is GP4
    gpio_set_function(_pins.scl, GPIO_FUNC_I2C); // PICO_DEFAULT_I2C_SCL_PIN is GP5
    gpio_pull_up(_pins.sda);
    gpio_pull_up(_pins.scl);
    // Make the I2C pins available to pictooof
    bi_decl(bi_2pins_with_func(_pins.sda, _pins.scl, GPIO_FUNC_I2C));

#elif defined(FRAMEWORK_ESPIDF)

    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = 0,//TEST_I2C_PORT,
        .sda_io_num = static_cast<gpio_num_t>(_pins.sda),
        .scl_io_num = static_cast<gpio_num_t>(_pins.scl),
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags {
            .enable_internal_pullup = true,
            .allow_pd = true
        }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &_bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _I2C_address,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags {
            .disable_ack_check = true
        }
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &dev_cfg, &_dev_handle));

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO

#if defined(USE_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
    _wire.begin(_pins.sda, _pins.scl);
#else
    _wire.begin();
#endif

#if defined(USE_FREERTOS)
    _dataReadyQueue = xQueueCreateStatic(IMU_DATA_READY_QUEUE_LENGTH, sizeof(_dataReadyQueueItem), &_dataReadyQueueStorageArea[0], &_dataReadyQueueStatic);
    configASSERT(_dataReadyQueue);
    const UBaseType_t messageCount = uxQueueMessagesWaiting(_dataReadyQueue);
    assert(messageCount == 0);
#endif

#endif // FRAMEWORK
}

#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_TEST)
BUS_I2C::BUS_I2C(uint8_t I2C_address, TwoWire& wire, const pins_t& pins) :
    _I2C_index(I2C_INDEX_0),
    _pins(pins),
    _wire(wire),
    _I2C_address(I2C_address)
{
    bus = this;

#if defined(USE_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
    _wire.begin(_pins.sda, _pins.scl);
#else
    _wire.begin();
#endif
#if defined(USE_FREERTOS)
    _dataReadyQueue = xQueueCreateStatic(IMU_DATA_READY_QUEUE_LENGTH, sizeof(_dataReadyQueueItem), &_dataReadyQueueStorageArea[0], &_dataReadyQueueStatic);
    configASSERT(_dataReadyQueue);
    const UBaseType_t messageCount = uxQueueMessagesWaiting(_dataReadyQueue);
    assert(messageCount == 0);
#endif
}
#endif

BUS_I2C::BUS_I2C(uint8_t I2C_address, i2c_index_t I2C_index)
#if defined(FRAMEWORK_RPI_PICO)
    : BUS_I2C(I2C_address, I2C_index,
        I2C_index == I2C_INDEX_0 ? pins_t{.sda=PICO_DEFAULT_I2C_SDA_PIN, .scl=PICO_DEFAULT_I2C_SCL_PIN, .irq=IRQ_NOT_SET, .irqLevel=0} : pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET, .irqLevel=0})
#elif defined(FRAMEWORK_ESPIDF)
    : BUS_I2C(I2C_address, I2C_index, pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET, .irqLevel=0})
#elif defined(FRAMEWORK_TEST)
    : BUS_I2C(I2C_address, I2C_index, pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET, .irqLevel=0})
#else // defaults to FRAMEWORK_ARDUINO
    : BUS_I2C(I2C_address, I2C_index, pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET, .irqLevel=0})
#endif // FRAMEWORK
{
}


void BUS_I2C::setDeviceRegister(uint8_t deviceRegister, uint8_t* readBuf, size_t readLength)
{
    _deviceRegister = deviceRegister;
    _readBuf = readBuf;
    _readLength = readLength;
}

void BUS_I2C::setInterruptDriven() // NOLINT(readability-make-member-function-const)
{
    assert(_pins.irq != IRQ_NOT_SET);

#if defined(FRAMEWORK_RPI_PICO)
    assert(_pins.irqLevel != 0);
    gpio_init(_pins.irq);
    enum { IRQ_ENABLED = true };
    gpio_set_irq_enabled_with_callback(_pins.irq, _pins.irqLevel, IRQ_ENABLED, &dataReadyISR);
#elif defined(USE_ARDUINO_ESP32)
    //pinMode(_pins.irq, INPUT);
    //attachInterrupt(digitalPinToInterrupt(_pins.irq), &dataReadyISR, _pins.irqLevel); // esp32-hal-gpio.h
#endif
}

uint8_t BUS_I2C::readRegister(uint8_t reg) const
{
#if defined(FRAMEWORK_RPI_PICO)
    i2c_write_blocking(_I2C, _I2C_address, &reg, 1, RETAIN_CONTROL_OF_BUS);
    uint8_t ret;
    i2c_read_blocking(_I2C, _I2C_address, &ret, 1, DONT_RETAIN_CONTROL_OF_BUS);
    return ret;
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.endTransmission();

    if (_wire.requestFrom(_I2C_address, 1U)) {
        return static_cast<uint8_t>(_wire.read());
    }
    return 0;
#endif
    return 0;
}

uint8_t BUS_I2C::readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const
{
#if defined(FRAMEWORK_RPI_PICO)
    i2c_write_blocking(_I2C, _I2C_address, &reg, 1, RETAIN_CONTROL_OF_BUS);
    uint8_t ret;
    i2c_read_timeout_us(_I2C, _I2C_address, &ret, 1, DONT_RETAIN_CONTROL_OF_BUS, timeoutMs * 1000);
    return ret;
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)timeoutMs;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)timeoutMs;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.endTransmission(false);

    _wire.requestFrom(_I2C_address, 1U);
    for (uint32_t ii = 0; ii < timeoutMs; ++ii) {
        if (_wire.available() > 0) {
            return static_cast<uint8_t>(_wire.read());
        }
        delay(1);
    }
#endif
    return 0;
}

bool BUS_I2C::readDeviceRegister()
{
    return readRegister(_deviceRegister, _readBuf + SPI_BUFFER_SIZE, _readLength - SPI_BUFFER_SIZE); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

bool BUS_I2C::readRegister(uint8_t reg, uint8_t* data, size_t length) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    i2c_write_blocking(_I2C, _I2C_address, &reg, 1, RETAIN_CONTROL_OF_BUS);
    i2c_read_blocking(_I2C, _I2C_address, data, length, DONT_RETAIN_CONTROL_OF_BUS);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.endTransmission();

    return readBytes(data, length);
#endif
    return false;
}

bool BUS_I2C::readBytes(uint8_t* data, size_t length) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    i2c_read_blocking(_I2C, _I2C_address, data, length, false);
    return true;
#elif defined(FRAMEWORK_ESPIDF)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    if (_wire.requestFrom(_I2C_address, static_cast<uint8_t>(length))) {
        for (size_t ii = 0; ii < length; ++ii) {
            data[ii] = static_cast<uint8_t>(_wire.read()); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
        return true;
    }
#endif
    return false;
}

bool BUS_I2C::readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    i2c_read_timeout_us(_I2C, _I2C_address, data, length, false, timeoutMs *1000);
    return true;
#elif defined(FRAMEWORK_ESPIDF)
    *data = 0;
    (void)length;
    (void)timeoutMs;
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
    (void)timeoutMs;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.requestFrom(_I2C_address, static_cast<uint8_t>(length));
    for (uint32_t ii = 0; ii < timeoutMs; ++ii) {
        if (_wire.available() > 0) {
            for (size_t jj = 0; jj < length; ++jj) {
                data[jj] = static_cast<uint8_t>(_wire.read()); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            }
            return true;
        }
        delay(1);
    }
    return false;
#endif
    return false;
}

uint8_t BUS_I2C::writeRegister(uint8_t reg, uint8_t data)
{
#if defined(FRAMEWORK_RPI_PICO)
    std::array<uint8_t, 2> buf = { reg, data };
    i2c_write_blocking(_I2C, _I2C_address, &buf[0], sizeof(buf), false);
    return 0;
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.write(data);
    return _wire.endTransmission();
#endif
    return 0;
}

uint8_t BUS_I2C::writeRegister(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
    i2c_write_blocking(_I2C, _I2C_address, &reg, 1, false);
    return i2c_write_blocking(_I2C, _I2C_address, data, length, false);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.write(data, length);
    return _wire.endTransmission();
#endif
    return 0;
}

uint8_t BUS_I2C::writeBytes(const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
    return i2c_write_blocking(_I2C, _I2C_address, data, length, false);
#elif defined(FRAMEWORK_ESPIDF)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_I2C_address);
    _wire.write(data, length);
    return _wire.endTransmission();
#endif
    return 0;
}
