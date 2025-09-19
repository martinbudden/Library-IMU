#include "BUS_I2C.h"

#include <cassert>
#if defined(FRAMEWORK_RPI_PICO)
//#include <boards/pico.h> // for PICO_DEFAULT_LED_PIN
#include <hardware/i2c.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
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
void __not_in_flash_func(BUS_I2C::dataReadyISR)(unsigned int gpio, uint32_t events) // NOLINT(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
{
    (void)gpio;
    (void)events;
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
    // reading the IMU register resets the interrupt
    bus->readDeviceData();
    bus->SIGNAL_DATA_READY_FROM_ISR();
}
#else
FAST_CODE void BUS_I2C::dataReadyISR()
{
    // reading the IMU register resets the interrupt
    bus->readDeviceData();
    bus->SIGNAL_DATA_READY_FROM_ISR();
}
#endif

BUS_I2C::BUS_I2C(uint8_t I2C_address, bus_index_e I2C_index, const port_pins_t& pins) :
    _I2C_index(I2C_index),
    _pins(pins),
#if defined(FRAMEWORK_RPI_PICO)
    _I2C(I2C_index == BUS_INDEX_1 ? i2c1 : i2c0),
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    _wire(Wire),
#endif
    _I2C_address(I2C_address)
{
#if defined(FRAMEWORK_STM32_CUBE)
    _I2C.Instance = (I2C_index == BUS_INDEX_1) ? I2C2 : I2C1;
#endif
    init();
}

BUS_I2C::BUS_I2C(uint8_t I2C_address, bus_index_e I2C_index, const pins_t& pins) :
    _I2C_index(I2C_index),
#if defined(FRAMEWORK_RPI_PICO)
    _I2C(I2C_index == BUS_INDEX_1 ? i2c1 : i2c0),
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    _wire(Wire),
#endif
    _I2C_address(I2C_address)
{
#if defined(FRAMEWORK_STM32_CUBE)
    _I2C.Instance = (I2C_index == BUS_INDEX_1) ? I2C2 : I2C1;
#endif

    _pins.sda = {0,pins.sda};
    _pins.scl = {0,pins.scl};
    _pins.irq = {0,pins.irq};
    init();
}

void BUS_I2C::init()
{
    bus = this;

#if defined(FRAMEWORK_RPI_PICO)
    static_assert(static_cast<int>(IRQ_LEVEL_LOW) == GPIO_IRQ_LEVEL_LOW);
    static_assert(static_cast<int>(IRQ_LEVEL_HIGH) == GPIO_IRQ_LEVEL_HIGH);
    static_assert(static_cast<int>(IRQ_EDGE_FALL) == GPIO_IRQ_EDGE_FALL);
    static_assert(static_cast<int>(IRQ_EDGE_RISE) == GPIO_IRQ_EDGE_RISE);

    enum { BUS_400_KHZ = 400000 };
    i2c_init(_I2C, BUS_400_KHZ);
    gpio_set_function(_pins.sda.pin, GPIO_FUNC_I2C); // PICO_DEFAULT_I2C_SDA_PIN is GP4
    gpio_set_function(_pins.scl.pin, GPIO_FUNC_I2C); // PICO_DEFAULT_I2C_SCL_PIN is GP5
    gpio_pull_up(_pins.sda.pin);
    gpio_pull_up(_pins.scl.pin);
    // Make the I2C pins available to pictooof
    bi_decl(bi_2pins_with_func(_pins.sda.pin, _pins.scl.pin, GPIO_FUNC_I2C));

#elif defined(FRAMEWORK_ESPIDF)

    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = 0,//TEST_I2C_PORT,
        .sda_io_num = static_cast<gpio_num_t>(_pins.sda.pin),
        .scl_io_num = static_cast<gpio_num_t>(_pins.scl.pin),
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

#elif defined(FRAMEWORK_STM32_CUBE)

    _I2C.Init.ClockSpeed = 400000;
    _I2C.Init.DutyCycle = I2C_DUTYCYCLE_2;
    _I2C.Init.OwnAddress1 = 0;
    _I2C.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    _I2C.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    _I2C.Init.OwnAddress2 = 0;
    _I2C.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    _I2C.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&_I2C);

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO

#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
    _wire.begin(_pins.sda.pin, _pins.scl.pin);
#else
    _wire.begin();
#endif

#if defined(FRAMEWORK_USE_FREERTOS)
    _dataReadyQueue = xQueueCreateStatic(IMU_DATA_READY_QUEUE_LENGTH, sizeof(_dataReadyQueueItem), &_dataReadyQueueStorageArea[0], &_dataReadyQueueStatic);
    configASSERT(_dataReadyQueue);
    const UBaseType_t messageCount = uxQueueMessagesWaiting(_dataReadyQueue);
    assert(messageCount == 0);
    (void)messageCount;
#endif

#endif // FRAMEWORK
}

#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
BUS_I2C::BUS_I2C(uint8_t I2C_address, TwoWire& wire, const pins_t& pins) :
    _I2C_index(BUS_INDEX_0),
    _wire(wire),
    _I2C_address(I2C_address)
{
    bus = this;

    _pins.sda = {0,pins.sda};
    _pins.scl = {0,pins.scl};
    _pins.irq = {0,pins.irq};
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
    _wire.begin(pins.sda, pins.scl);
#else
    _wire.begin();
#endif
#if defined(FRAMEWORK_USE_FREERTOS)
    _dataReadyQueue = xQueueCreateStatic(IMU_DATA_READY_QUEUE_LENGTH, sizeof(_dataReadyQueueItem), &_dataReadyQueueStorageArea[0], &_dataReadyQueueStatic);
    configASSERT(_dataReadyQueue);
    const UBaseType_t messageCount = uxQueueMessagesWaiting(_dataReadyQueue);
    assert(messageCount == 0);
    (void)messageCount;
#endif
}
#endif

BUS_I2C::BUS_I2C(uint8_t I2C_address, bus_index_e I2C_index)
#if defined(FRAMEWORK_RPI_PICO)
    : BUS_I2C(I2C_address, I2C_index,
        I2C_index == BUS_INDEX_0 ? pins_t{.sda=PICO_DEFAULT_I2C_SDA_PIN, .scl=PICO_DEFAULT_I2C_SCL_PIN, .irq=IRQ_NOT_SET} :
                                   pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET})
#elif defined(FRAMEWORK_ESPIDF)
    : BUS_I2C(I2C_address, I2C_index, pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET})
#elif defined(FRAMEWORK_STM32_CUBE)
    : BUS_I2C(I2C_address, I2C_index, pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET})
#elif defined(FRAMEWORK_TEST)
    : BUS_I2C(I2C_address, I2C_index, pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET})
#else // defaults to FRAMEWORK_ARDUINO
    : BUS_I2C(I2C_address, I2C_index, pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET})
#endif // FRAMEWORK
{
}

void BUS_I2C::setInterruptDriven(irq_level_e irqLevel) // NOLINT(readability-make-member-function-const)
{
    assert(_pins.irq.pin != IRQ_NOT_SET);

#if defined(FRAMEWORK_RPI_PICO)
    gpio_init(_pins.irq.pin);
    enum { IRQ_ENABLED = true };
    gpio_set_irq_enabled_with_callback(_pins.irq.pin, irqLevel, IRQ_ENABLED, &dataReadyISR);
#elif defined(FRAMEWORK_ARDUINO_ESP32)
    //pinMode(_pins.irq.pin, INPUT);
    // map to ESP32 constants
    enum { LEVEL_LOW = 0x04, LEVEL_HIGH = 0x05, EDGE_FALL = 0x02, EDGE_RISE = 0x01, EDGE_CHANGE = 0x03 };
    const uint8_t level =
        (irqLevel == IRQ_LEVEL_LOW) ? LEVEL_LOW :
        (irqLevel == IRQ_LEVEL_HIGH) ? LEVEL_HIGH :
        (irqLevel == IRQ_EDGE_FALL) ? EDGE_FALL :
        (irqLevel == IRQ_EDGE_RISE) ? EDGE_RISE : EDGE_CHANGE;
    (void)level;
    //attachInterrupt(digitalPinToInterrupt(_pins.irq), &dataReadyISR, irqLevel); // esp32-hal-gpio.h
#else
    enum { LEVEL_LOW = 0x04, LEVEL_HIGH = 0x05, EDGE_FALL = 0x02, EDGE_RISE = 0x01, EDGE_CHANGE = 0x03 };
    const uint8_t level =
        (irqLevel == IRQ_LEVEL_LOW) ? LEVEL_LOW :
        (irqLevel == IRQ_LEVEL_HIGH) ? LEVEL_HIGH :
        (irqLevel == IRQ_EDGE_FALL) ? EDGE_FALL :
        (irqLevel == IRQ_EDGE_RISE) ? EDGE_RISE : EDGE_CHANGE;
    (void)level;
#endif
}

FAST_CODE uint8_t BUS_I2C::readRegister(uint8_t reg) const
{
    uint8_t ret = 0; // NOLINT(misc-const-correctness)
#if defined(FRAMEWORK_RPI_PICO)
    i2c_write_blocking(_I2C, _I2C_address, &reg, 1, RETAIN_CONTROL_OF_BUS);
    i2c_read_blocking(_I2C, _I2C_address, &ret, 1, DONT_RETAIN_CONTROL_OF_BUS);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_I2C, _I2C_address, &reg, 1, HAL_MAX_DELAY); //Sending in Blocking mode
    HAL_I2C_Master_Receive (&_I2C, _I2C_address, &ret, 1, HAL_MAX_DELAY);
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.endTransmission();

    if (_wire.requestFrom(_I2C_address, 1U)) {
        return static_cast<uint8_t>(_wire.read());
    }
#endif
    return ret;
}

FAST_CODE uint8_t BUS_I2C::readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const
{
    uint8_t ret = 0; // NOLINT(misc-const-correctness)
#if defined(FRAMEWORK_RPI_PICO)
    i2c_write_blocking(_I2C, _I2C_address, &reg, 1, RETAIN_CONTROL_OF_BUS);
    enum { MILLISECONDS_TO_MICROSECONDS = 1000 };
    i2c_read_timeout_us(_I2C, _I2C_address, &ret, 1, DONT_RETAIN_CONTROL_OF_BUS, timeoutMs * MILLISECONDS_TO_MICROSECONDS);
    return ret;
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)timeoutMs;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_I2C,_I2C_address, &reg, 1, timeoutMs); //Sending in Blocking mode
    HAL_I2C_Master_Receive (&_I2C,_I2C_address, &ret, 1, timeoutMs);
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
    return ret;
}

FAST_CODE bool BUS_I2C::readDeviceData()
{
    return readRegister(_deviceDataRegister, _readBuf + SPI_BUFFER_SIZE, _readLength - SPI_BUFFER_SIZE); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

FAST_CODE bool BUS_I2C::readRegister(uint8_t reg, uint8_t* data, size_t length) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    i2c_write_blocking(_I2C, _I2C_address, &reg, 1, RETAIN_CONTROL_OF_BUS);
    const int bytesRead = i2c_read_blocking(_I2C, _I2C_address, data, length, DONT_RETAIN_CONTROL_OF_BUS);
    if (bytesRead <= 0) {
        return false;
    }
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_I2C,_I2C_address, &reg, 1, HAL_MAX_DELAY); //Sending in Blocking mode
    const HAL_StatusTypeDef status = HAL_I2C_Master_Receive (&_I2C,_I2C_address, data, length, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return false;
    }
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    const uint8_t err = _wire.endTransmission(false);
    if (err == 0) {
        return readBytes(data, length);
    }
#endif
    return false;
}

FAST_CODE bool BUS_I2C::readBytes(uint8_t* data, size_t length) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    i2c_read_blocking(_I2C, _I2C_address, data, length, false);
    return true;
#elif defined(FRAMEWORK_ESPIDF)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_STM32_CUBE)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    if (_wire.requestFrom(_I2C_address, static_cast<uint8_t>(length))) {
        //while (_wire.available() < 1) { delay(1); }
        for (size_t ii = 0; ii < length; ++ii) {
            data[ii] = static_cast<uint8_t>(_wire.read()); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
        return true;
    }
#endif
    return false;
}

FAST_CODE bool BUS_I2C::readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    enum { MILLISECONDS_TO_MICROSECONDS = 1000 };
    i2c_read_timeout_us(_I2C, _I2C_address, data, length, false, timeoutMs * MILLISECONDS_TO_MICROSECONDS);
    return true;
#elif defined(FRAMEWORK_ESPIDF)
    *data = 0;
    (void)length;
    (void)timeoutMs;
#elif defined(FRAMEWORK_STM32_CUBE)
    const HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&_I2C,_I2C_address, data, length, timeoutMs);
    if (status != HAL_OK) {
        return false;
    }
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

FAST_CODE uint8_t BUS_I2C::writeRegister(uint8_t reg, uint8_t data)
{
#if defined(FRAMEWORK_RPI_PICO)
    std::array<uint8_t, 2> buf = { reg, data };
    i2c_write_blocking(_I2C, _I2C_address, &buf[0], sizeof(buf), false);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
#elif defined(FRAMEWORK_STM32_CUBE)
    std::array<uint8_t, 2> buf = { reg, data };
    HAL_I2C_Master_Transmit(&_I2C,_I2C_address, &buf[0], sizeof(buf), HAL_MAX_DELAY); //Sending in Blocking mode
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

FAST_CODE uint8_t BUS_I2C::writeRegister(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
    i2c_write_blocking(_I2C, _I2C_address, &reg, 1, false);
    return i2c_write_blocking(_I2C, _I2C_address, data, length, false);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_I2C,_I2C_address, &reg, 1, HAL_MAX_DELAY); //Sending in Blocking mode
    HAL_I2C_Master_Transmit(&_I2C,_I2C_address, const_cast<uint8_t*>(data), length, HAL_MAX_DELAY); //Sending in Blocking mode
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

FAST_CODE uint8_t BUS_I2C::writeBytes(const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
    return i2c_write_blocking(_I2C, _I2C_address, data, length, false);
#elif defined(FRAMEWORK_ESPIDF)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_I2C,_I2C_address, const_cast<uint8_t*>(data), length, HAL_MAX_DELAY); //Sending in Blocking mode
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
