#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#if defined(FRAMEWORK_USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif
#if defined(FRAMEWORK_RPI_PICO)
#include <pico/mutex.h>
#endif

/*!
Base class for BUS_I2C and BUS_SPI.

Note
bus_index is zero-based.

RPI Pico bus index is also zero-based so, for SPI, BUS_INDEX_0 corresponds to spi0
STM32 bus index is also one-based so, for SPI, BUS_INDEX_0 corresponds to SPI1
*/
class BUS_BASE {
public:
    enum bus_index_e : uint8_t { BUS_INDEX_0, BUS_INDEX_1, BUS_INDEX_2, BUS_INDEX_3, BUS_INDEX_4, BUS_INDEX_5, BUS_INDEX_6, BUS_INDEX_7 };
    enum { SPI_BUFFER_SIZE = 2};
    enum { IRQ_NOT_SET = 0xFF };
    enum irq_level_e { IRQ_LEVEL_LOW = 0x01, IRQ_LEVEL_HIGH = 0x02, IRQ_EDGE_FALL = 0x04, IRQ_EDGE_RISE = 0x08, IRQ_EDGE_CHANGE = 0x04|0x08 };
    struct port_pin_t {
        uint8_t port;
        uint8_t pin;
    };
public:
    inline void setDeviceDataRegister(uint8_t deviceDataRegister, uint8_t* readBuf, size_t readLength) {
        _deviceDataRegister = deviceDataRegister;
        _readBuf = readBuf;
        _readLength = readLength;
    }
protected:
    uint8_t _deviceDataRegister {}; // the device register that is read in the readDeviceData() function
    uint8_t* _readBuf {};
    size_t _readLength {};
#if defined(FRAMEWORK_USE_FREERTOS)
    mutable uint32_t _dataReadyQueueItem {}; // this is just a dummy item whose value is not used
    enum { IMU_DATA_READY_QUEUE_LENGTH = 1 };
    std::array<uint8_t, IMU_DATA_READY_QUEUE_LENGTH * sizeof(_dataReadyQueueItem)> _dataReadyQueueStorageArea {};
    StaticQueue_t _dataReadyQueueStatic {};
    QueueHandle_t _dataReadyQueue {};
public:
    inline int32_t WAIT_DATA_READY() const { return xQueueReceive(_dataReadyQueue, &_dataReadyQueueItem, portMAX_DELAY); }
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) const { return xQueueReceive(_dataReadyQueue, &_dataReadyQueueItem, ticksToWait); } // returns pdPASS(1) if queue read, pdFAIL(0) if timeout
    inline void SIGNAL_DATA_READY_FROM_ISR() const { xQueueSendFromISR(_dataReadyQueue, &_dataReadyQueueItem, nullptr); }
#elif defined(FRAMEWORK_RPI_PICO)
    mutable mutex_t _dataReadyMutex{};
public:
    inline int32_t WAIT_DATA_READY() const { mutex_enter_blocking(&_dataReadyMutex); return 0; }
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) const { return mutex_enter_timeout_ms(&_dataReadyMutex, ticksToWait); } // returns true if mutex owned, false if timeout
    inline void SIGNAL_DATA_READY_FROM_ISR() const { mutex_exit(&_dataReadyMutex); }
#else
public:
    inline int32_t WAIT_DATA_READY() const { return 0; }
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) const { (void)ticksToWait; return 0; }
    inline void SIGNAL_DATA_READY_FROM_ISR() const {}
#endif
};
