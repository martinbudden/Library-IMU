#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif
#if defined(FRAMEWORK_RPI_PICO)
#include <pico/mutex.h>
#endif

class BUS_BASE {
public:
    enum { SPI_BUFFER_SIZE = 2};
    enum { IRQ_NOT_SET = 0xFF };
#if defined(FRAMEWORK_RPI_PICO)
    enum { IRQ_LEVEL_LOW = 0x1U, IRQ_LEVEL_HIGH = 0x2U, IRQ_EDGE_FALL = 0x4U, IRQ_EDGE_RISE = 0x8U, IRQ_EDGE_CHANGE = 0x4U|0x8U };
#else
    enum { IRQ_LEVEL_LOW = 0x04, IRQ_LEVEL_HIGH = 0x05, IRQ_EDGE_FALL = 0x02, IRQ_EDGE_RISE = 0x01, IRQ_EDGE_CHANGE = 0x03 };
#endif
protected:
    uint8_t _deviceRegister {}; // the device register that is read in the ISR
    uint8_t* _readBuf {};
    size_t _readLength {};
#if defined(FRAMEWORK_RPI_PICO)
    mutable mutex_t _dataReadyMutex{};
public:
    inline int32_t WAIT_DATA_READY() const { mutex_enter_blocking(&_dataReadyMutex); return 0; }
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) const { return mutex_enter_timeout_ms(&_dataReadyMutex, ticksToWait); } // returns true if mutex owned, false if timeout
    inline void SIGNAL_DATA_READY_FROM_ISR() const { mutex_exit(&_dataReadyMutex); }
#elif defined(USE_FREERTOS)
    mutable uint32_t _dataReadyQueueItem {}; // this is just a dummy item whose value is not used
    enum { IMU_DATA_READY_QUEUE_LENGTH = 1 };
    std::array<uint8_t, IMU_DATA_READY_QUEUE_LENGTH * sizeof(_dataReadyQueueItem)> _dataReadyQueueStorageArea {};
    StaticQueue_t _dataReadyQueueStatic {};
    QueueHandle_t _dataReadyQueue {};
public:
    inline int32_t WAIT_DATA_READY() const { return xQueueReceive(_dataReadyQueue, &_dataReadyQueueItem, portMAX_DELAY); }
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) const { return xQueueReceive(_dataReadyQueue, &_dataReadyQueueItem, ticksToWait); } // returns pdPASS(1) if queue read, pdFAIL(0) if timeout
    inline void SIGNAL_DATA_READY_FROM_ISR() const { xQueueSendFromISR(_dataReadyQueue, &_dataReadyQueueItem, nullptr); }
#else
public:
    inline int32_t WAIT_DATA_READY() const { return 0; }
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) const { (void)ticksToWait; return 0; }
    inline void SIGNAL_DATA_READY_FROM_ISR() const {}
#endif
};
