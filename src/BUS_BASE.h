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
    uint8_t _imuRegister {}; // the IMU register that we want to read
    uint8_t* _readBuf {};
    size_t _readLength {};
    int _userIrq {0};
#if defined(USE_FREERTOS)
    mutable uint32_t _imuDataReadyQueueItem {}; // this is just a dummy item whose value is not used
    enum { IMU_DATA_READY_QUEUE_LENGTH = 8 };
    std::array<uint8_t, IMU_DATA_READY_QUEUE_LENGTH * sizeof(_imuDataReadyQueueItem)> _imuDataReadyQueueStorageArea {};
    StaticQueue_t _imuDataReadyQueueStatic {};
    QueueHandle_t _imuDataReadyQueue {};
public:
    inline void LOCK_IMU_DATA_READY() const { xQueueReceive(_imuDataReadyQueue, &_imuDataReadyQueueItem, portMAX_DELAY); }
    inline void UNLOCK_IMU_DATA_READY_FROM_ISR() const { xQueueSendFromISR(_imuDataReadyQueue, &_imuDataReadyQueueItem, nullptr); }
#elif defined(FRAMEWORK_RPI_PICO)
    mutable mutex_t _imuDataReadyMutex{};
public:
    //inline void LOCK_IMU_DATA_READY() const { mutex_enter_blocking(&_imuDataReadyMutex); }
    //inline void UNLOCK_IMU_DATA_READY_FROM_ISR() const { mutex_exit(&_imuDataReadyMutex); }
#else
public:
    inline void LOCK_IMU_DATA_READY() const {}
    inline void UNLOCK_IMU_DATA_READY_FROM_ISR() const {}
#endif
};
