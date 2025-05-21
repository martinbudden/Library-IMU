#include <IMU_Base.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>


class AHRS {
public:
    explicit AHRS(IMU_Base& imu);
    inline void LOCK_IMU_DATA_READY() const { xQueueReceive(_imuDataReadyQueue, &_imuDataReadyQueueItem, portMAX_DELAY); }
private:
    IMU_Base& _IMU;
    mutable uint32_t _imuDataReadyQueueItem {}; // this is just a dummy item whose value is not used
    enum { IMU_DATA_READY_QUEUE_LENGTH = 8 };
    std::array<uint8_t, IMU_DATA_READY_QUEUE_LENGTH * sizeof(_imuDataReadyQueueItem)> _imuDataReadyQueueStorageArea {};
    StaticQueue_t _imuDataReadyQueueStatic {};
    QueueHandle_t _imuDataReadyQueue {};
};
