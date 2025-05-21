#include "AHRS.h"


AHRS::AHRS(IMU_Base& imu) :
    _IMU(imu)
{
    static_assert(sizeof(_imuDataReadyQueue) == sizeof(int));

    // this fails if created in the initializer
    _imuDataReadyQueue = xQueueCreateStatic(IMU_DATA_READY_QUEUE_LENGTH, sizeof(_imuDataReadyQueueItem), &_imuDataReadyQueueStorageArea[0], &_imuDataReadyQueueStatic);
    configASSERT(_imuDataReadyQueue);
    const UBaseType_t messageCount = uxQueueMessagesWaiting(_imuDataReadyQueue);
    assert(messageCount == 0);

    /*uint8_t* pucQueueStorage;
    StaticQueue_t* pxStaticQueue;
    const BaseType_t err = xQueueGetStaticBuffers(_imuDataReadyQueue, &pucQueueStorage, &pxStaticQueue);
    assert(pucQueueStorage == &_imuDataReadyQueueStorageArea[0]);
    assert(pxStaticQueue == &_imuDataReadyQueueStatic);*/

    _IMU.setInterrupt(reinterpret_cast<int>(_imuDataReadyQueue));
}
