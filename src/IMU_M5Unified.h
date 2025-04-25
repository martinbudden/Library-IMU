#pragma once

#include "IMU_Base.h"


class IMU_M5_UNIFIED : public IMU_Base {
public:
    IMU_M5_UNIFIED(axis_order_t axisOrder, void* i2cMutex);
public:
    virtual int init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity) override;
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() override;

private:
    uint8_t _fifoBuffer[1024] {};
};
