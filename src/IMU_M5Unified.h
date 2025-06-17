#pragma once

#include "IMU_Base.h"


class IMU_M5_UNIFIED : public IMU_Base {
public:
    explicit IMU_M5_UNIFIED(axis_order_e axisOrder);
public:
    virtual int init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) override;
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual accGyroRPS_t readAccGyroRPS() override;

    void setAxisOrder(axis_order_e axisOrder);
};
