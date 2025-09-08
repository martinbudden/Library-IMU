#pragma once

#include "IMU_Base.h"


class IMU_M5_STACK : public IMU_Base {
public:
    explicit IMU_M5_STACK(axis_order_e axisOrder);
public:
    virtual int init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) override;
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual accGyroRPS_t readAccGyroRPS() override;

private:
    struct acc_temperature_gyro_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 14 };
        uint8_t acc_x_h;
        uint8_t acc_x_l;
        uint8_t acc_y_h;
        uint8_t acc_y_l;
        uint8_t acc_z_h;
        uint8_t acc_z_l;
        uint8_t temperature_h;
        uint8_t temperature_l;
        uint8_t gyro_x_h;
        uint8_t gyro_x_l;
        uint8_t gyro_y_h;
        uint8_t gyro_y_l;
        uint8_t gyro_z_h;
        uint8_t gyro_z_l;
    };
    accGyroRPS_t accGyroRPSFromRaw(const acc_temperature_gyro_data_t& data) const;
};

