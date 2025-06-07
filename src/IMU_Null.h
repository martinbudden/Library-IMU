#pragma once

#include "IMU_Base.h"

/*!
NULL IMU, useful for test code.
*/
class IMU_Null : public IMU_Base {
public:
    virtual ~IMU_Null() override = default;
    IMU_Null(const IMU_Null&) = delete;
    IMU_Null& operator=(const IMU_Null&) = delete;
    IMU_Null(IMU_Null&&) = delete;
    IMU_Null& operator=(IMU_Null&&) = delete;

    explicit IMU_Null(axis_order_e axisOrder) : IMU_Base(axisOrder) {}
    IMU_Null() : IMU_Null(IMU_Base::XPOS_YPOS_ZPOS) {}

// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
    virtual int init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) override;
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
public: // for testing
    void setGyroRaw(const xyz_int32_t& gyroRaw) { _gyroRaw = gyroRaw; }
    void setAccRaw(const xyz_int32_t& accRaw) { _accRaw = accRaw; }
private:
    xyz_int32_t _gyroRaw {};
    xyz_int32_t _accRaw {};
};
