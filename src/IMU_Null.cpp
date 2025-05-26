#include "IMU_Null.h"


int IMU_Null::init(uint32_t outputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex)
{
    (void)outputDataRateHz;
    (void)gyroSensitivity;
    (void)accSensitivity;
    (void)i2cMutex;
    return 0;
}

IMU_Base::xyz_int32_t IMU_Null::readGyroRaw()
{
    return _gyroRaw;
}

IMU_Base::xyz_int32_t IMU_Null::readAccRaw()
{
    return _accRaw;
}
