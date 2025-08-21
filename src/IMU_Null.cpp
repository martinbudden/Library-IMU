#include "IMU_Null.h"


int IMU_Null::init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex)
{
    (void)gyroSensitivity;
    (void)accSensitivity;
    (void)i2cMutex;

    _gyroIdMSP = MSP_GYRO_ID_VIRTUAL;
    _accIdMSP = MSP_ACC_ID_VIRTUAL;

    return static_cast<int>(targetOutputDataRateHz);
}

IMU_Base::xyz_int32_t IMU_Null::readGyroRaw()
{
    return _gyroRaw;
}

IMU_Base::xyz_int32_t IMU_Null::readAccRaw()
{
    return _accRaw;
}
