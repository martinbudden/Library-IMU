#if defined(LIBRARY_IMU_USE_M5_STACK)

#include "IMU_M5Stack.h"
#include <M5Stack.h>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_8G_RES { 8.0 / 32768.0 };
} // end namespace


IMU_M5_STACK::IMU_M5_STACK(axis_order_e axisOrder) :
    IMU_Base(axisOrder, IMU_AUTO_CALIBRATES)
{
    _gyroResolutionDPS = GYRO_2000DPS_RES;
    _gyroResolutionRPS = GYRO_2000DPS_RES * degreesToRadians;
    _accResolution = ACC_8G_RES;
    _gyroSampleRateHz = 500;
    _accSampleRateHz = 500;
}

int IMU_M5_STACK::init(uint32_t outputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex)
{
    (void)outputDataRateHz;
    (void)gyroSensitivity;
    (void)accSensitivity;

    // MSP compatible gyro and acc identifiers, use defaults, since no MSP value for MPU6886
    _gyroIdMSP = MSP_GYRO_ID_DEFAULT;
    _accIdMSP = MSP_ACC_ID_DEFAULT;

#if defined(FRAMEWORK_USE_FREERTOS)
    _i2cMutex = static_cast<SemaphoreHandle_t>(i2cMutex);
#else
    _i2cMutex = i2cMutex;
#endif

    // Set up FIFO for IMU
    // IMU data frequency is 500Hz
    i2cSemaphoreTake(_i2cMutex);
    M5.IMU.setFIFOEnable(true);
    M5.IMU.RestFIFO();
    i2cSemaphoreGive(_i2cMutex);

    // return the gyro sample rate actually set
    return _gyroSampleRateHz;
}

IMU_Base::xyz_int32_t IMU_M5_STACK::readAccRaw()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake(_i2cMutex);
    M5.IMU.getAccelAdc(&x, &y, &z);
    i2cSemaphoreGive(_i2cMutex);

    return xyz_int32_t {.x = x, .y = y, .z = z };
}

xyz_t IMU_M5_STACK::readAcc()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake(_i2cMutex);
    M5.IMU.getAccelAdc(&x, &y, &z);
    i2cSemaphoreGive(_i2cMutex);

    const xyz_t acc {
        .x = static_cast<float>(x - _accOffset.x) * _accResolution,
        .y = static_cast<float>(y - _accOffset.y) * _accResolution,
        .z = static_cast<float>(z - _accOffset.z) * _accResolution
    };
    return mapAxes(acc);
}

IMU_Base::xyz_int32_t IMU_M5_STACK::readGyroRaw()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake(_i2cMutex);
    M5.IMU.getGyroAdc(&x, &y, &z);
    i2cSemaphoreGive(_i2cMutex);

    return xyz_int32_t {.x = x, .y = y, .z = z };
}

xyz_t IMU_M5_STACK::readGyroRPS()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake(_i2cMutex);
    M5.IMU.getGyroAdc(&x, &y, &z);
    i2cSemaphoreGive(_i2cMutex);

    const xyz_t gyroRPS {
        .x = static_cast<float>(x - _gyroOffset.x) * _gyroResolutionRPS,
        .y = static_cast<float>(y - _gyroOffset.y) * _gyroResolutionRPS,
        .z = static_cast<float>(z - _gyroOffset.z) * _gyroResolutionRPS,
    };
    return mapAxes(gyroRPS);
}

xyz_t IMU_M5_STACK::readGyroDPS()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake(_i2cMutex);
    M5.IMU.getGyroAdc(&x, &y, &z);
    i2cSemaphoreGive(_i2cMutex);

    const xyz_t gyroDPS {
        .x = static_cast<float>(x - _gyroOffset.x) * _gyroResolutionDPS,
        .y = static_cast<float>(y - _gyroOffset.y) * _gyroResolutionDPS,
        .z = static_cast<float>(z - _gyroOffset.z) * _gyroResolutionDPS,
    };
    return mapAxes(gyroDPS);
}

IRAM_ATTR IMU_Base::accGyroRPS_t IMU_M5_STACK::readAccGyroRPS()
{
    const accGyroRPS_t gyroAcc {
        .gyroRPS = readGyroRPS(),
        .acc = readAcc()
    };

    return gyroAcc;
}

size_t IMU_M5_STACK::readFIFO_ToBuffer()
{
    i2cSemaphoreTake(_i2cMutex);
    const uint32_t fifoCount = M5.IMU.ReadFIFOCount();
    if (fifoCount != 0) {
        M5.IMU.ReadFIFOBuff(&_fifoBuffer[0], fifoCount);
    }
    i2cSemaphoreGive(_i2cMutex);
    return fifoCount / acc_temperature_gyro_data_t::DATA_SIZE;
}

IMU_Base::accGyroRPS_t IMU_M5_STACK::readFIFO_Item(size_t index)
{
    const acc_temperature_gyro_data_t* imu_data = reinterpret_cast<acc_temperature_gyro_data_t*>(&_fifoBuffer[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    const acc_temperature_gyro_data_t& imuData = imu_data[index]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const accGyroRPS_t gyroAcc = IMU_M5_STACK::accGyroRPSFromRaw(imuData);

    return gyroAcc;
}

IMU_Base::accGyroRPS_t IMU_M5_STACK::accGyroRPSFromRaw(const acc_temperature_gyro_data_t& data) const
{
// NOLINTBEGIN(hicpp-signed-bitwise)
#if defined(LIBRARY_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_XNEG_YNEG_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS,
            .z = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution,
            .z = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution
        }
    };
#else
    // Axis order mapping done at run-time
    const accGyroRPS_t accGyroRPS {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x) * _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y) * _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z) * _accResolution
        }
    };

    switch (_axisOrder) {
    case XPOS_YPOS_ZPOS:
        return accGyroRPS;
        break;
    case YNEG_XPOS_ZPOS:
        return accGyroRPS_t {
            .gyroRPS = {
                .x = -accGyroRPS.gyroRPS.y,
                .y =  accGyroRPS.gyroRPS.x,
                .z =  accGyroRPS.gyroRPS.z
            },
            .acc = {
                .x = -accGyroRPS.acc.y,
                .y =  accGyroRPS.acc.x,
                .z =  accGyroRPS.acc.z
            }
        };
        break;
    case XNEG_YNEG_ZPOS:
        return accGyroRPS_t {
            .gyroRPS = {
                .x = -accGyroRPS.gyroRPS.x,
                .y = -accGyroRPS.gyroRPS.y,
                .z =  accGyroRPS.gyroRPS.z
            },
            .acc = {
                .x = -accGyroRPS.acc.x,
                .y = -accGyroRPS.acc.y,
                .z =  accGyroRPS.acc.z
            }
        };
        break;
    case YPOS_XNEG_ZPOS:
        return accGyroRPS_t {
            .gyroRPS = {
                .x =  accGyroRPS.gyroRPS.y,
                .y = -accGyroRPS.gyroRPS.x,
                .z =  accGyroRPS.gyroRPS.z
            },
            .acc = {
                .x =  accGyroRPS.acc.y,
                .y = -accGyroRPS.acc.x,
                .z =  accGyroRPS.acc.z
            }
        };
        break;
    case XPOS_ZPOS_YNEG:
        return accGyroRPS_t {
            .gyroRPS = {
                .x =  accGyroRPS.gyroRPS.x,
                .y =  accGyroRPS.gyroRPS.z,
                .z = -accGyroRPS.gyroRPS.y
            },
            .acc = {
                .x = -accGyroRPS.acc.x,
                .y =  accGyroRPS.acc.z,
                .z = -accGyroRPS.acc.y
            }
        };
        break;
    default:
        return accGyroRPS_t {
            .gyroRPS = mapAxes(accGyroRPS.gyroRPS),
            .acc = mapAxes(accGyroRPS.acc)
        };
        break;
    } // end switch

    return accGyroRPS;
#endif
// NOLINTEND(hicpp-signed-bitwise)
}
#endif
