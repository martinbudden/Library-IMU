#if defined(USE_IMU_M5_UNIFIED)
#include <M5Unified.h>

#include <IMU_M5Unified.h>
#include <cassert>

IMU_M5_UNIFIED::IMU_M5_UNIFIED(axis_order_e axisOrder) :
    IMU_Base(axisOrder)
{
}

int IMU_M5_UNIFIED::init(uint32_t outputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex)
{
    (void)outputDataRateHz;
    (void)gyroSensitivity;
    (void)accSensitivity;

    // MSP compatible gyro and acc identifiers, use defaults, since no MSP value for MPU6886
    _gyroIdMSP = MSP_GYRO_ID_DEFAULT;
    _accIdMSP = MSP_ACC_ID_DEFAULT;

    _gyroSampleRateHz = 500;
    _accSampleRateHz = 500;

#if defined(USE_FREERTOS)
    _i2cMutex = static_cast<SemaphoreHandle_t>(i2cMutex);
#else
    _i2cMutex = i2cMutex;
#endif

    i2cSemaphoreTake(_i2cMutex);
#if defined(IMU_BUILD_YNEG_XPOS_ZPOS)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos);
#elif defined(IMU_BUILD_YPOS_XNEG_ZPOS)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos);
#elif defined(IMU_BUILD_XPOS_ZPOS_YNEG)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg);
#elif defined(IMU_BUILD_XPOS_YPOS_ZPOS)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_pos);
#elif defined(IMU_BUILD_ZPOS_XNEG_YNEG)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_neg);
#else
    switch (_axisOrder) {
    case XPOS_YPOS_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_pos);
        break;
    case YNEG_XPOS_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos);
        break;
    case YPOS_XNEG_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos);
        break;
    case XPOS_ZPOS_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg);
        break;
    case ZPOS_XNEG_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_neg);
        break;
    default:
        assert(false && "IMU orientation not implemented for M5Unified.");
        break;
    }
#endif
    i2cSemaphoreGive(_i2cMutex);

    return 0;
}

IMU_Base::xyz_int32_t IMU_M5_UNIFIED::readAccRaw()
{
    xyz_int32_t acc {};
    assert(false && ("M5Unified variants should not call readAccRaw")); // NOLINT(readability-simplify-boolean-expr)
    return acc;
}

xyz_t IMU_M5_UNIFIED::readAcc()
{
    // This is very slow on the M5 Atom.
    i2cSemaphoreTake(_i2cMutex);
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    i2cSemaphoreGive(_i2cMutex);

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    const xyz_t acc = {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .x = data.accel.x,
        .y = data.accel.y,
        .z = data.accel.z
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };

    return acc;
}

IMU_Base::xyz_int32_t IMU_M5_UNIFIED::readGyroRaw()
{
    xyz_int32_t gyro {};
    assert(false && ("M5Unified variants should not call readGyroRaw")); // NOLINT(readability-simplify-boolean-expr)
    return gyro;
}

xyz_t IMU_M5_UNIFIED::readGyroRPS()
{
    // This is very slow on the M5 Atom.
    i2cSemaphoreTake(_i2cMutex);
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    i2cSemaphoreGive(_i2cMutex);

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    const xyz_t gyroRPS {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .x = data.gyro.x * degreesToRadians,
        .y = data.gyro.y * degreesToRadians,
        .z = data.gyro.z * degreesToRadians
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };
    return gyroRPS;
}

xyz_t IMU_M5_UNIFIED::readGyroDPS()
{
    // This is very slow on the M5 Atom.
    i2cSemaphoreTake(_i2cMutex);
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    i2cSemaphoreGive(_i2cMutex);

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    const xyz_t gyroDPS {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .x = data.gyro.x,
        .y = data.gyro.y,
        .z = data.gyro.z
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };
    return gyroDPS;
}

IRAM_ATTR IMU_Base::accGyroRPS_t IMU_M5_UNIFIED::readAccGyroRPS()
{
    // This is very slow on the M5 Atom.
    i2cSemaphoreTake(_i2cMutex);
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    i2cSemaphoreGive(_i2cMutex);

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    return accGyroRPS_t {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .gyroRPS = {
            .x = data.gyro.x * degreesToRadians,
            .y = data.gyro.y * degreesToRadians,
            .z = data.gyro.z * degreesToRadians
        },
        .acc = {
            .x = data.accel.x,
            .y = data.accel.y,
            .z = data.accel.z
        }
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };
}

#endif