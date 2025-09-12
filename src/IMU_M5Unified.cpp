#if defined(LIBRARY_IMU_USE_M5_UNIFIED)
#include <M5Unified.h>

#include <IMU_M5Unified.h>
#include <cassert>

IMU_M5_UNIFIED::IMU_M5_UNIFIED(axis_order_e axisOrder) :
    IMU_Base(axisOrder, IMU_AUTO_CALIBRATES)
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

#if defined(FRAMEWORK_USE_FREERTOS)
    _i2cMutex = static_cast<SemaphoreHandle_t>(i2cMutex);
#else
    _i2cMutex = i2cMutex;
#endif

#if defined(LIBRARY_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos);
#elif defined(LIBRARY_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos);
#elif defined(LIBRARY_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg);
#elif defined(LIBRARY_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_pos);
#elif defined(LIBRARY_IMU_FIXED_AXES_ZPOS_XNEG_YNEG)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_neg);
#else
    setAxisOrder(_axisOrder);
#endif

    // return the gyro sample rate actually set
    return _gyroSampleRateHz;
}

void IMU_M5_UNIFIED::setAxisOrder(axis_order_e axisOrder)
{
    _axisOrder = axisOrder;

    switch (axisOrder) {
    case XPOS_YPOS_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_pos);
        break;
    case YPOS_XNEG_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos);
        break;
    case XNEG_YNEG_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_z_pos);
        break;
    case YNEG_XPOS_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos);
        break;

    case XPOS_YNEG_ZNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_z_neg);
        break;
    case YPOS_XPOS_ZNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_neg);
        break;
    case XNEG_YPOS_ZNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_neg);
        break;
    case YNEG_XNEG_ZNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_neg);
        break;

    case ZPOS_YNEG_XPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos);
        break;
    case YPOS_ZPOS_XPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_pos);
        break;
    case ZNEG_YPOS_XPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_pos);
        break;
    case YNEG_ZNEG_XPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_x_pos);
        break;

    case ZPOS_YPOS_XNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg);
        break;
    case YPOS_ZNEG_XNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_x_neg);
        break;
    case ZNEG_YNEG_XNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_neg);
        break;
    case YNEG_ZPOS_XNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_neg);
        break;

    case ZPOS_XPOS_YPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_pos);
        break;
    case XNEG_ZPOS_YPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_pos);
        break;
    case ZNEG_XNEG_YPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_pos);
        break;
    case XPOS_ZNEG_YPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_y_pos);
        break;

    case ZPOS_XNEG_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_neg);
        break;
    case XNEG_ZNEG_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_y_neg);
        break;
    case ZNEG_XPOS_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_neg);
        break;
    case XPOS_ZPOS_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg);
        break;

    default:
        assert(false && "IMU orientation invalid.");
        break;
    }
}

[[noreturn]] IMU_Base::xyz_int32_t IMU_M5_UNIFIED::readAccRaw()
{
    assert(false && ("M5Unified variants should not call readAccRaw")); // NOLINT(readability-simplify-boolean-expr)
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

[[noreturn]] IMU_Base::xyz_int32_t IMU_M5_UNIFIED::readGyroRaw()
{
    assert(false && ("M5Unified variants should not call readGyroRaw")); // NOLINT(readability-simplify-boolean-expr)
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

FAST_CODE IMU_Base::accGyroRPS_t IMU_M5_UNIFIED::readAccGyroRPS()
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