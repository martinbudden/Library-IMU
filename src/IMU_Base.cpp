#include "IMU_Base.h"
#include <cassert>


#if defined(FRAMEWORK_RPI_PICO)
#include <pico/time.h>
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#endif


IMU_Base::IMU_Base(axis_order_e axisOrder, BUS_BASE& busBase, uint32_t flags) :
    _axisOrder(axisOrder),
    _busBase(&busBase),
    _flags(flags)
{
}

IMU_Base::IMU_Base(axis_order_e axisOrder, BUS_BASE& busBase) :
    IMU_Base(axisOrder, busBase, 0)
{
}

IMU_Base::IMU_Base(axis_order_e axisOrder, uint32_t flags):
    _axisOrder(axisOrder),
    _busBase(nullptr),
    _flags(flags)
{
}

IMU_Base::IMU_Base(axis_order_e axisOrder) :
    IMU_Base(axisOrder, 0)
{
}

int IMU_Base::init(uint32_t outputDataRateHz, void* i2cMutex)
{
    return init(outputDataRateHz, GYRO_FULL_SCALE_MAX, ACC_FULL_SCALE_MAX, i2cMutex);
}

int IMU_Base::init(uint32_t outputDataRateHz)
{
    return init(outputDataRateHz, nullptr);
}

int IMU_Base::init(void* i2cMutex)
{
    return init(TARGET_OUTPUT_DATA_RATE_MAX, GYRO_FULL_SCALE_MAX, ACC_FULL_SCALE_MAX, i2cMutex);
}

int IMU_Base::init()
{
    return init(nullptr);
}

void IMU_Base::delayMs(int ms)
{
#if defined(FRAMEWORK_RPI_PICO)
    sleep_ms(ms);
#elif defined(FRAMEWORK_ESPIDF)
    (void)ms;
#elif defined(FRAMEWORK_STM32_CUBE)
    (void)ms;
#elif defined(FRAMEWORK_TEST)
    (void)ms;
#else // defaults to FRAMEWORK_ARDUINO
    delay(ms);
#endif
}

void IMU_Base::setInterruptDriven()
{
}

IMU_Base::xyz_int32_t IMU_Base::getGyroOffset() const
{
    return _gyroOffset;
}

void IMU_Base::setGyroOffset(const xyz_int32_t& gyroOffset)
{
    _gyroOffset = gyroOffset;
}

IMU_Base::xyz_int32_t IMU_Base::getAccOffset() const
{
    return _accOffset;
}

void IMU_Base::setAccOffset(const xyz_int32_t& accOffset)
{
    _accOffset = accOffset;
}

int32_t IMU_Base::getAccOneG_Raw() const
{
    return static_cast<int32_t>(1.0F / _accResolution);
}

xyz_t IMU_Base::readGyroRPS()
{
    const xyz_int32_t gyroRaw = readGyroRaw();
    return mapAxes({
        .x = static_cast<float>(gyroRaw.x - _gyroOffset.x) * _gyroResolutionRPS,
        .y = static_cast<float>(gyroRaw.y - _gyroOffset.y) * _gyroResolutionRPS,
        .z = static_cast<float>(gyroRaw.z - _gyroOffset.z) * _gyroResolutionRPS
    });
}

xyz_t IMU_Base::readGyroDPS()
{
    const xyz_int32_t gyroRaw = readGyroRaw();
    return mapAxes({
        .x = static_cast<float>(gyroRaw.x - _gyroOffset.x) * _gyroResolutionDPS,
        .y = static_cast<float>(gyroRaw.y - _gyroOffset.y) * _gyroResolutionDPS,
        .z = static_cast<float>(gyroRaw.z - _gyroOffset.z) * _gyroResolutionDPS
    });
}

xyz_t IMU_Base::readAcc()
{
    const xyz_int32_t accRaw = readAccRaw();
    return mapAxes({
        .x = static_cast<float>(accRaw.x - _accOffset.x) * _accResolution,
        .y = static_cast<float>(accRaw.y - _accOffset.y) * _accResolution,
        .z = static_cast<float>(accRaw.z - _accOffset.z) * _accResolution
    });
}

IMU_Base::accGyroRPS_t IMU_Base::readAccGyroRPS()
{
    return accGyroRPS_t {
        .gyroRPS = readGyroRPS(),
        .acc = readAcc()
    };
}

IMU_Base::accGyroRPS_t IMU_Base::getAccGyroRPS() const
{
    return accGyroRPS_t {};
}

Quaternion IMU_Base::readOrientation()
{
    return Quaternion {};
}

xyz_t IMU_Base::mapAxes(const xyz_t& data, axis_order_e axisOrder)
{
// NOLINTBEGIN(bugprone-branch-clone) false positive
    switch (axisOrder) {
    case XPOS_YPOS_ZPOS:
        return data;
    case YPOS_XNEG_ZPOS: // rotate  90 degrees anticlockwise
        // .x =   data.x*cos90(0) + data.y*sin90(1)
        // .y =  -data.x*sin90(1) + data.y*cos90(0)
        return xyz_t {
            .x =  data.y,
            .y = -data.x,
            .z =  data.z
        };
    case XNEG_YNEG_ZPOS: // rotate 180 degrees
        // .x =   data.x*cos180(-1) + data.y*sin180(0)
        // .y =  -data.x*sin180(0) + data.y*cos180(-1)
        return xyz_t {
            .x = -data.x,
            .y = -data.y,
            .z =  data.z
        };
    case YNEG_XPOS_ZPOS: // rotate 270 degrees anticlockwise
        // .x =   data.x*cos270(0) + data.y*sin270(-1)
        // .y =  -data.x*sin270(-1) + data.y*cos270(0)
        return xyz_t {
            .x = -data.y,
            .y =  data.x,
            .z =  data.z
        };
    case XPOS_YNEG_ZNEG:
        return xyz_t {
            .x =  data.x,
            .y = -data.y,
            .z = -data.z
        };
    case YPOS_XPOS_ZNEG:
        return xyz_t {
            .x =  data.y,
            .y =  data.x,
            .z = -data.z
        };
    case XNEG_YPOS_ZNEG:
        return xyz_t {
            .x = -data.x,
            .y =  data.y,
            .z = -data.z
        };
    case YNEG_XNEG_ZNEG:
        return xyz_t {
            .x = -data.y,
            .y = -data.x,
            .z = -data.z
        };
    case ZPOS_YNEG_XPOS:
        return xyz_t {
            .x =  data.z,
            .y = -data.y,
            .z =  data.x
        };
    case YPOS_ZPOS_XPOS:
        return xyz_t {
            .x =  data.y,
            .y =  data.z,
            .z =  data.x
        };
    case ZNEG_YPOS_XPOS:
        return xyz_t {
            .x = -data.z,
            .y =  data.y,
            .z =  data.x
        };
    case YNEG_ZNEG_XPOS:
        return xyz_t {
            .x = -data.y,
            .y = -data.z,
            .z =  data.x
        };
    case ZPOS_YPOS_XNEG:
        return xyz_t {
            .x =  data.z,
            .y =  data.y,
            .z = -data.x
        };
    case YPOS_ZNEG_XNEG:
        return xyz_t {
            .x =  data.y,
            .y = -data.z,
            .z = -data.x
        };
    case ZNEG_YNEG_XNEG:
        return xyz_t {
            .x = -data.z,
            .y = -data.y,
            .z = -data.x
        };
    case YNEG_ZPOS_XNEG:
        return xyz_t {
            .x = -data.y,
            .y =  data.z,
            .z = -data.x
        };
    case ZPOS_XPOS_YPOS:
        return xyz_t {
            .x =  data.z,
            .y =  data.x,
            .z =  data.y
        };
    case XNEG_ZPOS_YPOS:
        return xyz_t {
            .x = -data.x,
            .y =  data.z,
            .z =  data.y
        };
    case ZNEG_XNEG_YPOS:
        return xyz_t {
            .x = -data.z,
            .y = -data.x,
            .z =  data.y
        };
    case XPOS_ZNEG_YPOS:
        return xyz_t {
            .x =  data.x,
            .y = -data.z,
            .z =  data.y
        };
    case ZPOS_XNEG_YNEG:
        return xyz_t {
            .x =  data.z,
            .y = -data.x,
            .z = -data.y
        };
    case XNEG_ZNEG_YNEG:
        return xyz_t {
            .x = -data.x,
            .y = -data.z,
            .z = -data.y
        };
    case ZNEG_XPOS_YNEG:
        return xyz_t {
            .x = -data.z,
            .y =  data.x,
            .z = -data.y
        };
    case XPOS_ZPOS_YNEG:
        return xyz_t {
            .x =  data.x,
            .y =  data.z,
            .z = -data.y
        };
    case XPOS_YPOS_ZPOS_45: // 45
        return xyz_t {
            .x =  data.x*cos45f + data.y*sin45f,
            .y = -data.x*sin45f + data.y*cos45f,
            .z =  data.z
        };
    case YPOS_XNEG_ZPOS_45: {// 135
        static constexpr float sin135f =  sin45f;
        static constexpr float cos135f = -cos45f;
        return xyz_t {
            .x =  data.x*cos135f + data.y*sin135f,
            .y = -data.x*sin135f + data.y*cos135f,
            .z =  data.z
        };
    }
    case XNEG_YNEG_ZPOS_45: {// 225
        static constexpr float sin225f = -sin45f;
        static constexpr float cos225f = -cos45f;
        return xyz_t {
            .x =  data.x*cos225f + data.y*sin225f,
            .y = -data.x*sin225f + data.y*cos225f,
            .z =  data.z
        };
    }
    case YNEG_XPOS_ZPOS_45: {// 315
        static constexpr float sin315f = -sin45f;
        static constexpr float cos315f =  cos45f;
        return xyz_t {
            .x =  data.x*cos315f + data.y*sin315f,
            .y = -data.x*sin315f + data.y*cos315f,
            .z =  data.z
        };
    }
    default:
        assert(false && "IMU axis order not supported"); // NOLINT(readability-implicit-bool-conversion,readability-simplify-boolean-expr)
        break;
    } // end switch
// NOLINTEND(bugprone-branch-clone)

    return data;
}

IMU_Base::axis_order_e IMU_Base::axisOrderInverse(axis_order_e axisOrder)
{
    switch (axisOrder) {
    case YPOS_XNEG_ZPOS:
        return YNEG_XPOS_ZPOS;
    case YNEG_XPOS_ZPOS:
        return YPOS_XNEG_ZPOS;
    case YPOS_ZPOS_XPOS:
        return ZPOS_XPOS_YPOS;
    case ZNEG_YPOS_XPOS:
        return ZPOS_YPOS_XNEG;
    case YNEG_ZNEG_XPOS:
        return ZPOS_XNEG_YNEG;
    case ZPOS_YPOS_XNEG:
        return ZNEG_YPOS_XPOS;
    case YPOS_ZNEG_XNEG:
        return ZNEG_XPOS_YNEG;
    case YNEG_ZPOS_XNEG:
        return ZNEG_XNEG_YPOS;
    case ZPOS_XPOS_YPOS:
        return YPOS_ZPOS_XPOS;
    case ZNEG_XNEG_YPOS:
        return YNEG_ZPOS_XNEG;
    case XPOS_ZNEG_YPOS:
        return XPOS_ZPOS_YNEG;
    case ZPOS_XNEG_YNEG:
        return YNEG_ZNEG_XPOS;
    case ZNEG_XPOS_YNEG:
        return YPOS_ZNEG_XNEG;
    case XPOS_ZPOS_YNEG:
        return XPOS_ZNEG_YPOS;
    case XPOS_YPOS_ZPOS_45: // 45
        return YNEG_XPOS_ZPOS_45; // 315
    case YPOS_XNEG_ZPOS_45: // 135
        return XNEG_YNEG_ZPOS_45; // 225
    case XNEG_YNEG_ZPOS_45: // 225
        return YPOS_XNEG_ZPOS_45; // 135
    case YNEG_XPOS_ZPOS_45: // 315
        return XPOS_YPOS_ZPOS_45; // 45
    default:
        // other axis orders are self-inverting
        return axisOrder;
    }
}

IMU_Base::xyz_alignment_t IMU_Base::alignmentFromAxisOrder(axis_order_e axisOrder)
{
    (void)axisOrder;
    xyz_alignment_t alignment {
        .x = 0,
        .y = 0,
        .z = 0
    };
    return alignment;
}

IMU_Base::axis_order_e IMU_Base::axisOrderFromAlignment(const xyz_alignment_t& alignment)
{
    (void)alignment;
    return XPOS_YPOS_ZPOS;
}
