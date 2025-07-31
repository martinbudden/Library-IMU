#include "IMU_MPU6000.h"
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#if defined(USE_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
#include <HardwareSerial.h>
#else
#include <Arduino.h>
#endif
#endif
#include <cassert>

namespace { // use anonymous namespace to make items local to this translation unit

constexpr uint8_t REG_ACCEL_XOUT_H          = 0x3B;
constexpr uint8_t REG_ACCEL_XOUT_L          = 0x3C;
constexpr uint8_t REG_ACCEL_YOUT_H          = 0x3D;
constexpr uint8_t REG_ACCEL_YOUT_L          = 0x3E;
constexpr uint8_t REG_ACCEL_ZOUT_H          = 0x3F;
constexpr uint8_t REG_ACCEL_ZOUT_L          = 0x40;
constexpr uint8_t REG_TEMP_OUT_H            = 0x41;
constexpr uint8_t REG_TEMP_OUT_L            = 0x42;
constexpr uint8_t REG_GYRO_XOUT_H           = 0x43;
constexpr uint8_t REG_GYRO_XOUT_L           = 0x44;
constexpr uint8_t REG_GYRO_YOUT_H           = 0x45;
constexpr uint8_t REG_GYRO_YOUT_L           = 0x46;
constexpr uint8_t REG_GYRO_ZOUT_H           = 0x47;
constexpr uint8_t REG_GYRO_ZOUT_L           = 0x48;

} // end namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

#if defined(USE_IMU_MPU6000_SPI)
IMU_MPU6000::IMU_MPU6000(axis_order_e axisOrder, uint32_t frequency, BUS_SPI::spi_index_e SPI_index, const BUS_SPI::pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);
}
#else
IMU_MPU6000::IMU_MPU6000(axis_order_e axisOrder, BUS_I2C::i2c_index_e I2C_index, const BUS_I2C::pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, I2C_index, pins)
{
}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_TEST)
IMU_MPU6000::IMU_MPU6000(axis_order_e axisOrder, TwoWire& wire, const BUS_I2C::pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, wire, pins)
{
}
#endif
#endif // USE_IMU_MPU6000_SPI

int IMU_MPU6000::init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) // NOLINT(readability-function-cognitive-complexity)
{
    (void)targetOutputDataRateHz;
    (void)gyroSensitivity;
    (void)accSensitivity;
#if defined(I2C_MUTEX_REQUIRED)
    _i2cMutex = static_cast<SemaphoreHandle_t>(i2cMutex);
#else
    (void)i2cMutex;
#endif
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_temperature_gyro_data_t) == acc_temperature_gyro_data_t::DATA_SIZE);

    // MSP compatible gyro and acc identifiers
    enum { MSP_GYRO_ID_MPU6050 = 2, MSP_GYRO_ID_MPU6000 = 4 };
    enum { MSP_ACC_ID_MPU6050 = 2, MSP_ACC_ID_MPU6000 = 3 };

    _gyroIdMSP = MSP_GYRO_ID_MPU6000;
    _accIdMSP = MSP_ACC_ID_MPU6000;

    _bus.setDeviceRegister(REG_ACCEL_XOUT_H, reinterpret_cast<uint8_t*>(&_spiAccGyroData), sizeof(_spiAccGyroData));

    return 0;
}

void IMU_MPU6000::setInterruptDriven()
{
    _bus.setInterruptDriven();
}

IMU_Base::xyz_int32_t IMU_MPU6000::readGyroRaw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake();
    _bus.readRegister(REG_ACCEL_XOUT_H, &gyro.data[0], sizeof(gyro));
    i2cSemaphoreGive();

// NOLINTBEGIN(hicpp-signed-bitwise)
    return xyz_int32_t {
        .x = static_cast<int16_t>((gyro.value.x_h << 8U) | gyro.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((gyro.value.y_h << 8U) | gyro.value.y_l),
        .z = static_cast<int16_t>((gyro.value.z_h << 8U) | gyro.value.z_l)
    };
// NOLINTEND(hicpp-signed-bitwise)
}

IMU_Base::xyz_int32_t IMU_MPU6000::readAccRaw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake();
    _bus.readRegister(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    i2cSemaphoreGive();

// NOLINTBEGIN(hicpp-signed-bitwise)
    return xyz_int32_t {
        .x = static_cast<int16_t>((acc.value.x_h << 8U) | acc.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((acc.value.y_h << 8U) | acc.value.y_l),
        .z = static_cast<int16_t>((acc.value.z_h << 8U) | acc.value.z_l)
    };
// NOLINTEND(hicpp-signed-bitwise)
}

xyz_t IMU_MPU6000::readGyroRPS()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_GYRO_XOUT_H, &gyro.data[0], sizeof(gyro));
    i2cSemaphoreGive(_i2cMutex);

    return gyroRPS_FromRaw(gyro.value);
}

xyz_t IMU_MPU6000::readGyroDPS()
{
    return readGyroRPS() * radiansToDegrees;
}

xyz_t IMU_MPU6000::readAcc()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    i2cSemaphoreGive(_i2cMutex);

    return accFromRaw(acc.value);
}

IRAM_ATTR IMU_Base::accGyroRPS_t IMU_MPU6000::readAccGyroRPS()
{
    i2cSemaphoreTake(_i2cMutex);
    _bus.readDeviceRegister();
    //_bus.readDeviceRegisterDMA();
    i2cSemaphoreGive(_i2cMutex);

    return accGyroRPSFromRaw(_spiAccGyroData.accGyro.value);
}

/*!
Return the gyroAcc data that was read in the ISR
*/
IRAM_ATTR IMU_Base::accGyroRPS_t IMU_MPU6000::getAccGyroRPS() const
{
    return accGyroRPSFromRaw(_spiAccGyroData.accGyro.value);
}

xyz_t IMU_MPU6000::gyroRPS_FromRaw(const mems_sensor_data_t::value_t& data) const
{
    // static cast to int16_t to sign extend the 8 bit values
#if defined(IMU_BUILD_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _gyroOffset.y) * _gyroResolutionRPS,
        .y =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _gyroOffset.x) * _gyroResolutionRPS,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _gyroOffset.z) * _gyroResolutionRPS
    };
#elif defined(IMU_BUILD_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _gyroOffset.y) * _gyroResolutionRPS,
        .y = -static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _gyroOffset.x) * _gyroResolutionRPS,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _gyroOffset.z) * _gyroResolutionRPS
    };
#elif defined(IMU_BUILD_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _gyroOffset.x) * _gyroResolutionRPS,
        .y =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _gyroOffset.z) * _gyroResolutionRPS,
        .z = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _gyroOffset.y) * _gyroResolutionRPS
    };
#elif defined(IMU_BUILD_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _gyroOffset.x) * _gyroResolutionRPS,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _gyroOffset.y) * _gyroResolutionRPS,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _gyroOffset.z) * _gyroResolutionRPS
    };
#else
    const xyz_t gyro {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _gyroOffset.x) * _gyroResolutionRPS,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _gyroOffset.y) * _gyroResolutionRPS,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _gyroOffset.z) * _gyroResolutionRPS
    };
    return mapAxes(gyro);
#endif
}

xyz_t IMU_MPU6000::accFromRaw(const mems_sensor_data_t::value_t& data) const
{
#if defined(IMU_BUILD_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _accOffset.y)* _accResolution,
        .y =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _accOffset.x)* _accResolution,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _accOffset.z)* _accResolution
    };
#elif defined(IMU_BUILD_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _accOffset.y)* _accResolution,
        .y = -static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _accOffset.x)* _accResolution,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _accOffset.z)* _accResolution
    };
#elif defined(IMU_BUILD_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _accOffset.x)* _accResolution,
        .y =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _accOffset.z)* _accResolution,
        .z = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _accOffset.y)* _accResolution
    };
#elif defined(IMU_BUILD_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _accOffset.x)* _accResolution,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _accOffset.y)* _accResolution,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _accOffset.z)* _accResolution
    };
#else
    const xyz_t acc = {
        .x = static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _accOffset.x) * _accResolution,
        .y = static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _accOffset.y) * _accResolution,
        .z = static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _accOffset.z) * _accResolution
    };
    return mapAxes(acc);
#endif
}

IMU_Base::accGyroRPS_t IMU_MPU6000::accGyroRPSFromRaw(const acc_temperature_gyro_data_t::value_t& data) const
{
// NOLINTBEGIN(hicpp-signed-bitwise)
#if defined(IMU_BUILD_XPOS_YPOS_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_YPOS_XNEG_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_XNEG_YNEG_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_YNEG_XPOS_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_XPOS_ZPOS_YNEG)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS,
            .z = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l) - _accOffset.z)* _accResolution,
            .z = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l) - _accOffset.y)* _accResolution
        }
    };
#else
    // Axis order mapping done at run-time
    const accGyroRPS_t accGyroRPS {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l) - _accOffset.x) * _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l) - _accOffset.y) * _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l) - _accOffset.z) * _accResolution
        }
    };
// NOLINTEND(hicpp-signed-bitwise)

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
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
