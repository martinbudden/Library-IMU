#include "IMU_LSM6DS3TR_C.h"
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
#include <HardwareSerial.h>
#else
#include <Arduino.h>
#endif
#endif
#include <cassert>

/*
https://github.com/STMicroelectronics/lsm6ds3tr-c-pid
https://github.com/STMicroelectronics/ism330dhcx-pid
https://github.com/STMicroelectronics/lsm6dsox-pid
*/

namespace { // use anonymous namespace to make items local to this translation unit

constexpr uint8_t REG_RESERVED_00           = 0x00;
constexpr uint8_t REG_FUNC_CFG_ACCESS       = 0x01;
constexpr uint8_t REG_RESERVED_03           = 0x03;

#if defined(USE_IMU_LSM6DS3TR_C)

constexpr uint8_t REG_RESERVED_02           = 0x02;
constexpr uint8_t REG_SENSOR_SYNC_TIME_FRAME= 0x04;
constexpr uint8_t REG_SENSOR_SYNC_RES_RATIO = 0x05;
constexpr uint8_t REG_FIFO_CTRL1            = 0x06;
constexpr uint8_t REG_FIFO_CTRL2            = 0x07;
constexpr uint8_t REG_FIFO_CTRL3            = 0x08;
constexpr uint8_t REG_FIFO_CTRL4            = 0x09;
constexpr uint8_t REG_FIFO_CTRL5            = 0x0A;
constexpr uint8_t REG_DRDY_PULSE_CFG_G      = 0x0B;
constexpr uint8_t REG_RESERVED_0C           = 0x0C;
constexpr uint8_t REG_MASTER_CONFIG         = 0x1A;

#elif defined(USE_IMU_ISM330DHCX)

constexpr uint8_t REG_PIN_CTRL              = 0x02;
constexpr uint8_t REG_RESERVED_04           = 0x04;
constexpr uint8_t REG_RESERVED_05           = 0x05;
constexpr uint8_t REG_RESERVED_06           = 0x06;
constexpr uint8_t REG_FIFO_CTRL1            = 0x07;
constexpr uint8_t REG_FIFO_CTRL2            = 0x08;
constexpr uint8_t REG_FIFO_CTRL3            = 0x09;
constexpr uint8_t REG_FIFO_CTRL4            = 0x0A;
constexpr uint8_t REG_COUNTER_BDR_REG1      = 0x0B;
constexpr uint8_t REG_COUNTER_BDR_REG2      = 0x0C;
constexpr uint8_t REG_ALL_INT_SRC           = 0x1A;

#elif defined(USE_IMU_LSM6DSOX)

constexpr uint8_t REG_PIN_CTRL              = 0x02;
constexpr uint8_t REG_S4S_TPH_L             = 0x04;
constexpr uint8_t REG_S4S_TPH_H             = 0x05;
constexpr uint8_t REG_S4S_RR                = 0x06;
constexpr uint8_t REG_FIFO_CTRL1            = 0x07;
constexpr uint8_t REG_FIFO_CTRL2            = 0x08;
constexpr uint8_t REG_FIFO_CTRL3            = 0x09;
constexpr uint8_t REG_FIFO_CTRL4            = 0x0A;
constexpr uint8_t REG_COUNTER_BDR_REG1      = 0x0B;
constexpr uint8_t REG_COUNTER_BDR_REG2      = 0x0C;
constexpr uint8_t REG_ALL_INT_SRC           = 0x1A;

#endif

constexpr uint8_t REG_DATA_READY_PULSE_CONFIG = 0x0B;
    const uint8_t DATA_READY_PULSED         = 0b10000000;
constexpr uint8_t REG_INT1_CTRL             = 0x0D;
    constexpr uint8_t INT1_DRDY_G           =0b00000010;
constexpr uint8_t REG_INT2_CTRL             = 0x0E;
    constexpr uint8_t INT2_DRDY_G           =0b00000010;
constexpr uint8_t REG_WHO_AM_I              = 0x0F;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C = 0x6A;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE_ISM330DHCX = 0x6B;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE_LSM6DSOX = 0x6C;
constexpr uint8_t REG_CTRL1_XL              = 0x10;
    constexpr uint8_t ACC_RANGE_2G    =     0b0000;
    constexpr uint8_t ACC_RANGE_4G    =     0b1000;
    constexpr uint8_t ACC_RANGE_8G    =     0b1100;
    constexpr uint8_t ACC_RANGE_16G   =     0b0100;
    constexpr uint8_t ACC_ODR_12p5_HZ = 0b00010000;
    constexpr uint8_t ACC_ODR_26_HZ   = 0b00100000;
    constexpr uint8_t ACC_ODR_52_HZ   = 0b00110000;
    constexpr uint8_t ACC_ODR_104_HZ  = 0b01000000;
    constexpr uint8_t ACC_ODR_208_HZ  = 0b010100000;
    constexpr uint8_t ACC_ODR_416_HZ  = 0b01100000;
    constexpr uint8_t ACC_ODR_833_HZ  = 0b01110000;
    constexpr uint8_t ACC_ODR_1666_HZ = 0b10000000;
    constexpr uint8_t ACC_ODR_3332_HZ = 0b10010000;
    constexpr uint8_t ACC_ODR_6664_HZ = 0b10100000;
constexpr uint8_t REG_CTRL2_G               = 0x11;
    constexpr uint8_t GYRO_RANGE_125_DPS   = 0b0010;
    constexpr uint8_t GYRO_RANGE_245_DPS   = 0b0000; // LSM6DS3TR_C
    constexpr uint8_t GYRO_RANGE_250_DPS   = 0b0000; // ISM330DHCX, LSM6DSOX
    constexpr uint8_t GYRO_RANGE_500_DPS   = 0b0100;
    constexpr uint8_t GYRO_RANGE_1000_DPS  = 0b1000;
    constexpr uint8_t GYRO_RANGE_2000_DPS  = 0b1100;
    constexpr uint8_t GYRO_ODR_12p5_HZ = 0b00010000;
    constexpr uint8_t GYRO_ODR_26_HZ   = 0b00100000;
    constexpr uint8_t GYRO_ODR_52_HZ   = 0b00110000;
    constexpr uint8_t GYRO_ODR_104_HZ  = 0b01000000;
    constexpr uint8_t GYRO_ODR_208_HZ  = 0b010100000;
    constexpr uint8_t GYRO_ODR_416_HZ  = 0b01100000;
    constexpr uint8_t GYRO_ODR_833_HZ  = 0b01110000;
    constexpr uint8_t GYRO_ODR_1666_HZ = 0b10000000;
    constexpr uint8_t GYRO_ODR_3332_HZ = 0b10010000;
    constexpr uint8_t GYRO_ODR_6664_HZ = 0b10100000;
constexpr uint8_t REG_CTRL3_C               = 0x12;
    constexpr uint8_t BDU                   = 0b01000000;
    constexpr uint8_t IF_INC                = 0b00000100;
    constexpr uint8_t SW_RESET              = 0b00000001;
constexpr uint8_t REG_CTRL4_C               = 0x13;
    constexpr uint8_t I2C_DISABLE           = 0b00000100;
    constexpr uint8_t LPF1_SEL_G            = 0b00000010;
constexpr uint8_t REG_CTRL5_C               = 0x14;
constexpr uint8_t REG_CTRL6_C               = 0x15;
    constexpr uint8_t XL_HM_MODE_DISABLE    = 0b00010000;
    constexpr uint8_t LPF1_MEDIUM_HI        = 0x00;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz - LSM6DS3TR_C: 351Hz, ISM330DHCX: 297Hz, LSM6DSOX: 335.5Hz
    constexpr uint8_t LPF1_MEDIUM_LO        = 0x01;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz - LSM6DS3TR_C: 237Hz, ISM330DHCX: 223Hz, LSM6DSOX: 232.0Hz
    constexpr uint8_t LPF1_LO               = 0x02;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz - LSM6DS3TR_C: 172Hz, ISM330DHCX: 154Hz, LSM6DSOX: 171.1Hz
    constexpr uint8_t LPF1_HI               = 0x03;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz   LSM6DS3TR_C: 937Hz, ISM330DHCX: 470Hz, LSM6DSOX: 609.0Hz
constexpr uint8_t REG_CTRL7_G               = 0x16; // this includes high pass filter (HPF)
constexpr uint8_t REG_CTRL8_XL              = 0x17;
constexpr uint8_t REG_CTRL9_XL              = 0x18;
constexpr uint8_t REG_CTRL10_C              = 0x19;
constexpr uint8_t REG_WAKE_UP_SRC           = 0x1B;
constexpr uint8_t REG_TAP_SRC               = 0x1C;
constexpr uint8_t REG_D6D_SRC               = 0x1D;
constexpr uint8_t REG_STATUS_REG            = 0x1E;
constexpr uint8_t REG_RESERVED_1F           = 0x1F;

constexpr uint8_t REG_OUT_TEMP_L            = 0x20;
constexpr uint8_t REG_OUT_TEMP_H            = 0x22;
constexpr uint8_t REG_OUTX_L_G              = 0x22;
constexpr uint8_t REG_OUTX_H_G              = 0x23;
constexpr uint8_t REG_OUTY_L_G              = 0x24;
constexpr uint8_t REG_OUTY_H_G              = 0x25;
constexpr uint8_t REG_OUTZ_L_G              = 0x26;
constexpr uint8_t REG_OUTZ_H_G              = 0x27;
constexpr uint8_t REG_OUTX_L_ACC            = 0x28;
constexpr uint8_t REG_OUTX_H_ACC            = 0x29;
constexpr uint8_t REG_OUTY_L_ACC            = 0x2A;
constexpr uint8_t REG_OUTY_H_ACC            = 0x2B;
constexpr uint8_t REG_OUTZ_L_ACC            = 0x2C;
constexpr uint8_t REG_OUTZ_H_ACC            = 0x2D;

} // end namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

/*!
Gyroscope data rates up to 6.4 kHz, accelerometer up to 1.6 kHz
*/
#if defined(LIBRARY_IMU_USE_SPI_BUS)
IMU_LSM6DS3TR_C::IMU_LSM6DS3TR_C(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::port_pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins)
{
}
IMU_LSM6DS3TR_C::IMU_LSM6DS3TR_C(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins)
{
}
#else
IMU_LSM6DS3TR_C::IMU_LSM6DS3TR_C(axis_order_e axisOrder, BUS_BASE::bus_index_e I2C_index, const BUS_I2C::port_pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, I2C_index, pins)
{
}
IMU_LSM6DS3TR_C::IMU_LSM6DS3TR_C(axis_order_e axisOrder, BUS_BASE::bus_index_e I2C_index, const BUS_I2C::pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, I2C_index, pins)
{
}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) &&!defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
IMU_LSM6DS3TR_C::IMU_LSM6DS3TR_C(axis_order_e axisOrder, TwoWire& wire, const BUS_I2C::pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, wire, pins)
{
}
#endif
#endif

int IMU_LSM6DS3TR_C::init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) // NOLINT(readability-function-cognitive-complexity)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);

#if defined(LIBRARY_IMU_I2C_MUTEX_REQUIRED)
    _i2cMutex = static_cast<SemaphoreHandle_t>(i2cMutex);
#else
    (void)i2cMutex;
#endif
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);

    // MSP compatible gyro and acc identifiers
    _gyroIdMSP = 18;
    _accIdMSP = 19;

    _bus.setDeviceDataRegister(REG_OUTX_L_G, reinterpret_cast<uint8_t*>(&_spiAccGyroData), sizeof(_spiAccGyroData));

    _bus.writeRegister(REG_CTRL3_C, SW_RESET); // software reset
    delayMs(100);

    const uint8_t chipID = _bus.readRegisterWithTimeout(REG_WHO_AM_I, 100);
    delayMs(1);
#if defined(SERIAL_OUTPUT)
    Serial.print("IMU init, chipID:0x");
    Serial.println(chipID, HEX);
#endif
    //assert(chipID == REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C || chipID == REG_WHO_AM_I_RESPONSE_ISM330DHCX || chipID == REG_WHO_AM_I_RESPONSE_LSM6DSOX);
    if (chipID != REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C && chipID != REG_WHO_AM_I_RESPONSE_ISM330DHCX && chipID != REG_WHO_AM_I_RESPONSE_LSM6DSOX) {
        return NOT_DETECTED;
    }

    // set data ready pulsed
    _bus.writeRegister(REG_DATA_READY_PULSE_CONFIG, DATA_READY_PULSED);
    delayMs(1);
    // INT pins are by default forced to ground, so active high
    _bus.writeRegister(REG_INT1_CTRL, INT1_DRDY_G); // Enable gyro data ready on INT1 pin
    delayMs(1);
    _bus.writeRegister(REG_INT2_CTRL, INT2_DRDY_G); // Enable gyro data ready on INT2 pin
    delayMs(1);
    _bus.writeRegister(REG_CTRL3_C, BDU | IF_INC); // Block Data Update and automatically increment registers when read via serial interface (I2C or SPI)
    delayMs(1);
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    _bus.writeRegister(REG_CTRL4_C, LPF1_SEL_G | I2C_DISABLE);  // enable gyro LPF, disable I2C
#else
    _bus.writeRegister(REG_CTRL4_C, LPF1_SEL_G); // enable gyro LPF
#endif
    delayMs(1);

    // calculate the GYRO_ODR bit values to write to the REG_CTRL2_G register
    const uint8_t GYRO_ODR =
        targetOutputDataRateHz == 0 ? GYRO_ODR_6664_HZ :
        targetOutputDataRateHz > 3332 ? GYRO_ODR_6664_HZ :
        targetOutputDataRateHz > 1666 ? GYRO_ODR_3332_HZ :
        targetOutputDataRateHz > 833 ? GYRO_ODR_1666_HZ :
        targetOutputDataRateHz > 416 ? GYRO_ODR_833_HZ :
        targetOutputDataRateHz > 208 ? GYRO_ODR_416_HZ :
        targetOutputDataRateHz > 104 ? GYRO_ODR_208_HZ :
        targetOutputDataRateHz > 52 ? GYRO_ODR_104_HZ :
        targetOutputDataRateHz > 26 ? GYRO_ODR_52_HZ :
        targetOutputDataRateHz > 13 ? GYRO_ODR_26_HZ : GYRO_ODR_12p5_HZ;
    // report the value that was actually set
    _gyroSampleRateHz =
        GYRO_ODR == GYRO_ODR_6664_HZ ? 6664 :
        GYRO_ODR == GYRO_ODR_3332_HZ ? 3332 :
        GYRO_ODR == GYRO_ODR_1666_HZ ? 1666 :
        GYRO_ODR == GYRO_ODR_833_HZ ? 833 :
        GYRO_ODR == GYRO_ODR_416_HZ ? 416 :
        GYRO_ODR == GYRO_ODR_208_HZ ? 208 :
        GYRO_ODR == GYRO_ODR_104_HZ ? 104:
        GYRO_ODR == GYRO_ODR_52_HZ ? 52 :
        GYRO_ODR == GYRO_ODR_26_HZ ? 26 : 12;

    // set the anti-alias LPF filter according to the gyro sampling rate
    _bus.writeRegister(REG_CTRL6_C, _gyroSampleRateHz > 3000 ? LPF1_HI : LPF1_MEDIUM_HI);
    delayMs(1);

    switch (gyroSensitivity) {
    case GYRO_FULL_SCALE_125_DPS: // NOLINT(bugprone-branch-clone) false positive
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_125_DPS | GYRO_ODR);
        _gyroResolutionDPS = 245.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_250_DPS:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_245_DPS | GYRO_ODR); // cppcheck-suppress badBitmaskCheck
        _gyroResolutionDPS = 245.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_500_DPS:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_500_DPS | GYRO_ODR);
        _gyroResolutionDPS = 500.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_1000_DPS:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_1000_DPS | GYRO_ODR);
        _gyroResolutionDPS = 1000.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_2000_DPS:
        [[fallthrough]];
    default:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_2000_DPS | GYRO_ODR);
        _gyroResolutionDPS = 2000.0F / 32768.0F;
        break;
    }
    _gyroResolutionRPS = _gyroResolutionDPS * degreesToRadians;
    delayMs(1);

    // calculate the ACC_ODR bit values to write to the REG_CTRL1_XL register
    const uint8_t ACC_ODR =
        targetOutputDataRateHz == 0 ? ACC_ODR_6664_HZ :
        targetOutputDataRateHz > 3332 ? ACC_ODR_6664_HZ :
        targetOutputDataRateHz > 1666 ? ACC_ODR_3332_HZ :
        targetOutputDataRateHz > 833 ? ACC_ODR_1666_HZ :
        targetOutputDataRateHz > 416 ? ACC_ODR_833_HZ :
        targetOutputDataRateHz > 208 ? ACC_ODR_416_HZ :
        targetOutputDataRateHz > 104 ? ACC_ODR_208_HZ :
        targetOutputDataRateHz > 52 ? ACC_ODR_104_HZ :
        targetOutputDataRateHz > 26 ? ACC_ODR_52_HZ :
        targetOutputDataRateHz > 13 ? ACC_ODR_26_HZ : ACC_ODR_12p5_HZ;
    // report the value that was actually set
    _accSampleRateHz =
        ACC_ODR == ACC_ODR_6664_HZ ? 6664 :
        ACC_ODR == ACC_ODR_3332_HZ ? 3332 :
        ACC_ODR == ACC_ODR_1666_HZ ? 1666 :
        ACC_ODR == ACC_ODR_833_HZ ? 833 :
        ACC_ODR == ACC_ODR_416_HZ ? 416 :
        ACC_ODR == ACC_ODR_208_HZ ? 208 :
        ACC_ODR == ACC_ODR_104_HZ ? 104:
        ACC_ODR == ACC_ODR_52_HZ ? 52 :
        ACC_ODR == ACC_ODR_26_HZ ? 26 : 12;

    switch (accSensitivity) {
    case ACC_FULL_SCALE_2G:
        _bus.writeRegister(REG_CTRL1_XL, ACC_RANGE_2G | ACC_ODR); // cppcheck-suppress badBitmaskCheck
        _accResolution = 2.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_4G:
        _bus.writeRegister(REG_CTRL1_XL, ACC_RANGE_4G | ACC_ODR);
        _accResolution = 4.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_8G:
        _bus.writeRegister(REG_CTRL1_XL, ACC_RANGE_8G | ACC_ODR);
        _accResolution = 8.0F / 32768.0F;
        break;
    default:
        _bus.writeRegister(REG_CTRL1_XL, ACC_RANGE_16G | ACC_ODR);
        _accResolution = 16.0F / 32768.0F;
        break;
    }
    delayMs(1);

    // return the gyro sample rate actually set
    return static_cast<int>(_gyroSampleRateHz);
}

void IMU_LSM6DS3TR_C::setInterruptDriven()
{
    // set interrupt level as configured in init()
    _bus.setInterruptDriven(BUS_BASE::IRQ_EDGE_RISE);
}

IMU_Base::xyz_int32_t IMU_LSM6DS3TR_C::readGyroRaw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_G, &gyro.data[0], sizeof(gyro));
    i2cSemaphoreGive();

    return xyz_int32_t {
        .x = gyro.value.x,
        .y = gyro.value.y,
        .z = gyro.value.z
    };
}

IMU_Base::xyz_int32_t IMU_LSM6DS3TR_C::readAccRaw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_ACC, &acc.data[0], sizeof(acc));
    i2cSemaphoreGive();

    return xyz_int32_t {
        .x = acc.value.x,
        .y = acc.value.y,
        .z = acc.value.z
    };
}

FAST_CODE IMU_Base::accGyroRPS_t IMU_LSM6DS3TR_C::readAccGyroRPS()
{
    i2cSemaphoreTake();
    //_bus.readRegister(REG_OUTX_L_G, &_spiAccGyroData.accGyro.data[0], sizeof(_spiAccGyroData.accGyro));
    _bus.readDeviceData();
    i2cSemaphoreGive();

    return accGyroRPSFromRaw(_spiAccGyroData.accGyro.value);
}

/*!
Return the gyroAcc data that was read in the ISR
*/
FAST_CODE IMU_Base::accGyroRPS_t IMU_LSM6DS3TR_C::getAccGyroRPS() const
{
    return accGyroRPSFromRaw(_spiAccGyroData.accGyro.value);
}

IMU_Base::accGyroRPS_t IMU_LSM6DS3TR_C::accGyroRPSFromRaw(const acc_gyro_data_t::value_t& data) const
{
#if defined(LIBRARY_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =  static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
        }
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =  static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .y = -static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .y = -static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
        }
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_XNEG_YNEG_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x = -static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y = -static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y = -static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
        }
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x = -static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .y =  static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .y =  static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
        }
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =  static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS,
            .z = -static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution,
            .z = -static_cast<float>(data.acc_y - _accOffset.y)* _accResolution
        }
    };
#else
    // Axis order mapping done at run-time
    const accGyroRPS_t accGyroRPS {
        .gyroRPS = {
            .x =  static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
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
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
