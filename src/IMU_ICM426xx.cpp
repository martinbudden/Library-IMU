#include "IMU_ICM426xx.h"
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

constexpr uint8_t REG_SENSOR_CONFIG0        = 0x03;
constexpr uint8_t REG_GYRO_CONFIG_STATIC2   = 0x0B;
constexpr uint8_t REG_GYRO_CONFIG_STATIC3   = 0x0C;
constexpr uint8_t REG_GYRO_CONFIG_STATIC4   = 0x0D;
constexpr uint8_t REG_GYRO_CONFIG_STATIC5   = 0x0E;
constexpr uint8_t REG_GYRO_CONFIG_STATIC6   = 0x0F;
constexpr uint8_t REG_GYRO_CONFIG_STATIC7   = 0x10;
constexpr uint8_t REG_GYRO_CONFIG_STATIC8   = 0x11;
constexpr uint8_t REG_GYRO_CONFIG_STATIC9   = 0x12;
constexpr uint8_t REG_GYRO_CONFIG_STATIC10  = 0x13;

constexpr uint8_t REG_INT_CONFIG            = 0x14;
    constexpr uint8_t INT1_MODE_LATCHED             = 0b00000100;
    constexpr uint8_t INT1_MODE_PULSED              = 0b00000000;
    constexpr uint8_t INT1_DRIVE_CIRCUIT_PUSH_PULL  = 0b00000010;
    constexpr uint8_t INT1_POLARITY_ACTIVE_HIGH     = 0b00000001;
constexpr uint8_t REG_FIFO_CONFIG           = 0x16;

constexpr uint8_t REG_TEMP_DATA1            = 0x1D;
constexpr uint8_t REG_TEMP_DATA0            = 0x1E;
constexpr uint8_t REG_ACCEL_DATA_X1         = 0x1F;
constexpr uint8_t REG_ACCEL_DATA_X0         = 0x20;
constexpr uint8_t REG_ACCEL_DATA_Y1         = 0x21;
constexpr uint8_t REG_ACCEL_DATA_Y0         = 0x22;
constexpr uint8_t REG_ACCEL_DATA_Z1         = 0x23;
constexpr uint8_t REG_ACCEL_DATA_Z0         = 0x24;
constexpr uint8_t REG_GYRO_DATA_X1          = 0x25;
constexpr uint8_t REG_GYRO_DATA_X0          = 0x26;
constexpr uint8_t REG_GYRO_DATA_Y1          = 0x27;
constexpr uint8_t REG_GYRO_DATA_Y0          = 0x28;
constexpr uint8_t REG_GYRO_DATA_Z1          = 0x29;
constexpr uint8_t REG_GYRO_DATA_Z0          = 0x2A;

constexpr uint8_t REG_TMST_FSYNCH           = 0x2B;
constexpr uint8_t REG_TMST_FSYNCHL          = 0x2C;
constexpr uint8_t REG_INT_STATUS            = 0x2D;

constexpr uint8_t REG_FIFO_COUNTH           = 0x2E;
constexpr uint8_t REG_FIFO_COUNTL           = 0x2F;
constexpr uint8_t REG_FIFO_DATA             = 0x30;

constexpr uint8_t REG_APEX_DATA0            = 0x31;
constexpr uint8_t REG_APEX_DATA1            = 0x33;
constexpr uint8_t REG_APEX_DATA2            = 0x34;
constexpr uint8_t REG_APEX_DATA3            = 0x34;
constexpr uint8_t REG_APEX_DATA4            = 0x35;
constexpr uint8_t REG_APEX_DATA5            = 0x36;

constexpr uint8_t REG_INT_STATUS2           = 0x37;
constexpr uint8_t REG_INT_STATUS3           = 0x38;

constexpr uint8_t REG_SIGNAL_PATH_RESET     = 0x4B;
constexpr uint8_t REG_INTF_CONFIG0          = 0x4C;
    constexpr uint8_t SENSOR_DATA_BIG_ENDIAN    = 0b00010000; // default
    constexpr uint8_t SENSOR_DATA_LITTLE_ENDIAN = 0b00000000; // datasheet gives very little information on how this works
    constexpr uint8_t UI_SIFS_CFG_DISABLE_I2C   = 0b00000011;
constexpr uint8_t REG_INTF_CONFIG1          = 0x4D;
    constexpr uint8_t AFSR_DISABLE          = 0x40;

constexpr uint8_t REG_PWR_MGMT0             = 0x4E;
    constexpr uint8_t PWR_OFF               = 0x00;
    constexpr uint8_t PWR_TEMP_ENABLED      = 0b00000000;
    constexpr uint8_t PWR_GYRO_LOW_NOISE    = 0b00001100;
    constexpr uint8_t PWR_ACCEL_LOW_NOISE   = 0b00000011;

constexpr uint8_t REG_GYRO_CONFIG0          = 0x4F;
    constexpr uint8_t GYRO_RANGE_15p625_DPS = 0b00000000;
    constexpr uint8_t GYRO_RANGE_31p25_DPS  = 0b00100000;
    constexpr uint8_t GYRO_RANGE_62p5_DPS   = 0b01000000;
    constexpr uint8_t GYRO_RANGE_125_DPS    = 0b01100000;
    constexpr uint8_t GYRO_RANGE_250_DPS    = 0b10000000;
    constexpr uint8_t GYRO_RANGE_500_DPS    = 0b10100000;
    constexpr uint8_t GYRO_RANGE_1000_DPS   = 0b11000000;
    constexpr uint8_t GYRO_RANGE_2000_DPS   = 0b11100000;

    constexpr uint8_t GYRO_ODR_32000_HZ   = 0b00000001;
    constexpr uint8_t GYRO_ODR_16000_HZ   = 0b00000010;
    constexpr uint8_t GYRO_ODR_8000_HZ    = 0b00000011;
    constexpr uint8_t GYRO_ODR_4000_HZ    = 0b00000100;
    constexpr uint8_t GYRO_ODR_2000_HZ    = 0b00000101;
    constexpr uint8_t GYRO_ODR_1000_HZ    = 0b00000110;
    constexpr uint8_t GYRO_ODR_200_HZ     = 0b00000111;
    constexpr uint8_t GYRO_ODR_100_HZ     = 0b00001000;
    constexpr uint8_t GYRO_ODR_50_HZ      = 0b00001001;
    constexpr uint8_t GYRO_ODR_25_HZ      = 0b00001010;
    constexpr uint8_t GYRO_ODR_12p5_HZ    = 0b00001011;
    constexpr uint8_t GYRO_ODR_500_HZ     = 0b00001111;

constexpr uint8_t REG_ACCEL_CONFIG0         = 0x50;
    constexpr uint8_t ACCEL_RANGE_16G = 0b00000000;
    constexpr uint8_t ACCEL_RANGE_8G  = 0b00100000;
    constexpr uint8_t ACCEL_RANGE_4G  = 0b01000000;
    constexpr uint8_t ACCEL_RANGE_2G  = 0b01100000;

    constexpr uint8_t ACCEL_ODR_32000_HZ   = 0b00000001;
    constexpr uint8_t ACCEL_ODR_16000_HZ   = 0b00000010;
    constexpr uint8_t ACCEL_ODR_8000_HZ    = 0b00000011;
    constexpr uint8_t ACCEL_ODR_4000_HZ    = 0b00000100;
    constexpr uint8_t ACCEL_ODR_2000_HZ    = 0b00000101;
    constexpr uint8_t ACCEL_ODR_1000_HZ    = 0b00000110;
    constexpr uint8_t ACCEL_ODR_200_HZ     = 0b00000111;
    constexpr uint8_t ACCEL_ODR_100_HZ     = 0b00001000;
    constexpr uint8_t ACCEL_ODR_50_HZ      = 0b00001001;
    constexpr uint8_t ACCEL_ODR_25_HZ      = 0b00001010;
    constexpr uint8_t ACCEL_ODR_12p5_HZ    = 0b00001011;
    constexpr uint8_t ACCEL_ODR_500_HZ     = 0b00001111;

constexpr uint8_t REG_GYRO_CONFIG1          = 0x51;
constexpr uint8_t REG_GYRO_ACCEL_CONFIG0    = 0x52;
    constexpr uint8_t ACCEL_FILTER_LOW_LATENCY  = 0b11110000;
    constexpr uint8_t GYRO_FILTER_LOW_LATENCY   = 0b00001111;
constexpr uint8_t REG_ACCEL_CONFIG1         = 0x53;

constexpr uint8_t REG_TMST_CONFIG           = 0x54;
constexpr uint8_t REG_APEX_CONFIG           = 0x56;
constexpr uint8_t REG_SMD_CONFIG            = 0x57;
constexpr uint8_t REG_FIFO_CONFIG1          = 0x5F;
constexpr uint8_t REG_FIFO_CONFIG2          = 0x60;
constexpr uint8_t REG_FIFO_CONFIG3          = 0x61;
constexpr uint8_t REG_FSYNC_CONFIG          = 0x62;
constexpr uint8_t REG_INT_CONFIG0           = 0x63;
    constexpr uint8_t INT_CLEAR_ON_STATUS_BIT_READ = 0b00000000;
constexpr uint8_t REG_INT_CONFIG1           = 0x64;
    constexpr uint8_t INT_ASYNC_RESET           = 0b00001000; // this bit should be set to 0 for proper INT1 and INT2 pin operation
    constexpr uint8_t INT_TPULSE_DURATION_8US   = 0b01000000; // interrupt puls duration 8us, required for ODR >=4kHz
    constexpr uint8_t INT_TDEASSERT_DISABLE     = 0b00100000; // required for ODR >= 4kHz

constexpr uint8_t REG_INT_SOURCE0           = 0x65;
    constexpr uint8_t INT1_UI_DATA_READY_ENABLED     = 0b00001000;
constexpr uint8_t REG_INT_SOURCE1           = 0x66;
constexpr uint8_t REG_INT_SOURCE3           = 0x68;
constexpr uint8_t REG_INT_SOURCE4           = 0x69;

constexpr uint8_t REG_FIFO_LOST_PKT0        = 0x6C;
constexpr uint8_t REG_FIFO_LOST_PKT1        = 0x6D;
constexpr uint8_t REG_SELF_TEST_CONFIG3     = 0x70;

constexpr uint8_t REG_WHO_AM_I              = 0x75;
    constexpr uint8_t WHO_AM_I_RESPONSE_ICM42605  = 0x43;
    constexpr uint8_t WHO_AM_I_RESPONSE_ICM42688P = 0x47;

constexpr uint8_t REG_BANK_SEL              = 0x76;
constexpr uint8_t REG_INTF_CONFIG4          = 0x7A;
constexpr uint8_t REG_INTF_CONFIG5          = 0x7B;
constexpr uint8_t REG_INTF_CONFIG6          = 0x7C;

// User Bank 2 Register Map
constexpr uint8_t REG_BANK2_ACCEL_CONFIG_STATIC2    = 0x03;
constexpr uint8_t REG_BANK2_ACCEL_CONFIG_STATIC3    = 0x04;
constexpr uint8_t REG_BANK2_ACCEL_CONFIG_STATIC4    = 0x05;

} // end namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

/*!
Gyroscope data rates up to 6.4 kHz, accelerometer up to 1.6 kHz
*/
#if defined(USE_IMU_ICM426XX_SPI)
IMU_ICM426xx::IMU_ICM426xx(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::port_pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);
}
IMU_ICM426xx::IMU_ICM426xx(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins)
{
}
#else
IMU_ICM426xx::IMU_ICM426xx(axis_order_e axisOrder, BUS_BASE::bus_index_e I2C_index, const BUS_I2C::pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, I2C_index, pins)
{
}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_TEST)
IMU_ICM426xx::IMU_ICM426xx(axis_order_e axisOrder, TwoWire& wire, const BUS_I2C::pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, wire, pins)
{
}
#endif
#endif

int IMU_ICM426xx::init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) // NOLINT(readability-function-cognitive-complexity)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);

#if defined(I2C_MUTEX_REQUIRED)
    _i2cMutex = static_cast<SemaphoreHandle_t>(i2cMutex);
#else
    (void)i2cMutex;
#endif

    // MSP compatible gyro and acc identifiers
    enum { MSP_GYRO_ID_ICM42605 = 12, MSP_GYRO_ID_ICM42688P = 13 };
    enum { MSP_ACC_ID_ICM42605 = 11, MSP_ACC_ID_ICM42688P = 12 };

    _gyroIdMSP = MSP_GYRO_ID_ICM42605;
    _accIdMSP = MSP_ACC_ID_ICM42605;

    _bus.setDeviceDataRegister(REG_ACCEL_DATA_X1, reinterpret_cast<uint8_t*>(&_spiAccGyroData), sizeof(_spiAccGyroData));

    _bus.writeRegister(REG_BANK_SEL, 0); // software reset
    _bus.writeRegister(REG_PWR_MGMT0, PWR_OFF); // turn off IMU

    // Configure gyro and acc UI Filters
    _bus.writeRegister(REG_BANK_SEL, 0);

    _bus.writeRegister(REG_GYRO_ACCEL_CONFIG0, ACCEL_FILTER_LOW_LATENCY | GYRO_FILTER_LOW_LATENCY);

    // Configure interrupts

    _bus.writeRegister(REG_INT_CONFIG, INT1_MODE_PULSED | INT1_DRIVE_CIRCUIT_PUSH_PULL | INT1_POLARITY_ACTIVE_HIGH); // cppcheck-suppress badBitmaskCheck

    _bus.writeRegister(REG_INT_CONFIG0, INT_CLEAR_ON_STATUS_BIT_READ);

    _bus.writeRegister(REG_INT_SOURCE0, INT1_UI_DATA_READY_ENABLED);

    _bus.writeRegister(REG_INT_CONFIG1, INT_TPULSE_DURATION_8US | INT_TDEASSERT_DISABLE);

    //_bus.writeRegister(REG_INTF_CONFIG0, SENSOR_DATA_LITTLE_ENDIAN | UI_SIFS_CFG_DISABLE_I2C);

    // Disable AFSR to prevent stalls in gyro output
    uint8_t intFConfig1 = _bus.readRegister(REG_INTF_CONFIG1);
    //constexpr uint8_t CONFIG1_RESET_VALUE   = 0b10010001;
    constexpr uint8_t CONFIG1_AFSR_MASK     = 0b00001100U;
    constexpr uint8_t CONFIG1_AFSR_DISABLE  = 0b00000100U;
    intFConfig1 &= ~CONFIG1_AFSR_MASK; // NOLINT(hicpp-signed-bitwise)
    intFConfig1 |= CONFIG1_AFSR_DISABLE;
    _bus.writeRegister(REG_INTF_CONFIG1, intFConfig1);

    // Turn on gyro and acc on again so Output Data Rate(ODR) and Full Scale Rate (FSRP can be configured
    _bus.writeRegister(REG_PWR_MGMT0, PWR_TEMP_ENABLED | PWR_GYRO_LOW_NOISE | PWR_ACCEL_LOW_NOISE); // cppcheck-suppress badBitmaskCheck
    delayMs(1);

    // calculate the GYRO_ODR bit values to write to the REG_GYRO_CONFIG0 register
    const uint8_t GYRO_ODR =
        targetOutputDataRateHz == 0 ? GYRO_ODR_8000_HZ : // default to 8kHz
        targetOutputDataRateHz > 16000 ? GYRO_ODR_32000_HZ :
        targetOutputDataRateHz > 8000 ? GYRO_ODR_16000_HZ :
        targetOutputDataRateHz > 4000 ? GYRO_ODR_8000_HZ :
        targetOutputDataRateHz > 2000 ? GYRO_ODR_4000_HZ :
        targetOutputDataRateHz > 1000 ? GYRO_ODR_2000_HZ :
        targetOutputDataRateHz > 500 ? GYRO_ODR_1000_HZ :
        targetOutputDataRateHz > 200 ? GYRO_ODR_500_HZ :
        targetOutputDataRateHz > 100 ? GYRO_ODR_200_HZ :
        targetOutputDataRateHz > 50 ? GYRO_ODR_100_HZ :
        targetOutputDataRateHz > 25 ? GYRO_ODR_50_HZ :
        targetOutputDataRateHz > 12 ? GYRO_ODR_25_HZ : GYRO_ODR_12p5_HZ;
    // report the value that was actually set
    _gyroSampleRateHz =
        GYRO_ODR == GYRO_ODR_32000_HZ  ? 32000 :
        GYRO_ODR == GYRO_ODR_16000_HZ ? 16000 :
        GYRO_ODR == GYRO_ODR_8000_HZ ? 8000 :
        GYRO_ODR == GYRO_ODR_4000_HZ ? 4000 :
        GYRO_ODR == GYRO_ODR_2000_HZ ? 2000 :
        GYRO_ODR == GYRO_ODR_1000_HZ ? 1000 :
        GYRO_ODR == GYRO_ODR_500_HZ ? 500 :
        GYRO_ODR == GYRO_ODR_200_HZ ? 200 :
        GYRO_ODR == GYRO_ODR_100_HZ ? 100 :
        GYRO_ODR == GYRO_ODR_50_HZ ? 50 :
        GYRO_ODR == GYRO_ODR_25_HZ ? 25 : 12;

    // calculate the GYRO_RANGE bit values to write to the REG_GYRO_CONFIG0 register
    uint8_t GYRO_RANGE = 0;
    switch (gyroSensitivity) {
    case GYRO_FULL_SCALE_125_DPS:
        GYRO_RANGE = GYRO_RANGE_125_DPS;
        _gyroResolutionDPS = 125.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_250_DPS:
        GYRO_RANGE = GYRO_RANGE_250_DPS;
        _gyroResolutionDPS = 250.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_500_DPS:
        GYRO_RANGE = GYRO_RANGE_500_DPS;
        _gyroResolutionDPS = 500.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_1000_DPS:
        GYRO_RANGE = GYRO_RANGE_1000_DPS;
        _gyroResolutionDPS = 1000.0F / 32768.0F;
        break;
    default:
        GYRO_RANGE = GYRO_RANGE_2000_DPS;
        _gyroResolutionDPS = 2000.0F / 32768.0F;
        break;
    }
    _gyroResolutionRPS = _gyroResolutionDPS * degreesToRadians;

    _bus.writeRegister(REG_GYRO_CONFIG0, GYRO_RANGE | GYRO_ODR);
    delayMs(1);


    // calculate the ACCEL_ODR bit values to write to the REG_ACCEL_CONFIG0 register
    const uint8_t ACCEL_ODR =
        targetOutputDataRateHz == 0 ? ACCEL_ODR_8000_HZ : // default to 8kHz
        targetOutputDataRateHz > 16000 ? ACCEL_ODR_32000_HZ :
        targetOutputDataRateHz > 8000 ? ACCEL_ODR_16000_HZ :
        targetOutputDataRateHz > 4000 ? ACCEL_ODR_8000_HZ :
        targetOutputDataRateHz > 2000 ? ACCEL_ODR_4000_HZ :
        targetOutputDataRateHz > 1000 ? ACCEL_ODR_2000_HZ :
        targetOutputDataRateHz > 500 ? ACCEL_ODR_1000_HZ :
        targetOutputDataRateHz > 200 ? ACCEL_ODR_500_HZ :
        targetOutputDataRateHz > 100 ? ACCEL_ODR_200_HZ :
        targetOutputDataRateHz > 50 ? ACCEL_ODR_100_HZ :
        targetOutputDataRateHz > 25 ? ACCEL_ODR_50_HZ :
        targetOutputDataRateHz > 12 ? ACCEL_ODR_25_HZ : ACCEL_ODR_12p5_HZ;
    // report the value that was actually set
    _accSampleRateHz =
        ACCEL_ODR == ACCEL_ODR_32000_HZ  ? 32000 :
        ACCEL_ODR == ACCEL_ODR_16000_HZ ? 16000 :
        ACCEL_ODR == ACCEL_ODR_8000_HZ ? 8000 :
        ACCEL_ODR == ACCEL_ODR_4000_HZ ? 4000 :
        ACCEL_ODR == ACCEL_ODR_2000_HZ ? 2000 :
        ACCEL_ODR == ACCEL_ODR_1000_HZ ? 1000 :
        ACCEL_ODR == ACCEL_ODR_500_HZ ? 500 :
        ACCEL_ODR == ACCEL_ODR_200_HZ ? 200 :
        ACCEL_ODR == ACCEL_ODR_100_HZ ? 100 :
        ACCEL_ODR == ACCEL_ODR_50_HZ ? 50 :
        ACCEL_ODR == ACCEL_ODR_25_HZ ? 25 : 12;

    uint8_t ACCEL_RANGE = 0;
    switch (accSensitivity) {
    case ACC_FULL_SCALE_2G:
        ACCEL_RANGE = ACCEL_RANGE_2G;
        _accResolution = 2.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_4G:
        ACCEL_RANGE = ACCEL_RANGE_4G;
        _accResolution = 4.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_8G:
        ACCEL_RANGE = ACCEL_RANGE_8G;
        _accResolution = 8.0F / 32768.0F;
        break;
    default:
        ACCEL_RANGE = ACCEL_RANGE_16G;
        _accResolution = 16.0F / 32768.0F;
        break;
    }

    _bus.writeRegister(REG_ACCEL_CONFIG0, ACCEL_RANGE | ACCEL_ODR);
    delayMs(1);

    // return the gyro sample rate actually set
    return static_cast<int>(_gyroSampleRateHz);
}

void IMU_ICM426xx::setInterruptDriven()
{
    // set interrupt level as configured in init()
    _bus.setInterruptDriven(BUS_BASE::IRQ_EDGE_RISE);
}

IMU_Base::xyz_int32_t IMU_ICM426xx::readGyroRaw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake();
    _bus.readRegister(REG_GYRO_DATA_X1, &gyro.data[0], sizeof(gyro));
    i2cSemaphoreGive();

// NOLINTBEGIN(hicpp-signed-bitwise)
    return xyz_int32_t {
        .x = static_cast<int16_t>((gyro.value.x_h << 8U) | gyro.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((gyro.value.y_h << 8U) | gyro.value.y_l),
        .z = static_cast<int16_t>((gyro.value.z_h << 8U) | gyro.value.z_l)
    };
// NOLINTEND(hicpp-signed-bitwise)
}

IMU_Base::xyz_int32_t IMU_ICM426xx::readAccRaw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake();
    _bus.readRegister(REG_ACCEL_DATA_X1, &acc.data[0], sizeof(acc));
    i2cSemaphoreGive();

// NOLINTBEGIN(hicpp-signed-bitwise)
    return xyz_int32_t {
        .x = static_cast<int16_t>((acc.value.x_h << 8U) | acc.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((acc.value.y_h << 8U) | acc.value.y_l),
        .z = static_cast<int16_t>((acc.value.z_h << 8U) | acc.value.z_l)
    };
// NOLINTEND(hicpp-signed-bitwise)
}

xyz_t IMU_ICM426xx::readGyroRPS()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_GYRO_DATA_X1, &gyro.data[0], sizeof(gyro));
    i2cSemaphoreGive(_i2cMutex);

    return gyroRPS_FromRaw(gyro.value);
}

xyz_t IMU_ICM426xx::readGyroDPS()
{
    return readGyroRPS() * radiansToDegrees;
}

xyz_t IMU_ICM426xx::readAcc()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_ACCEL_DATA_X1, &acc.data[0], sizeof(acc));
    i2cSemaphoreGive(_i2cMutex);

    return accFromRaw(acc.value);
}

IRAM_ATTR IMU_Base::accGyroRPS_t IMU_ICM426xx::readAccGyroRPS()
{
    i2cSemaphoreTake(_i2cMutex);
    _bus.readDeviceData();
    i2cSemaphoreGive(_i2cMutex);

    return accGyroRPSFromRaw(_spiAccGyroData.accGyro.value);
}

/*!
Return the gyroAcc data that was read in the ISR
*/
IRAM_ATTR IMU_Base::accGyroRPS_t IMU_ICM426xx::getAccGyroRPS() const
{
    return accGyroRPSFromRaw(_spiAccGyroData.accGyro.value);
}

xyz_t IMU_ICM426xx::gyroRPS_FromRaw(const mems_sensor_data_t::value_t& data) const
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

xyz_t IMU_ICM426xx::accFromRaw(const mems_sensor_data_t::value_t& data) const
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

IMU_Base::accGyroRPS_t IMU_ICM426xx::accGyroRPSFromRaw(const acc_gyro_data_t::value_t& data) const
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
