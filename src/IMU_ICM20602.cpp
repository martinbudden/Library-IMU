#include "IMU_ICM20602.h"
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif

#include <cassert>


namespace { // use anonymous namespace to make items local to this translation unit

constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_8G_RES { 8.0 / 32768.0 };

constexpr uint8_t REG_XG_OFFS_TC_H          = 0x04;
constexpr uint8_t REG_XG_OFFS_TC_L          = 0x05;
constexpr uint8_t REG_YG_OFFS_TC_H          = 0x07;
constexpr uint8_t REG_YG_OFFS_TC_L          = 0x08;
constexpr uint8_t REG_ZG_OFFS_TC_H          = 0x0A;
constexpr uint8_t REG_ZG_OFFS_TC_L          = 0x0B;

constexpr uint8_t REG_SELF_TEST_X_ACCEL     = 0x0D;
constexpr uint8_t REG_SELF_TEST_Y_ACCEL     = 0x0E;
constexpr uint8_t REG_SELF_TEST_Z_ACCEL     = 0x0F;

//  GYRO OFFSET ADJUSTMENT REGISTERS
constexpr uint8_t REG_XG_OFFS_USRH          = 0x13;
constexpr uint8_t REG_XG_OFFS_USRL          = 0x14;
constexpr uint8_t REG_YG_OFFS_USRH          = 0x15;
constexpr uint8_t REG_YG_OFFS_USRL          = 0x16;
constexpr uint8_t REG_ZG_OFFS_USRH          = 0x17;
constexpr uint8_t REG_ZG_OFFS_USRL          = 0x18;

constexpr uint8_t REG_SAMPLE_RATE_DIVIDER   = 0x19;
    constexpr uint8_t DIVIDE_BY_1 = 0x00;
    constexpr uint8_t DIVIDE_BY_2 = 0x01;
constexpr uint8_t REG_CONFIG                = 0x1A;
    // Update rate: 1kHz, Filter:176 3-DB BW (Hz), default value used by M5Stack, the least filtered 1kHz update variant
    constexpr uint8_t DLPF_CFG_1 = 0x01;
    // Update rate: 8kHz, Filter:3281 3-DB BW (Hz) - only non-32kHz variant less filtered than DLPF_CFG_1
    constexpr uint8_t DLPF_CFG_7 = 0x07;
constexpr uint8_t REG_GYRO_CONFIG           = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG          = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2         = 0x1D;

constexpr uint8_t REG_FIFO_ENABLE           = 0x23;
    constexpr uint8_t GYRO_FIFO_EN = 0b00001000;
    constexpr uint8_t ACC_FIFO_EN  = 0b00000100;

constexpr uint8_t REG_INT_PIN_CFG           = 0x37;
constexpr uint8_t REG_INT_ENABLE            = 0x38;
constexpr uint8_t FIFO_WM_INT_STATUS        = 0x39; // FIFO watermark interrupt status

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

constexpr uint8_t REG_FIFO_WM_TH1           = 0x60; // FIFO watermark threshold in number of bytes
constexpr uint8_t REG_FIFO_WM_TH2           = 0x61;

constexpr uint8_t REG_SIGNAL_PATH_RESET     = 0x68;
constexpr uint8_t REG_ACCEL_INTEL_CTRL      = 0x69;
constexpr uint8_t REG_USER_CTRL             = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1            = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2            = 0x6C;

constexpr uint8_t REG_FIFO_COUNT_H          = 0x72;
constexpr uint8_t REG_FIFO_COUNT_L          = 0x73;
constexpr uint8_t REG_FIFO_R_W              = 0x74;

constexpr uint8_t REG_WHO_AM_I              = 0x75;

// ACCELEROMETER OFFSET REGISTERS
constexpr uint8_t REG_XA_OFFSET_H           = 0x77;
constexpr uint8_t REG_XA_OFFSET_L           = 0x78;
constexpr uint8_t REG_YA_OFFSET_H           = 0x7A;
constexpr uint8_t REG_YA_OFFSET_L           = 0x7B;
constexpr uint8_t REG_ZA_OFFSET_H           = 0x7D;
constexpr uint8_t REG_ZA_OFFSET_L           = 0x7E;

} // end namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,hicpp-signed-bitwise)

#if defined(LIBRARY_IMU_USE_SPI_BUS)
IMU_ICM20602::IMU_ICM20602(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::stm32_spi_pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins)
{
}
IMU_ICM20602::IMU_ICM20602(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins)
{
}
#else
IMU_ICM20602::IMU_ICM20602(axis_order_e axisOrder, BUS_BASE::bus_index_e I2C_index, const BUS_I2C::pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, I2C_index, pins)
{
}
#endif

int IMU_ICM20602::init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex)
{
    (void)targetOutputDataRateHz;
    (void)gyroSensitivity;
    (void)accSensitivity;

    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_temperature_gyro_data_t) == acc_temperature_gyro_data_t::DATA_SIZE);

#if defined(FRAMEWORK_USE_FREERTOS)
    _i2cMutex = static_cast<SemaphoreHandle_t>(i2cMutex);
#else
    _i2cMutex = i2cMutex;
#endif

    // MSP compatible gyro and acc identifiers, use defaults, since no MSP value for MPU6886
    _gyroIdMSP = MSP_GYRO_ID_DEFAULT;
    _accIdMSP = MSP_ACC_ID_DEFAULT;

    _bus.setDeviceDataRegister(REG_GYRO_XOUT_H, reinterpret_cast<uint8_t*>(&_spiAccTemperatureGyroData), sizeof(_spiAccTemperatureGyroData));

    i2cSemaphoreTake(_i2cMutex);

    const uint8_t chipID = _bus.readRegister(REG_WHO_AM_I);
    delayMs(1);
#if defined(SERIAL_OUTPUT)
    Serial.printf("IMU init, chipID=%02x\r\n", chipID);
#else
    (void)chipID;
#endif

    _bus.writeRegister(REG_PWR_MGMT_1, 0); // clear the power management register
    delayMs(10);

    constexpr uint8_t DEVICE_RESET = 0x01U << 7U;
    _bus.writeRegister(REG_PWR_MGMT_1, DEVICE_RESET); // reset the device
    delayMs(10);

    constexpr uint8_t CLKSEL_1 = 0x01;
    _bus.writeRegister(REG_PWR_MGMT_1, CLKSEL_1); // CLKSEL must be set to 001 to achieve full gyroscope performance.
    delayMs(10);

    // Gyro scale is fixed at 2000DPS, the maximum supported.
    constexpr uint8_t GYRO_FCHOICE_B = 0x00; // enables gyro update rate and filter configuration using REG_CONFIG
    _bus.writeRegister(REG_GYRO_CONFIG, (GFS_2000DPS << 3) | GYRO_FCHOICE_B); // cppcheck-suppress badBitmaskCheck
    _gyroResolutionDPS = GYRO_2000DPS_RES;
    _gyroResolutionRPS = GYRO_2000DPS_RES * degreesToRadians;
    delayMs(1);

    // Accelerometer scale is fixed at 8G, the maximum supported.
    _bus.writeRegister(REG_ACCEL_CONFIG, AFS_8G << 3U);
    _accResolution = ACC_8G_RES;
    delayMs(1);

    constexpr uint8_t ACC_FCHOICE_B = 0x00; // Filter:218.1 3-DB BW (Hz), least filtered 1kHz update variant
    _bus.writeRegister(REG_ACCEL_CONFIG2, ACC_FCHOICE_B);
    delayMs(1);

    constexpr uint8_t FIFO_MODE_OVERWRITE = 0b01000000;
    _bus.writeRegister(REG_CONFIG, DLPF_CFG_1 | FIFO_MODE_OVERWRITE);
    delayMs(1);

    // M5Stack default divider is two, giving 500Hz output rate
    _bus.writeRegister(REG_SAMPLE_RATE_DIVIDER, DIVIDE_BY_2);
    delayMs(1);
    _gyroSampleRateHz = 500;
    _accSampleRateHz = 500;

    _bus.writeRegister(REG_FIFO_ENABLE, 0x00); // FIFO disabled
    delayMs(1);

    // M5 Unified settings
    //_bus.writeRegister(REG_INT_PIN_CFG, 0b11000000); // Active low, open drain 50us pulse width, clear on read
    _bus.writeRegister(REG_INT_PIN_CFG, 0x22);
    delayMs(1);

    constexpr uint8_t DATA_RDY_INT_EN = 0x01;
    _bus.writeRegister(REG_INT_ENABLE, DATA_RDY_INT_EN); // data ready interrupt enabled
    delayMs(10);

    _bus.writeRegister(REG_USER_CTRL, 0x00);

    i2cSemaphoreGive(_i2cMutex);
    delayMs(1);

    // return the gyro sample rate actually set
    return static_cast<int>(_gyroSampleRateHz);
}

/*!
Gyro offset adjustment.
These values are used to remove DC bias from the sensor output. The values are
added to the gyroscope sensor value before going into the sensor register.
So the offset value is negated.
*/
IMU_ICM20602::mems_sensor_data_t::value_t IMU_ICM20602::gyroOffsetFromXYZ(const xyz_int32_t& data)
{
    return mems_sensor_data_t::value_t {
        .x_h = static_cast<uint8_t>((-data.x) >> 8U),
        .x_l = static_cast<uint8_t>((-data.x) & 0xFFU),
        .y_h = static_cast<uint8_t>((-data.y) >> 8U),
        .y_l = static_cast<uint8_t>((-data.y) & 0xFFU),
        .z_h = static_cast<uint8_t>((-data.z) >> 8U),
        .z_l = static_cast<uint8_t>((-data.z) & 0xFFU)
    };
}

void IMU_ICM20602::setInterruptDriven()
{
    // set interrupt level as configured in init()
    _bus.setInterruptDriven(BUS_BASE::IRQ_EDGE_RISE);
}

void IMU_ICM20602::setGyroOffset(const xyz_int32_t& gyroOffset)
{
#if false
    // Setting the XG_OFFS registers seems to have no effect, so this code disabled
    const mems_sensor_data_t offset = gyroOffsetFromXYZ(gyroOffset);
    _bus.writeRegister(REG_XG_OFFS_USRH, reinterpret_cast<const uint8_t*>(&offset), sizeof(offset)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    delayMs(1);
#else
    _gyroOffset = gyroOffset;
#endif
}

IMU_Base::xyz_int32_t IMU_ICM20602::readGyroRaw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_GYRO_XOUT_H, &gyro.data[0], sizeof(gyro));
    i2cSemaphoreGive(_i2cMutex);

    xyz_int32_t ret {
        .x = static_cast<int16_t>((gyro.value.x_h << 8U) | gyro.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((gyro.value.y_h << 8U) | gyro.value.y_l),
        .z = static_cast<int16_t>((gyro.value.z_h << 8U) | gyro.value.z_l)
    };
    return ret;
}

IMU_Base::xyz_int32_t IMU_ICM20602::readAccRaw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    i2cSemaphoreGive(_i2cMutex);

     return xyz_int32_t {
        .x = static_cast<int16_t>((acc.value.x_h << 8U) | acc.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((acc.value.y_h << 8U) | acc.value.y_l),
        .z = static_cast<int16_t>((acc.value.z_h << 8U) | acc.value.z_l)
    };
}

xyz_t IMU_ICM20602::readGyroRPS()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_GYRO_XOUT_H, &gyro.data[0], sizeof(gyro));
    i2cSemaphoreGive(_i2cMutex);

    return gyroRPS_FromRaw(gyro.value);
}

xyz_t IMU_ICM20602::readGyroDPS()
{
    return readGyroRPS() * radiansToDegrees;
}

xyz_t IMU_ICM20602::readAcc()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    i2cSemaphoreGive(_i2cMutex);

    return accFromRaw(acc.value);
}

FAST_CODE IMU_Base::accGyroRPS_t IMU_ICM20602::readAccGyroRPS()
{
    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_ACCEL_XOUT_H, &_spiAccTemperatureGyroData.accGyro.data[0], sizeof(_spiAccTemperatureGyroData.accGyro));
    //_bus.readDeviceData();
    i2cSemaphoreGive(_i2cMutex);

    return accGyroRPSFromRaw(_spiAccTemperatureGyroData.accGyro.value);
}

/*!
Return the gyroAcc data that was read in the ISR
*/
FAST_CODE IMU_Base::accGyroRPS_t IMU_ICM20602::getAccGyroRPS() const
{
    return accGyroRPSFromRaw(_spiAccTemperatureGyroData.accGyro.value);
}

int32_t IMU_ICM20602::readTemperatureRaw() const
{
    std::array<uint8_t, 2> data;

    i2cSemaphoreTake(_i2cMutex);
    _bus.readRegister(REG_TEMP_OUT_H, &data[0], sizeof(data));
    i2cSemaphoreGive(_i2cMutex);

    const int32_t temperature = static_cast<int16_t>((data[0] << 8U) | data[1]); // NOLINT(hicpp-use-auto,modernize-use-auto)
    return temperature;
}

float IMU_ICM20602::readTemperature() const
{
    const int32_t temperature = readTemperatureRaw();

    return static_cast<float>(temperature) / 326.8F + 25.0F; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

xyz_t IMU_ICM20602::gyroRPS_FromRaw(const mems_sensor_data_t::value_t& data) const
{
    // static cast to int16_t to sign extend the 8 bit values
#if defined(LIBRARY_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _gyroOffset.y) * _gyroResolutionRPS,
        .y =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _gyroOffset.x) * _gyroResolutionRPS,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _gyroOffset.z) * _gyroResolutionRPS
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _gyroOffset.y) * _gyroResolutionRPS,
        .y = -static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _gyroOffset.x) * _gyroResolutionRPS,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _gyroOffset.z) * _gyroResolutionRPS
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _gyroOffset.x) * _gyroResolutionRPS,
        .y =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _gyroOffset.z) * _gyroResolutionRPS,
        .z = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _gyroOffset.y) * _gyroResolutionRPS
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
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

xyz_t IMU_ICM20602::accFromRaw(const mems_sensor_data_t::value_t& data) const
{
#if defined(LIBRARY_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _accOffset.y)* _accResolution,
        .y =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _accOffset.x)* _accResolution,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _accOffset.z)* _accResolution
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _accOffset.y)* _accResolution,
        .y = -static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _accOffset.x)* _accResolution,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _accOffset.z)* _accResolution
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l) - _accOffset.x)* _accResolution,
        .y =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l) - _accOffset.z)* _accResolution,
        .z = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l) - _accOffset.y)* _accResolution
    };
#elif defined(LIBRARY_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
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

IMU_Base::accGyroRPS_t IMU_ICM20602::accGyroRPSFromRaw(const acc_temperature_gyro_data_t::value_t& data) const
{
#if defined(LIBRARY_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
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
#elif defined(LIBRARY_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
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
#elif defined(LIBRARY_IMU_FIXED_AXES_XNEG_YNEG_ZPOS)
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
#elif defined(LIBRARY_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
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
#elif defined(LIBRARY_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
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
// NOLINTEND(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,hicpp-signed-bitwise)
