#pragma once

#include "BUS_I2C.h"
#include "BUS_SPI.h"
#include "IMU_Base.h"

class IMU_LSM6DS3TR_C : public IMU_Base {
public:
    static constexpr uint8_t I2C_ADDRESS = 0x6A;
    static constexpr uint8_t I2C_ADDRESS_ALTERNATIVE = 0x6B;
#pragma pack(push, 1)
    union mems_sensor_data_t {
        enum { DATA_SIZE = 6 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            int16_t x;
            int16_t y;
            int16_t z;
        } value;
    };
    union acc_gyro_data_t {
        enum { DATA_SIZE = 12 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            int16_t gyro_x;
            int16_t gyro_y;
            int16_t gyro_z;
            int16_t acc_x;
            int16_t acc_y;
            int16_t acc_z;
        } value;
    };
#pragma pack(pop)
public:
#if defined(USE_IMU_LSM6DS3TR_C_SPI) || defined(USE_IMU_ISM330DHCX_SPI) || defined(USE_LSM6DSOX_SPI)
    // SPI constructor
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, uint32_t frequency, BUS_SPI::spi_index_t SPI_index, uint8_t CS_pin, uint8_t SCK_pin, uint8_t CIPO_pin, uint8_t COPI_pin);
#else
    // I2C constructors
#if !defined(FRAMEWORK_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_TEST)
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, TwoWire& wire, uint8_t SDA_pin, uint8_t SCL_pin, uint8_t I2C_address, void* i2cMutex);
#endif
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, BUS_I2C::i2c_index_t I2C_index, uint8_t SDA_pin, uint8_t SCL_pin, uint8_t I2C_address, void* i2cMutex);
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, uint8_t I2C_address, void* i2cMutex) : IMU_LSM6DS3TR_C(axisOrder, BUS_I2C::I2C_INDEX_0, SDA_pin, SCL_pin, I2C_address, i2cMutex) {}
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, uint8_t I2C_address) : IMU_LSM6DS3TR_C(axisOrder, SDA_pin, SCL_pin, I2C_address, nullptr) {}
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex) : IMU_LSM6DS3TR_C(axisOrder, SDA_pin, SCL_pin, I2C_ADDRESS, i2cMutex) {}
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin) : IMU_LSM6DS3TR_C(axisOrder, SDA_pin, SCL_pin, I2C_ADDRESS, nullptr) {}
#endif
public:
    virtual int init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity) override;
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() override;
private:
    gyroRPS_Acc_t gyroRPS_AccFromRaw(const acc_gyro_data_t::value_t& data) const;
private:
#if defined(USE_IMU_LSM6DS3TR_C_SPI) || defined(USE_IMU_ISM330DHCX_SPI) || defined(USE_LSM6DSOX_SPI)
    BUS_SPI _bus; //!< SPI bus interface,
#else
    BUS_I2C _bus; //!< I2C bus interface
#endif
    acc_gyro_data_t _accGyroData {};
};
