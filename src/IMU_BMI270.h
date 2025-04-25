#pragma once

#include "BUS_I2C.h"
#include "BUS_SPI.h"
#include "IMU_Base.h"


class IMU_BMI270 : public IMU_Base {
public:
    static constexpr uint8_t I2C_ADDRESS = 0x68;
    static constexpr uint8_t I2C_ADDRESS_ALTERNATIVE = 0x69;

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
            int16_t acc_x;
            int16_t acc_y;
            int16_t acc_z;
            int16_t gyro_x;
            int16_t gyro_y;
            int16_t gyro_z;
        } value;
    };
#pragma pack(pop)
public:
#if defined(USE_IMU_BMI270_SPI)
    // SPI constructors
    IMU_BMI270(axis_order_t axisOrder, uint32_t frequency, BUS_SPI::spi_index_t SPI_index, uint8_t CS_pin);
#else
    // I2C constructors
    IMU_BMI270(axis_order_t axisOrder, BUS_I2C::i2c_index_t I2C_index, uint8_t SDA_pin, uint8_t SCL_pin, uint8_t I2C_address, void* i2cMutex);
    IMU_BMI270(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, uint8_t I2C_address, void* i2cMutex) : IMU_BMI270(axisOrder, BUS_I2C::I2C_INDEX_0, SDA_pin, SCL_pin, I2C_address, i2cMutex) {}
    IMU_BMI270(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, uint8_t I2C_address) : IMU_BMI270(axisOrder, SDA_pin, SCL_pin, I2C_address, nullptr) {}
    IMU_BMI270(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex) : IMU_BMI270(axisOrder, SDA_pin, SCL_pin, I2C_ADDRESS, i2cMutex) {}
    IMU_BMI270(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin) : IMU_BMI270(axisOrder, SDA_pin, SCL_pin, I2C_ADDRESS, nullptr) {}
#endif
public:
    virtual int init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity) override;
    void loadConfigurationData();
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() override;
private:
    gyroRPS_Acc_t gyroRPS_AccFromRaw(const acc_gyro_data_t::value_t& data) const;
private:
#if defined(USE_IMU_BMI270_SPI)
    BUS_SPI _bus; //!< SPI bus interface,
#else
    BUS_I2C _bus; //!< I2C bus interface
#endif
    acc_gyro_data_t _accGyroData {};
};
