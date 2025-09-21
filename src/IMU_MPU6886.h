#pragma once

#include "BUS_I2C.h"
#include "BUS_SPI.h"
#include "IMU_Base.h"


class IMU_MPU6886 : public IMU_Base {
public:
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    // SPI constructors
    IMU_MPU6886(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::stm32_spi_pins_t& pins);
    IMU_MPU6886(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::pins_t& pins);
#else
    // I2C constructors
    IMU_MPU6886(axis_order_e axisOrder, BUS_BASE::bus_index_e I2C_index, const BUS_I2C::pins_t& pins, uint8_t I2C_address);
    IMU_MPU6886(axis_order_e axisOrder, const BUS_I2C::pins_t& pins, uint8_t I2C_address) : IMU_MPU6886(axisOrder, BUS_I2C::BUS_INDEX_0, pins, I2C_address) {}
    IMU_MPU6886(axis_order_e axisOrder, const BUS_I2C::pins_t& pins) : IMU_MPU6886(axisOrder, pins, I2C_ADDRESS) {}
#endif
public:
    virtual int init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) override;
public:
    static constexpr uint8_t I2C_ADDRESS = 0x68;
    enum acc_scale_e { AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G };
    enum gyro_scale_e { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };
#pragma pack(push, 1)
    union mems_sensor_data_t {
        enum { DATA_SIZE = 6 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            uint8_t x_h;
            uint8_t x_l;
            uint8_t y_h;
            uint8_t y_l;
            uint8_t z_h;
            uint8_t z_l;
        } value;
    };
private:
    union acc_temperature_gyro_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 14 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            uint8_t acc_x_h;
            uint8_t acc_x_l;
            uint8_t acc_y_h;
            uint8_t acc_y_l;
            uint8_t acc_z_h;
            uint8_t acc_z_l;
            uint8_t temperature_h;
            uint8_t temperature_l;
            uint8_t gyro_x_h;
            uint8_t gyro_x_l;
            uint8_t gyro_y_h;
            uint8_t gyro_y_l;
            uint8_t gyro_z_h;
            uint8_t gyro_z_l;
        } value;
    };
    struct spi_acc_temperature_gyro_data_t {
        std::array<uint8_t, BUS_BASE::SPI_BUFFER_SIZE> spiBuffer; // buffer for use when reading gyro by SPI
        acc_temperature_gyro_data_t accGyro;
    };
    union acc_temperature_gyro_array_t {
        enum { DATA_SIZE = 1036 };
        acc_temperature_gyro_data_t accTemperatureGyro[74];
        uint8_t data[DATA_SIZE];
    };
#pragma pack(pop)
public:
    virtual void setInterruptDriven() override;
    virtual void setGyroOffset(const xyz_int32_t& gyroOffset) override;

    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    FAST_CODE virtual accGyroRPS_t readAccGyroRPS() override;
    FAST_CODE virtual accGyroRPS_t getAccGyroRPS() const override;

    float readTemperature() const;
    int32_t readTemperatureRaw() const;

    static mems_sensor_data_t::value_t gyroOffsetFromXYZ(const xyz_int32_t& data);
private:
    xyz_t gyroRPS_FromRaw(const mems_sensor_data_t::value_t& data) const;
    xyz_t accFromRaw(const mems_sensor_data_t::value_t& data) const;
    accGyroRPS_t accGyroRPSFromRaw(const acc_temperature_gyro_data_t::value_t& data) const;
private:
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    BUS_SPI _bus; //!< SPI bus interface
#else
    BUS_I2C _bus; //!< I2C bus interface
#endif
    spi_acc_temperature_gyro_data_t _spiAccTemperatureGyroData {};
};
