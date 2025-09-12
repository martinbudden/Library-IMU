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
    struct spi_acc_gyro_data_t {
        std::array<uint8_t, BUS_BASE::SPI_BUFFER_SIZE> spiBuffer; // buffer for use when reading gyro by SPI
        acc_gyro_data_t accGyro;
    };
#pragma pack(pop)
public:
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    // SPI constructors
    IMU_BMI270(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::port_pins_t& pins);
    IMU_BMI270(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::pins_t& pins);
#else
    // I2C constructors
    IMU_BMI270(axis_order_e axisOrder, BUS_BASE::bus_index_e I2C_index, const BUS_I2C::pins_t& pins, uint8_t I2C_address);
    IMU_BMI270(axis_order_e axisOrder, const BUS_I2C::pins_t& pins, uint8_t I2C_address) : IMU_BMI270(axisOrder, BUS_I2C::BUS_INDEX_0, pins, I2C_address) {}
    IMU_BMI270(axis_order_e axisOrder, const BUS_I2C::pins_t& pins) : IMU_BMI270(axisOrder, pins, I2C_ADDRESS) {}
#endif
public:
    virtual int init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) override;
    void loadConfigurationData();
    virtual void setInterruptDriven() override;

    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    FAST_CODE virtual accGyroRPS_t readAccGyroRPS() override;
    FAST_CODE virtual accGyroRPS_t getAccGyroRPS() const override;
private:
    accGyroRPS_t accGyroRPSFromRaw(const acc_gyro_data_t::value_t& data) const;
private:
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    BUS_SPI _bus; //!< SPI bus interface,
#else
    BUS_I2C _bus; //!< I2C bus interface
#endif
    spi_acc_gyro_data_t _spiAccGyroData {};
};
