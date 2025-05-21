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
    enum { SPI_BUFFER_SIZE = 2};
    struct spi_acc_gyro_data_t {
        std::array<uint8_t, SPI_BUFFER_SIZE> spiBuffer; // buffer for use when reading gyro by SPI
        acc_gyro_data_t accGyro;
    };
#pragma pack(pop)
public:
#if defined(USE_IMU_BMI270_SPI)
    // SPI constructors
    IMU_BMI270(axis_order_t axisOrder, uint32_t frequency, BUS_SPI::spi_index_t SPI_index, const BUS_SPI::pins_t& pins);
#else
    // I2C constructors
    IMU_BMI270(axis_order_t axisOrder, BUS_I2C::i2c_index_t I2C_index, const BUS_I2C::pins_t& pins, uint8_t I2C_address);
    IMU_BMI270(axis_order_t axisOrder, const BUS_I2C::pins_t& pins, uint8_t I2C_address) : IMU_BMI270(axisOrder, BUS_I2C::I2C_INDEX_0, pins, I2C_address) {}
    IMU_BMI270(axis_order_t axisOrder, const BUS_I2C::pins_t& pins) : IMU_BMI270(axisOrder, pins, I2C_ADDRESS) {}
#endif
public:
    virtual int init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity, void* i2cMutex) override;
    void loadConfigurationData();
    void UNLOCK_IMU_DATA_READY() { _bus.UNLOCK_IMU_DATA_READY_FROM_ISR(); } // for debugging
    virtual void setInterrupt(int userIrq) override;
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual accGyroRPS_t readAccGyroRPS() override;
    virtual accGyroRPS_t getAccGyroRPS() const override;
private:
    accGyroRPS_t accGyroRPSFromRaw(const acc_gyro_data_t::value_t& data) const;
private:
#if defined(USE_IMU_BMI270_SPI)
    BUS_SPI _bus; //!< SPI bus interface,
#else
    BUS_I2C _bus; //!< I2C bus interface
#endif
    spi_acc_gyro_data_t _spiAccGyroData {};
    uint8_t _dmaSpiRegister {}; // register value used for DMA transfer
};
