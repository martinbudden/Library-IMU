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
    struct dma_acc_gyro_data_t {
        uint8_t filler;
        uint8_t spiReadStartByte; // dummy byte that receives data when imu register is written for DMA transfer
        acc_gyro_data_t accGyro;
    };
#pragma pack(pop)
public:
#if defined(USE_IMU_LSM6DS3TR_C_SPI) || defined(USE_IMU_ISM330DHCX_SPI) || defined(USE_LSM6DSOX_SPI)
    // SPI constructor
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, uint32_t frequency, BUS_SPI::spi_index_t SPI_index, const BUS_SPI::pins_t& pins);
#else
    // I2C constructors
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, BUS_I2C::i2c_index_t I2C_index, const BUS_I2C::pins_t& pins, uint8_t I2C_address);
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, const BUS_I2C::pins_t& pins, uint8_t I2C_address) : IMU_LSM6DS3TR_C(axisOrder, BUS_I2C::I2C_INDEX_0, pins, I2C_address) {}
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, const BUS_I2C::pins_t& pins) : IMU_LSM6DS3TR_C(axisOrder, pins, I2C_ADDRESS) {}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_TEST)
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, TwoWire& wire, const BUS_I2C::pins_t& pins, uint8_t I2C_address);
#endif
#endif
public:
    virtual int init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity, void* i2cMutex) override;
    virtual void setInterrupt(int userIrq) override;
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
    //acc_gyro_data_t _accGyroData {};
    dma_acc_gyro_data_t _accGyroData {};
    uint8_t _dmaSpiRegister {}; // register value used fro DMA transfer
};
