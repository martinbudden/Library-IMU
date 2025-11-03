#pragma once

#include "BarometerBase.h"
#include "BUS_I2C.h"
#include "BUS_SPI.h"


class Barometer_BMP280 : public BarometerBase {
public:
    static constexpr uint8_t I2C_ADDRESS = 0x77;
    static constexpr uint8_t I2C_ADDRESS_ALTERNATIVE = 0x76;
    static constexpr uint8_t CHIP_ID = 0x58;
    enum { MAX_SPI_FREQUENCY_HZ = 10000000 };
public:
    union calibration_data_u { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 24 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            // temperature calibration
            uint16_t dig_T1;
            int16_t dig_T2;
            int16_t dig_T3;
            // pressure calibration
            uint16_t dig_P1;
            int16_t dig_P2;
            int16_t dig_P3;
            int16_t dig_P4;
            int16_t dig_P5;
            int16_t dig_P6;
            int16_t dig_P7;
            int16_t dig_P8;
            int16_t dig_P9;
        } value;
    };
    union pressure_temperature_data_u { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 6 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            uint8_t pressure_msb;
            uint8_t pressure_lsb;
            uint8_t pressure_xlsb;
            uint8_t temperature_msb;
            uint8_t temperature_lsb;
            uint8_t temperature_xlsb;
        } value;
    };
public:
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    // SPI constructors
    Barometer_BMP280(uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::stm32_spi_pins_t& pins);
    Barometer_BMP280(uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::spi_pins_t& pins);
#else
    // I2C constructors
    Barometer_BMP280(BUS_BASE::bus_index_e I2C_index, const BUS_I2C::stm32_i2c_pins_t& pins, uint8_t I2C_address);
    Barometer_BMP280(const BUS_I2C::stm32_i2c_pins_t& pins, uint8_t I2C_address) : Barometer_BMP280(BUS_I2C::BUS_INDEX_0, pins, I2C_address) {}
    explicit Barometer_BMP280(const BUS_I2C::stm32_i2c_pins_t& pins) : Barometer_BMP280(pins, I2C_ADDRESS) {}

    Barometer_BMP280(BUS_BASE::bus_index_e I2C_index, const BUS_I2C::i2c_pins_t& pins, uint8_t I2C_address);
    Barometer_BMP280(const BUS_I2C::i2c_pins_t& pins, uint8_t I2C_address) : Barometer_BMP280(BUS_I2C::BUS_INDEX_0, pins, I2C_address) {}
    explicit Barometer_BMP280(const BUS_I2C::i2c_pins_t& pins) : Barometer_BMP280(pins, I2C_ADDRESS) {}
#endif
    virtual int init() override;
    virtual float readTemperatureCelsius() override;
    virtual float readPressurePascals() override;
    virtual float readAltitudeMeters() override;
    virtual float calculateAltitudeMeters(float pressure) override;
private:
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    BUS_SPI _bus; //!< SPI bus interface
#else
    BUS_I2C _bus; //!< I2C bus interface
#endif
    int32_t _temperatureFine {};
    calibration_data_u _calibrationData {};
};
