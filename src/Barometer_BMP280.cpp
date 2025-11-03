#include "Barometer_BMP280.h"
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif

#include <cmath>

namespace { // use anonymous namespace to make items local to this translation unit
    constexpr uint8_t REG_DIG_T1        = 0x88;
    constexpr uint8_t REG_DIG_T2        = 0x8A;
    constexpr uint8_t REG_DIG_T3        = 0x8C;
    constexpr uint8_t REG_DIG_P1        = 0x8E;
    constexpr uint8_t REG_DIG_P2        = 0x90;
    constexpr uint8_t REG_DIG_P3        = 0x92;
    constexpr uint8_t REG_DIG_P4        = 0x94;
    constexpr uint8_t REG_DIG_P5        = 0x96;
    constexpr uint8_t REG_DIG_P6        = 0x98;
    constexpr uint8_t REG_DIG_P7        = 0x9A;
    constexpr uint8_t REG_DIG_P8        = 0x9C;
    constexpr uint8_t REG_DIG_P9        = 0x9E;
    constexpr uint8_t REG_CHIPID        = 0xD0;
    constexpr uint8_t REG_VERSION       = 0xD1;
    constexpr uint8_t REG_SOFTRESET     = 0xE0;
    constexpr uint8_t REG_CAL26         = 0xE1; // R calibration = 0xE1-0xF0
    constexpr uint8_t REG_STATUS        = 0xF3;
    constexpr uint8_t REG_CONTROL       = 0xF4;
    constexpr uint8_t REG_CONFIG        = 0xF5;
    constexpr uint8_t REG_PRESSURE_MSB  = 0xF7;
    constexpr uint8_t REG_PRESSURE_LSB  = 0xF8;
    constexpr uint8_t REG_PRESSURE_XLSB = 0xF9;
    constexpr uint8_t REG_TEMPERATURE_MSB   = 0xFA;
    constexpr uint8_t REG_TEMPERATURE_LSB   = 0xFB;
    constexpr uint8_t REG_TEMPERATURE_XLSB  = 0xFC;

    constexpr uint8_t SAMPLING_NONE     = 0x00;
    constexpr uint8_t SAMPLING_X1       = 0x01;
    constexpr uint8_t SAMPLING_X2       = 0x02; // 2x oversampling
    constexpr uint8_t SAMPLING_X4       = 0x03; // 4x oversampling
    constexpr uint8_t SAMPLING_X8       = 0x04; // 8x oversampling
    constexpr uint8_t SAMPLING_X16      = 0x05; // 16x oversampling

    constexpr uint8_t MODE_SLEEP        = 0x00;
    constexpr uint8_t MODE_FORCED       = 0x01;
    constexpr uint8_t MODE_NORMAL       = 0x03;
    constexpr uint8_t MODE_SOFT_RESET_CODE = 0xB6;

  // Filtering level
    constexpr uint8_t FILTER_OFF        = 0x00;
    constexpr uint8_t FILTER_X2         = 0x01;
    constexpr uint8_t FILTER_X4         = 0x02;
    constexpr uint8_t FILTER_X8         = 0x03;
    constexpr uint8_t FILTER_X16        = 0x04;

  // Standby duration in milliseconds
    constexpr uint8_t STANDBY_MS_0p5    = 0x00; // 0.5ms
    constexpr uint8_t STANDBY_MS_62p5   = 0x01; // 62.5ms
    constexpr uint8_t STANDBY_MS_125    = 0x02;
    constexpr uint8_t STANDBY_MS_250    = 0x03;
    constexpr uint8_t STANDBY_MS_500    = 0x04;
    constexpr uint8_t STANDBY_MS_1000   = 0x05;
    constexpr uint8_t STANDBY_MS_2000   = 0x06;
    constexpr uint8_t STANDBY_MS_4000   = 0x07;

    constexpr uint8_t SPI_3WIRE_ENABLE  = 0x01;
} // end namespace/

#if defined(LIBRARY_SENSORS_BAROMETER_USE_SPI_BUS)
Barometer_BMP280::Barometer_BMP280(uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::stm32_spi_pins_t& pins) :
    BarometerBase(_bus),
    _bus(frequency, SPI_index, pins)
{
}
Barometer_BMP280::Barometer_BMP280(uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::spi_pins_t& pins) :
    BarometerBase(_bus),
    _bus(frequency, SPI_index, pins)
{
}
#else
Barometer_BMP280::Barometer_BMP280(BUS_BASE::bus_index_e I2C_index, const BUS_I2C::i2c_pins_t& pins, uint8_t I2C_address) :
    BarometerBase(_bus),
    _bus(I2C_address, I2C_index, pins)
{
}
#endif

int Barometer_BMP280::init()
{
    /*_calibrationData.value.dig_T1 = readRegister16_LE(REG_DIG_T1);
    _calibrationData.value.dig_T2 = readRegister16S_LE(REG_DIG_T2);
    _calibrationData.value.dig_T3 = readRegister16S_LE(REG_DIG_T3);

    _calibrationData.value.dig_P1 = readRegister16_LE(REG_DIG_P1);
    _calibrationData.value.dig_P2 = readRegister16S_LE(REG_DIG_P2);
    _calibrationData.value.dig_P3 = readRegister16S_LE(REG_DIG_P3);
    _calibrationData.value.dig_P4 = readRegister16S_LE(REG_DIG_P4);
    _calibrationData.value.dig_P5 = readRegister16S_LE(REG_DIG_P5);
    _calibrationData.value.dig_P6 = readRegister16S_LE(REG_DIG_P6);
    _calibrationData.value.dig_P7 = readRegister16S_LE(REG_DIG_P7);
    _calibrationData.value.dig_P8 = readRegister16S_LE(REG_DIG_P8);
    _calibrationData.value.dig_P9 = readRegister16S_LE(REG_DIG_P9);*/

    _bus.readRegister(REG_DIG_T1, &_calibrationData.data[0], sizeof(_calibrationData));
    delayMs(1);


    /*
    don't need to set REG_CONFIG, since default of 0 gives us what we need, namely:
    const uint8_t standbyMs = STANDBY_MS_0p5;
    const uint8_t filter = FILTER_OFF;
    const uint8_t spi3wire = 0;
    _bus.writeRegister(REG_CONFIG, (standbyMs << 5) | (filter << 2) | spi3wire); // cppcheck-suppress badBitmaskCheck
    delayMs(1);
    */

    const uint8_t mode = MODE_FORCED;
    const uint8_t TEMPERATURE_OSR = SAMPLING_X1;
    const uint8_t PRESSURE_OSR = SAMPLING_X8;
    _bus.writeRegister(REG_CONTROL, (TEMPERATURE_OSR << 5) | (PRESSURE_OSR << 2) | mode);
    delayMs(1);

    _pressureAtReferenceAltitude = readPressurePascals();
    delayMs(1);
    _referenceAltitude = 0.0F;

    enum { TIME_INIT_MAX = 20 }; // 20/16 = 1.25 ms
    enum { TIME_MEASURE_PER_OSRS_MAX = 37 }; // 37/16 = 2.3125 ms
    enum { TIME_SETUP_PRESSURE_MAX = 10 }; // 10/16 = 0.625 ms

    constexpr uint32_t delayMs = ((TIME_INIT_MAX + TIME_MEASURE_PER_OSRS_MAX * (((1 << TEMPERATURE_OSR) >> 1) + ((1 << PRESSURE_OSR) >> 1)) + TIME_SETUP_PRESSURE_MAX + 15) / 16);
    _sampleRateHz = 1000 / delayMs; // delay ~ 23ms, sample rate ~ 43.5Hz
    return static_cast<int>(_sampleRateHz);
}

float Barometer_BMP280::readTemperatureCelsius()
{
    const int32_t T1 = static_cast<int32_t>(_calibrationData.value.dig_T1);
    const int32_t T2 = static_cast<int32_t>(_calibrationData.value.dig_T2);
    const int32_t T3 = static_cast<int32_t>(_calibrationData.value.dig_T3);

    pressure_temperature_data_u pt; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)
    _bus.readRegister(REG_TEMPERATURE_MSB, &pt.data[3], 3);
    const int32_t adcT = static_cast<int32_t>(
            ((static_cast<uint32_t>(pt.value.temperature_msb) << 16) |
            (static_cast<uint32_t>(pt.value.temperature_lsb) << 8) |
            static_cast<uint32_t>(pt.value.temperature_xlsb)) >> 4
        );
    const int32_t vt1 = (((adcT >> 3) - (T1 << 1))) * T2;
    const int32_t vt2 = ((((adcT >> 4) - T1) * ((adcT >> 4) - T1)) >> 12) * T3;
    _temperatureFine = (vt1 >> 11) + (vt2 >> 14);
    _temperatureCelsius = static_cast<float>((_temperatureFine * 5 + 128) >> 8) / 100.0F;

    return _temperatureCelsius;
}

float Barometer_BMP280::readPressurePascals()
{
    const int32_t T1 = static_cast<int32_t>(_calibrationData.value.dig_T1);
    const int32_t T2 = static_cast<int32_t>(_calibrationData.value.dig_T2);
    const int32_t T3 = static_cast<int32_t>(_calibrationData.value.dig_T3);
    const int64_t P1 = static_cast<int64_t>(_calibrationData.value.dig_P1);
    const int64_t P2 = static_cast<int64_t>(_calibrationData.value.dig_P2);
    const int64_t P3 = static_cast<int64_t>(_calibrationData.value.dig_P3);
    const int64_t P4 = static_cast<int64_t>(_calibrationData.value.dig_P4);
    const int64_t P5 = static_cast<int64_t>(_calibrationData.value.dig_P5);
    const int64_t P6 = static_cast<int64_t>(_calibrationData.value.dig_P6);
    const int64_t P7 = static_cast<int64_t>(_calibrationData.value.dig_P7);
    const int64_t P8 = static_cast<int64_t>(_calibrationData.value.dig_P8);
    const int64_t P9 = static_cast<int64_t>(_calibrationData.value.dig_P9);

    // burst read of temperature and pressure data
    // read together in burst so data is consistent, as specified in datasheet
    pressure_temperature_data_u pt; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)
    _bus.readRegister(REG_PRESSURE_MSB, &pt.data[0], sizeof(pt));
    const int32_t adcT = static_cast<int32_t>(
            ((static_cast<uint32_t>(pt.value.temperature_msb) << 16) |
            (static_cast<uint32_t>(pt.value.temperature_lsb) << 8) |
            static_cast<uint32_t>(pt.value.temperature_xlsb)) >> 4
        );

    // set _temperatureFine.
    const int32_t vt1 = (((adcT >> 3) - (T1 << 1))) * T2;
    const int32_t vt2 = ((((adcT >> 4) - T1) * ((adcT >> 4) - T1)) >> 12) * T3;
    _temperatureFine = (vt1 >> 11) + (vt2 >> 14);
    _temperatureCelsius = static_cast<float>((_temperatureFine * 5 + 128) >> 8) / 100.0F;

    int64_t vp1 = static_cast<int64_t>(_temperatureFine) - 128000;
    int64_t vp2 = vp1 * vp1 * P6;
    vp2 = vp2 + ((vp1 * P5) << 17);
    vp2 = vp2 + (P4 << 35);
    vp1 = ((vp1 * vp1 * P3) >> 8) + ((vp1 * P2) << 12);
    vp1 = (((((int64_t)1) << 47) + vp1)) * P1 >> 33;

    if (vp1 == 0) {
        return 0.0F; // avoid division by zero
    }
    const int32_t adcP = static_cast<int32_t>(
            ((static_cast<uint32_t>(pt.value.pressure_msb) << 16) |
            (static_cast<uint32_t>(pt.value.pressure_lsb) << 8) |
            static_cast<uint32_t>(pt.value.pressure_xlsb)) >> 4
        );
    int64_t p = 1048576 - adcP;
    p = (((p << 31) - vp2) * 3125) / vp1;
    vp1 = (P9 * (p >> 13) * (p >> 13)) >> 25;
    vp2 = (P8 * p) >> 19;

    p = ((p + vp1 + vp2) >> 8) + (P7 << 4);
    _pressurePascals =  static_cast<float>(p) / 256.0F;
    return _pressurePascals;
}

float Barometer_BMP280::readAltitudeMeters()
{
    return calculateAltitudeMeters(readPressurePascals());
}

float Barometer_BMP280::calculateAltitudeMeters(float pressure)
{
    return 44330.0F * (1.0F - std::pow(pressure/_pressureAtReferenceAltitude, 0.1903F));
}
