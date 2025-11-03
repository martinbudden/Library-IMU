#include <Barometer_BMP280.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_bmp280()
{
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 2000000;
    static const Barometer_BMP280 barometer(spiFrequency, BUS_SPI::BUS_INDEX_0, BUS_SPI::spi_pins_t{});
#else
    static const Barometer_BMP280 barometer(BUS_I2C::i2c_pins_t{});
#endif
    barometer.init();
    TEST_ASSERT_EQUAL(0.0F, Barometer_BMP280::calculateAltitude(1.0F, 1.0F));
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_bps280);

    UNITY_END();
}
