#include <IMU_BMI270.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_bmi270()
{
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 2000000;
    static const IMU_BMI270 imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::BUS_INDEX_0, BUS_SPI::pins_t{});
#else
    static const IMU_BMI270 imu(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::pins_t{});
#endif
    TEST_ASSERT_EQUAL(4096, imu.getAccOneG_Raw());
    TEST_ASSERT_EQUAL(0, imu.getFlags());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_bmi270);

    UNITY_END();
}
