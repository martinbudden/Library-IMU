#include <IMU_ICM426xx.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_icm426xx()
{
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 2000000;
    static IMU_ICM426xx imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::BUS_INDEX_0, BUS_SPI::spi_pins_t{});
#else
    static IMU_ICM426xx imu(IMU_Base::XPOS_YPOS_ZPOS, IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN);
#endif
    TEST_ASSERT_EQUAL(4096, imu.getAccOneG_Raw());
    TEST_ASSERT_EQUAL(0, imu.getFlags());

    imu.init(8000, IMU_Base::GYRO_FULL_SCALE_MAX, IMU_Base::ACC_FULL_SCALE_MAX, nullptr);
    TEST_ASSERT_EQUAL(12, imu.getGyroIdMSP());
    TEST_ASSERT_EQUAL(11, imu.getAccIdMSP());

    TEST_ASSERT_EQUAL_FLOAT(2000.0F / 32768.0F, imu.getGyroResolutionDPS());
    TEST_ASSERT_EQUAL_FLOAT(16.0F / 32768.0F, imu.getAccResolution());
    TEST_ASSERT_EQUAL(8000, imu.getGyroSampleRateHz());
    TEST_ASSERT_EQUAL(8000, imu.getAccSampleRateHz());

    imu.init(4000, IMU_Base::GYRO_FULL_SCALE_2000_DPS, IMU_Base::ACC_FULL_SCALE_16G, nullptr);
    TEST_ASSERT_EQUAL_FLOAT(2000.0F / 32768.0F, imu.getGyroResolutionDPS());
    TEST_ASSERT_EQUAL_FLOAT(16.0F / 32768.0F, imu.getAccResolution());
    TEST_ASSERT_EQUAL(4000, imu.getGyroSampleRateHz());
    TEST_ASSERT_EQUAL(4000, imu.getAccSampleRateHz());

    imu.init(2000, IMU_Base::GYRO_FULL_SCALE_1000_DPS, IMU_Base::ACC_FULL_SCALE_8G, nullptr);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F / 32768.0F, imu.getGyroResolutionDPS());
    TEST_ASSERT_EQUAL_FLOAT(8.0F / 32768.0F, imu.getAccResolution());
    TEST_ASSERT_EQUAL(2000, imu.getGyroSampleRateHz());
    TEST_ASSERT_EQUAL(2000, imu.getAccSampleRateHz());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_icm426xx);

    UNITY_END();
}
