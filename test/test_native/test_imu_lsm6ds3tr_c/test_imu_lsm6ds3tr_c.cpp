#include <IMU_LSM6DS3TR_C.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_lsm6ds3tr_c()
{
    constexpr uint32_t spiFrequency = 2000000;
    IMU_LSM6DS3TR_C imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, IMU_SPI_CS_PIN, IMU_SPI_SCK_PIN, IMU_SPI_CIPO_PIN, IMU_SPI_COPI_PIN);
    TEST_ASSERT_EQUAL(4096, imu.getAccOneG_Raw());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_lsm6ds3tr_c);

    UNITY_END();
}
