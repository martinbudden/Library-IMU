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
#if defined(USE_IMU_LSM6DS3TR_C_SPI) || defined(USE_IMU_ISM330DHCX_SPI) || defined(USE_IMU_LSM6DSOX_SPI)
    constexpr uint32_t spiFrequency = 2000000;
    const IMU_LSM6DS3TR_C imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::SPI_INDEX_0,
        BUS_SPI::pins_t{ .cs=IMU_SPI_CS_PIN, .sck=IMU_SPI_SCK_PIN, .cipo=IMU_SPI_CIPO_PIN, .copi=IMU_SPI_COPI_PIN, .irq=IMU_SPI_INTERRUPT_PIN, .irqLevel=0 });
#else
    constexpr uint8_t SDA_pin = 0;
    constexpr uint8_t SCL_pin = 0;
    const IMU_LSM6DS3TR_C imu(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::pins_t{.sda=IMU_I2C_SDA_PIN, .scl=IMU_I2C_SCL_PIN});
#endif
    TEST_ASSERT_EQUAL(4096, imu.getAccOneG_Raw());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_lsm6ds3tr_c);

    UNITY_END();
}
