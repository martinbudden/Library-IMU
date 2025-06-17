#include <IMU_MPU6886.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_mpu_6886()
{
    const IMU_Base::xyz_int32_t input { .x =70, .y = -70, .z = 400 };
    const IMU_MPU6886::mems_sensor_data_t::value_t output = IMU_MPU6886::gyroOffsetFromXYZ(input);

    TEST_ASSERT_EQUAL_UINT8(0xFF, output.x_h);
    TEST_ASSERT_EQUAL_UINT8(0xBA, output.x_l);

    TEST_ASSERT_EQUAL_UINT8(0x00, output.y_h);
    TEST_ASSERT_EQUAL_UINT8(0x46, output.y_l);

    TEST_ASSERT_EQUAL_UINT8(0xFE, output.z_h);
    TEST_ASSERT_EQUAL_UINT8(0x70, output.z_l);

#if defined(USE_IMU_MPU6886_SPI)
    constexpr uint32_t spiFrequency = 2000000;
    static const IMU_MPU6886 imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::SPI_INDEX_0, {});
#else
    static const IMU_MPU6886 imu(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::pins_t{.sda=IMU_I2C_SDA_PIN, .scl=IMU_I2C_SCL_PIN});
#endif
    TEST_ASSERT_EQUAL(4096, imu.getAccOneG_Raw());
    TEST_ASSERT_EQUAL(0, imu.getFlags());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_mpu_6886);

    UNITY_END();
}
