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
    static const IMU_BMI270 imu(IMU_Base::XPOS_YPOS_ZPOS, IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN);
    TEST_ASSERT_EQUAL(4096, imu.getAccOneG_Raw());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_bmi270);

    UNITY_END();
}
