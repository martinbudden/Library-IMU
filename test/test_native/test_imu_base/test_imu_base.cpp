#include <IMU_Null.h>
#include <unity.h>



void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_map_axes()
{
    const xyz_t input { .x =3.0F, .y = 5.0F, .z = 7.0F };
    xyz_t output {};

    static const IMU_Null XPOS_YPOS_ZPOS(IMU_Base::XPOS_YPOS_ZPOS);
    output = XPOS_YPOS_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    static const IMU_Null YPOS_XNEG_ZPOS(IMU_Base::YPOS_XNEG_ZPOS);
    output = YPOS_XNEG_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    static const IMU_Null XNEG_YNEG_ZPOS(IMU_Base::XNEG_YNEG_ZPOS);
    output = XNEG_YNEG_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    static const IMU_Null YNEG_XPOS_ZPOS(IMU_Base::YNEG_XPOS_ZPOS);
    output = YNEG_XPOS_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);


    static const IMU_Null XPOS_YNEG_ZNEG(IMU_Base::XPOS_YNEG_ZNEG);
    output = XPOS_YNEG_ZNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.z);

    static const IMU_Null YPOS_XPOS_ZNEG(IMU_Base::YPOS_XPOS_ZNEG);
    output = YPOS_XPOS_ZNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.z);

    static const IMU_Null XNEG_YPOS_ZNEG(IMU_Base::XNEG_YPOS_ZNEG);
    output = XNEG_YPOS_ZNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.z);

    static const IMU_Null YNEG_XNEG_ZNEG(IMU_Base::YNEG_XNEG_ZNEG);
    output = YNEG_XNEG_ZNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.z);


    static const IMU_Null ZPOS_YNEG_XPOS(IMU_Base::ZPOS_YNEG_XPOS);
    output = ZPOS_YNEG_XPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.z);

    static const IMU_Null YPOS_ZPOS_XPOS(IMU_Base::YPOS_ZPOS_XPOS);
    output = YPOS_ZPOS_XPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.z);

    static const IMU_Null ZNEG_YPOS_XPOS(IMU_Base::ZNEG_YPOS_XPOS);
    output = ZNEG_YPOS_XPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.z);

    static const IMU_Null YNEG_ZNEG_XPOS(IMU_Base::YNEG_ZNEG_XPOS);
    output = YNEG_ZNEG_XPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.z);


    static const IMU_Null ZPOS_YPOS_XNEG(IMU_Base::ZPOS_YPOS_XNEG);
    output = ZPOS_YPOS_XNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.z);

    static const IMU_Null YPOS_ZNEG_XNEG(IMU_Base::YPOS_ZNEG_XNEG);
    output = YPOS_ZNEG_XNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.z);

    static const IMU_Null ZNEG_YNEG_XNEG(IMU_Base::ZNEG_YNEG_XNEG);
    output = ZNEG_YNEG_XNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.z);

    static const IMU_Null YNEG_ZPOS_XNEG(IMU_Base::YNEG_ZPOS_XNEG);
    output = YNEG_ZPOS_XNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.z);


    static const IMU_Null ZPOS_XPOS_YPOS(IMU_Base::ZPOS_XPOS_YPOS);
    output = ZPOS_XPOS_YPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.z);

    static const IMU_Null XNEG_ZPOS_YPOS(IMU_Base::XNEG_ZPOS_YPOS);
    output = XNEG_ZPOS_YPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.z);

    static const IMU_Null ZNEG_XNEG_YPOS(IMU_Base::ZNEG_XNEG_YPOS);
    output = ZNEG_XNEG_YPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.z);

    static const IMU_Null XPOS_ZNEG_YPOS(IMU_Base::XPOS_ZNEG_YPOS);
    output = XPOS_ZNEG_YPOS.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.z);


    static const IMU_Null ZPOS_XNEG_YNEG(IMU_Base::ZPOS_XNEG_YNEG);
    output = ZPOS_XNEG_YNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.z);

    static const IMU_Null XNEG_ZNEG_YNEG(IMU_Base::XNEG_ZNEG_YNEG);
    output = XNEG_ZNEG_YNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.z);

    static const IMU_Null ZNEG_XPOS_YNEG(IMU_Base::ZNEG_XPOS_YNEG);
    output = ZNEG_XPOS_YNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.z);

    static const IMU_Null XPOS_ZPOS_YNEG(IMU_Base::XPOS_ZPOS_YNEG);
    output = XPOS_ZPOS_YNEG.mapAxes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.z);
}

void test_map_axes_inversion()
{
    const xyz_t input { .x =3.0F, .y = 5.0F, .z = 7.0F };

    for (int ii = IMU_Base::XPOS_YPOS_ZPOS; ii <= IMU_Base::YNEG_XPOS_ZPOS_45; ++ii) {
        const auto axisOrder = static_cast<IMU_Base::axis_order_e>(ii);
        const xyz_t intermediate = IMU_Base::mapAxes(input, axisOrder);
        const xyz_t output =       IMU_Base::mapAxes(intermediate, IMU_Base::axisOrderInverse(axisOrder));
        TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
        TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
        TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);
    }

    auto axisOrder = IMU_Base::XPOS_YPOS_ZPOS_45;
    xyz_t intermediate = IMU_Base::mapAxes(input, axisOrder);
    xyz_t output =       IMU_Base::mapAxes(intermediate, IMU_Base::axisOrderInverse(axisOrder));
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    axisOrder = IMU_Base::YPOS_XNEG_ZPOS_45;
    intermediate = IMU_Base::mapAxes(input, axisOrder);
    output =       IMU_Base::mapAxes(intermediate, IMU_Base::axisOrderInverse(axisOrder));
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    axisOrder = IMU_Base::XNEG_YNEG_ZPOS_45;
    intermediate = IMU_Base::mapAxes(input, axisOrder);
    output =       IMU_Base::mapAxes(intermediate, IMU_Base::axisOrderInverse(axisOrder));
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    axisOrder = IMU_Base::YNEG_XPOS_ZPOS_45;
    intermediate = IMU_Base::mapAxes(input, axisOrder);
    output =       IMU_Base::mapAxes(intermediate, IMU_Base::axisOrderInverse(axisOrder));
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);
}

void test_raw()
{
    static IMU_Null imu;

    IMU_Base::xyz_int32_t gyroRaw = imu.readGyroRaw();
    TEST_ASSERT_EQUAL(0, gyroRaw.x);
    TEST_ASSERT_EQUAL(0, gyroRaw.y);
    TEST_ASSERT_EQUAL(0, gyroRaw.z);

    imu.setGyroRaw({3, 5, 7});
    gyroRaw = imu.readGyroRaw();
    TEST_ASSERT_EQUAL(3, gyroRaw.x);
    TEST_ASSERT_EQUAL(5, gyroRaw.y);
    TEST_ASSERT_EQUAL(7, gyroRaw.z);

    IMU_Base::xyz_int32_t accRaw = imu.readAccRaw();
    TEST_ASSERT_EQUAL(0, accRaw.x);
    TEST_ASSERT_EQUAL(0, accRaw.y);
    TEST_ASSERT_EQUAL(0, accRaw.z);

    imu.setAccRaw({11, 13, 17});
    accRaw = imu.readAccRaw();
    TEST_ASSERT_EQUAL(11, accRaw.x);
    TEST_ASSERT_EQUAL(13, accRaw.y);
    TEST_ASSERT_EQUAL(17, accRaw.z);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_map_axes);
    RUN_TEST(test_map_axes_inversion);
    RUN_TEST(test_raw);

    UNITY_END();
}
