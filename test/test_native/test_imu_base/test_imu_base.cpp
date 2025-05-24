#include <IMU_Base.h>
#include <unity.h>

class IMU_Test : public IMU_Base {
public:
// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
    virtual ~IMU_Test() override = default;
    IMU_Test(const IMU_Test&) = delete;
    IMU_Test& operator=(const IMU_Test&) = delete;
    IMU_Test(IMU_Test&&) = delete;
    IMU_Test& operator=(IMU_Test&&) = delete;
    IMU_Test() = default;
    explicit IMU_Test(axis_order_e axisOrder) : IMU_Base(axisOrder) {}
    virtual int init(uint32_t outputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) override;
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
};

int IMU_Test::init(uint32_t outputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex)
{
    (void)outputDataRateHz;
    (void)gyroSensitivity;
    (void)accSensitivity;
    (void)i2cMutex;

    return 0;
}

IMU_Base::xyz_int32_t IMU_Test::readGyroRaw()
{
    return xyz_int32_t {};
}

IMU_Base::xyz_int32_t IMU_Test::readAccRaw()
{
    return xyz_int32_t {};
}


void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_map_axes()
{
    const xyz_t input { .x =3, .y = 5, .z = 7 };
    xyz_t output {};

    static const IMU_Test XPOS_YPOS_ZPOS(IMU_Base::XPOS_YPOS_ZPOS);
    output = XPOS_YPOS_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(input.x, output.x);
    TEST_ASSERT_EQUAL(input.y, output.y);
    TEST_ASSERT_EQUAL(input.z, output.z);

    static const IMU_Test YPOS_XNEG_ZPOS(IMU_Base::YPOS_XNEG_ZPOS);
    output = YPOS_XNEG_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(input.y, output.x);
    TEST_ASSERT_EQUAL(-input.x, output.y);
    TEST_ASSERT_EQUAL(input.z, output.z);

    static const IMU_Test XNEG_YNEG_ZPOS(IMU_Base::XNEG_YNEG_ZPOS);
    output = XNEG_YNEG_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.x, output.x);
    TEST_ASSERT_EQUAL(-input.y, output.y);
    TEST_ASSERT_EQUAL(input.z, output.z);

    static const IMU_Test YNEG_XPOS_ZPOS(IMU_Base::YNEG_XPOS_ZPOS);
    output = YNEG_XPOS_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.y, output.x);
    TEST_ASSERT_EQUAL(input.x, output.y);
    TEST_ASSERT_EQUAL(input.z, output.z);


    static const IMU_Test XPOS_YNEG_ZNEG(IMU_Base::XPOS_YNEG_ZNEG);
    output = XPOS_YNEG_ZNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(input.x, output.x);
    TEST_ASSERT_EQUAL(-input.y, output.y);
    TEST_ASSERT_EQUAL(-input.z, output.z);

    static const IMU_Test YPOS_XPOS_ZNEG(IMU_Base::YPOS_XPOS_ZNEG);
    output = YPOS_XPOS_ZNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(input.y, output.x);
    TEST_ASSERT_EQUAL(input.x, output.y);
    TEST_ASSERT_EQUAL(-input.z, output.z);

    static const IMU_Test XNEG_YPOS_ZNEG(IMU_Base::XNEG_YPOS_ZNEG);
    output = XNEG_YPOS_ZNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.x, output.x);
    TEST_ASSERT_EQUAL(input.y, output.y);
    TEST_ASSERT_EQUAL(-input.z, output.z);

    static const IMU_Test YNEG_XNEG_ZNEG(IMU_Base::YNEG_XNEG_ZNEG);
    output = YNEG_XNEG_ZNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.y, output.x);
    TEST_ASSERT_EQUAL(-input.x, output.y);
    TEST_ASSERT_EQUAL(-input.z, output.z);


    static const IMU_Test ZPOS_YNEG_XPOS(IMU_Base::ZPOS_YNEG_XPOS);
    output = ZPOS_YNEG_XPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(input.z, output.x);
    TEST_ASSERT_EQUAL(-input.y, output.y);
    TEST_ASSERT_EQUAL(input.x, output.z);

    static const IMU_Test YPOS_ZPOS_XPOS(IMU_Base::YPOS_ZPOS_XPOS);
    output = YPOS_ZPOS_XPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(input.y, output.x);
    TEST_ASSERT_EQUAL(input.z, output.y);
    TEST_ASSERT_EQUAL(input.x, output.z);

    static const IMU_Test ZNEG_YPOS_XPOS(IMU_Base::ZNEG_YPOS_XPOS);
    output = ZNEG_YPOS_XPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.z, output.x);
    TEST_ASSERT_EQUAL(input.y, output.y);
    TEST_ASSERT_EQUAL(input.x, output.z);

    static const IMU_Test YNEG_ZNEG_XPOS(IMU_Base::YNEG_ZNEG_XPOS);
    output = YNEG_ZNEG_XPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.y, output.x);
    TEST_ASSERT_EQUAL(-input.z, output.y);
    TEST_ASSERT_EQUAL(input.x, output.z);


    static const IMU_Test ZPOS_YPOS_XNEG(IMU_Base::ZPOS_YPOS_XNEG);
    output = ZPOS_YPOS_XNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(input.z, output.x);
    TEST_ASSERT_EQUAL(input.y, output.y);
    TEST_ASSERT_EQUAL(-input.x, output.z);

    static const IMU_Test YPOS_ZNEG_XNEG(IMU_Base::YPOS_ZNEG_XNEG);
    output = YPOS_ZNEG_XNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(input.y, output.x);
    TEST_ASSERT_EQUAL(-input.z, output.y);
    TEST_ASSERT_EQUAL(-input.x, output.z);

    static const IMU_Test ZNEG_YNEG_XNEG(IMU_Base::ZNEG_YNEG_XNEG);
    output = ZNEG_YNEG_XNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.z, output.x);
    TEST_ASSERT_EQUAL(-input.y, output.y);
    TEST_ASSERT_EQUAL(-input.x, output.z);

    static const IMU_Test YNEG_ZPOS_XNEG(IMU_Base::YNEG_ZPOS_XNEG);
    output = YNEG_ZPOS_XNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.y, output.x);
    TEST_ASSERT_EQUAL(input.z, output.y);
    TEST_ASSERT_EQUAL(-input.x, output.z);


    static const IMU_Test ZPOS_XPOS_YPOS(IMU_Base::ZPOS_XPOS_YPOS);
    output = ZPOS_XPOS_YPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(input.z, output.x);
    TEST_ASSERT_EQUAL(input.x, output.y);
    TEST_ASSERT_EQUAL(input.y, output.z);

    static const IMU_Test XNEG_ZPOS_YPOS(IMU_Base::XNEG_ZPOS_YPOS);
    output = XNEG_ZPOS_YPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.x, output.x);
    TEST_ASSERT_EQUAL(input.z, output.y);
    TEST_ASSERT_EQUAL(input.y, output.z);

    static const IMU_Test ZNEG_XNEG_YPOS(IMU_Base::ZNEG_XNEG_YPOS);
    output = ZNEG_XNEG_YPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.z, output.x);
    TEST_ASSERT_EQUAL(-input.x, output.y);
    TEST_ASSERT_EQUAL(input.y, output.z);

    static const IMU_Test XPOS_ZNEG_YPOS(IMU_Base::XPOS_ZNEG_YPOS);
    output = XPOS_ZNEG_YPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(input.x, output.x);
    TEST_ASSERT_EQUAL(-input.z, output.y);
    TEST_ASSERT_EQUAL(input.y, output.z);


    static const IMU_Test ZPOS_XNEG_YNEG(IMU_Base::ZPOS_XNEG_YNEG);
    output = ZPOS_XNEG_YNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(input.z, output.x);
    TEST_ASSERT_EQUAL(-input.x, output.y);
    TEST_ASSERT_EQUAL(-input.y, output.z);

    static const IMU_Test XNEG_ZNEG_YNEG(IMU_Base::XNEG_ZNEG_YNEG);
    output = XNEG_ZNEG_YNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.x, output.x);
    TEST_ASSERT_EQUAL(-input.z, output.y);
    TEST_ASSERT_EQUAL(-input.y, output.z);

    static const IMU_Test ZNEG_XPOS_YNEG(IMU_Base::ZNEG_XPOS_YNEG);
    output = ZNEG_XPOS_YNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(-input.z, output.x);
    TEST_ASSERT_EQUAL(input.x, output.y);
    TEST_ASSERT_EQUAL(-input.y, output.z);

    static const IMU_Test XPOS_ZPOS_YNEG(IMU_Base::XPOS_ZPOS_YNEG);
    output = XPOS_ZPOS_YNEG.mapAxes(input);
    TEST_ASSERT_EQUAL(input.x, output.x);
    TEST_ASSERT_EQUAL(input.z, output.y);
    TEST_ASSERT_EQUAL(-input.y, output.z);
}

void test_map_axes_inversion()
{
    const xyz_t input { .x =3, .y = 5, .z = 7 };

    for (int ii = IMU_Base::XPOS_YPOS_ZPOS; ii <= IMU_Base::XPOS_ZPOS_YNEG; ++ii) {
        const auto axisOrder = static_cast<IMU_Base::axis_order_e>(ii);
        const xyz_t intermediate = IMU_Base::mapAxes(input, axisOrder);
        const xyz_t output =       IMU_Base::mapAxes(intermediate, IMU_Base::axisOrderInverse(axisOrder));
        TEST_ASSERT_EQUAL(input.x, output.x);
        TEST_ASSERT_EQUAL(input.y, output.y);
        TEST_ASSERT_EQUAL(input.z, output.z);
    }
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_map_axes);
    RUN_TEST(test_map_axes_inversion);

    UNITY_END();
}
