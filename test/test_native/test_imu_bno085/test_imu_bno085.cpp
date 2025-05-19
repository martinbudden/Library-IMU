#include <IMU_BNO085.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

IMU_BNO085& newBNO085()
{
#if defined(USE_IMU_BNO085_SPI)
    constexpr uint32_t spiFrequency = 2000000;
    constexpr uint8_t CS_pin = 0;
    static IMU_BNO085 imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::SPI_INDEX_0, CS_pin);
#else
    static IMU_BNO085 imu(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::pins_t{.sda=IMU_I2C_SDA_PIN, .scl=IMU_I2C_SCL_PIN, .irq=BUS_I2C::IRQ_NOT_SET, .irqLevel=0});
#endif
    return imu;
}
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_bno085_channel_input_sensor_reports()
{
    IMU_BNO085 imu = newBNO085();
    IMU_BNO085::SHTP_Packet packet;

    packet.header.channel = IMU_BNO085::CHANNEL_INPUT_SENSOR_REPORTS;
    packet.data[9] = 0x03;
    packet.data[10] = 0x05;
    packet.data[11] = 0x38;
    packet.data[12] = 0x75;
    packet.data[13] = 0x61;
    packet.data[14] = 0xD7;
    packet.data[15] = 0x11;
    packet.data[16] = 0xF9;

    packet.data[5] = IMU_BNO085::SENSOR_REPORTID_ACCELEROMETER;
    TEST_ASSERT_EQUAL(IMU_BNO085::SENSOR_REPORTID_ACCELEROMETER, imu.parseInputSensorReport(packet));
    const IMU_BNO085::sensor_output_t acc = imu.getAccData();
    TEST_ASSERT_EQUAL(0x0503, acc.x);
    TEST_ASSERT_EQUAL(0x7538, acc.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), acc.z);

    packet.data[5] = IMU_BNO085::SENSOR_REPORTID_GYROSCOPE_CALIBRATED;
    TEST_ASSERT_EQUAL(IMU_BNO085::SENSOR_REPORTID_GYROSCOPE_CALIBRATED, imu.parseInputSensorReport(packet));
    const IMU_BNO085::sensor_output_t gyroRPS = imu.getGyroRPS_Data();
    TEST_ASSERT_EQUAL(0x0503, gyroRPS.x);
    TEST_ASSERT_EQUAL(0x7538, gyroRPS.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), gyroRPS.z);

    packet.data[5] = IMU_BNO085::SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED;
    TEST_ASSERT_EQUAL(IMU_BNO085::SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED, imu.parseInputSensorReport(packet));
    const IMU_BNO085::sensor_output_t mag = imu.getMagData();
    TEST_ASSERT_EQUAL(0x0503, mag.x);
    TEST_ASSERT_EQUAL(0x7538, mag.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), mag.z);

    packet.data[5] = IMU_BNO085::SENSOR_REPORTID_LINEAR_ACCELERATION;
    TEST_ASSERT_EQUAL(IMU_BNO085::SENSOR_REPORTID_LINEAR_ACCELERATION, imu.parseInputSensorReport(packet));
    const IMU_BNO085::sensor_output_t accLinear = imu.getAccLinearData();
    TEST_ASSERT_EQUAL(0x0503, accLinear.x);
    TEST_ASSERT_EQUAL(0x7538, accLinear.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), accLinear.z);

    packet.data[5] = IMU_BNO085::SENSOR_REPORTID_GRAVITY;
    TEST_ASSERT_EQUAL(IMU_BNO085::SENSOR_REPORTID_GRAVITY, imu.parseInputSensorReport(packet));
    const IMU_BNO085::sensor_output_t gravity = imu.getGravityData();
    TEST_ASSERT_EQUAL(0x0503, gravity.x);
    TEST_ASSERT_EQUAL(0x7538, gravity.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), gravity.z);

    packet.data[5] = IMU_BNO085::SENSOR_REPORTID_GAME_ROTATION_VECTOR;
    TEST_ASSERT_EQUAL(IMU_BNO085::SENSOR_REPORTID_GAME_ROTATION_VECTOR, imu.parseInputSensorReport(packet));
    const IMU_BNO085::rotation_vector_t rotationVector = imu.getRotationVectorData();
    TEST_ASSERT_EQUAL(0x0503, rotationVector.i);
    TEST_ASSERT_EQUAL(0x7538, rotationVector.j);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), rotationVector.k);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xF911), rotationVector.real);
}

void test_bno085_channel_gyro_integrated_rotation_vector_report()
{
    IMU_BNO085 imu = newBNO085();

    IMU_BNO085::SHTP_Packet packet;

    packet.header.channel = IMU_BNO085::CHANNEL_GYRO_INTEGRATED_ROTATION_VECTOR_REPORT;
    packet.data[0] = 0x03;
    packet.data[1] = 0x05;
    packet.data[2] = 0x38;
    packet.data[3] = 0x75;
    packet.data[4] = 0x61;
    packet.data[5] = 0xD7;
    packet.data[6] = 0x11;
    packet.data[7] = 0xF9;
    TEST_ASSERT_EQUAL(IMU_BNO085::SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, imu.parseGyroIntegratedRotationVectorReport(packet));

    const IMU_BNO085::gyro_integrated_rotation_vector_t gyroRotation = imu.getGyroIntegratedRotationVectorData();
    TEST_ASSERT_EQUAL(0x0503, gyroRotation.i);
    TEST_ASSERT_EQUAL(0x7538, gyroRotation.j);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), gyroRotation.k);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xF911), gyroRotation.real);

    const Quaternion orientation = imu.readOrientation();
    constexpr int orientation_Q_point = 14;
    //constexpr int gyro_Q_point = 10;

    // BNO085 uses [real, i, j, k] for quaternion, VectorQuaternionMatrix uses [w, x, y, z]
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(gyroRotation.real) * powf(2, -orientation_Q_point), orientation.getW());
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(gyroRotation.i) * powf(2, -orientation_Q_point), orientation.getX());
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(gyroRotation.j) * powf(2, -orientation_Q_point), orientation.getY());
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(gyroRotation.k) * powf(2, -orientation_Q_point), orientation.getZ());

    constexpr int Q_point = 14;
    const float multiplier = pow(2, Q_point * -1);
    constexpr float multiplier2 = 1.0F / (1 << Q_point); // NOLINT(hicpp-signed-bitwise)
    TEST_ASSERT_EQUAL_FLOAT(multiplier, multiplier2);
}

void test_bno085()
{
#if defined(USE_IMU_BNO085_SPI)
    constexpr uint32_t spiFrequency = 2000000;
    constexpr uint8_t CS_pin = 0;
    static const IMU_BNO085 imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::SPI_INDEX_0, CS_pin);
#else
    static const IMU_BNO085 imu(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::pins_t{.sda=IMU_I2C_SDA_PIN, .scl=IMU_I2C_SCL_PIN, .irq=BUS_I2C::IRQ_NOT_SET, .irqLevel=0});
#endif
    TEST_ASSERT_EQUAL(4096, imu.getAccOneG_Raw());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_bno085_channel_input_sensor_reports);
    RUN_TEST(test_bno085_channel_gyro_integrated_rotation_vector_report);
    RUN_TEST(test_bno085);

    UNITY_END();
}
