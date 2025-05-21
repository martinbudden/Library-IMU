#include <Arduino.h>
#include <IMU_BNO085.h>

static constexpr uint8_t I2C_SDA_PIN = 8;
static constexpr uint8_t I2C_SCL_PIN = 9;

static IMU_Base* imu;

void setup()
{
    Serial.begin(115200);

    // statically allocate a BNO085 IMU object
    static IMU_BNO085 imuStatic(IMU_Base::XPOS_YPOS_ZPOS, I2C_SDA_PIN, I2C_SCL_PIN);
    imu = &imuStatic;
    // initialize the IMU
    imu->init();
}

void loop()
{
    // read the gyro data and convert from radians per second to degrees per second
    const xyz_t gyroDPS = imu->readGyroRPS() * IMU_Base::radiansToDegrees;
    Serial.print("gyroX:");
    Serial.print(gyroDPS.x);
    Serial.print(" gyroY:");
    Serial.print(gyroDPS.y);
    Serial.print(" gyroZ:");
    Serial.println(gyroDPS.z);

    // read the orientation quaternion
    const Quaternion orientation = imu->readOrientation();
    // calculate the roll and pitch angles from the quaternion
    const float rollAngleDegrees = orientation.calculateRollDegrees();
    const float pitchAngleDegrees = orientation.calculatePitchDegrees();
    const float yawAngleDegrees = orientation.calculateYawDegrees();
    Serial.print("roll:");
    Serial.print(rollAngleDegrees);
    Serial.print(" pitch:");
    Serial.print(pitchAngleDegrees);
    Serial.print(" yaw:");
    Serial.println(yawAngleDegrees);
    Serial.println();

    delay(250);
}
