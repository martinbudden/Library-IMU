#include <Arduino.h>
#include <IMU_BMI270.h>
#include <M5Unified.h>


static constexpr uint8_t I2C_SDA_PIN = 45;
static constexpr uint8_t I2C_SCL_PIN = 0;

static IMU_Base* imuSensor;

void setup()
{
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();

    Serial.begin(115200);

    // create a BMI270 sensor object
    static IMU_BMI270 imuSensorStatic(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::pins_t{.sda=I2C_SDA_PIN, .scl=I2C_SCL_PIN});
    imuSensor = &imuSensorStatic;
    // initialize the sensor object
    imuSensor->init();
}

void loop()
{
    // take a gyro reading
    const xyz_t gyroDPS =  imuSensor->readGyroDPS();

    Serial.println();
    Serial.print("gyroX:");
    Serial.print(gyroDPS.x, 1);
    Serial.print(" gyroY:");
    Serial.print(gyroDPS.y, 1);
    Serial.print(" gyroZ:");
    Serial.println(gyroDPS.z, 1);

    // take an accelerometer reading
    const xyz_t acc =  imuSensor->readAcc();

    Serial.print("accX:");
    Serial.print(acc.x);
    Serial.print(" accY:");
    Serial.print(acc.y);
    Serial.print(" accZ:");
    Serial.println(acc.z);

    delay(500);
}
