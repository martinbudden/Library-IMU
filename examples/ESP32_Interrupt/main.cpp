#include "AHRS.h"
#include <Arduino.h>
#include <IMU_BMI270.h>
#include <M5Unified.h>


static constexpr uint8_t I2C_SDA_PIN = 45;
static constexpr uint8_t I2C_SCL_PIN = 0;
static constexpr uint8_t I2C_IRQ_PIN = 16; // pin is pulled high

static IMU_Base* imu;
static AHRS* ahrs;

void setup()
{
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();
    Serial.begin(115200);

    // statically allocate a BMI270 IMU object
    static IMU_BMI270 imuStatic(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::pins_t{.sda=I2C_SDA_PIN, .scl=I2C_SCL_PIN, .irq=I2C_IRQ_PIN, .irqLevel=BUS_I2C::IRQ_LEVEL_HIGH});

    imu = &imuStatic;

    // initialize the IMU
    imu->init();

    // statically create an AHRS object to handle the data synchronization with the IMU
    static AHRS ahrsStatic(imuStatic);
    ahrs = &ahrsStatic;

    Serial.println("\r\n****Ready****\r\n");
}

void loop()
{
    // interrupt not working, so test UNLOCK directly
    static_cast<IMU_BMI270*>(imu)->UNLOCK_IMU_DATA_READY();
    imu->readGyroRPS_Acc();

    // wait for the IMU data ready interrupt
    ahrs->LOCK_IMU_DATA_READY();

    // get the gyro data read in the Interrupt Service Routine
    const IMU_Base::gyroRPS_Acc_t gyroRPS_Acc = imu->getGyroRPS_Acc();

    // convert the gyro data from radians per second to degrees per second
    const xyz_t gyroDPS = gyroRPS_Acc.gyroRPS * IMU_Base::radiansToDegrees;
    Serial.println();
    Serial.print("gyroX:");
    Serial.print(gyroDPS.x, 1);
    Serial.print(" gyroY:");
    Serial.print(gyroDPS.y, 1);
    Serial.print(" gyroZ:");
    Serial.println(gyroDPS.z, 1);

    // take an accelerometer reading
    const xyz_t acc = gyroRPS_Acc.acc;

    Serial.print("accX:");
    Serial.print(acc.x);
    Serial.print(" accY:");
    Serial.print(acc.y);
    Serial.print(" accZ:");
    Serial.println(acc.z);

    delay(500);
}
