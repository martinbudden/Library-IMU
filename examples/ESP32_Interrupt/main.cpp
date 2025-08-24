#include <Arduino.h>
#include <IMU_BMI270.h>
#include <M5Unified.h>


static constexpr uint8_t I2C_SDA_PIN = 45;
static constexpr uint8_t I2C_SCL_PIN = 0;
static constexpr uint8_t I2C_IRQ_PIN = 16; // pin is pulled high
static constexpr uint8_t I2C_IRQ_LEVEL = BUS_I2C::IRQ_LEVEL_HIGH;

static IMU_Base* imu;

//#define INTERRUPT_DRIVEN

void setup()
{
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();
    Serial.begin(115200);

    delay(1000);

    // statically allocate a BMI270 IMU object
    static IMU_BMI270 imuStatic(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::pins_t{.sda=I2C_SDA_PIN, .scl=I2C_SCL_PIN, .irq=I2C_IRQ_PIN});

    imu = &imuStatic;

    // initialize the IMU
    imu->init();

#if defined(INTERRUPT_DRIVEN)
    imu->setInterruptDriven();
#endif

    Serial.println("\r\n****Ready****\r\n");

}

void loop()
{
#if defined(INTERRUPT_DRIVEN)
    imu->WAIT_IMU_DATA_READY();
#else
    imu->SIGNAL_IMU_DATA_READY_FROM_ISR();
    imu->WAIT_IMU_DATA_READY();
    imu->readAccGyroRPS();
#endif

    // get the gyro data read in the ISR
    const IMU_Base::accGyroRPS_t accGyroRPS = imu->getAccGyroRPS();

    // convert the gyro data from radians per second to degrees per second
    const xyz_t gyroDPS = accGyroRPS.gyroRPS * IMU_Base::radiansToDegrees;
    Serial.println();
    Serial.print("gyroX:");
    Serial.print(gyroDPS.x, 1);
    Serial.print(" gyroY:");
    Serial.print(gyroDPS.y, 1);
    Serial.print(" gyroZ:");
    Serial.println(gyroDPS.z, 1);

    const xyz_t acc = accGyroRPS.acc;

    Serial.print("accX:");
    Serial.print(acc.x);
    Serial.print(" accY:");
    Serial.print(acc.y);
    Serial.print(" accZ:");
    Serial.println(acc.z);

    delay(500);
}
