#include <Arduino.h>
#include <IMU_BMI270.h>
#include <M5Unified.h>



#define IMU_I2C_PINS i2c_pins_t{.sda=45,.scl=0,.irq=16}

//static constexpr uint8_t I2C_SDA_PIN = 45;
//static constexpr uint8_t I2C_SCL_PIN = 0;
//static constexpr uint8_t I2C_IRQ_PIN = 16; // pin is pulled high
//enum irq_level_e { IRQ_LEVEL_LOW = 0x04, IRQ_LEVEL_HIGH = 0x05, IRQ_EDGE_FALL = 0x02, IRQ_EDGE_RISE = 0x01, IRQ_EDGE_CHANGE = 0x03 };
//static constexpr uint8_t I2C_IRQ_LEVEL = IRQ_LEVEL_HIGH;


static IMU_Base* imu;

//#define INTERRUPT_DRIVEN

void setup()
{
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};

    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();
    Serial.begin(115200);

    delay(1000);

    // statically allocate a BMI270 IMU object
    static IMU_BMI270 imuStatic(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::IMU_I2C_PINS);

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
