#include <Arduino.h>
#include <IMU_LSM6DS3TR_C.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

static IMU_Base* imu;
int ret;

void setup()
{
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};
    enum {P0=0, P1=1, P2=2, P3=3, P4=4, P5=5, P6=6, P7=7};

#if defined(M5_UNIFIED)
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();
#endif
    Serial.begin(115200);

    // create an LSM6DS3TR_C IMU object
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 20000000; // 20 MHz
    static IMU_LSM6DS3TR_C imuStatic(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#else
#if defined(LIBRARY_IMU_USE_I2C_WIRE_1)
    static IMU_LSM6DS3TR_C imuStatic(IMU_Base::XPOS_YPOS_ZPOS, Wire1, BUS_I2C::port_pins_t{.sda=IMU_I2C_SDA_PIN, .scl=IMU_I2C_SCL_PIN, .irq=BUS_SPI::IRQ_NOT_SET}, IMU_LSM6DS3TR_C::I2C_ADDRESS);
#else
    static IMU_LSM6DS3TR_C imuStatic(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::port_pins_t{.sda=IMU_I2C_SDA_PIN, .scl=IMU_I2C_SCL_PIN, .irq=BUS_I2C::IRQ_NOT_SET});
#endif
#endif
    imu = &imuStatic;

    delay(1000);
    Serial.println("\r\n****Ready****\r\n");

    // initialize the IMU
    ret = imu->init();
    Serial.print("imuInit:");
    Serial.println(ret);
}

void loop()
{
    // take an IMU reading
    const IMU_Base::accGyroRPS_t accGyroRPS = imu->readAccGyroRPS();

    // convert the gyro radians per second value to degrees per second
    const xyz_t gyroDPS = accGyroRPS.gyroRPS * IMU_Base::radiansToDegrees;

    Serial.print("imuInit:");
    Serial.println(ret);
    Serial.println();
    Serial.print("gyroX:");
    Serial.print(gyroDPS.x, 1);
    Serial.print(" gyroY:");
    Serial.print(gyroDPS.y, 1);
    Serial.print(" gyroZ:");
    Serial.println(gyroDPS.z, 1);

    // get the acc part of the accGyro reading
    const xyz_t acc =  accGyroRPS.acc;

    Serial.print("accX:");
    Serial.print(acc.x);
    Serial.print(" accY:");
    Serial.print(acc.y);
    Serial.print(" accZ:");
    Serial.println(acc.z);

    delay(500);
}
