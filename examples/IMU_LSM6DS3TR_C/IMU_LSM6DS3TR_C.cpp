#include <Arduino.h>
#include <IMU_LSM6DS3TR_C.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
static IMU_Base* imuSensor;

void setup()
{
#if defined(M5_UNIFIED)
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();
#endif
    Serial.begin(115200);

    // create an LSM6DS3TR_C sensor object
#if defined(USE_IMU_LSM6DS3TR_C_SPI)
    constexpr uint32_t spiFrequency = 20000000;
    static IMU_LSM6DS3TR_C imuSensorStatic(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::SPI_INDEX_0, {IMU_SPI_CS_PIN, IMU_SPI_SCK_PIN, IMU_SPI_CIPO_PIN, IMU_SPI_COPI_PIN});
#else
#if defined(USE_I2C_WIRE_1)
    static IMU_LSM6DS3TR_C imuSensorStatic(IMU_Base::XPOS_YPOS_ZPOS, Wire1, IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN, IMU_LSM6DS3TR_C::I2C_ADDRESS);
#else
    static IMU_LSM6DS3TR_C imuSensorStatic(IMU_Base::XPOS_YPOS_ZPOS, IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN);
#endif
#endif
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
