#include "Targets.h"
#include <Arduino.h>
#include <IMU_LSM6DS3TR_C.h>
#include <boards/pico.h>


static IMU_Base* imu;
static bool ledOn = false;

#define INTERRUPT_DRIVEN

void setup()
{
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};

    Serial.begin(115200);
    gpio_init(PICO_DEFAULT_LED_PIN); // 25
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, ledOn);

    // statically allocate an LSM6DS3TR_C IMU object
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    static IMU_LSM6DS3TR_C imuStatic(IMU_Base::XPOS_YPOS_ZPOS, SPI_FREQUENCY, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#else
    static IMU_LSM6DS3TR_C imuStatic(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::IMU_I2C_PINS);
#endif

    imu = &imuStatic;

    // initialize the IMU
    imu->init();
#if defined(INTERRUPT_DRIVEN)
    imu->setInterruptDriven();
#endif
}

void loop()
{
    // wait for the IMU data ready interrupt
#if defined(INTERRUPT_DRIVEN)
    imu->WAIT_IMU_DATA_READY();
#else
    imu->readAccGyroRPS();
#endif

    Serial.println();

    // get the gyro data read in the Interrupt Service Routine
    const IMU_Base::accGyroRPS_t accGyroRPS =  imu->getAccGyroRPS();

    // convert the gyro data from radians per second to degrees per second
    const xyz_t gyroDPS =  accGyroRPS.gyroRPS * IMU_Base::radiansToDegrees;
    Serial.println();
    Serial.print("gyroX:");
    Serial.print(gyroDPS.x, 1);
    Serial.print(" gyroY:");
    Serial.print(gyroDPS.y, 1);
    Serial.print(" gyroZ:");
    Serial.println(gyroDPS.z, 1);

    const xyz_t acc =  accGyroRPS.acc;

    Serial.print("accX:");
    Serial.print(acc.x);
    Serial.print(" accY:");
    Serial.print(acc.y);
    Serial.print(" accZ:");
    Serial.println(acc.z);

    delay(500);
    ledOn = !ledOn;
    gpio_put(PICO_DEFAULT_LED_PIN, ledOn);
}
