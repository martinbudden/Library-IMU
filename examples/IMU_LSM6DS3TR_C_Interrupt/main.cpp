#include "AHRS.h"
#include <Arduino.h>
#include <IMU_LSM6DS3TR_C.h>
#if defined(FRAMEWORK_RPI_PICO)
#include <boards/pico.h>
#endif

static constexpr uint8_t IMU_I2C_SDA_PIN=4;
static constexpr uint8_t IMU_I2C_SCL_PIN=5;
static constexpr uint8_t IMU_I2C_IRQ_PIN=3;

static constexpr uint8_t IMU_SPI_CS_PIN=17;
static constexpr uint8_t IMU_SPI_SCK_PIN=18;
static constexpr uint8_t IMU_SPI_CIPO_PIN=16; // RX
static constexpr uint8_t IMU_SPI_COPI_PIN=19; // TX
static constexpr uint8_t IMU_SPI_IRQ_PIN=20;
static constexpr uint8_t IMU_SPI_IRQ_LEVEL = BUS_SPI::IRQ_LEVEL_HIGH;
static IMU_Base* imuSensor;
static AHRS* ahrs;

void setup()
{
    Serial.begin(115200);
#if defined(FRAMEWORK_RPI_PICO)
    gpio_init(PICO_DEFAULT_LED_PIN); // 25
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

    // create an LSM6DS3TR_C sensor object
#if defined(USE_IMU_LSM6DS3TR_C_SPI)
    constexpr uint32_t spiFrequency = 20000000; // 20 MHz
    const BUS_SPI::pins_t pins{.cs=IMU_SPI_CS_PIN, .sck=IMU_SPI_SCK_PIN, .cipo=IMU_SPI_CIPO_PIN, .copi=IMU_SPI_COPI_PIN, .irq=IMU_SPI_IRQ_PIN, .irqLevel=IMU_SPI_IRQ_LEVEL};
    static IMU_LSM6DS3TR_C imuSensorStatic(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_LSM6DS3TR_C_I2C)
    const BUS_I2C::pins_t pins{.sda=IMU_I2C_SDA_PIN, .scl=IMU_I2C_SCL_PIN, .irq=IMU_I2C_IRQ_PIN, .irqLevel=BUS_SPI::IRQ_LEVEL_HIGH});
    static IMU_LSM6DS3TR_C imuSensorStatic(IMU_Base::XPOS_YPOS_ZPOS, pins);
#endif

    imuSensor = &imuSensorStatic;

    // initialize the sensor object
    imuSensor->init();

    static AHRS ahrsStatic(imuSensorStatic);
    ahrs = &ahrsStatic;
}

void loop()
{
    // wait for the IMU data ready interrupt
    ahrs->LOCK_IMU_DATA_READY();
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    Serial.println();
    Serial.print("dataReadyCount:");
    Serial.print(ahrs->getImuDataReadyCount());

    // get the gyro data read in the Interrupt Service Routine
    const IMU_Base::gyroRPS_Acc_t gyroRPS_Acc =  imuSensor->getGyroRPS_Acc();

    const xyz_t gyroDPS =  gyroRPS_Acc.gyroRPS * IMU_Base::radiansToDegrees;
    Serial.println();
    Serial.print("gyroX:");
    Serial.print(gyroDPS.x, 1);
    Serial.print(" gyroY:");
    Serial.print(gyroDPS.y, 1);
    Serial.print(" gyroZ:");
    Serial.println(gyroDPS.z, 1);

    // take an accelerometer reading
    const xyz_t acc =  gyroRPS_Acc.acc;

    Serial.print("accX:");
    Serial.print(acc.x);
    Serial.print(" accY:");
    Serial.print(acc.y);
    Serial.print(" accZ:");
    Serial.println(acc.z);

    delay(500);
}
