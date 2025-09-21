#include "Targets.h"
#include <Arduino.h>
#include <IMU_LSM6DS3TR_C.h>
#include <IMU_MPU6000.h>

static IMU_Base* imu;
static bool ledOn = false;
//#define INTERRUPT_DRIVEN


enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};
struct port_pin_t {
    uint8_t port;
    uint8_t pin;
};
static inline GPIO_TypeDef* gpioPort(port_pin_t portPin) { return reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + portPin.port*(GPIOB_BASE - GPIOA_BASE)); }
static inline uint16_t gpioPin(port_pin_t portPin) { return static_cast<uint16_t>(1U << portPin.pin); }


void setup()
{
    Serial.begin(115200);

    // Configure LED pin
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = gpioPin(GPIO_LED_0),
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
#if !defined(FRAMEWORK_STM32_CUBE_F1)
        .Alternate = 0
#endif
    };
    HAL_GPIO_Init(gpioPort(GPIO_LED_0), &GPIO_InitStruct);
    HAL_GPIO_WritePin(gpioPort(GPIO_LED_0), gpioPin(GPIO_LED_0), GPIO_PIN_RESET);

    // statically allocate the IMU object
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    static IMU_CLASS imuStatic(IMU_Base::XPOS_YPOS_ZPOS, SPI_FREQUENCY, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#else
    static IMU_CLASS imuStatic(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::IMU_I2C_PINS);
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
    HAL_GPIO_WritePin(gpioPort(GPIO_LED_0), gpioPin(GPIO_LED_0), ledOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
