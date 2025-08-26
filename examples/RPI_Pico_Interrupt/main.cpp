#include <Arduino.h>
#include <IMU_LSM6DS3TR_C.h>
#include <boards/pico.h>

#define IMU_I2C_PINS pins_t{.sda=4,.scl=5,.irq=6}
#define IMU_SPI_INDEX BUS_INDEX_0
#define IMU_SPI_PINS pins_t{.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}

/*
static constexpr uint8_t IMU_I2C_SDA_PIN=4;
static constexpr uint8_t IMU_I2C_SCL_PIN=5;
static constexpr uint8_t IMU_I2C_IRQ_PIN=6;

static constexpr uint8_t IMU_SPI_CS_PIN=17;
static constexpr uint8_t IMU_SPI_SCK_PIN=18;
static constexpr uint8_t IMU_SPI_CIPO_PIN=16; // RX
static constexpr uint8_t IMU_SPI_COPI_PIN=19; // TX
static constexpr uint8_t IMU_SPI_IRQ_PIN=20;
*/

static IMU_Base* imu;

#define INTERRUPT_DRIVEN

void setup()
{
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};

    Serial.begin(115200);
    gpio_init(PICO_DEFAULT_LED_PIN); // 25
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // statically allocate an LSM6DS3TR_C IMU object
#if defined(USE_IMU_LSM6DS3TR_C_SPI)
    constexpr uint32_t spiFrequency = 20000000; // 20 MHz
    static IMU_LSM6DS3TR_C imuStatic(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_LSM6DS3TR_C_I2C)
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

    //gpio_put(PICO_DEFAULT_LED_PIN, 0);
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
}
