#include <Arduino.h>
#include <IMU_LSM6DS3TR_C.h>
#include <boards/pico.h>

static constexpr uint8_t IMU_I2C_SDA_PIN=4;
static constexpr uint8_t IMU_I2C_SCL_PIN=5;
static constexpr uint8_t IMU_I2C_IRQ_PIN=6;
static constexpr uint8_t IMU_I2C_IRQ_LEVEL = BUS_I2C::IRQ_LEVEL_HIGH;

static constexpr uint8_t IMU_SPI_CS_PIN=17;
static constexpr uint8_t IMU_SPI_SCK_PIN=18;
static constexpr uint8_t IMU_SPI_CIPO_PIN=16; // RX
static constexpr uint8_t IMU_SPI_COPI_PIN=19; // TX
static constexpr uint8_t IMU_SPI_IRQ_PIN=20;
static constexpr uint8_t IMU_SPI_IRQ_LEVEL = BUS_SPI::IRQ_LEVEL_HIGH;

static IMU_Base* imu;

#define INTERRUPT_DRIVEN

void setup()
{
    Serial.begin(115200);
    gpio_init(PICO_DEFAULT_LED_PIN); // 25
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // statically allocate an LSM6DS3TR_C IMU object
#if defined(USE_IMU_LSM6DS3TR_C_SPI)
    constexpr uint32_t spiFrequency = 20000000; // 20 MHz
    const BUS_SPI::pins_t pins{.cs=IMU_SPI_CS_PIN, .sck=IMU_SPI_SCK_PIN, .cipo=IMU_SPI_CIPO_PIN, .copi=IMU_SPI_COPI_PIN, .irq=IMU_SPI_IRQ_PIN, .irqLevel=IMU_SPI_IRQ_LEVEL};
    static IMU_LSM6DS3TR_C imuStatic(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_LSM6DS3TR_C_I2C)
    const BUS_I2C::pins_t pins{.sda=IMU_I2C_SDA_PIN, .scl=IMU_I2C_SCL_PIN, .irq=IMU_I2C_IRQ_PIN, .irqLevel=IMU_I2C_IRQ_LEVEL};
    static IMU_LSM6DS3TR_C imuStatic(IMU_Base::XPOS_YPOS_ZPOS, pins);
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
