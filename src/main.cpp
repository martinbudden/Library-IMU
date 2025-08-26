#if defined (USE_EMPTY_SETUP_LOOP)

#include <Arduino.h>

#include "IMU_LSM6DS3TR_C.h"

//#define IMU_SPI_PINS BUS_SPI::pins_t{.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}
//#define IMU_SPI_PORT_PINS BUS_SPI::port_pins_t{.cs={PB,11},.sck={PB,14},.cipo={PB,14},.copi={PB,15},.irq={PB,10}}

void setup()
{
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};

    //const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    //const BUS_SPI::port_pins_t port_pins = IMU_SPI_PORT_PINS;

    static constexpr uint32_t spiFrequency = 20000000;
    const IMU_LSM6DS3TR_C imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
    //const IMU_LSM6DS3TR_C imu2(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::BUS_INDEX_0, IMU_SPI_PORT_PINS);
    (void)imu;
}

void loop()
{
}
#endif