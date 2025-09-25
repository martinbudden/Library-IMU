#pragma once

/*!
Targets
*/


#if defined(TARGET_RPI_PICO_SPI)
    #define BOARD_IDENTIFIER    "RPI_Pico_SPI"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
    #define IMU_SPI_INDEX       BUS_INDEX_0
    #define SPI_FREQUENCY       20000000 // 20 MHz
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    #define IMU_SPI_PINS        spi_pins_t{.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}
#else
    //#define IMU_SPI_PINS        spi_pins_t{.cs=13,.sck=14,.cipo=12,.copi=15,.irq=20}
    #define IMU_I2C_PINS        i2c_pins_t{.sda=4,.scl=5,.irq=6}
#endif
#endif

#if defined(TARGET_RPI_PICO_I2C)
    #define BOARD_IDENTIFIER    "RPI_Pico_I2C"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
    #define IMU_SPI_INDEX       BUS_INDEX_0
    #define SPI_FREQUENCY       20000000 // 20 MHz
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    #define IMU_SPI_PINS        spi_pins_t{.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}
    //#define IMU_SPI_PINS        spi_pins_t{.cs=13,.sck=14,.cipo=12,.copi=15,.irq=20}
#else
    #define IMU_I2C_PINS        i2c_pins_t{.sda=07,.scl=27,.irq=BUS_I2C::IRQ_NOT_SET}
    //#define IMU_I2C_PINS        i2c_pins_t{.sda=4,.scl=5,.irq=6}
#endif
#endif
