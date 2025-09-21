#pragma once

/*!
Targets
*/


#if defined(TARGET_ADAFRUIT_FEATHER_F405)
    #define BOARD_IDENTIFIER    "Feather_F405"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define IMU_CLASS IMU_LSM6DS3TR_C
    //#define IMU_CLASS IMU_ICM426xx
    //#define IMU_CLASS IMU_MPU6000
    enum { SPI_FREQUENCY = 20000000 }; // 20 MHz
// SPI1 NSS-PA4-A0 SCK-PA5-A1 MISO-PA6-A2 MOSI-PA7-A3 EXTI4-PC4-A4
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    #define IMU_SPI_INDEX       BUS_INDEX_0
    //                                      A0         A1          A2           A3           A4
    #define IMU_SPI_PINS        port_pins_t{.cs={PA,4},.sck={PA,5},.cipo={PA,6},.copi={PA,7},.irq={PC,4}}
#else
    // function-pin-marking on board
    // I2C1 SDA-PB7-SCL SCL-PB6-SDA
    #define IMU_I2C_INDEX       BUS_INDEX_0
    //                                      SDA         SCL 
    #define IMU_I2C_PINS        port_pins_t{.sda={PB,7},.scl={PB,6},.irq={0,0xFF}}
#endif

    #define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX 2
    // UART3 RX-PB11-RX_D0 TX TX-PB10-TX_D1
    //                                    D0               D1
    #define RECEIVER_PINS     port_pins_t{.rx={PB,11},.tx={PB,10}}

// PB13-SCK
// PB14-MISO
// PB15-MOSI
// PC6-D6
// PC7-D5
// PB8-D9
// PB9-D10
// PC3-D11
// PC2-D12
// PC1-D13

// PB
    #define USE_MOTOR_MIXER_QUAD_X_PWM
    //                                      D5         D6         D9         D10
    #define MOTOR_PINS          port_pins_t{.br={PC,7},.fr={PC,6},.bl={PB,8},.fl={PB,9}}

    #define GPIO_LED_0          port_pin_t{PC,1} 
#endif


#if defined(TARGET_AFROFLIGHT_F301CB)
    #define BOARD_IDENTIFIER    "AfroFlight_F301CB"

    #define IMU_CLASS IMU_MPU6000
    //#define IMU_CLASS IMU_LSM6DS3TR_C

    #define USART_1_PINS        port_pins_t{.rx={PA,10},.tx={PA,9}} // TX output is always inverted (for FrSky). Internally connected to USB port via CP2102 IC
    #define USART_2_PINS        port_pins_t{.rx={PA,3},.tx={PA,2}}
    #define SOFT_SERIAL_1_PINS  port_pins_t{.rx={PA,6},.tx={PA,7}}
    #define SOFT_SERIAL_2_PINS  port_pins_t{.rx={PB,0},.tx={PB,1}}
    #define I2C_1_PINS          port_pins_t{.sda={PB,7},.scl={PB,6}}

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    //!!TODO: add port_pins to IMU_MPU6000
    #define IMU_I2C_PINS        port_pins_t{.sda={PB,11},.scl={PB,10},.irq={PB,13}}
    // On afroflight Rev 5 MPU6050 is connected to IC2 index 2
    #define IMU_SPI_PINS        port_pins_t{.cs={PA,4},.sck={PA,5},.cipo={PA,6},.copi={PA,7},.irq={PA,0xFF}}

    #define USE_RECEIVER_SBUS
    #define RECEIVER_UART_INDEX 0
    #define RECEIVER_PINS       port_pins_t{.rx={PB,10},.tx={PB,9}}

    #define USE_MOTOR_MIXER_QUAD_X_PWM
    #define MOTOR_PINS          port_pins_t{.br={PA,8},.fr={PA,11},.bl={PB,6},.fl={PB,7}}

    #define GPIO_LED_0          port_pin_t{PB,4} 
    #define GPIO_LED_1          port_pin_t{PB,3} 

    #define GPIO_RC_CH1         port_pin_t{PA,0} // T2C1
    #define GPIO_RC_CH2         port_pin_t{PA,1} // T2C2
    #define GPIO_RC_CH3         port_pin_t{PA,2} // T2C3/UA2_TX
    #define GPIO_RC_CH4         port_pin_t{PA,3} // T2C4/UA2_RX
    #define GPIO_RC_CH5         port_pin_t{PA,6} // T3C1
    #define GPIO_RC_CH6         port_pin_t{PA,7} // T3C2
    #define GPIO_RC_CH7         port_pin_t{PB,0} // T3C3
    #define GPIO_RC_CH8         port_pin_t{PB,1} // T3C4

    #define GPIO_PWM1           port_pin_t{PA,8} // T1C1
    #define GPIO_PMW2           port_pin_t{PA,11} // T1C4
    #define GPIO_PMW3           port_pin_t{PB,6} // T4C1
    #define GPIO_PMW4           port_pin_t{PB,7} // T4C2
    #define GPIO_PMW5           port_pin_t{PB,8} // T4C3
    #define GPIO_PMW6           port_pin_t{PB,9} // T4C4
    #define GPIO_SONAR_INT      port_pin_t{PA,15}
    #define GPIO_GPIO_BOTTOM    port_pin_t{PB,5}
    #define GPIO_TELEM_OUT      port_pin_t{PA,13} // Warning, SWD access is lost when using this pin, bootloader via uart is required after

    #define GPIO_BAT_ADC        port_pin_t{PA,4} // Connected to 6 pin header Battery voltage in via resistor divider
    #define GPIO_ACC_INT        port_pin_t{PA,5} // Connected to Interrupt pin of MMA84520 accelerometer I2C

    #define GPIO_MAG_DRD        port_pin_t{PB,12} //Connected to HMC5883L compass I2C
    #define GPIO_BEEP           port_pin_t{PA,12} //Connected to Beep out transistor on 6 pin header

#endif
