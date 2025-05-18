# IMU Library ![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

This library implements drivers for  IMUs (Inertial Management Units), that is combined gyroscopes and accelerometers.

The following IMUs are currently implemented:

| IMU                                                                                  | ID          | SPI Build Flag            |
| -------------------------------------------------------------------------------------| ----------- | ------------------------- |
| [Bosch BMI270](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/) | BMI270      | `USE_IMU_BMI270_SPI`      |
| [CEVA BNO085](https://www.ceva-ip.com/product/bno-9-axis-imu/)                       | BNO085      | `USE_IMU_BNO085_SPI`      |
| [ST LSM6DS3TR-C](https://www.st.com/en/mems-and-sensors/lsm6ds3tr-c.html)            | LSM6DS3TR_C | `USE_IMU_LSM6DS3TR_C_SPI` |
| [ST ISM330DHCX](https://www.st.com/en/mems-and-sensors/ism330dhcx.html)              | ISM330DHCX  | `USE_IMU_ISM330DHCX_SPI`  |
| [ST LSM6DSOX](https://www.st.com/en/mems-and-sensors/lsm6dsox.html)                  | LSM6DSOX    | `USE_LSM6DSOX_SPI`        |
| InvenSense MPU-6886                                                                  | MPU6886     | `USE_IMU_MPU6886_SPI`     |

The LSM6DS3TR-C, ISM330DHCX, and LSM6DSOX are broadly compatible and share the same driver.

## SPI Build Flags

By default the drivers are configured to us I2C. To use SPI, define the relevant SPI build flag for the IMU. This can be done in the `build_flags` section of `platformio.ini`.

The SPI versions of the drivers are not yet working and are still under development.

## Frameworks

Currently only the Arduino framework is supported.

## Dependencies

This library uses the [VectorQuaternionMatrix library](https://github.com/martinbudden/Library-VectorQuaternionMatrix) for its `xyz_t`(3D vector) and `Quaternion` classes.
