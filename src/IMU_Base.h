#pragma once

#include "BUS_BASE.h"

#include <Quaternion.h>


/*!
IMU virtual base class.

Base class for an IMU (Inertial Management Unit) including a gyroscope and accelerometer.

Gyroscope readings can be returned as raw values, in RPS (Radians Per Second), or in DPS (Degrees Per Second).

Accelerometer readings are returned in units of standard gravity (g - 9.80665 meters per second squared).

The gyro and accelerometer can be read together using readAccGyroRPS. For typical IMUs this involves reading 12 bytes of data:
if the IMU is read via a 10 MHz  SPI bus this takes approximately 10 microseconds,
if the IMU is read via a 400 kHz I2C bus this takes approximately 270 microseconds.
*/
class IMU_Base {
public:
    enum  { NOT_DETECTED = -1 };
    /*!
    Axes order describing the sensor axes relative to the body axes.
    For example, if the sensor is rotated relative to the body so that the
    sensor X-axis points right, the sensor Z-axis points forward, and the sensor Y-axis points down,
    then the axis order is XPOS_ZPOS_YNEG.

    For example, if the sensor is rotated relative to the body so that the
    sensor y-axis points right, the sensor X-axis points back, and the sensor Z-axis points up,
    then the axis order is YPOS_XNEG_ZPOS
    */
   enum axis_order_e {
        XPOS_YPOS_ZPOS,
        YPOS_XNEG_ZPOS, // rotate  90 degrees anticlockwise
        XNEG_YNEG_ZPOS, // rotate 180 degrees
        YNEG_XPOS_ZPOS, // rotate 270 degrees anticlockwise

        XPOS_YNEG_ZNEG,
        YPOS_XPOS_ZNEG,
        XNEG_YPOS_ZNEG,
        YNEG_XNEG_ZNEG,

        ZPOS_YNEG_XPOS,
        YPOS_ZPOS_XPOS,
        ZNEG_YPOS_XPOS,
        YNEG_ZNEG_XPOS,

        ZPOS_YPOS_XNEG,
        YPOS_ZNEG_XNEG,
        ZNEG_YNEG_XNEG,
        YNEG_ZPOS_XNEG,

        ZPOS_XPOS_YPOS,
        XNEG_ZPOS_YPOS,
        ZNEG_XNEG_YPOS,
        XPOS_ZNEG_YPOS,

        ZPOS_XNEG_YNEG,
        XNEG_ZNEG_YNEG,
        ZNEG_XPOS_YNEG,
        XPOS_ZPOS_YNEG,

        XPOS_YPOS_ZPOS_45, // rotate  45 degrees anticlockwise
        YPOS_XNEG_ZPOS_45, // rotate 135 degrees anticlockwise
        XNEG_YNEG_ZPOS_45, // rotate 225 degrees anticlockwise
        YNEG_XPOS_ZPOS_45,  // rotate 315 degrees anticlockwise

        XPOS_YPOS_ZPOS_135 = YPOS_XNEG_ZPOS_45,
        XPOS_YPOS_ZPOS_225 = XNEG_YNEG_ZPOS_45,
        XPOS_YPOS_ZPOS_315 = YNEG_XPOS_ZPOS_45
    };

    enum { TARGET_OUTPUT_DATA_RATE_MAX = 0 };
    enum gyro_sensitivity_e {
        GYRO_FULL_SCALE_MAX,
        GYRO_FULL_SCALE_125_DPS,
        GYRO_FULL_SCALE_250_DPS,
        GYRO_FULL_SCALE_500_DPS,
        GYRO_FULL_SCALE_1000_DPS,
        GYRO_FULL_SCALE_2000_DPS,
        GYRO_FULL_SCALE_4000_DPS,
    };
    enum acc_sensitivity_e {
        ACC_FULL_SCALE_MAX,
        ACC_FULL_SCALE_1G,
        ACC_FULL_SCALE_2G,
        ACC_FULL_SCALE_4G,
        ACC_FULL_SCALE_8G,
        ACC_FULL_SCALE_16G,
        ACC_FULL_SCALE_32G,
    };
    // Values for reporting gyro and acc type back to MSP (MultiWii Serial Protocol)
    enum {
        MSP_GYRO_ID_NONE = 0, MSP_GYRO_ID_DEFAULT = 1, MSP_GYRO_ID_VIRTUAL = 20,
        MSP_ACC_ID_DEFAULT = 0, MSP_ACC_ID_NONE = 1 , MSP_ACC_ID_VIRTUAL = 21
    };

    // IMU characteristics flag values
    enum : uint32_t { IMU_AUTO_CALIBRATES = 0x01, IMU_PERFORMS_SENSOR_FUSION = 0x02 };

    static constexpr float sin45f = 0.7071067811865475F;
    static constexpr float cos45f = 0.7071067811865475F;
    const std::array<Quaternion, 24> axisOrientations = {
        Quaternion(  1.0F,    0.0F,    0.0F,    0.0F ),
        Quaternion(  sin45f,  0.0F,    0.0F,    sin45f ),
        Quaternion(  0.0F,    0.0F,    0.0F,    1.0F ),
        Quaternion(  sin45f,  0.0F,    0.0F,   -sin45f ),
        Quaternion(  0.0F,    0.0F,   -1.0F,    0.0F ),
        Quaternion(  0.0F,   -sin45f, -sin45f,  0.0F ),
        Quaternion(  0.0F,   -1.0F,    0.0F,    0.0F ),
        Quaternion(  0.0F,   -sin45f,  sin45f,  0.0F ),
        Quaternion(  0.0F,    0.0F,   -sin45f, sin45f ),
        Quaternion(  0.5F,   -0.5F,   -0.5F,    0.5F ),
        Quaternion(  sin45f, -sin45f,  0.0F,    0.0F ),
        Quaternion(  0.5F,   -0.5F,    0.5F,   -0.5F ),
        Quaternion(  sin45f, -sin45f,  0.0F,    0.0F ),
        Quaternion( -0.5F,   -0.5F,   -0.5F,   -0.5F ),
        Quaternion(  0.0F,    0.0F,   -sin45f, -sin45f ),
        Quaternion(  0.5F,    0.5F,   -0.5F,   -0.5F ),
        Quaternion( -0.5F,   -0.5F,   -0.5F,    0.5F ),
        Quaternion(  0.0F,   -sin45f,  0.0F,    sin45f ),
        Quaternion(  0.5F,   -0.5F,    0.5F,    0.5F ),
        Quaternion( -sin45f,  0.0F,   -sin45f,  0.0F ),
        Quaternion(  0.5F,    0.5F,   -0.5F,    0.5F ),
        Quaternion(  0.0F,   -sin45f,  0.0F,   -sin45f ),
        Quaternion(  0.5F,   -0.5F,   -0.5F,   -0.5F ),
        Quaternion(  sin45f,  0.0F,   -sin45f,  0.0F )
    };
public:
    virtual ~IMU_Base() = default;
    IMU_Base(axis_order_e axisOrder, BUS_BASE& busBase, uint32_t flags);
    IMU_Base(axis_order_e axisOrder, BUS_BASE& busBase);
    IMU_Base(axis_order_e axisOrder, uint32_t flags);
    explicit IMU_Base(axis_order_e axisOrder);
public:
    struct xyz_alignment_t {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct xyz_int32_t {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    struct accGyroRPS_t {
        xyz_t gyroRPS;
        xyz_t acc;
    };
    static constexpr float degreesToRadians = static_cast<float>(M_PI / 180.0);
    static constexpr float radiansToDegrees = static_cast<float>(180.0 / M_PI);
public:
    static void delayMs(int ms);
    virtual int init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* i2cMutex) = 0;
    virtual int init(uint32_t targetOutputDataRateHz, void* i2cMutex) final;
    virtual int init(uint32_t targetOutputDataRateHz) final;
    virtual int init(void* i2cMutex) final;
    virtual int init() final;

    float getGyroResolutionDPS() const { return _gyroResolutionDPS; }
    float getAccResolution() const { return _accResolution; }
    uint32_t getGyroSampleRateHz() const { return _gyroSampleRateHz; }
    uint32_t getAccSampleRateHz() const { return _accSampleRateHz; }
    uint16_t getGyroIdMSP() const { return _gyroIdMSP; }
    uint16_t getAccIdMSP() const { return _accIdMSP; }

    virtual void setInterruptDriven();
    int32_t WAIT_IMU_DATA_READY() { return _busBase->WAIT_DATA_READY(); }
    int32_t WAIT_IMU_DATA_READY(uint32_t ticksToWait) { return _busBase->WAIT_DATA_READY(ticksToWait); }
    void SIGNAL_IMU_DATA_READY_FROM_ISR() { _busBase->SIGNAL_DATA_READY_FROM_ISR(); } //! should not be used directly, made public for debugging purposes

    virtual xyz_int32_t getGyroOffset() const;
    virtual void setGyroOffset(const xyz_int32_t& gyroOffset);
    virtual xyz_int32_t getAccOffset() const;
    virtual void setAccOffset(const xyz_int32_t& accOffset);

    virtual xyz_int32_t readGyroRaw() = 0;
    virtual xyz_int32_t readAccRaw() = 0;
    virtual int32_t getAccOneG_Raw() const;

    // read functions have default implementations in the base class for convenience,
    // but should be reimplemented in derived classes for efficiency
    virtual xyz_t readGyroRPS();
    virtual xyz_t readGyroDPS();
    virtual xyz_t readAcc();
    virtual accGyroRPS_t readAccGyroRPS();
    virtual accGyroRPS_t getAccGyroRPS() const;

    virtual Quaternion readOrientation();

    inline axis_order_e getAxisOrder() const { return _axisOrder; }
    inline void setAxisOrder(axis_order_e axisOrder) { _axisOrder = axisOrder; }
    static xyz_t mapAxes(const xyz_t& data, axis_order_e axisOrder);
    inline xyz_t mapAxes(const xyz_t& data) const { return mapAxes(data, _axisOrder); }
    static axis_order_e axisOrderInverse(axis_order_e axisOrder);
    static xyz_alignment_t alignmentFromAxisOrder(axis_order_e axisOrder);
    static axis_order_e axisOrderFromAlignment(const xyz_alignment_t& alignment);

    inline uint32_t getFlags() const { return _flags; }
#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(LIBRARY_IMU_I2C_MUTEX_REQUIRED)
    inline void i2cSemaphoreTake() const { xSemaphoreTake(_i2cMutex, portMAX_DELAY); }
    inline void i2cSemaphoreGive() const { xSemaphoreGive(_i2cMutex); }
#else
    inline void i2cSemaphoreTake() const {}
    inline void i2cSemaphoreGive() const {}
#endif
    // functions to allow an IMU implementation to do run time checking if a mutex is required. Used by M5Stack implementations.
    inline void i2cSemaphoreTake(SemaphoreHandle_t i2cMutex) const { if (i2cMutex) {xSemaphoreTake(i2cMutex, portMAX_DELAY);}  }
    inline void i2cSemaphoreGive(SemaphoreHandle_t i2cMutex) const { if (i2cMutex) {xSemaphoreGive(i2cMutex);} }
    SemaphoreHandle_t _i2cMutex {};
#else
    inline void i2cSemaphoreTake() const {}
    inline void i2cSemaphoreGive() const {}
    inline void i2cSemaphoreTake(void* i2cMutex) const { (void)i2cMutex; }
    inline void i2cSemaphoreGive(void* i2cMutex) const { (void)i2cMutex; }
    void* _i2cMutex {};
#endif // FRAMEWORK_USE_FREERTOS
protected:
    axis_order_e _axisOrder;
    BUS_BASE* _busBase;
    const uint32_t _flags; //!< Flags for describing IMU characteristics
    float _gyroResolutionRPS {};
    float _gyroResolutionDPS {};
    float _accResolution { 8.0F / 32768.0F };
    uint32_t _gyroSampleRateHz {};
    uint32_t _accSampleRateHz {};
    xyz_int32_t _gyroOffset {};
    xyz_int32_t _accOffset {};
    uint16_t _gyroIdMSP {};
    uint16_t _accIdMSP {};
};
