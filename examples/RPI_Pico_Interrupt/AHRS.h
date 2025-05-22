#include <IMU_Base.h>
#include <pico/mutex.h>

class AHRS {
public:
    ~AHRS();
    explicit AHRS(IMU_Base& imu);
    static void imuDataReadyISR();
    inline void LOCK_IMU_DATA_READY() const { mutex_enter_blocking(&_imuDataReadyMutex); }
    inline uint32_t getImuDataReadyCount() const { return _imuDataReadyCount; }
private:
    inline void UNLOCK_IMU_DATA_READY_FROM_ISR() const { _imuDataReadyCount = 0; mutex_exit(&_imuDataReadyMutex); }
private:
    static AHRS* ahrs; //!< alias of `this` to be used in dataReadyISR
    IMU_Base& _IMU;
    mutable mutex_t _imuDataReadyMutex{};
    //auto_init_mutex(_imuDataReadyMutex);
    int _userIrq {0};
    mutable uint32_t _imuDataReadyCount {0}; //<! data ready count, set in interrupt service routine for instrumentation
};
