#pragma once

#include "BUS_BASE.h"


/*!
Barometer virtual base class.
*/
class BarometerBase {
public:
    enum  { NOT_DETECTED = -1 };
    // Values for reporting barometer type back to MSP (MultiWii Serial Protocol)
    enum barometer_type_e {
        MSP_BAROMETER_ID_DEFAULT = 0,
        MSP_BAROMETER_ID_NONE = 1,
        MSP_BAROMETER_ID_BMP280 = 4,
        MSP_BAROMETER_ID_VIRTUAL = 11,
    };

public:
    virtual ~BarometerBase() = default;
    explicit BarometerBase(BUS_BASE& busBase);
    virtual int init() = 0;

    uint32_t getSampleRateHz() const { return _sampleRateHz; }
    float getTemperatureCelsius() const { return _temperatureCelsius; }
    float getPressurePascals() const { return _pressurePascals; }

    virtual float readTemperatureCelsius() = 0;
    virtual float readPressurePascals() = 0;
    virtual float readAltitudeMeters() = 0;
    virtual float calculateAltitudeMeters(float pressure) = 0;

    void setReferenceAltitude(float referenceAltitude) { _referenceAltitude = referenceAltitude; }
    void setPressureAtReferenceAltitude(float pressureAtReferenceAltitude) { _pressureAtReferenceAltitude = pressureAtReferenceAltitude; }

    static void delayMs(int ms);
protected:
    BUS_BASE* _busBase;
    uint32_t _sampleRateHz {};
    float _referenceAltitude {};
    float _pressureAtReferenceAltitude {};
    float _pressurePascals {};
    float _temperatureCelsius {};
};
