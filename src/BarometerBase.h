#pragma once

#include "BUS_BASE.h"


/*!
BPS virtual base class.

Base class for an Barometric Pressure Sensor.
*/
class BarometerBase {
public:
    enum  { NOT_DETECTED = -1 };
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
