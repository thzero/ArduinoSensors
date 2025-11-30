#ifndef _SENSOR_BAROMETER_H
#define _SENSOR_BAROMETER_H

// #define KALMAN
// #define KALMAN_ALTITUDE
// #define KALMAN_HUMIDITY
// #define KALMAN_PRESSURE
// #define KALMAN_TEMPERATURE

#include "kalman.h"
#include "sensorBase.h"
#include "sensorData.h"

class sensorBarometer: public sensorBase {
  public:
    sensorValuesStruct initialize() override;
    sensorValuesStruct read(unsigned long current, unsigned long delta) override;
    virtual float readAltitude();
    virtual float readAltitude(atmosphereValues values);
    virtual atmosphereValues readAtmosphere(bool update = true);
    void sleep() override;

  protected:
    virtual atmosphereValues readAtmosphereI(bool update = true);

  private:
    // float _previousAltitude = 0;
    // float _previousHumidity = 0;
    // float _previousPressure = 0;
    // float _previousTemperature = 0;
#if KALMAN
    kalman _kalmanAltitude;
    kalman _kalmanHumidity;
    kalman _kalmanPressure;
    kalman _kalmanTemperature;
#endif
};

#endif