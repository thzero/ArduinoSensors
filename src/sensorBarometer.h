#ifndef _SENSOR_BAROMETER_H
#define _SENSOR_BAROMETER_H

#include "sensorBase.h"
#include "sensorData.h"

class sensorBarometer: public sensorBase {
  public:
    virtual float readAltitude();
    virtual float readAltitude(atmosphereValues values);
    virtual atmosphereValues readAtmosphere();
};

#endif