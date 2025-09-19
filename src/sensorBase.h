#ifndef _SENSOR_BASE_H
#define _SENSOR_BASE_H

#include "sensorData.h"

class sensorBase {
  public:
    virtual void calibrationResetCommand();
    virtual sensorValuesStruct initialize();
    virtual accelerometerValues readAccelerometer();
    virtual float readAltitude();
    virtual float readAltitude(atmosphereValues values);
    virtual atmosphereValues readAtmosphere();
    virtual gyroscopeValues readGyroscope();
    virtual void sleep();
    virtual void setup();
};

#endif