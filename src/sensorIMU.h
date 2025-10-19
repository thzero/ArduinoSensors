#ifndef _SENSOR_IMU_H
#define _SENSOR_IMU_H

#include "sensorBase.h"
#include "sensorData.h"

class sensorIMU: public sensorBase {
  public:
    virtual void calibrationResetCommand();
    virtual accelerometerValues readAccelerometer();
    virtual gyroscopeValues readGyroscope();
    virtual magnetometerValues readMagnetometer();
};

#endif