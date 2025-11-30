#ifndef _SENSOR_IMU_H
#define _SENSOR_IMU_H

#include "madgwick.h"
#include "sensorBase.h"
#include "sensorData.h"

class sensorIMU: public sensorBase {
  public:
    virtual void calibrationResetCommand();
    virtual accelerometerValues readAccelerometer(bool update = true);
    virtual gyroscopeValues readGyroscope(bool update = true);
    virtual magnetometerValues readMagnetometer(bool update = true);
    virtual void setVerticalTolerance(float tolerance);

  protected:
    float _verticalTolerance = 5;
    virtual bool hasMagnetometer();

  private:
    Madgwick _filter;
};

#endif