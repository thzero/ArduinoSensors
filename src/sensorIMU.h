#ifndef _SENSOR_IMU_H
#define _SENSOR_IMU_H

// #define KALMAN
// #define KALMAN_ACCELEROMETER
// #define KALMAN_GYRO
// #define KALMAN_MEGNETOMETER

#include "kalman.h"
#include "madgwick.h"
#include "sensorBase.h"
#include "sensorData.h"

class sensorIMU: public sensorBase {
  public:
    virtual void calibrationResetCommand();
    sensorValuesStruct initialize() override;
    sensorValuesStruct read(unsigned long current, unsigned long delta) override;
    virtual accelerometerValues readAccelerometer(bool update = true);
    virtual gyroscopeValues readGyroscope(bool update = true);
    virtual magnetometerValues readMagnetometer(bool update = true);
    virtual void setVerticalTolerance(float tolerance);
    void sleep() override;

  protected:
    virtual bool hasMagnetometer();
    virtual accelerometerValues readAccelerometerI(bool update = true);
    virtual gyroscopeValues readGyroscopeI(bool update = true);
    virtual magnetometerValues readMagnetometerI(bool update = true);

    float _verticalTolerance = 5;

  private:
    Madgwick _filter;
#if KALMAN
    kalman _kalmanAccelerometerX;
    kalman _kalmanAccelerometerY;
    kalman _kalmanAccelerometerZ;
    kalman _kalmanGyroX;
    kalman _kalmanGyroY;
    kalman _kalmanGyroZ;
    kalman _kalmanMagnetometerX;
    kalman _kalmanMagnetometerY;
    kalman _kalmanMagnetometerZ;
#endif
};

#endif