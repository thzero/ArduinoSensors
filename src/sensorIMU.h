#ifndef _SENSOR_IMU_H
#define _SENSOR_IMU_H

// #define KALMAN
// #define KALMAN_ACCELEROMETER
// #define KALMAN_GYRO
// #define KALMAN_MEGNETOMETER

#include <Wire.h>

#include "kalman.h"
#include "madgwick.h"
#include "sensorBase.h"
#include "sensorData.h"

class sensorIMU: public sensorBase {
  public:
    virtual void calibrationResetCommand();
    sensorValuesStruct initialize() override;
    void read(sensorValuesStruct* values, unsigned long current, unsigned long delta) override;
    // virtual accelerometerValues readAccelerometer(bool update = true);
    // virtual gyroscopeValues readGyroscope(bool update = true);
    // virtual magnetometerValues readMagnetometer(bool update = true);
    virtual int8_t setup(TwoWire* wire, uint8_t calibrationId, uint8_t calibrationStatusId, int gyro_config_num = 1, int acc_config_num = 0);
    virtual void setVerticalTolerance(float tolerance);
    void sleep() override;

  protected:
    virtual bool hasMagnetometer();
    virtual void calculate(sensorValuesStruct* values, unsigned long current, unsigned long delta);
    virtual void readI(sensorValuesStruct* values);
    // virtual accelerometerValues readAccelerometerI(accelerometerValues* values, bool update = true);
    // virtual gyroscopeValues readGyroscopeI(gyroscopeValues* values, bool update = true);
    // virtual magnetometerValues readMagnetometerI(magnetometerValues* values, bool update = true);
    float _verticalTolerance = 5.0;

  private:
    Madgwick _filter;
    float pitch;
    float roll;
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