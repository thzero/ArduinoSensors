#ifndef _SENSOR_MPU6050_H
#define _SENSOR_MPU6050_H

#include "FastIMU.h"

// #include "kalman.h"
#include "sensorBase.h"
#include "sensorData.h"

class sensorMPU6050: public sensorBase {
  public:
    sensorMPU6050();
    void calibrationResetCommand() override;
    sensorValuesStruct initialize() override;
    accelerometerValues readAccelerometer() override;
    gyroscopeValues readGyroscope() override;
    void sleep() override;
    void setup() override;
    
  private:
    void calibrationDisplay(calData calibrationData, const char* offset);
    void setupCalibration();

#if defined(TEENSYDUINO)
    MPU6050 _imu = MPU6050(Wire2); // TODO: need to make this configurable...
#else
    MPU6050 _imu = MPU6050(Wire1); // TODO: need to make this configurable...
#endif
    calData calibrationData = { 0 };  //Calibration data
    // kalman _kalmanAccelX;
    // kalman _kalmanAccelY;
    // kalman _kalmanAccelZ;
    // kalman _kalmanGyroX;
    // kalman _kalmanGyroY;
    // kalman _kalmanGyroZ;
};

#endif