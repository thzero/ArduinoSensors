#ifndef _SENSOR_MPU6050_H
#define _SENSOR_MPU6050_H

#include "FastIMU.h"

// #include "kalman.h"
#include "sensorData.h"

class sensorMPU6050 {
  public:
    sensorMPU6050();
    void sleepSensors();
    void setupSensors();
    accelerometerValues readSensorAccelerometer();
    gyroscopeValues readSensorGyroscope();
    void calibrationResetCommand();
    
  private:
    void calibrationDisplay(calData calibrationData, const char* offset);
    void setupSensorCalibration();

    MPU6050 _imu = MPU6050(Wire2);
    calData calibrationData = { 0 };  //Calibration data
    // kalman _kalmanAccelX;
    // kalman _kalmanAccelY;
    // kalman _kalmanAccelZ;
    // kalman _kalmanGyroX;
    // kalman _kalmanGyroY;
    // kalman _kalmanGyroZ;
};

#endif