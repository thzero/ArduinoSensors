#ifndef _SENSOR_MPU6050_H
#define _SENSOR_MPU6050_H

#include "FastIMU.h"

// #include "kalman.h"
#include "sensorIMU.h"
#include "sensorData.h"

class sensorMPU6050: public sensorIMU {
  public:
    sensorMPU6050();
    void calibrationResetCommand() override;
    sensorValuesStruct initialize() override;
    accelerometerValues readAccelerometer() override;
    gyroscopeValues readGyroscope() override;
    magnetometerValues readMagnetometer() override;
    void sleep() override;
    byte setup(uint8_t calibrationId, uint8_t calibrationStatusId);
    
  private:
    void calibrationDisplay(calData calibrationData, const char* offset);
    void setupCalibration();

#if defined(TEENSYDUINO)
    MPU6050 _imu = MPU6050(Wire2); // TODO: need to make this configurable...
#else
    MPU6050 _imu = MPU6050(Wire1); // TODO: need to make this configurable...
#endif
    calData _calibrationData = { 0 };  // Calibration data
    // kalman _kalmanAccelX;
    // kalman _kalmanAccelY;
    // kalman _kalmanAccelZ;
    // kalman _kalmanGyroX;
    // kalman _kalmanGyroY;
    // kalman _kalmanGyroZ;
};

#endif