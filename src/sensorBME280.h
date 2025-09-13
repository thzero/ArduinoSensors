#ifndef _SENSOR_BME_H
#define _SENSOR_BME_H

#include <BME280_LITE.h>

// #include "kalman.h"
#include "sensorData.h"

class sensorBME280 {
  public:
    sensorBME280();
    // void init(BME280I2C bme);
    atmosphereValues initializeSensors();
    void initSensors();
    atmosphereValues readSensor();
    float readSensorAltitude();
    float readSensorAltitude(atmosphereValues values);
    void sleepSensors();
    void setupSensors();
    
  private:
    BME280_LITE _sensor;
    // BMP280 _sensor = BMP280();
    int _count;
    // kalman _kalmanAltitude;
    // kalman _kalmanHumidity;
    // kalman _kalmanPressure;
    // kalman _kalmanTemperature;
};

#endif