#ifndef _SENSOR_BME_H
#define _SENSOR_BME_H

#include <BME280_LITE.h>

// #define KALMAN
// #define KALMAN_ALTITUDE
// #define KALMAN_HUMIDITY
// #define KALMAN_PRESSURE
// #define KALMAN_TEMPERATURE

#include "kalman.h"
#include "sensorBarometer.h"
#include "sensorData.h"

class sensorBME280: public sensorBarometer {
  public:
    sensorBME280();
    // void init(BME280I2C bme);
    sensorValuesStruct initialize() override;
    atmosphereValues readAtmosphere() override;
    float readAltitude() override;
    float readAltitude(atmosphereValues values) override;
    void sleep() override;
    int8_t setup(uint8_t calibrationId, uint8_t calibrationStatusId) override;
    
  private:
    BME280_LITE _sensor;
    // BMP280 _sensor = BMP280();
    int _count;
    // float _previousAltitude = 0;
    // float _previousHumidity = 0;
    // float _previousPressure = 0;
    // float _previousTemperature = 0;
#if KALMAN
    kalman _kalmanAltitude;
    kalman _kalmanHumidity;
    kalman _kalmanPressure;
    kalman _kalmanTemperature;
#endif
};

#endif