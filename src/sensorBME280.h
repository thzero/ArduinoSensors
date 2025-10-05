#ifndef _SENSOR_BME_H
#define _SENSOR_BME_H

#include <BME280_LITE.h>

// #include "kalman.h"
#include "sensorBase.h"
#include "sensorData.h"

class sensorBME280: public sensorBase {
  public:
    sensorBME280();
    // void init(BME280I2C bme);
    sensorValuesStruct initialize() override;
    atmosphereValues readAtmosphere() override;
    float readAltitude() override;
    float readAltitude(atmosphereValues values) override;
    void sleep() override;
    byte setup() override;
    
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