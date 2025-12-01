#ifndef _SENSOR_BME_H
#define _SENSOR_BME_H

#include <BME280_LITE.h>

#include "sensorBarometer.h"
#include "sensorData.h"

class sensorBME280: public sensorBarometer {
  public:
    sensorBME280();
    int8_t setup(uint8_t calibrationId, uint8_t calibrationStatusId) override;

  protected:
    atmosphereValues readAtmosphereI(bool update = true) override;
    
  private:
    BME280_LITE _sensor;
    // BMP280 _sensor = BMP280();
};

#endif