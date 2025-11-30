#include <debug.h>

#include "sensorBME280.h"
#include <utilities.h>

#define BME_ADDR 0x76 

sensorBME280::sensorBME280() {
}

atmosphereValues sensorBME280::readAtmosphereI(bool update) {
  atmosphereValues values;

  values.humidity = -1;
  BME_SensorData humidityData = _sensor.readHumidity(BME_ADDR);
  // debug("humidityData.valid", humidityData.isValid);
  if (humidityData.isValid) {
    // debug(F("H"), humidityData.data);
    values.humidity = humidityData.data;
  }

  values.pressure = -1;
  BME_SensorData pressureData = _sensor.readPressure(BME_ADDR);
  // debug("pressureData.valid", pressureData.isValid);
  if (pressureData.isValid) {
    // debug(F("P"), pressureData.data);
    values.pressure = pressureData.data;
  }

  values.temperature = -1;
  BME_SensorData temperatureData = _sensor.readTemperature(BME_ADDR);
  // debug("temperatureData.valid", temperatureData.isValid);
  if (temperatureData.isValid) {
    // debug(F("T"), temperatureData.data);
    values.temperature = temperatureData.data;
  }

#if defined(DEBUG_SENSOR)
  debug(F("humidity"), values.humidity);
  debug(F("pressure"), values.pressure);
  debug(F("temperature"), values.temperature);
#endif

  return values;
}

int8_t sensorBME280::setup(uint8_t calibrationId, uint8_t calibrationStatusId) {
  Serial.println(F("\tSetup sensor atmosphere..."));

  sensorBase::setup(calibrationId, calibrationStatusId);

  bool results = _sensor.begin(BME_ADDR, // returns a T/F based on initialization.
            BME_H_X1, // All settings can be found in the docs.
            BME_T_X1, 
            BME_P_X16, 
            BME_NORMAL, 
            BME_TSB_0_5MS, 
            BME_FILTER_2);
  if (!results) {
    Serial.println(F("\t...sensor atmosphere unsuccessful."));
    return 1;
  }

  _sensor.calibrate(BME_ADDR);

  Serial.println(F("\t...sensor atmosphere successful."));

  return 0;
}