#ifndef _SENSOR_DATA_H
#define _SENSOR_DATA_H

// Assumed environmental values
extern float pressureReference; // hPa local QFF (official meteor-station reading)
extern float temperatureOutdoor; // °C  measured local outdoor temp

struct atmosphereValues {
  float altitude;
  float humidity;
  float pressure;
  float temperature;
};

struct accelerometerValues {
  float x;
  float y;
  float z;
};

struct gyroscopeValues {
  float x;
  float y;
  float z;
};

struct sensorValuesStruct {
  atmosphereValues atmosphere;
  accelerometerValues acceleration;
  gyroscopeValues gyroscope;
};

#endif