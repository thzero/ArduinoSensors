#ifndef _SENSOR_DATA_H
#define _SENSOR_DATA_H

// Assumed environmental values
extern float pressureReference; // hPa local QFF (official meteor-station reading)
extern float temperatureOutdoor; // °C  measured local outdoor temp

// struct atmosphereValues {
struct __attribute__((packed)) atmosphereValues {
  float altitude;
  float humidity;
  float pressure;
  float temperature;
};

// struct accelerometerValues {
struct __attribute__((packed)) accelerometerValues {
  float x;
  float y;
  float z;
};

// struct gyroscopeValues {
struct __attribute__((packed)) gyroscopeValues {
  float x;
  float y;
  float z;
};

// struct sensorValuesStruct {
struct __attribute__((packed)) sensorValuesStruct {
  atmosphereValues atmosphere;
  accelerometerValues acceleration;
  gyroscopeValues gyroscope;
};

#endif