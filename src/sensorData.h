#ifndef _SENSOR_DATA_H
#define _SENSOR_DATA_H

#include <stdint.h>

// Assumed environmental values
extern float pressureReference; // hPa local QFF (official meteor-station reading)
extern float temperatureOutdoor; // °C  measured local outdoor temp

struct __attribute__((packed)) accelerometerValues {
  float x;
  float y;
  float z;
};

struct atmosphereValues {
  float altitude;
  float humidity;
  float pressure;
  float temperature;
};

struct __attribute__((packed)) gpsValues {
  int32_t altitude;
  int32_t altitudeFiltered;
  bool altitudeValid;
  int32_t altitudeMSL;
  int32_t altitudeMSLFiltered;
  bool altitudeMSLValid;
  int32_t altitudeGeoidSep;
  int32_t altitudeGeoidSepFiltered;
  bool altitudeGeoidSepValid;
  int8_t fix;
  bool fixValid;
  int32_t fixAge;
  int32_t fixAgeFiltered;
  bool fixAgeValid;
  int8_t fixType;
  bool fixTypeValid;
  uint16_t hdop;
  uint16_t hdopFiltered;
  bool hdopValid;
  int32_t latitude;
  int32_t latitudeFiltered;
  bool latitudeValid;
  int32_t longitude;
  int32_t longitudeFiltered;
  bool longitudeValid;
  uint16_t pdop;
  uint16_t pdopFiltered;
  bool pdopValid;
  uint16_t satellites;
  uint16_t satellitesFiltered;
  bool satellitesValid;
  uint16_t vdop;
  uint16_t vdopFiltered;
  bool vdopValid;
  
  uint32_t failed;
  uint32_t failedOver;
  uint32_t processed;
  uint32_t processedOver;
  uint32_t processedChecksum;
  uint32_t processedChecksumOver;
  uint32_t processedFix;
  uint32_t processedFixOver;
};

struct __attribute__((packed)) gyroscopeValues {
  float x;
  float y;
  float z;
};

struct __attribute__((packed)) magnetometerValues {
  float x;
  float y;
  float z;
};

struct __attribute__((packed)) velocityValues {
  float x;
  float y;
  float z;
};

struct sensorValuesStruct {
  accelerometerValues acceleration;
  atmosphereValues atmosphere;
  gpsValues gps;
  gyroscopeValues gyroscope;
  magnetometerValues magnetometer;
  velocityValues velocity;
};

#endif