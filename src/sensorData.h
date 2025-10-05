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
  float altitude;
  float altitudeFiltered;
  bool altitudeValid;
  float altitudeMSL;
  float altitudeMSLFiltered;
  bool altitudeMSLValid;
  float altitudeGeoidSep;
  float altitudeGeoidSepFiltered;
  bool altitudeGeoidSepValid;
  bool fix;
  bool fixValid;
  int fixAge;
  int fixAgeFiltered;
  bool fixAgeValid;
  float hdop;
  float hdopFiltered;
  bool hdopValid;
  float latitude;
  float latitudeFiltered;
  bool latitudeValid;
  float longitude;
  float longitudeFiltered;
  bool longitudeValid;
  float pdop;
  float pdopFiltered;
  bool pdopValid;
  int satellites;
  int satellitesFiltered;
  bool satellitesValid;
  float vdop;
  float vdopFiltered;
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

struct sensorValuesStruct {
  accelerometerValues acceleration;
  atmosphereValues atmosphere;
  gpsValues gps;
  gyroscopeValues gyroscope;
};

#endif