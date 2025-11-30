#include <debug.h>

#include "sensorBarometer.h"

sensorValuesStruct sensorBarometer::initialize() {
  sensorValuesStruct values;
  float resultAltitude = 0;
  float resultHumidity = 0;
  float resultPressure = 0;
  float resultTemperature = 0;
  int8_t samples = 20;
  float sumAltitude = 0;
  float sumHumidity = 0;
  float sumPressure = 0;
  float sumTemperature = 0;
  for (int i = 0; i < samples; i++) {
    // debug(F("i"), i);
    atmosphereValues values2 = readAtmosphere();
    resultAltitude = values2.altitude;
    resultHumidity = values2.humidity;
    resultPressure = values2.pressure;
    resultTemperature = values2.temperature;
    // debug(F("resultAltitude"), resultAltitude);
    // debug(F("resultHumidity"), resultHumidity);
    // debug(F("resultPressure"), resultPressure);
    // debug(F("resultTemperature"), resultTemperature);
    sumAltitude += resultAltitude;
    sumHumidity += resultHumidity;
    sumPressure += resultPressure;
    sumTemperature += resultTemperature;
    // debug(F("sumAltitude"), sumAltitude);
    // debug(F("sumHumidity"), sumHumidity);
    // debug(F("sumPressure"), sumPressure);
    // debug(F("sumTemperature"), sumTemperature);
    delay(50);
  }
  values.atmosphere.altitude = (sumAltitude / samples);
  values.atmosphere.humidity = (sumHumidity / samples);
  values.atmosphere.pressure = (sumPressure / samples);
  values.atmosphere.temperature = (sumTemperature / samples);
  // debug(F("atmosphereValues.humidity"), values.humidity);
  // debug(F("atmosphereValues.pressure"), values.pressure);
  // debug(F("temperatureOutdoor"), values.temperature);
  
  // float result = 0;
  // float sum = 0;
  // for (int i = 0; i < samples; i++) {
  //   // debug(F("i"), i);
  //   result = readAltitude();
  //   // debug(F("result"), result);
  //   sum += result;
  //   // debug(F("sum"), sum);
  //   delay(50);
  // }
  // float altitudeInitial = (sum / samples);
  // // debug(F("altitudeInitial"), altitudeInitial);
  // values.atmosphere.altitude = altitudeInitial;

//   // Serial.println(F("\t\tSetup Kalman filter..."));

  return values;
}

sensorValuesStruct sensorBarometer::read(unsigned long current, unsigned long delta) {
  sensorValuesStruct values;
  values.atmosphere = readAtmosphere();
  values.atmosphere.altitude = readAltitude(values.atmosphere);
  return values;
}

float sensorBarometer::readAltitude() {
  atmosphereValues values = readAtmosphere();
  return readAltitude(values);
}

float sensorBarometer::readAltitude(atmosphereValues values) {
  float altitude = 44330.0f * (1.0f - powf(values.pressure / pressureReference, 0.1903f));

//  float altitude = _previousAltitude; 
// #if defined(DEBUG_SENSOR)
  // debug(F("\tpressure"), values.pressure);
  // debug(F("\tpressureReference"), pressureReference);
  // debug(F("\taltitude"), altitude);
  // BME_SensorData altitudeS = _sensor.readAltitude(BME_ADDR, pressureReference);
  // if (altitudeS.isValid)
  //   altitude = altitudeS.data;
  // debug(F("\taltitudeS.valid"), altitudeS.isValid);
  // debug(F("\taltitudeS.data"), altitudeS.data);
// #endif
  values.altitude = altitude;
  // _previousAltitude = altitude;

// #if defined(KALMAN) && defined(KALMAN_ALTITUDE)
//   float altitudeK = _kalmanAltitude.kalmanCalc(altitude);
// // #if defined(DEBUG_SENSOR)
//   debug("_kalmanAltitude", altitudeK);
// // #endif
//   altitude = altitudeK;
// #endif
//   debug(F("altitude"), altitude);
//   values.altitude = altitude;

// #if defined(DEV) && defined(DEV_SIM)
//   if (_funcOverride != nullptr) {
//     sensorValuesStruct valuesOverride = _funcOverride();
//     values.altitude = valuesOverride.atmosphere.altitude;
// #if defined(DEBUG_SENSOR) && defined(DEBUG_SIM)
//     debug("sim.altitude", values.altitude);
// #endif
//   }
// #endif

//   // alternate to compare...
//   float altitude2 = NAN;
//   if (!isnan(values.pressure) && !isnan(pressureReference) && !isnan(temperatureOutdoor))
//   {
//       altitude2 = pow(pressureReference / values.pressure, 0.190234) - 1;
//       altitude2 *= ((temperatureOutdoor + 273.15) / 0.0065);
//   }
//   values.altitude = altitude2;
// #if defined(DEBUG_SENSOR)
//   debug("altitude2", altitude2);
// #endif

// #if defined(KALMAN) && defined(KALMAN_ALTITUDE)
//   float altitudeK = _kalmanAltitude.kalmanCalc(altitude);
// #if defined(DEBUG_SENSOR)
//   Serial.print(F("_kalmanAltitude="));
//   Serial.println(altitudeK);
// #endif
//   values.altitude = altitudeK;
// #endif

  return values.altitude;
}

atmosphereValues sensorBarometer::readAtmosphere(bool update) {
  atmosphereValues values = readAtmosphereI( update);

  // debug("humidity.valid", values.humidity);
  if (values.humidity > -1) {
    // debug(F("H"), values.humidity);

  #if defined(KALMAN) && defined(KALMAN_HUMIDITY)
      float humidityK = _kalmanHumidity.kalmanCalc(values.humidity);
  #if defined(DEBUG_SENSOR)
      debug(F("_kalmanHumidity"), humidityK.data);
  #endif
      values.humidity = humidityK;
  #endif
    // debug(F("humidity"), values.humidity);
  }

  // debug("pressureData.valid", values.pressure);
  if (values.pressure) {
    // debug(F("P"), values.pressure);
    values.altitude = 44330.0f * (1.0f - powf(values.pressure / pressureReference, 0.1903f));

#if defined(KALMAN) && defined(KALMAN_PRESSURE)
  float pressureK = _kalmanPressure.kalmanCalc(pressure);
#if defined(DEBUG_SENSOR)
    debug("_kalmanPressure", pressureK);
#endif
    values.pressure = pressureK;
    values.altitude = 44330.0f * (1.0f - powf(pressureK / pressureReference, 0.1903f))
#endif
    // debug(F("pressure"), values.pressure);
  }

  // debug("temperatureData.valid", values.temperature);
  if (values.temperature > -1) {
    // debug(F("T"), values.temperature);

#if defined(KALMAN) && defined(KALMAN_TEMPERATURE)
    float temperatureK = _kalmanTemperature.kalmanCalc(values.temperature);
#if defined(DEBUG_SENSOR)
    debug("_kalmanTemperature", temperatureK);
#endif
  values.temperature = temperatureK;
#endif
    // debug(F("temperature"), values.temperature);
  }

#if defined(DEBUG_SENSOR)
  Serial.print(F("humidity="));
  Serial.print(humidity);
  Serial.print(F("\tpressure="));
  Serial.print(pressure);
  Serial.print(F("\ttemperature="));
  Serial.println(temperature);
  Serial.print(F("pressure (reference)="));
  Serial.print(pressureReference);
  Serial.print(F("\taltitude="));
  Serial.println(altitude);
#endif

  return values;
}

void sensorBarometer::sleep() {
  Serial.println(F("\tSleep sensor atmosphere..."));

  Serial.println(F("\t...sensor atmosphere sleep successful."));
}