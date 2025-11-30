#include <debug.h>

#include "sensorBME280.h"
#include <utilities.h>

#define BME_ADDR 0x76 

sensorBME280::sensorBME280() {
}

sensorValuesStruct sensorBME280::initialize() {
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

  return values;
}

atmosphereValues sensorBME280::readAtmosphere(bool update) {
  atmosphereValues values;

  float humidity = 0;
  BME_SensorData humidityData = _sensor.readHumidity(BME_ADDR);
  // debug("humidityData.valid", humidityData.isValid);
  if (humidityData.isValid) {
    // debug(F("H"), humidityData.data);
    humidity = humidityData.data;

  #if defined(KALMAN) && defined(KALMAN_HUMIDITY)
      float humidityK = _kalmanHumidity.kalmanCalc(humidity);
  #if defined(DEBUG_SENSOR)
      debug(F("_kalmanHumidity"), humidityK.data);
  #endif
      humidity = humidityK;
  #endif
      // debug(F("humidity"), humidity);
      values.humidity = humidity;
  }

  float altitude = 0;
  float pressure = 0;
  BME_SensorData pressureData = _sensor.readPressure(BME_ADDR);
  // debug("pressureData.valid", pressureData.isValid);
  if (pressureData.isValid) {
    // debug(F("P"), pressureData.data);
    pressure = pressureData.data;
    altitude = 44330.0f * (1.0f - powf(pressure / pressureReference, 0.1903f));

#if defined(KALMAN) && defined(KALMAN_PRESSURE)
  float pressureK = _kalmanPressure.kalmanCalc(pressure);
#if defined(DEBUG_SENSOR)
    debug("_kalmanPressure", pressureK);
#endif
    pressure = pressureK;
    altitude = 44330.0f * (1.0f - powf(pressureK / pressureReference, 0.1903f))
#endif
    // debug(F("pressure"), pressure);
    values.pressure = pressure;
    values.altitude = altitude;
  }

  float temperature = 0;
  BME_SensorData temperatureData = _sensor.readTemperature(BME_ADDR);
  // debug("temperatureData.valid", temperatureData.isValid);
  if (temperatureData.isValid) {
    // debug(F("T"), temperatureData.data);
    temperature = temperatureData.data;

#if defined(KALMAN) && defined(KALMAN_TEMPERATURE)
    float temperatureK = _kalmanTemperature.kalmanCalc(temperature);
#if defined(DEBUG_SENSOR)
    debug("_kalmanTemperature", temperatureK);
#endif
  temperature = temperatureK;
#endif
    // debug(F("temperature"), temperature);
    values.temperature = temperature;
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

float sensorBME280::readAltitude() {
  atmosphereValues values = readAtmosphere();
  return readAltitude(values);
}

float sensorBME280::readAltitude(atmosphereValues values) {
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

void sensorBME280::sleep() {
  Serial.println(F("\tSleep sensor atmosphere..."));

  Serial.println(F("\t...sensor atmosphere sleep successful."));
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

//   Serial.println(F("\t...BME280 initialized."));

//   // Serial.println(F("\t\tSetup Kalman filter..."));

//   // let's do some dummy altitude reading to initialise the Kalman filter
// // #if defined(KALMAN)
// //   for (int i = 0; i < 50; i++) {
// // // #ifdef DEBUG
// // //   Serial.print(F("Reading atmosphere...");
// // //   Serial.println(i);
// // // #endif
// // //   readAtmosphere();
// //   }
// // #endif

//   // Serial.println(F("\t\t...Kalman filter initialized."));

  Serial.println(F("\t...sensor atmosphere successful."));

  return 0;
}