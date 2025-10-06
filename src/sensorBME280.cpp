#include <debug.h>
#include "sensorBME280.h"
// #ifdef DEV_SIM
// #include "simulation.h"
// #endif
#include <utilities.h>

#define BME_ADDR 0x76 

sensorBME280::sensorBME280() {
}

sensorValuesStruct sensorBME280::initialize() {
  sensorValuesStruct values;

  float resultHumidity = 0;
  float resultPressure = 0;
  float resultTemperature = 0;
  byte samples = 20;
  float sumHumidity = 0;
  float sumPressure = 0;
  float sumTemperature = 0;
  for (int i = 0; i < samples; i++) {
    // debug(F("i"), i);
    atmosphereValues values2 = readAtmosphere();
    resultHumidity = values2.humidity;
    resultPressure = values2.pressure;
    resultTemperature = values2.temperature;
    // debug(F("resultHumidity"), resultHumidity);
    // debug(F("resultPressure"), resultPressure);
    // debug(F("resultTemperature"), resultTemperature);
    sumHumidity += resultHumidity;
    sumPressure += resultPressure;
    sumTemperature += resultTemperature;
    // debug(F("sumHumidity"), sumHumidity);
    // debug(F("sumPressure"), sumPressure);
    // debug(F("sumTemperature"), sumTemperature);
    delay(50);
  }
  values.atmosphere.humidity = (sumHumidity / samples);
  values.atmosphere.pressure = (sumPressure / samples);
  values.atmosphere.temperature = (sumTemperature / samples);
  // debug(F("atmosphereValues.humidity"), values.humidity);
  // debug(F("atmosphereValues.pressure"), values.pressure);
  // debug(F("temperatureOutdoor"), values.temperature);
  
  float result = 0;
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    // debug(F("i"), i);
    result = readAltitude();
    // debug(F("result"), result);
    sum += result;
    // debug(F("sum"), sum);
    delay(50);
  }
  float altitudeInitial = (sum / samples);
  // debug(F("altitudeInitial"), altitudeInitial);
  values.atmosphere.altitude = altitudeInitial;

  return values;
}

atmosphereValues sensorBME280::readAtmosphere() {
  atmosphereValues values;

  uint32_t humidity = 0;
  BME_SensorData humidityData = _sensor.readHumidity(BME_ADDR);
  // debug("humidityData.valid", humidityData.isValid);
  if (humidityData.isValid) {
    // debug(F("H"), humidityData.data);
    humidity = humidityData.data;
  }
// #if defined(KALMAN) && defined(KALMAN_HUMIDITY)
//   float humidityK = _kalmanHumidity.kalmanCalc(humidity);
// #if defined(DEBUG_SENSOR)
//    debug(F("__kalmanHumidity"), humidityK.data);
// #endif
  // humidity = humidityK;
// #endif
  // debug(F("__humidity"), humidity);
  values.humidity = (float)humidity;

  uint32_t pressure = 0;
  BME_SensorData pressureData = _sensor.readPressure(BME_ADDR);
  // debug("pressureData.valid", pressureData.isValid);
  if (pressureData.isValid) {
    // debug(F("P"), pressureData.data);
    pressure = pressureData.data;
  }
// #if defined(KALMAN) && defined(KALMAN_PRESSURE)
//   float pressureK = _kalmanPressure.kalmanCalc(pressure);
// #if defined(DEBUG_SENSOR)
//   debug("__kalmanPressure", pressureK);
// #endif
  // pressure = pressureK;
// #endif
  // debug(F("__pressure"), pressure);
  values.pressure = (float)pressure / 100.0;

  uint32_t temperature = 0;
  BME_SensorData temperatureData = _sensor.readTemperature(BME_ADDR);
  // debug("temperatureData.valid", temperatureData.isValid);
  if (temperatureData.isValid) {
    // debug(F("T"), temperatureData.data);
    temperature = temperatureData.data;
  }
// #if defined(KALMAN) && defined(KALMAN_TEMPERATURE)
//   float temperatureK = _kalmanTemperature.kalmanCalc(temperature);
// #if defined(DEBUG_SENSOR)
//   debug("__kalmanTemperature", temperatureK);
// #endif
  // temperature = temperatureK;
// #endif
  // debug(F("__temperature"), temperature);
  values.temperature = temperature;

#if defined(DEBUG_SENSOR)
  Serial.print(F("pressure (reference)="));
  Serial.print(pressureReference);
  Serial.print(F("humidity="));
  Serial.print(humidity);
  Serial.print(F("\tpressure="));
  Serial.print(pressure);
  Serial.print(F("\ttemperature="));
  Serial.println(temperature);
#endif

  return values;
}

float sensorBME280::readAltitude() {
  atmosphereValues values = readAtmosphere();
  return readAltitude(values);
}

float sensorBME280::readAltitude(atmosphereValues values) {
  float pressure = values.pressure * 100;

  // float altitude = _sensor.calAltitude(pressure, pressureReference);
  float altitude = 44330.0f * (1.0f - pow(pressure / pressureReference, 0.190f));
#if defined(DEBUG_SENSOR)
  debug(F("\tpressure"), values.pressure);
  debug(F("\taltitud"), altitude);
#endif
  values.altitude = altitude;

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

#ifdef DEV_SIM
  if (_simulation.isRunning() && _initialized) { 
    // values.altitude = simulationValueAltitude();
    values.altitude = _simulation.valueAltitude();
#if defined(DEBUG_SENSOR) && defined(DEBUG_SIM)
    debug("sim.altitude", values.altitude);
#endif
  }
#endif

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

byte sensorBME280::setup() {
  Serial.println(F("\tSetup sensor atmosphere..."));

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