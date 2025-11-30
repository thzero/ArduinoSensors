#include <debug.h>

#include "sensorIMU.h"

void sensorIMU::calibrationResetCommand() {
}

bool sensorIMU::hasMagnetometer() {
  return false;
}

accelerometerValues sensorIMU::readAccelerometer(bool update) {
  accelerometerValues values;
  values.x = 0.0;
  values.y = 0.0;
  values.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("accelerometer.x"), values.x);
  debug(F("accelerometer.y"), values.y);
  debug(F("accelerometer.z"), values.z);
#endif

  return values;
}

gyroscopeValues sensorIMU::readGyroscope(bool update) {
  gyroscopeValues values;
  values.x = 0.0;
  values.y = 0.0;
  values.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("gyroscopeV.x"), values.x);
  debug(F("gyroscopeV.y"), values.y);
  debug(F("gyroscopeV.z"), values.z);
#endif

  return values;
}

magnetometerValues sensorIMU::readMagnetometer(bool update) {
  magnetometerValues values;
  values.x = 0.0;
  values.y = 0.0;
  values.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("magnetometer.x"), values.x);
  debug(F("magnetometer.y"), values.y);
  debug(F("magnetometer.z"), values.z);
#endif

  return values;
}

void sensorIMU::setVerticalTolerance(float tolerance) {
  _verticalTolerance = tolerance;
}