#include <debug.h>
#include "sensorBase.h"

void sensorBase::calibrationResetCommand() {
}

accelerometerValues sensorBase::readAccelerometer() {
  accelerometerValues values = {};
  return values;
}

float sensorBase::readAltitude() {
  return 0;
}

float sensorBase::readAltitude(atmosphereValues values) {
  return 0;
}

atmosphereValues sensorBase::readAtmosphere() {
  atmosphereValues values;
  return values;
}

gyroscopeValues sensorBase::readGyroscope() {
  gyroscopeValues values;
  return values;
}

void sensorBase::sleep() {
}