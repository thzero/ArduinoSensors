#include <debug.h>

#include "sensorBase.h"

sensorValuesStruct sensorBase::initialize() {
  sensorValuesStruct values;
  return values;
}

int8_t sensorBase::initOverride(SensorOverrideFunctionPtr funcOverride) {
  _funcOverride = funcOverride;
  return 0;
}

int8_t sensorBase::setup(uint8_t calibrationId, uint8_t calibrationStatusId) {
  _calibrationId = calibrationId;
  _calibrationStatusId = calibrationStatusId;
  return 0;
}

void sensorBase::sleep() {
}
