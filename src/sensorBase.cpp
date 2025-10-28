#include <debug.h>

#include "sensorBase.h"

sensorValuesStruct sensorBase::initialize() {
  sensorValuesStruct values;
  return values;
}

void sensorBase::sleep() {
}

byte sensorBase::setup(uint8_t calibrationId, uint8_t calibrationStatusId) {
  _calibrationId = calibrationId;
  _calibrationStatusId = calibrationStatusId;
}
