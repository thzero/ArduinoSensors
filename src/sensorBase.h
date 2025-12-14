#ifndef _SENSOR_BASE_H
#define _SENSOR_BASE_H

#include <stdint.h>

#include "sensorData.h"

typedef sensorValuesStruct (*SensorOverrideFunctionPtr)();

class sensorBase {
  public:
    virtual sensorValuesStruct initialize();
    virtual void read(sensorValuesStruct* values, unsigned long current, unsigned long delta);
    int8_t initOverride(SensorOverrideFunctionPtr funcOverride);
    virtual void sleep();

  protected:
    virtual int8_t setupI(uint8_t calibrationId, uint8_t calibrationStatusId);

    uint8_t _calibrationId;
    uint8_t _calibrationStatusId;
    SensorOverrideFunctionPtr _funcOverride;
};

#endif