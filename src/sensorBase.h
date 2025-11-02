#ifndef _SENSOR_BASE_H
#define _SENSOR_BASE_H

#include <stdint.h>

#include "sensorData.h"

class sensorBase {
  public:
    virtual sensorValuesStruct initialize();
    virtual void sleep();
    virtual int8_t setup(uint8_t calibrationId, uint8_t calibrationStatusId);

  protected:
    uint8_t _calibrationId;
    uint8_t _calibrationStatusId;
};

#endif