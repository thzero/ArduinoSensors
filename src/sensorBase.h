#ifndef _SENSOR_BASE_H
#define _SENSOR_BASE_H

#include "sensorData.h"

class sensorBase {
  public:
    virtual sensorValuesStruct initialize();
    virtual void sleep();
    virtual byte setup();
};

#endif