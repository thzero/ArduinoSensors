#ifndef _SENSOR_UTILITIES_H
#define _SENSOR_UTILITIES_H

#include <stdint.h>

extern int16_t convertAcc(float value);
extern float convertAltitudeF(int32_t value);
extern int32_t convertAltitude(float value);
extern int16_t convertAtmosphere(float value);
extern float convertDopF(int16_t value);
extern int16_t convertGyro(float value);
extern int16_t convertMagnetometer(float value);
extern float convertLatLngF(int32_t value);

#endif