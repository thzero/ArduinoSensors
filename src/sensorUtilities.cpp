#include <Arduino.h>
#include <math.h>

#include "sensorUtilities.h"

int16_t convertAcc(float value) {
    return round(value * 100);
}

int32_t convertAltitude(float value) {
    return round(value * 1000);
}

float convertAltitudeF(int32_t value) {
    return value / 100.0;
}

int16_t convertAtmosphere(float value) {
    return round(value * 100);
}

float convertDopF(int16_t value) {
    return value / 100.0;
}

int16_t convertGyro(float value) {
    return round(value * 100);
}

float convertLatLngF(int32_t value) {
    return value / 10000000.0;
}

int16_t convertMagnetometer(float value) {
    return round(value * 100);
}