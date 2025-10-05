// based on nmea from madflight
// https://github.com/qqqlab/madflight/blob/main/src/gps/nmea/gps_nmea_pubx_parser.h

#include <stdint.h>
#include <Arduino.h>
#include <limits.h>

// #define DEBUG

class sensorLora {
  public:
    sensorLora() {
    }

    virtual int setup(int baud);
};