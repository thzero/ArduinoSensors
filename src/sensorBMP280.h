// #ifndef _SENSOR_BME_H
// #define _SENSOR_BME_H

// #include <BMP280.h>

// // #include "kalman.h"
// #include "sensorData.h"

// class sensorBMP {
//   public:
//     sensorBMP();
//     // void init(BME280I2C bme);
//     atmosphereValues initializeSensors();
//     void initSensors();
//     atmosphereValues readSensor();
//     float readSensorAltitude();
//     float readSensorAltitude(atmosphereValues values);
//     void sleepSensors();
//     void setupSensors();
    
//   private:
//     BMP280 _sensor = BMP280(0x76);
//     // BMP280 _sensor = BMP280();
//     int _count;
//     // kalman _kalmanAltitude;
//     // kalman _kalmanHumidity;
//     // kalman _kalmanPressure;
//     // kalman _kalmanTemperature;
// };

// #endif