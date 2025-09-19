// #ifndef _SENSOR_BME_H
// #define _SENSOR_BME_H

// #include <BMP280.h>

// // #include "kalman.h"
// #include "sensorBase.h"
// #include "sensorData.h"

// class sensorBMP: public sensorBase {
//   public:
//     sensorBMP();
//     // void init(BME280I2C bme);
//     sensorValuesStruct initialize() override;
//     atmosphereValues readAtmosphere() override;
//     float readAltitude() override;
//     float readAltitude(atmosphereValues values) override;
//     void sleep() override;
//     void setup() override;
    
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