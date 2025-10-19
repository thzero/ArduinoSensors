#include "EEPROM.h"

#include <debug.h>
// #include "kalman.h"
#include "sensorMPU6050.h"
// #include "simulation.h"
#include <utilities.h>

//////////////////////////////////////////////////////////////////////
// Constants
#define IMU_ADDRESS 0x68
#define IMU_GEOMETRY 5
#define IMU_EEPROM_CALIBRATION_ID 200
#define IMU_EEPROM_CALIBRATION_STATUS 99
#define IMU_EEPROM_CALIBRATION_STATUS_ID 100

sensorMPU6050::sensorMPU6050() {
}

sensorValuesStruct sensorMPU6050::initialize() {
  sensorValuesStruct values;
  values.acceleration = readAccelerometer();
  values.gyroscope = readGyroscope();
  return values;
}

accelerometerValues sensorMPU6050::readAccelerometer() {
  accelerometerValues values;

  AccelData imuAccel; 
  _imu.update();
  _imu.getAccel(&imuAccel);

  values.x = (float)imuAccel.accelX;
  values.y = (float)imuAccel.accelY;
  values.z = (float)imuAccel.accelZ;

// #ifdef DEV_SIM
//   if (_simulation.isRunning()) { 
//     values.x = 0; // TODO: Simulation currently does not have these values.
//     values.y = 0; // TODO: Simulation currently does not have these values.
//     values.z = _simulation.valueAltitude();
//   }
// #endif

// #if defined(KALMAN) && defined(KALMAN_ACCEL)
//         float value = _kalmanAccelX.kalmanCalc(values.x);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanAccelX="));
//         Serial.println(value);
//   #endif
//         values.x = value;

//         value = _kalmanAccelY.kalmanCalc(values.y);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanAccelY="));
//         Serial.println(value);
//   #endif
//         values.y = value;
        
//         value = _kalmanAccelZ.kalmanCalc(values.z);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanAccelZ="));
//         Serial.println(value);
//   #endif
//         values.z = value;
// #endif

#if defined(DEBUG_SENSOR)
  debug(F("accelerometer.x"), values.x);
  debug(F("accelerometer.y"), values.y);
  debug(F("accelerometer.z"), values.z);
#endif

    return values;
}

gyroscopeValues sensorMPU6050::readGyroscope() {
  gyroscopeValues values;

  GyroData imuGyro;
  _imu.update();
  _imu.getGyro(&imuGyro);

  values.x = (float)imuGyro.gyroX;
  values.y = (float)imuGyro.gyroY;
  values.z = (float)imuGyro.gyroZ;

// #ifdef DEV_SIM
//   if (_simulation.isRunning()) { 
//     // TODO: Simulation currently does not have these values.
//     values.x = 0;
//     values.y = 0;
//     values.z = 0;
//   }
// #endif

// #if defined(KALMAN) && defined(KALMAN_ACCEL)
//         float value = _kalmanGyroX.kalmanCalc(values.x);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanGyroX="));
//         Serial.println(value);
//   #endif
//         values.x = value;

//         value = _kalmanGyroY.kalmanCalc(values.y);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanGyroY="));
//         Serial.println(value);
//   #endif
//         values.y = value;
        
//         value = _kalmanGyroZ.kalmanCalc(values.z);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanGyroZ="));
//         Serial.println(value);
//   #endif
//         values.z = value;
// #endif

#if defined(DEBUG_SENSOR)
  debug(F("gyro.x"), values.x);
  debug(F("gyro.y"), values.y);
  debug(F("gyro.z"), values.z);
#endif

    return values;
}

magnetometerValues sensorMPU6050::readMagnetometer() {
  magnetometerValues values;

  values.x = 0.0;
  values.y = 0.0;
  values.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("magnetometer.x"), values.x);
  debug(F("magnetometer.y"), values.y);
  debug(F("magnetometer.z"), values.z);
#endif

    return values;
}

void sensorMPU6050::sleep() {
  Serial.println(F("\tSleep sensor IMU..."));
  
  // _qmi.disableGyroscope();
  // _qmi.disableAccelerometer();

  Serial.println(F("\t...sensor IMU sleep successful."));
}

byte sensorMPU6050::setup() {
  Serial.println(F("\tSetup sensor IMU..."));
  
  _imu.setIMUGeometry(IMU_GEOMETRY);

  int calibrationDataStatus = 0;
  // debug(F("calibrationDataStatus"), calibrationDataStatus);
  EEPROM.get(IMU_EEPROM_CALIBRATION_STATUS_ID, calibrationDataStatus);
  if (calibrationDataStatus == IMU_EEPROM_CALIBRATION_STATUS)
    EEPROM.get(IMU_EEPROM_CALIBRATION_ID, calibrationData);
  else
    setupCalibration();

  calibrationDisplay(calibrationData, "\t\t");

  int err = _imu.init(calibrationData, IMU_ADDRESS);
  if (err != 0) {
    Serial.print(F("\t...sensor IMU error: "));
    Serial.println(err);
    return 1;
  }

  Serial.println(F("\t...sensor IMU successful."));
  return 0;
}

void sensorMPU6050::setupCalibration() {
  Serial.println(F("\t\tSetup sensor IMU calibrating... keep rocket perpendicular to a level surface"));

  calibrationData = { 0 };
  _imu.init(calibrationData, IMU_ADDRESS);

  delay(1000);

  _imu.calibrateAccelGyro(&calibrationData);
  _imu.init(calibrationData, IMU_ADDRESS);

  Serial.println(F("\t\tAccelerometer and Gyroscope calibrated!"));

  // debug(F("\t\thasMagnetometer"), _imu.hasMagnetometer());
  if (_imu.hasMagnetometer()) {
    delay(1000);
    Serial.println(F("\t\tMagnetometer calibration: move IMU in figure 8 pattern until done."));
    delay(5000);
    _imu.calibrateMag(&calibrationData);
    Serial.println(F("\t\tMagnetic calibration done!"));
  }

  Serial.println(F("\t\tIMU Calibration complete!"));
  
  Serial.println(F("\t\tSaving Calibration values to EEPROM!"));
  EEPROM.put(IMU_EEPROM_CALIBRATION_ID, calibrationData);
  EEPROM.put(IMU_EEPROM_CALIBRATION_STATUS_ID, IMU_EEPROM_CALIBRATION_STATUS);

  delay(1000);

  Serial.println(F("\t\t...sensor IMU calibration successful."));
}

void sensorMPU6050::calibrationDisplay(calData calibrationData, const char* offset) {
  Serial.print(offset);
  Serial.println(F("IMU Bias"));

  Serial.print(offset);
  Serial.print(F("\tAccel biases X/Y/Z: "));
  Serial.print(calibrationData.accelBias[0]);
  Serial.print(F(", "));
  Serial.print(calibrationData.accelBias[1]);
  Serial.print(F(", "));
  Serial.println(calibrationData.accelBias[2]);
  Serial.print(offset);
  Serial.print(F("\tGyro biases X/Y/Z: "));
  Serial.print(calibrationData.gyroBias[0]);
  Serial.print(F(", "));
  Serial.print(calibrationData.gyroBias[1]);
  Serial.print(F(", "));
  Serial.println(calibrationData.gyroBias[2]);
  // debug(F("hasMagnetometer"), _imu.hasMagnetometer());
  if (_imu.hasMagnetometer()) {
    Serial.print(offset);
    Serial.print(F("\tMag biases X/Y/Z: "));
    Serial.print(calibrationData.magBias[0]);
    Serial.print(F(", "));
    Serial.print(calibrationData.magBias[1]);
    Serial.print(F(", "));
    Serial.println(calibrationData.magBias[2]);
    Serial.print(offset);
    Serial.print(F("\tMag Scale X/Y/Z: "));
    Serial.print(calibrationData.magScale[0]);
    Serial.print(F(", "));
    Serial.print(calibrationData.magScale[1]);
    Serial.print(F(", "));
    Serial.println(calibrationData.magScale[2]);
  }
}

void sensorMPU6050::calibrationResetCommand() {
  Serial.println(F("\tSetup sensor IMU calibrating..."));
  
  setupCalibration();

  calibrationDisplay(calibrationData, "");

  int err = _imu.init(calibrationData, IMU_ADDRESS);
  if (err != 0) {
    Serial.print(F("...sensor IMU error: "));
    Serial.println(err);
  }

  Serial.println(F("\t...sensor IMU calibrated successful."));
}