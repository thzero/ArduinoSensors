#include "EEPROM.h"

#include <debug.h>
// #include "kalman.h"
#include "sensorMPU6050.h"
// #include "simulation.h"
#include <utilities.h>

//////////////////////////////////////////////////////////////////////
// Constants
#define IMU_ADDRESS 0x68
#define IMU_GEOMETRY 0
#define IMU_EEPROM_CALIBRATION_STATUS 99

sensorMPU6050::sensorMPU6050() {
}

void sensorMPU6050::calibrationResetCommand() {
  Serial.println(F("\tSetup sensor IMU calibrating..."));
  
  setupCalibration();

  calibrationDisplay(_calibrationData, "");

  int err = _imu.init(_calibrationData, IMU_ADDRESS);
  if (err != 0) {
    Serial.print(F("...sensor IMU error: "));
    Serial.println(err);
  }

  Serial.println(F("\t...sensor IMU calibrated successful."));
}

bool sensorMPU6050::hasMagnetometer() {
  return _imu.hasMagnetometer();
}

accelerometerValues sensorMPU6050::readAccelerometerI(bool update) {
  accelerometerValues values;

  AccelData imuAccel;
  if (update)
    _imu.update();
  _imu.getAccel(&imuAccel);

  values.x = (float)imuAccel.accelX;
  values.y = (float)imuAccel.accelY;
  values.z = (float)imuAccel.accelZ;

#if defined(DEBUG_SENSOR)
  debug(F("accelerometer.x"), values.x);
  debug(F("accelerometer.y"), values.y);
  debug(F("accelerometer.z"), values.z);
#endif

    return values;
}

gyroscopeValues sensorMPU6050::readGyroscopeI(bool update) {
  gyroscopeValues values;

  GyroData imuGyro;
  if (update)
    _imu.update();
  _imu.getGyro(&imuGyro);

  values.x = (float)imuGyro.gyroX;
  values.y = (float)imuGyro.gyroY;
  values.z = (float)imuGyro.gyroZ;

#if defined(DEBUG_SENSOR)
  debug(F("gyro.x"), values.x);
  debug(F("gyro.y"), values.y);
  debug(F("gyro.z"), values.z);
#endif

    return values;
}

int8_t sensorMPU6050::setup(uint8_t calibrationId, uint8_t calibrationStatusId) {
  Serial.println(F("\tSetup sensor IMU..."));

  sensorBase::setup(calibrationId, calibrationStatusId);
  
  _imu.setIMUGeometry(IMU_GEOMETRY);

  int calibrationDataStatus = 0;
  // debug(F("calibrationDataStatus"), calibrationDataStatus);
  EEPROM.get(_calibrationStatusId, calibrationDataStatus);
  if (calibrationDataStatus == IMU_EEPROM_CALIBRATION_STATUS)
    EEPROM.get(_calibrationId, _calibrationData);
  else
    setupCalibration();

  calibrationDisplay(_calibrationData, "\t\t");

  int err = _imu.init(_calibrationData, IMU_ADDRESS);
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

  _calibrationData = { 0 };
  _imu.init(_calibrationData, IMU_ADDRESS);

  delay(1000);

  if (_imu.hasMagnetometer())
    _imu.calibrateMag(&_calibrationData);
  _imu.calibrateAccelGyro(&_calibrationData);
  _imu.init(_calibrationData, IMU_ADDRESS);

  Serial.println(F("\t\tAccelerometer and Gyroscope calibrated!"));

  // debug(F("\t\thasMagnetometer"), _imu.hasMagnetometer());
  if (_imu.hasMagnetometer()) {
    delay(1000);
    Serial.println(F("\t\tMagnetometer calibration: move IMU in figure 8 pattern until done."));
    delay(5000);
    _imu.calibrateMag(&_calibrationData);
    Serial.println(F("\t\tMagnetic calibration done!"));
  }

  Serial.println(F("\t\tIMU Calibration complete!"));
  
  Serial.println(F("\t\tSaving Calibration values to EEPROM!"));
  EEPROM.put(_calibrationId, _calibrationData);
  EEPROM.put(_calibrationStatusId, IMU_EEPROM_CALIBRATION_STATUS);

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